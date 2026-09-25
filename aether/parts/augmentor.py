"""Lobed mixer, tail cone, augmentor case and liner, spray rings and the
V-gutter flameholder.

The core and the bypass air arrive at the mixer at the same total pressure
-- the cycle solves the bypass ratio for exactly that -- and sixteen lobes
fold them into each other so the augmentor burns a mixed stream at 980 K
rather than a hot core wrapped in cold air.

There are no V-gutter rings. The flameholders are sixteen radial vanes from
the tail cone to the liner, each a gutter with the spraybar inside it: the
flame holds in the wake of the vane's blunt trailing edge. That takes the
rings out of the dry-power gas path, where they are nothing but drag, and
out of the line of sight to the hot turbine.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec          # noqa: E402
import mesh          # noqa: E402
import blades        # noqa: E402
from parts import common   # noqa: E402

SEG = common.SEG
A = spec.AUGMENTOR
P = spec.PATHS


def third_in(x):
    return spec.annulus(P["third"], x)[0]


def tailcone_r(x):
    """An ellipse from the LP turbine's hub radius down to a point."""
    x0, x1 = A["tailcone_x0"], A["tailcone_x1"]
    r0 = common.hub("core", spec.CORE_PATH[-1][0])
    t = min(max((x - x0) / (x1 - x0), 0.0), 1.0)
    return r0 * math.sqrt(max(0.0, 1.0 - t * t))


def build():
    out = {}
    out["mixer"] = _mixer()
    out["tailcone"] = _tailcone()
    out["augmentor_case"] = common.ring(
        A["x_mixer0"], A["x_liner1"], lambda x: third_in(x) - A["case_t"],
        third_in, (P["third"],), step=50.0)
    out.update(_liner())
    out["flameholder"] = _flameholder()
    out["ab_fuel_manifold"] = _ab_fuel()
    out["ab_spraybars"] = _spraybars()
    out["ab_spray_rings"] = _spray_rings()
    out["ab_igniter"] = _igniter()
    out["ab_fuel_control"] = _fuel_control()
    return out


def _mixer():
    """A lobed sheet from the turbine case's trailing edge to the start of
    the augmentor. At its front it is a plain ring on the core cowl's line;
    the lobes grow to full depth by its trailing edge, alternately reaching
    out into the bypass air and down into the core."""
    x0, x1 = A["x_mixer0"], A["x_mixer1"]
    rm = spec.annulus(P["bypass"], x0)[0]
    t = A["mixer_t"]
    n_l = A["n_lobes"]
    n_th = n_l * 12
    n_x = 22
    depth = A["lobe_depth"]
    outer, inner = [], []
    for i in range(n_x + 1):
        s = i / n_x
        x = x0 + (x1 - x0) * s
        amp = depth * (3 * s * s - 2 * s * s * s)      # smoothstep
        for j in range(n_th):
            th = 2.0 * math.pi * j / n_th
            r = rm - 1.0 + amp * math.cos(n_l * th)
            for lst, rr in ((outer, r + t / 2), (inner, r - t / 2)):
                lst.append((x, rr * math.cos(th), rr * math.sin(th)))
    verts = outer + inner
    no = len(outer)
    faces = []
    for i in range(n_x):
        for j in range(n_th):
            j2 = (j + 1) % n_th
            a, b = i * n_th + j, i * n_th + j2
            c, d = (i + 1) * n_th + j2, (i + 1) * n_th + j
            faces.append((a, b, c, d))
            faces.append((no + d, no + c, no + b, no + a))
    for i, flip in ((0, True), (n_x, False)):
        for j in range(n_th):
            j2 = (j + 1) % n_th
            o0, o1 = i * n_th + j, i * n_th + j2
            f = (o0, no + o0, no + o1, o1)
            faces.append(tuple(reversed(f)) if not flip else f)
    return common.orient((verts, faces))


def _tailcone():
    x0, x1 = A["tailcone_x0"], A["tailcone_x1"]
    n = 30
    prof = [(x0, 0.001)]
    for i in range(n + 1):
        # cluster points toward the point, where the ellipse turns fastest
        t = math.sin(0.5 * math.pi * i / n)
        x = x0 + (x1 - x0) * t
        prof.append((x, max(tailcone_r(x), 0.001)))
    prof[-1] = (x1, 0.001)
    return mesh.revolve_open(prof, SEG, cap_start=True, cap_end=True)


def _liner():
    """The screech liner: a perforated CMC sleeve inside the case, hung from
    its front flange and free to slide at the back. The holes are tuned
    Helmholtz dampers against the pressure oscillation a reheat flame drives
    in a long duct."""
    out = {}
    x0, x1 = A["x_liner0"], A["x_liner1"]
    r = A["liner_r"]
    t = A["liner_t"]
    case_bore = lambda x: third_in(x) - A["case_t"]
    # corrugated along its length, as a reheat liner is: the corrugations
    # let it grow hot without buckling and stiffen it against the screech
    out["augmentor_liner"] = mesh.join(
        common.ring(x0, x1, lambda x: liner_r(x), lambda x: liner_r(x) + t,
                    step=A["liner_pitch"] / 8.0),
        common.ring(x0, x0 + 10.0, r + t - 1.0, case_bore(x0 + 5.0)))
    holes = []
    nr, nc = A["screech_rows"], A["screech_per_row"]
    for i in range(nr):
        x = x0 + 60.0 + (x1 - x0 - 120.0) * i / (nr - 1)
        for k in range(nc):
            clock = 360.0 * (k + 0.5 * (i % 2)) / nc
            holes.append(common.radial_pin(x, r - 6.0, r + t + 6.0,
                                           A["screech_hole_r"], clock, 8))
    out["cut:augmentor_liner"] = mesh.join(*holes)
    return out


def liner_r(x):
    """The corrugated liner's bore: a whole number of waves along it, so it
    meets the nozzle's liner on its nominal radius."""
    return A["liner_r"] + A["liner_wave"] * math.sin(
        2.0 * math.pi * (x - A["x_liner0"]) / A["liner_pitch"])


def _plate_arm(x0, a0, x1, a1, r0, r1, t, clock):
    """One flat arm of a radial gutter: a plate from (x0, tangential a0) to
    (x1, a1) in the gutter's own section, extruded from r0 to r1, t thick,
    turned to its clock angle. Convex, so its ends cap cleanly."""
    L = math.hypot(x1 - x0, a1 - a0)
    nx, na = -(a1 - a0) / L * t / 2, (x1 - x0) / L * t / 2
    sect = [(x0 - nx, a0 - na), (x1 - nx, a1 - na), (x1 + nx, a1 + na),
            (x0 + nx, a0 + na)]
    c = math.radians(clock)
    cc, sc = math.cos(c), math.sin(c)
    rings = []
    for r in (r0, r1):
        rings.append([(x, r * cc - a * sc, r * sc + a * cc) for (x, a) in sect])
    v = [p for rg in rings for p in rg]
    f = [(0, 1, 2, 3), (7, 6, 5, 4), (0, 4, 5, 1), (1, 5, 6, 2), (2, 6, 7, 3),
         (3, 7, 4, 0)]
    return common.orient((v, f))


def _flameholder():
    """The flameholder: V-gutters, open aft, in whose wake the flame holds.

    Three concentric rings, tied together and held by sixteen radial gutters
    that run from the tail cone to the liner -- which is also what carries
    the tail cone. A gutter makes a pocket of slow recirculating gas behind
    its open mouth; fuel sprayed ahead of it burns there steadily and lights
    the stream going past. The inner ring, closest to the tail cone where
    the stream is slowest, is the pilot."""
    x0, dep = A["vane_x0"], A["vane_chord"]
    w, t = A["gutter_w"], A["gutter_t"]
    parts = []
    n = A["n_vanes"]
    r_in = tailcone_r(x0 + dep) - 3.0
    r_out = A["liner_r"] + A["liner_wave"] + 1.0
    for k in range(n):
        clock = 360.0 * k / n
        for sgn in (1.0, -1.0):
            parts.append(_plate_arm(x0, 0.0, x0 + dep, sgn * w / 2, r_in, r_out,
                                    t, clock))
    # the rings: a V in the meridional plane, apex forward
    rw, rd = A["ring_w"], A["ring_depth"]
    xa = x0 + 4.0
    for r in A["ring_r"]:
        loop = [(xa - 1.5, r), (xa + rd, r + rw / 2), (xa + rd, r + rw / 2 - t),
                (xa + 2.5, r), (xa + rd, r - rw / 2 + t), (xa + rd, r - rw / 2)]
        parts.append(mesh.revolve_ring(loop, SEG))
    return mesh.join(*parts)


TUBE = 9.0


def manifold_r(x):
    return spec.annulus(P["third"], x)[1] + spec.WALL["outer_case"] + TUBE + 3.0


def _ab_fuel():
    """The three reheat zone manifolds round the outer case, and zone 1's
    feeds down through the case and liner to its spray ring."""
    parts = []
    for x in A["zone_x"]:
        parts.append(mesh.ring_torus(x, manifold_r(x), TUBE, SEG, 12))
    x = A["zone_x"][0]
    n = A["n_vanes"]
    for k in range(n):
        parts.append(common.radial_pin(x, A["spray_ring_r"][0] - 2.0,
                                       manifold_r(x) + 3.0, 5.0,
                                       360.0 * (k + 0.5) / n, 10))
    return mesh.join(*parts)


def _spraybars():
    """Zones 2 and 3: a radial spraybar off each manifold between every
    pair of vanes, down through the case and liner into the stream, with a
    row of orifices on its downstream side. Zone 2 sprays the outer half of
    the stream, zone 3 reaches in to the tail cone."""
    n = A["n_vanes"]
    parts = []
    for x, r_in in zip(A["zone_x"][1:], A["spraybar_r"]):
        rm = manifold_r(x)
        for k in range(n):
            clock = 360.0 * (k + 0.5) / n
            parts.append(common.radial_pin(x, r_in, rm + 2.0, 6.0, clock, 12))
            # a rounded tip, and the orifices down the aft face
            parts.append(common.radial_pin(x, r_in - 3.0, r_in + 1.0, 4.5, clock, 12))
            for j in range(5):
                r = r_in + 12.0 + (A["liner_r"] - 20.0 - r_in - 12.0) * j / 4
                parts.append(mesh.pipe([common.polar(x + 4.0, r, clock),
                                        common.polar(x + 9.0, r, clock)], 2.4, 8))
    return mesh.join(*parts)


def _spray_rings():
    """A spray ring in the stream for each zone, on the ends of its feeds:
    a tube with orifices round its aft face, spraying into the gutters'
    wake. Zone 1 is the outer ring, just ahead of the gutters; zones 2 and 3
    further forward, on the tips of their spraybars."""
    parts = []
    tr = A["spray_tube_r"]
    for x, r in zip(A["zone_x"], A["spray_ring_r"]):
        parts.append(mesh.ring_torus(x, r, tr, SEG, 12))
        for k in range(48):
            clock = 360.0 * (k + 0.25) / 48
            parts.append(mesh.pipe([common.polar(x + tr - 2.0, r, clock),
                                    common.polar(x + tr + 4.0, r, clock)], 2.0, 8))
    return mesh.join(*parts)


def _igniter():
    """The reheat igniter: through a boss on the case at two o'clock, its
    tip in the wake of the middle gutter ring, and the lead's connector on
    top."""
    x, clock = A["igniter_x"], 60.0
    r_od = spec.annulus(P["third"], x)[1] + spec.WALL["outer_case"]
    return mesh.join(
        common.radial_pin(x, A["ring_r"][1] + 30.0, r_od + 34.0, 7.0, clock, 14),
        common.radial_pin(x, r_od - 2.0, r_od + 16.0, 17.0, clock, 18),
        common.radial_pin(x, r_od + 30.0, r_od + 52.0, 11.0, clock, 16))


FC_CLOCK = -112.0


def fuel_control_inlet():
    """Where the reheat feed plugs into the control unit's front face."""
    x0 = A["fuel_control_x"][0]
    return common.polar(x0 - 1.0, manifold_r(x0) + 26.0, FC_CLOCK)


def _fuel_control():
    """The reheat fuel control on the case at seven o'clock: it meters the
    reheat feed and splits it three ways through the zone valves, one short
    line to each zone's manifold."""
    x0, x1 = A["fuel_control_x"]
    r_od = spec.annulus(P["third"], x0)[1] + spec.WALL["outer_case"]
    t = math.radians(FC_CLOCK)
    body = common.sector_block(x0, x1, r_od - 1.0, r_od + 64.0, t - 0.1, t + 0.1, 6)
    # three zone valves standing on its back
    parts = [body]
    for j, dc in enumerate((-4.0, 0.0, 4.0)):
        xv = x0 + 30.0 + 30.0 * j
        parts.append(common.radial_pin(xv, r_od + 62.0, r_od + 82.0, 11.0,
                                       FC_CLOCK + dc, 16))
    # feeds from its aft face to each manifold, at a clock apiece
    for j, (x, dc) in enumerate(zip(A["zone_x"], (-5.0, 0.0, 5.0))):
        c = FC_CLOCK + dc
        rm = manifold_r(x)
        parts.append(mesh.pipe([common.polar(x1 - 2.0, r_od + 20.0 + 14.0 * j, c),
                                common.polar(x - 30.0, r_od + 20.0 + 14.0 * j, c),
                                common.polar(x, rm + TUBE + 8.0, c),
                                common.polar(x, rm + TUBE - 3.0, c)],
                               6.0, 12, bend=16.0))
    return mesh.join(*parts)
