"""Lobed mixer, tail cone, augmentor case and liner, and the integrated
flameholders.

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
    out["flameholder_vanes"] = _vanes()
    out["ab_fuel_manifold"] = _ab_fuel()
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
    out["augmentor_liner"] = mesh.join(
        common.ring(x0, x1, r, r + t, step=100.0),
        common.ring(x0, x0 + 10.0, r + t - 1.0, case_bore(x0 + 5.0)))
    holes = []
    nr, nc = A["screech_rows"], A["screech_per_row"]
    for i in range(nr):
        x = x0 + 60.0 + (x1 - x0 - 120.0) * i / (nr - 1)
        for k in range(nc):
            clock = 360.0 * (k + 0.5 * (i % 2)) / nc
            holes.append(common.radial_pin(x, r - 3.0, r + t + 3.0,
                                           A["screech_hole_r"], clock, 8))
    out["cut:augmentor_liner"] = mesh.join(*holes)
    return out


def _vanes():
    """Sixteen radial flameholder vanes from the tail cone to the liner: a
    thick symmetric section whose wake is where the flame sits."""
    row = spec.BladeRow("flameholder", "core", A["vane_x0"], A["vane_chord"],
                        A["n_vanes"], 0.0, 0.0,
                        thickness=A["vane_t"] / A["vane_chord"], camber=0.0,
                        rotor=False)
    one = blades.loft(row, lambda x: tailcone_r(x) - spec.ROOT_EMBED,
                      lambda x: A["liner_r"] + spec.ROOT_EMBED, 30, 9)
    return mesh.replicate(*one, row.count)


def _ab_fuel():
    """The reheat fuel manifold round the outer case, and a feed down into
    every flameholder vane, where the spraybar is."""
    x = A["ab_manifold_x"]
    r_od = spec.annulus(P["third"], x)[1] + spec.WALL["outer_case"]
    tube = 9.0
    rm = r_od + tube + 3.0
    parts = [mesh.ring_torus(x, rm, tube, SEG, 12)]
    n = A["n_vanes"]
    for k in range(n):
        clock = 360.0 * k / n
        parts.append(common.radial_pin(x, A["liner_r"] - 30.0, rm + 3.0, 5.0,
                                       clock, 10))
    return mesh.join(*parts)
