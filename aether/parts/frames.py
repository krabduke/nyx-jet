"""Static structure: the fan frame, the three walls that divide the streams,
the outer and core casings, flanges, the mid-turbine frame's service struts,
the third-stream mode valve and heat exchanger.

Three streams means three concentric walls where a conventional turbofan has
two. From the outside in:

    outer case         -- the engine's skin, over the third stream
    intermediate case  -- third stream outside it, core + bypass inside;
                          its nose is the third-stream splitter
    core cowl          -- bypass outside it, the core casings inside;
                          its nose is the core splitter

Each wall's two faces are read off the stream tables on either side of it,
so the wall is exactly as thick as the gap between the streams it divides.
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
P = spec.PATHS


def third_in(x):
    return spec.annulus(P["third"], x)[0]


def third_out(x):
    return spec.annulus(P["third"], x)[1]


def outer_od(x):
    return third_out(x) + spec.WALL["outer_case"]


def build():
    out = {}
    out.update(_walls())
    out.update(_fan_frame())
    out.update(_outer_cases())
    out.update(_flanges())
    out["service_struts"] = _service_struts()
    out["mode_valve"] = _mode_valve()
    out["tms_hx"] = _heat_exchanger()
    out["case_ribs"] = _case_ribs()
    return out


def _case_ribs():
    """The orthogrid on the outer cases: rings and stringers standing on the
    skin, 1 mm let into it, each stringer broken where something is bolted
    through or on to the case."""
    cr = spec.CASE_RIBS
    parts = []
    for x in cr["rings"]:
        parts.append(common.ring(x - cr["w"] / 2, x + cr["w"] / 2,
                                 lambda xx: outer_od(xx) - 1.0,
                                 lambda xx: outer_od(xx) + cr["h"], step=cr["w"]))
    # stringer runs between the gaps
    runs, x = [], cr["x0"]
    for g0, g1 in sorted(cr["gaps"]):
        runs.append((x, g0))
        x = g1
    runs.append((x, cr["x1"]))
    n = cr["n_stringers"]
    for k in range(n):
        clock = cr["phase_deg"] + 360.0 * k / n
        for a, b in runs:
            parts.append(common.strip(a, b, outer_od, cr["h"], cr["w"], clock,
                                      (P["third"],)))
    return mesh.join(*parts)


# --------------------------------------------------------------------------

def _walls():
    out = {}
    x3 = spec.SPLITTER3_X
    x2 = spec.SPLITTER2_X
    x_mix = spec.AUGMENTOR["x_mixer0"]

    def inter_in(x):
        # the CDFS tip line ahead of the core splitter, the bypass duct's
        # outer line behind it -- and never closer than 3 mm to the outside,
        # so the splitter's leading edge is a lip and not a knife
        r = common.tip("cdfs", x) if x < x2 else spec.annulus(P["bypass"], x)[1]
        return min(r, third_in(x) - 3.0)

    out["intermediate_case"] = common.ring(
        x3, x_mix, inter_in, third_in,
        (P["cdfs"], P["bypass"], P["third"]), step=20.0)

    x_hpc = spec.HPC_ROWS[0].x
    byp_in = lambda x: spec.annulus(P["bypass"], x)[0]

    def split_in(x):
        return min(common.tip("core", x), byp_in(x) - 3.0)

    out["core_splitter"] = common.ring(x2, x_hpc, split_in, byp_in,
                                       (P["core"], P["bypass"]), step=10.0)
    x_comb = _x_comb()
    out["core_cowl"] = common.ring(
        x_hpc, x_comb, lambda x: byp_in(x) - spec.WALL["core_cowl"], byp_in,
        (P["bypass"],), step=40.0)
    return out


def _x_comb():
    """Where the HP compressor case ends and the combustor case begins: just
    aft of the last rotor, under the exit guide vanes' leading edge."""
    return spec.COMBUSTOR["x_ogv"] - 10.0


# --------------------------------------------------------------------------

def _strut_row(name, t_c):
    """A pseudo blade row for a frame strut: symmetric, unstaggered, from
    the inner flow's hub to the outer case, so the loft follows both walls."""
    f = spec.FAN_FRAME
    return spec.BladeRow(name, "cdfs", f["x0"], f["x1"] - f["x0"], 1, 0.0, 0.0,
                         thickness=t_c, camber=0.0, rotor=False)


def _fan_frame():
    out = {}
    f = spec.FAN_FRAME
    w = spec.WALL["hub_ring"]
    x0 = spec.SPLITTER3_X
    x1 = f["hub_x1"]
    h = lambda x: common.hub("cdfs", x)
    # the inner hub ring, from the fan exit vanes' band to just short of the
    # CDFS rim; the sump hangs off its underside
    out["fan_frame_hub"] = common.ring(x0, x1, lambda x: h(x) - w, h,
                                       (P["cdfs"],), step=20.0)
    struts = []
    root = lambda x: common.hub("cdfs", x) - spec.ROOT_EMBED
    top = lambda x: third_out(x) + spec.ROOT_EMBED
    n = f["n_struts"]
    for k in range(n):
        clock = 360.0 * k / n
        king = abs(((clock - 270.0) + 180.0) % 360.0 - 180.0) < 1e-6
        row = _strut_row("fan_frame_strut", f["t_c_king"] if king else f["t_c"])
        v, fc = blades.loft(row, root, top, 30, 7)
        struts.append((mesh.rot_x(v, math.radians(clock)), fc))
    out["fan_frame_struts"] = mesh.join(*struts)
    return out


def x_hpc_front():
    return spec.HPC_ROWS[0].x


# --------------------------------------------------------------------------

def _outer_cases():
    out = {}
    for name, x0, x1 in spec.OUTER_CASES:
        if name == "case_fan":
            continue            # the fan module builds the fan case
        out[name] = common.ring(x0, x1, third_out, outer_od, (P["third"],),
                                step=40.0)
    return out


def _od_at(x):
    """Outside of whichever case is at station x."""
    if x < spec.SPINNER["x_base"]:
        from parts import fan
        return fan.case_od(spec.SPINNER["x_base"])
    if x < spec.SPLITTER3_X:
        from parts import fan
        return fan.case_od(x)
    return outer_od(x)


def _flanges():
    """A bolted collar over each joint, bored to the smaller of the two cases
    it joins, with a ring of bolt heads on its aft face."""
    out = {}
    for name, x, r_out, t, nb in spec.FLANGES:
        r_in = min(_od_at(x - t / 2.0), _od_at(x + t / 2.0 + 0.01))
        ringv = common.ring(x - t / 2.0, x + t / 2.0, r_in, r_out)
        pitch = 0.5 * (r_in + r_out)
        bolts = mesh.bolt_ring(x + t / 2.0, pitch, nb, 6.5, 6.0, 10)
        out[name] = mesh.join(ringv, bolts)
    return out


# --------------------------------------------------------------------------

def _service_struts():
    """Four struts across the bypass and third streams at the mid-turbine
    frame, tying the turbine case out to the outer case. The oil lines to and
    from the rear sump run inside the ones at 3 and 9 o'clock -- here the
    struts are solid, so the lines are inside them."""
    m = spec.MTF
    x0 = spec.row("mtf").x + 5.0
    row = spec.BladeRow("service_strut", "bypass", x0, m["service_chord"], 1,
                        0.0, 0.0, thickness=m["service_t"] / m["service_chord"],
                        camber=0.0, rotor=False)
    root = lambda x: spec.annulus(P["bypass"], x)[0] - spec.ROOT_EMBED
    top = lambda x: third_out(x) + spec.ROOT_EMBED
    v, f = blades.loft(row, root, top, 30, 7)
    out = []
    for k in range(m["n_service"]):
        out.append((mesh.rot_x(v, math.radians(90.0 * k)), f))
    return mesh.join(*out)


def _mode_valve():
    """Petals round the third stream's entrance, hinged on the intermediate
    case. Drawn at the maximum-thrust setting: lifted 14 degrees into the
    stream, which chokes the third stream down to the tenth of the airflow
    the cycle gives it. At cruise they lie flat and it takes a quarter."""
    mv = spec.MODE_VALVE
    x0 = mv["x_hinge"]
    r0 = third_in(x0)
    a = math.radians(mv["deflect"])
    t = mv["t"]
    L = mv["len"]
    p0 = (x0, r0)
    p1 = (x0 + L * math.cos(a), r0 + L * math.sin(a))
    p2 = (p1[0] - t * math.sin(a), p1[1] + t * math.cos(a))
    p3 = (x0 - t * math.sin(a), r0 + t * math.cos(a))
    parts = []
    n = mv["n"]
    dphi = 2.0 * math.pi / n
    for k in range(n):
        c = k * dphi
        prof = [p0, p1, p2, p3]
        v, f = [], []
        m = 4
        for j in range(m + 1):
            ang = c - 0.45 * dphi + 0.9 * dphi * j / m
            ca, sa = math.cos(ang), math.sin(ang)
            for (x, r) in prof:
                v.append((x, r * ca, r * sa))
        for j in range(m):
            b0, b1 = j * 4, (j + 1) * 4
            for i in range(4):
                i2 = (i + 1) % 4
                f.append((b0 + i, b0 + i2, b1 + i2, b1 + i))
        f.append((3, 2, 1, 0))
        b = m * 4
        f.append((b, b + 1, b + 2, b + 3))
        parts.append(common.orient((v, f)))
    # the hinge ring the petals turn on, let into the case
    parts.append(mesh.ring_torus(x0 + 2.0, r0 + 1.0, 4.0, SEG, 12))
    return mesh.join(*parts)


def _heat_exchanger():
    """Twelve plate-fin segments filling the third stream from wall to wall:
    the aircraft's heat load -- avionics, sensors, a directed-energy weapon's
    waste heat -- goes into the third stream here and out of the nozzle.
    Each segment is faced with its fins so it reads as a matrix, not a
    block."""
    h = spec.TMS_HX
    x0, x1 = h["x0"], h["x1"]
    # 0.2 mm off each wall: close enough to be held by them (the audits'
    # contact tolerance is 0.3), and not a coincident surface that reads as
    # neither in nor out
    r0, r1 = third_in(x0) + 0.2, third_out(x0) - 0.2
    n = h["n"]
    gap = math.radians(h["gap_deg"])
    parts = []
    for k in range(n):
        a0 = 2.0 * math.pi * k / n + gap / 2.0
        a1 = 2.0 * math.pi * (k + 1) / n - gap / 2.0
        parts.append(common.sector_block(x0, x1, r0, r1, a0, a1, 10))
        # fins standing proud of the front face
        for j in range(1, 8):
            rr = r0 + (r1 - r0) * j / 8.0
            parts.append(common.sector_block(x0 - 6.0, x0, rr - 0.8, rr + 0.8,
                                             a0 + 0.01, a1 - 0.01, 8))
    return mesh.join(*parts)
