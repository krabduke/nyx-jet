"""The core-driven fan stage and the six-stage HP compressor, both on the HP
spool, with the drum that carries them, their casings and the variable-vane
actuation.

The HP spool's rotating structure, front to back: the CDFS blisk, whose cone
comes down onto the HP shaft behind bearing 3; a front drum that runs under
the CDFS stator, the core splitter and the inlet guide vane to the
compressor's first disc; the compressor drum with a disc under every rotor;
and a rear cone from the last disc back down onto the shaft. The static
bands under every stator sit over the drum with a 3 mm seal gap.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec          # noqa: E402
import mesh          # noqa: E402
from parts import common   # noqa: E402

SEG = common.SEG
P = spec.PATHS
CDFS_R, CDFS_S = spec.CDFS_ROWS
HPC = spec.HPC_ROWS
ROTORS = [r for r in HPC if r.rotor]
STATORS = [r for r in HPC if not r.rotor]

DRUM_IN = 238.0       # inner skin of the compressor drum and the front drum
BORE_R = 150.0        # compressor disc bores, clear of the HP shaft


def build():
    out = {}
    out.update(_cdfs())
    out.update(_hpc_rows())
    out.update(_drums())
    out.update(_case())
    out["vsv_actuation"] = _vsv()
    return out


# --------------------------------------------------------------------------

def _cdfs():
    out = {}
    sh = spec.SHAFTS
    x0 = spec.FAN_FRAME["cdfs_rim_x0"]
    _, x1 = common.reach(CDFS_R)
    web = 0.5 * (x0 + x1)
    loop = common.rim_profile(x0, x1, "cdfs", 14.0, web, 22.0, 175.0, 34.0)
    # the cone from the disc's bore down onto the HP shaft, aft of bearing 3
    cone = common.revolve([(web + 12.0, 175.0), (web + 26.0, 175.0),
                           (web + 62.0, sh["hp_r_out"]),
                           (web + 48.0, sh["hp_r_out"])])
    out["cdfs_blisk"] = mesh.join(common.revolve(loop), cone,
                                  common.rotor_blades(CDFS_R))
    clr = spec.TIP_CLEARANCE["cdfs"]
    # The CDFS stator's band runs on to the core splitter's station, where
    # the inlet guide vane's band takes over the static inner wall.
    s0, _ = common.reach(CDFS_S)
    out["vanes_cdfs"] = mesh.join(
        common.stator_vanes(CDFS_S, band=False, top_embed=spec.ROOT_EMBED),
        common.ring(s0, spec.SPLITTER2_X,
                    lambda x: common.hub("cdfs", x) - spec.BAND_DEPTH,
                    lambda x: common.hub("cdfs", x), (P["cdfs"],), step=10.0))
    return out


def _hpc_rows():
    out = {}
    igv = HPC[0]
    _, i1 = common.reach(igv)
    # the swan neck's inner wall is the inlet guide vane's band, run forward
    # to the core splitter's station
    out["vanes_hpc_igv"] = mesh.join(
        common.stator_vanes(igv, band=False),
        common.ring(spec.SPLITTER2_X, i1,
                    lambda x: common.hub("core", x) - spec.BAND_DEPTH,
                    lambda x: common.hub("core", x), (P["core"],), step=8.0))
    for row in HPC[1:]:
        if row.rotor:
            out[f"blades_{row.name}"] = common.rotor_blades(row)
        else:
            out[f"vanes_{row.name}"] = common.stator_vanes(row)
    return out


# --------------------------------------------------------------------------

def _drum_top():
    """The drum's outside, front to back: the hub line over every rotor's
    rim, dropped below the static band over every stator."""
    pts = []
    drop = spec.BAND_DEPTH + spec.DRUM_RECESS
    for i, row in enumerate(ROTORS):
        a, b = common.reach(row)
        xs = common.knots(a, b, P["core"], step=6.0)
        if pts:
            # step down into the recess under the stator ahead
            pa = pts[-1][0]
            pts.append((pa, common.hub("core", pa) - drop))
            pts.append((a, common.hub("core", a) - drop))
        pts += [(x, common.hub("core", x)) for x in xs]
    return pts


def _drums():
    out = {}
    sh = spec.SHAFTS
    top = _drum_top()
    x_front, x_rear = top[0][0], top[-1][0]
    loop = top + [(x_rear, DRUM_IN), (x_front, DRUM_IN)]
    parts = [common.revolve(loop)]
    # a disc under every rotor
    for row in ROTORS:
        a, b = common.reach(row)
        c = 0.5 * (row.x + row.x_te)
        wt, bw = 0.45 * row.chord, 0.95 * row.chord
        parts.append(common.revolve([
            (c - wt / 2, DRUM_IN + 4.0), (c - wt / 2, BORE_R + 16.0),
            (c - bw / 2, BORE_R + 16.0), (c - bw / 2, BORE_R),
            (c + bw / 2, BORE_R), (c + bw / 2, BORE_R + 16.0),
            (c + wt / 2, BORE_R + 16.0), (c + wt / 2, DRUM_IN + 4.0)]))
    # the rear cone, from the last disc down onto the HP shaft
    parts.append(common.revolve([
        (x_rear - 16.0, DRUM_IN), (x_rear, DRUM_IN),
        (x_rear + 66.0, sh["hp_r_out"]), (x_rear + 52.0, sh["hp_r_out"])]))
    out["hpc_drum"] = mesh.join(*parts)

    # the front drum, from the CDFS rim under the static walls to the first
    # compressor disc's front face
    _, c1 = common.reach(CDFS_R)
    rim_in = min(common.hub("cdfs", x) for x in (spec.FAN_FRAME["cdfs_rim_x0"],
                                                   c1)) - 14.0
    out["hp_front_drum"] = common.revolve([
        (c1, DRUM_IN), (c1, rim_in + 2.0), (c1 + 6.0, rim_in + 2.0),
        (c1 + 16.0, DRUM_IN + 12.0), (x_front, DRUM_IN + 12.0),
        (x_front, DRUM_IN)])
    return out


# --------------------------------------------------------------------------

def case_bore(x):
    return common.tip("core", x)


def case_od(x):
    return case_bore(x) + spec.WALL["core_case"]


def _case():
    """The HP compressor case: its bore IS the core's tip line, which the
    rotor tips run 1 mm under and the stator vanes are let 2 mm into."""
    x0 = HPC[0].x
    x1 = spec.COMBUSTOR["x_ogv"] - 10.0
    return {"case_hpc": common.ring(x0, x1, case_bore, case_od, (P["core"],),
                                    step=20.0)}


def _vsv():
    """Variable stator actuation for the inlet guide vane and the first two
    stators: a unison ring round the case for each row, a spindle up from
    every vane through the case into its ring, and a tie rod along the top
    of the case that makes the three rings move as one. There are 3 mm
    between the rings and the core cowl, so the actuator that drives the rod
    lives outside the core, at the fan frame, and is not modelled."""
    parts = []
    rows = [r for r in HPC if r.variable]
    ring_r = 332.0
    for row in rows:
        xc = row.x + 0.4 * row.chord
        parts.append(common.ring(xc - 5.0, xc + 5.0, ring_r - 4.0,
                                 ring_r + 4.0, seg=SEG))
        one = common.radial_pin(xc, case_bore(xc) - 1.0, ring_r - 2.0, 2.6,
                                0.0, 8)
        parts.append(mesh.replicate(*one, row.count))
    # the tie rod runs through all three rings along the top of the case
    xa = rows[0].x + 0.4 * rows[0].chord
    xb = rows[-1].x + 0.4 * rows[-1].chord
    parts.append(mesh.pipe([common.polar(xa, ring_r, 90.0),
                            common.polar(xb, ring_r, 90.0)], 3.0, 12))
    return mesh.join(*parts)
