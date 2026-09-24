"""HP turbine, mid-turbine frame, LP turbine, their discs and case.

The two turbines turn opposite ways. The HP rotor leaves the gas swirling
against the LP rotor's direction of travel, which is exactly the inlet angle
the LP rotor wants -- so the vane row a co-rotating design needs between
them to turn the flow round is not there. What is there instead is the
mid-turbine frame: sixteen fat struts, faired as vanes that barely turn the
flow, that carry the rear bearings and the oil lines to them.

The hot rows are film-cooled. Each NGV and HP blade has a showerhead of
holes round its leading edge and a row down its pressure side, and they are
real holes: cut through the aerofoil, in every blade of the row.
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
NGV, HPT, MTF, LPT = spec.TURBINE_ROWS
C = spec.COMBUSTOR

HOLE_R = 1.2

# For the cooled rows, assemble.py cuts the holes in ONE aerofoil and arrays
# the result round the disc -- the same geometry as cutting the whole row,
# for a thirty-sixth or a fifty-sixth of the boolean work. build() still
# returns the full row and every hole, which is what the audits check.
# {part: (one aerofoil, its holes, count, the rest of the part)}
PROTO = {}


def build():
    PROTO.clear()
    out = {}
    out.update(_ngv())
    out.update(_hpt())
    out.update(_mtf())
    out.update(_lpt())
    out["case_turbine"] = common.ring(
        C["x_exit"], spec.AUGMENTOR["x_mixer0"],
        lambda x: common.tip("core", x),
        lambda x: spec.annulus(P["bypass"], x)[0], (P["core"],), step=10.0)
    return out


def _holes(row, r_root, r_top, n_span=6):
    """Cooling-hole cutters for one aerofoil: a showerhead of three rows
    round the leading edge and one row down the pressure side, each hole
    drilled along the surface normal."""
    cut = []
    for (u, side) in ((0.015, +1), (0.015, -1), (0.004, +1), (0.30, +1)):
        for k in range(n_span):
            s = 0.14 + 0.72 * k / (n_span - 1)
            p, d = blades.surface_point(row, r_root, r_top, u, s, side)
            L = row.chord * 0.25
            cut.append(mesh.pipe([tuple(p[i] - d[i] * L for i in range(3)),
                                  tuple(p[i] + d[i] * 3.0 for i in range(3))],
                                 HOLE_R, 8))
    return mesh.join(*cut)


def _ngv():
    out = {}
    root = lambda x: common.hub("core", x) - spec.ROOT_EMBED
    top = lambda x: common.tip("core", x) + spec.ROOT_EMBED
    one = blades.loft(NGV, root, top, spec.RES["airfoil_chord_pts"],
                      spec.RES["airfoil_span_pts"])
    _, x1 = common.reach(NGV)
    band = common.ring(C["x_exit"], x1,
                       lambda x: common.hub("core", x) - spec.BAND_DEPTH,
                       lambda x: common.hub("core", x), (P["core"],), step=8.0)
    holes = _holes(NGV, root, top)
    out["vanes_hpt_ngv"] = mesh.join(mesh.replicate(*one, NGV.count), band)
    out["cut:vanes_hpt_ngv"] = mesh.replicate(*holes, NGV.count)
    PROTO["vanes_hpt_ngv"] = (one, holes, NGV.count, band)
    return out


def _hpt():
    out = {}
    sh = spec.SHAFTS
    clr = spec.TIP_CLEARANCE["core"]
    root = lambda x: common.hub("core", x) - spec.ROOT_EMBED
    top = lambda x: common.tip("core", x) - clr
    one = blades.loft(HPT, root, top, spec.RES["airfoil_chord_pts"],
                      spec.RES["airfoil_span_pts"])
    holes = _holes(HPT, root, top)
    out["blades_hpt_r"] = mesh.replicate(*one, HPT.count)
    out["cut:blades_hpt_r"] = mesh.replicate(*holes, HPT.count)
    PROTO["blades_hpt_r"] = (one, holes, HPT.count, None)
    # The disc: the most highly stressed part in the engine, so a deep bore
    # and a short web. Its forward cone comes down onto the HP shaft.
    a, b = common.reach(HPT)
    c = 0.5 * (a + b)
    loop = common.rim_profile(a, b, "core", 18.0, c, 20.0, 140.0, 44.0, 26.0)
    cone = common.revolve([(c - 22.0, 140.0), (c - 10.0, 140.0),
                           (c - 50.0, sh["hp_r_out"]), (c - 62.0, sh["hp_r_out"])])
    out["hpt_disc"] = mesh.join(common.revolve(loop), cone)
    return out


def _mtf():
    out = {}
    root = lambda x: common.hub("core", x) - spec.ROOT_EMBED
    top = lambda x: common.tip("core", x) + spec.ROOT_EMBED
    one = blades.loft(MTF, root, top, spec.RES["airfoil_chord_pts"],
                      spec.RES["airfoil_span_pts"])
    a, b = common.reach(MTF)
    band = common.ring(a, b,
                       lambda x: common.hub("core", x) - spec.BAND_DEPTH,
                       lambda x: common.hub("core", x), (P["core"],), step=5.0)
    out["vanes_mtf"] = mesh.join(mesh.replicate(*one, MTF.count), band)
    return out


def _lpt():
    out = {}
    sh = spec.SHAFTS
    clr = spec.TIP_CLEARANCE["core"]
    root = lambda x: common.hub("core", x) - spec.ROOT_EMBED
    top = lambda x: common.tip("core", x) - clr
    one = blades.loft(LPT, root, top, spec.RES["airfoil_chord_pts"],
                      spec.RES["airfoil_span_pts"])
    out["blades_lpt_r"] = mesh.replicate(*one, LPT.count)
    a, _ = common.reach(LPT)
    b = spec.AUGMENTOR["tailcone_x0"] - 6.0
    c = 0.5 * (LPT.x + LPT.x_te)
    loop = common.rim_profile(a, b, "core", 16.0, c, 20.0, 118.0, 40.0, 22.0)
    # the LP disc's cone runs aft and down onto the LP shaft's tail, behind
    # bearing 5
    cone = common.revolve([(c + 10.0, 118.0), (c + 22.0, 118.0),
                           (sh["lp_x1"], sh["lp_r_out"]),
                           (sh["lp_x1"] - 12.0, sh["lp_r_out"])])
    out["lpt_disc"] = mesh.join(common.revolve(loop), cone)
    return out
