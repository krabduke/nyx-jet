"""Shafts, bearings and the two sumps that hold them.

The LP shaft runs inside the HP shaft from the fan to the LP turbine; the
two turn opposite ways, which cancels most of the gyroscopic couple an agile
airframe would otherwise have to fight in a fast pitch or yaw.

Five bearings in two sumps. In front, under the fan frame: No.1 (roller) and
No.2 (ball, the LP thrust bearing) carry the overhung fan, No.3 (ball, the HP
thrust bearing) carries the front of the HP spool. Behind, under the
mid-turbine frame: No.4 (roller) carries the back of the HP spool and No.5
(roller) the LP turbine. Every bearing's inner race sits on its shaft, its
outer race in a housing, and every housing on a web back to a frame -- so
the rotors are held by the static structure, not by being near it.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec          # noqa: E402
import mesh          # noqa: E402
from parts import common   # noqa: E402

SEG = common.SEG
SH = spec.SHAFTS
T = spec.SUMP_T
G = spec.GEARBOX


def build():
    out = {}
    out["shaft_lp"] = common.ring(SH["lp_x0"], SH["lp_x1"], SH["lp_r_in"],
                                  SH["lp_r_out"], step=200.0)
    out["shaft_hp"] = _hp_shaft()
    for b in spec.BEARINGS:
        out[b[0]] = _bearing(*b)
    out["sump_front"] = _sump_front()
    out["sump_rear"] = _sump_rear()
    return out


def _hp_shaft():
    """The HP shaft, with the bevel gear that drives the tower shaft cut on
    its front stub."""
    parts = [common.ring(SH["hp_x0"], SH["hp_x1"], SH["hp_r_in"],
                         SH["hp_r_out"], step=200.0)]
    xg = G["towershaft_x"]
    rb = G["bevel_r"]
    r0 = SH["hp_r_out"]
    # the gear blank: a cone whose face the tower shaft's pinion meets
    parts.append(common.revolve([(xg - 16.0, r0), (xg + 14.0, r0),
                                 (xg + 14.0, rb - 22.0), (xg - 4.0, rb - 6.0),
                                 (xg - 16.0, rb - 6.0)]))
    # 36 teeth on its face
    one = mesh.box(xg - 10.0, rb - 3.0, 0.0, 10.0, 8.0, 5.0)
    parts.append(mesh.replicate(*one, 36))
    return mesh.join(*parts)


def _bearing(name, x, r_bore, r_out, kind, spool):
    """Inner race on the shaft, outer race in the housing, and a full ring of
    rolling elements between them, each sitting 0.2 mm into both races --
    under the audits' tolerance, so they read as bearing on the races, which
    is what a preloaded rolling element does."""
    w = spec.BEARING_WIDTH[kind]
    race = 7.0
    r_i1 = r_bore + race             # outside of the inner race
    r_o0 = r_out - race              # inside of the outer race
    gap = r_o0 - r_i1
    d = gap + 0.4                    # element diameter
    pitch = 0.5 * (r_i1 + r_o0)
    inner = common.ring(x - w / 2, x + w / 2, r_bore, r_i1, seg=SEG)
    outer = common.ring(x - w / 2, x + w / 2, r_o0, r_out, seg=SEG)
    n = max(8, int(2.0 * math.pi * pitch / (1.35 * d)))
    if kind == "ball":
        # a sphere about (x, pitch): revolve a half circle about its own axis
        one = mesh.revolve_open([(px, max(abs(pr), 0.001)) for (px, pr) in
                                 [(x - 0.5 * d * math.cos(math.pi * i / 10.0),
                                   0.5 * d * math.sin(math.pi * i / 10.0))
                                  for i in range(11)]],
                                12, cap_start=True, cap_end=True)
        v, f = one
        v = [(px, py + pitch, pz) for (px, py, pz) in v]
    else:
        L = w * 0.7
        v, f = mesh.revolve_open([(x - L / 2, 0.001), (x - L / 2, d / 2),
                                  (x + L / 2, d / 2), (x + L / 2, 0.001)],
                                 12, cap_start=True, cap_end=True)
        v = [(px, py + pitch, pz) for (px, py, pz) in v]
    elements = mesh.replicate(v, f, n)
    return mesh.join(inner, outer, elements)


def _brg(name):
    return next(b for b in spec.BEARINGS if b[0] == name)


def _sump_front():
    """Housings for bearings 1, 2 and 3 and the two webs that hang them from
    the fan frame's hub ring. The webs stand either side of the tower shaft's
    bevel, which is why No.2 and No.3 are 75 mm apart."""
    hub_in = common.hub("cdfs", spec.SPLITTER3_X) - spec.WALL["hub_ring"]
    _, x1, _, r1, _, _ = _brg("brg_1_lp_roller")
    _, x2, _, r2, k2, _ = _brg("brg_2_lp_ball")
    _, x3, _, r3, k3, _ = _brg("brg_3_hp_ball")
    w1 = spec.BEARING_WIDTH["roller"]
    w2 = spec.BEARING_WIDTH[k2]
    w3 = spec.BEARING_WIDTH[k3]
    parts = []
    # one housing tube for Nos.1 and 2
    parts.append(common.ring(x1 - w1 / 2, x2 + w2 / 2, r1, r1 + T))
    # its web up to the hub ring, just ahead of No.2
    xw = x2 - w2 / 2 - 4.0 - T
    parts.append(common.ring(xw, xw + T, r1 + T - 1.0, hub_in))
    # No.3's housing and its web, at the hub ring's aft end
    parts.append(common.ring(x3 - w3 / 2, x3 + w3 / 2, r3, r3 + T))
    xa = spec.FAN_FRAME["hub_x1"] - T
    parts.append(common.ring(xa, xa + T, r3 + T - 1.0, hub_in))
    return mesh.join(*parts)


def _sump_rear():
    """Housings for bearings 4 and 5 and the web that hangs them from the
    mid-turbine frame's inner band, between the two turbine discs."""
    _, x4, _, r4, k4, _ = _brg("brg_4_hp_roller")
    _, x5, _, r5, k5, _ = _brg("brg_5_lp_roller")
    w4 = spec.BEARING_WIDTH[k4]
    w5 = spec.BEARING_WIDTH[k5]
    xw = x4 + w4 / 2 + 5.0
    band_bot = lambda x: common.hub("core", x) - spec.BAND_DEPTH
    parts = [
        common.ring(x4 - w4 / 2, xw, r4, r4 + T),
        common.ring(xw, xw + T, r5, band_bot, (spec.PATHS["core"],), step=T),
        common.ring(xw + T, x5 + w5 / 2, r5, r5 + T),
    ]
    return mesh.join(*parts)
