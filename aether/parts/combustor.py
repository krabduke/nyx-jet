"""Diffuser, combustor case, CMC liners, dome, swirlers, fuel nozzles and
igniters.

The compressor delivers air at 7 % of its inlet area and 37 times its inlet
pressure. The exit guide vanes take the swirl out of it, a short pre-diffuser
slows it, and it dumps into the case round a dome with eighteen swirlers.
The liners are single-skin SiC/SiC ceramic-matrix composite: CMC holds its
strength at 1,600 K and more without film cooling, so there are no cooling
rings and only one row of dilution holes -- which is what lets a combustor
feeding a 2,050 K turbine be 256 mm long.
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
C = spec.COMBUSTOR
P = spec.PATHS


def outer_liner(x):
    """Gas-side radius of the outer liner at x."""
    t = (x - C["x_dome"]) / (C["x_exit"] - C["x_dome"])
    return C["outer_liner_r0"] + (C["outer_liner_r1"] - C["outer_liner_r0"]) * t


def inner_liner(x):
    t = (x - C["x_dome"]) / (C["x_exit"] - C["x_dome"])
    return C["inner_liner_r0"] + (C["inner_liner_r1"] - C["inner_liner_r0"]) * t


def case_bore(x):
    """The combustor case's bore: the compressor's tip line to the end of the
    pre-diffuser, then out in a ramp to the dump."""
    pts = [(C["x_ogv"] - 10.0, common.tip("core", C["x_ogv"] - 10.0)),
           (C["x_dump"] - 16.0, common.tip("core", C["x_dump"] - 16.0)),
           (C["x_dome"] - 6.0, C["case_bore"]),
           (C["x_exit"], C["case_bore"])]
    return spec._interp(pts, x, 1)


def build():
    out = {}
    out["diffuser"] = _ogv()
    out.update(_cases())
    out.update(_liners())
    out.update(_dome())
    out.update(_fuel())
    out["igniters"] = _igniters()
    return out


def _ogv():
    """The compressor's exit guide vanes, ninety of them, rooted in the inner
    case and let into the combustor case's bore."""
    row = spec.BladeRow("hpc_ogv", "core", C["x_ogv"], C["ogv_chord"],
                        C["ogv_count"], 14.0, 6.0, thickness=0.07,
                        camber=0.08, rotor=False)
    one = blades.loft(row,
                      lambda x: common.hub("core", x) - spec.ROOT_EMBED,
                      lambda x: common.tip("core", x) + spec.ROOT_EMBED,
                      spec.RES["airfoil_chord_pts"], 7)
    return mesh.replicate(*one, row.count)


def _cases():
    out = {}
    x0 = C["x_ogv"] - 10.0
    byp_in = lambda x: spec.annulus(P["bypass"], x)[0]
    out["case_combustor"] = common.ring(
        x0, C["x_exit"], case_bore, byp_in, (P["bypass"],), step=8.0)
    # the inner case: the pre-diffuser's inner wall, down under the dome and
    # back up to meet the turbine nozzle's inner band
    hub_ogv = common.hub("core", C["x_ogv"])
    xd, xe = C["x_dump"], C["x_exit"]
    r_in = C["inner_case_r"]
    t = C["case_t"]
    # it rises to meet the inner liner's aft end and the nozzle's band,
    # stopping just under both
    r_end = inner_liner(xe) - C["liner_t"] - 1.0
    top = [(x0 + 6.0, hub_ogv), (xd - 16.0, hub_ogv), (xd + 4.0, hub_ogv - 18.0),
           (C["x_dome"] + 10.0, r_in + t), (xe - 38.0, r_in + t),
           (xe, r_end)]
    bot = [(xe, r_end - 8.0), (xe - 38.0, r_in),
           (C["x_dome"] + 10.0, r_in), (xd + 2.0, hub_ogv - 26.0),
           (xd - 16.0, hub_ogv - t), (x0 + 6.0, hub_ogv - t)]
    out["combustor_inner_case"] = common.revolve(top + bot)
    return out


def _liners():
    out = {}
    t = C["liner_t"]
    xd, xe = C["x_dome"], C["x_exit"]
    out["combustor_liner_outer"] = common.ring(
        xd, xe, outer_liner, lambda x: outer_liner(x) + t, step=16.0)
    out["combustor_liner_inner"] = common.ring(
        xd, xe, lambda x: inner_liner(x) - t, inner_liner, step=16.0)
    # one row of dilution holes through each liner
    xh = C["x_dilution"]
    holes_o, holes_i = [], []
    n = C["n_dilution"]
    for k in range(n):
        clock = 360.0 * (k + 0.5) / n
        holes_o.append(common.radial_pin(xh, outer_liner(xh) - 6.0,
                                         outer_liner(xh) + t + 6.0,
                                         C["dilution_r"], clock, 14))
        holes_i.append(common.radial_pin(xh, inner_liner(xh) - t - 6.0,
                                         inner_liner(xh) + 6.0,
                                         C["dilution_r"] * 0.8, clock, 14))
    out["cut:combustor_liner_outer"] = mesh.join(*holes_o)
    out["cut:combustor_liner_inner"] = mesh.join(*holes_i)
    # The liners hang on pins: eighteen from the case to the outer liner and
    # eighteen from the inner case to the inner liner, between the swirlers.
    # Pins let a ceramic liner grow and shrink against a metal case without
    # being loaded by it.
    xp = C["x_dome"] + 34.0
    pins = []
    for k in range(C["n_nozzles"]):
        clock = 360.0 * k / C["n_nozzles"]
        pins.append(common.radial_pin(xp, outer_liner(xp) + t - 1.0,
                                      case_bore(xp) + 1.0, 6.0, clock, 12))
        pins.append(common.radial_pin(xp, C["inner_case_r"] + C["case_t"] - 1.0,
                                      inner_liner(xp) - t + 1.0, 6.0, clock, 12))
    out["combustor_mount_pins"] = mesh.join(*pins)
    return out


def _swirler_centres():
    n = C["n_nozzles"]
    return [360.0 * (k + 0.5) / n for k in range(n)]


def _dome():
    """The dome closes the head of the annulus between the two liners, with
    a swirler through it for every fuel nozzle."""
    out = {}
    xd = C["x_dome"]
    th = C["dome_t"]
    # its edges follow the liners' taper across its thickness, so it meets
    # both skins face to face rather than cutting into them
    out["combustor_dome"] = common.ring(
        xd, xd + th, lambda x: inner_liner(x) + spec.SEAT,
        lambda x: outer_liner(x) - spec.SEAT, step=th)
    rs, L = C["swirler_r"], C["swirler_len"]
    rp = C["nozzle_pitch_r"]
    cut, sw = [], []
    x0 = xd + th / 2.0 - L / 2.0
    for clock in _swirler_centres():
        c = common.polar(0.0, rp, clock)
        # the hole in the dome
        # The hole in the dome is 0.6 mm bigger than the swirler: the swirler
        # floats in it, as a real one does to take the dome's thermal
        # growth, and is held by its flange against the dome's front face.
        cut.append(mesh.pipe([(xd - 4.0, c[1], c[2]), (xd + th + 4.0, c[1], c[2])],
                             rs + 0.6, 24))
        # the swirler: a short tube with a hub, eight vanes and its flange
        body = mesh.revolve_ring([(x0, rs - 7.0), (x0 + L, rs - 7.0),
                                  (x0 + L, rs), (x0, rs)], 24)
        flange = mesh.revolve_ring([(xd - 3.0, rs - 2.0), (xd, rs - 2.0),
                                    (xd, rs + 7.0), (xd - 3.0, rs + 7.0)], 24)
        hubv = mesh.revolve_ring([(x0, 4.0), (x0 + L * 0.6, 4.0),
                                  (x0 + L * 0.6, 9.0), (x0, 9.0)], 16)
        vanes = []
        for j in range(8):
            a = 2.0 * math.pi * j / 8
            v, f = mesh.box(x0 + L * 0.3, 0.0, 0.0, L * 0.5, rs - 7.0 - 8.0, 2.0)
            v = [(x, y + 0.5 * (rs - 7.0 + 8.0) - 0.5 * 0.0, z) for (x, y, z) in v]
            v = mesh.rot_x(v, a)
            vanes.append((v, f))
        one = mesh.join(body, flange, hubv, *vanes)
        v, f = one
        sw.append(([(x, y + c[1], z + c[2]) for (x, y, z) in v], f))
    out["cut:combustor_dome"] = mesh.join(*cut)
    out["swirlers"] = mesh.join(*sw)
    return out


def _fuel():
    """Eighteen fuel nozzles, fed from a manifold ring on the outer case.
    Each stem goes in radially through the outer case, the third stream,
    the intermediate case, the bypass duct and the combustor case, and turns
    aft into its swirler, where the tip seats in the swirler's bore. That is
    how a fuel nozzle is changed on the wing: unbolt it outside and draw it
    out."""
    out = {}
    xm = C["manifold_x"]
    r_od = spec.annulus(P["third"], xm)[1] + spec.WALL["outer_case"]
    rm = r_od + C["manifold_tube_r"] + 3.0
    out["fuel_manifold"] = mesh.ring_torus(xm, rm, C["manifold_tube_r"], SEG, 14)
    stems = []
    rp = C["nozzle_pitch_r"]
    rs = C["swirler_r"] - 7.0          # the swirler's bore
    xd = C["x_dome"]
    # the tip seats on the swirler's hub, in its bore
    x_tip = xd + C["dome_t"] / 2.0 - C["swirler_len"] / 2.0
    for clock in _swirler_centres():
        # the last 10 mm are straight, so the tip's face is square to the
        # swirler's and seats on its hub rather than digging a corner in
        path = [common.polar(xm, rm + 4.0, clock),
                common.polar(xm, rp + 22.0, clock),
                common.polar(x_tip - 12.0, rp, clock),
                common.polar(x_tip, rp, clock)]
        stems.append(mesh.pipe(path, [C["stem_r"], C["stem_r"], rs, rs],
                               16, bend=10.0))
        # the mounting flange that bolts the nozzle to the outer case
        stems.append(mesh.revolve_ring([(0.0, C["stem_r"]), (6.0, C["stem_r"]),
                                        (6.0, 20.0), (0.0, 20.0)], 16))
        v, f = stems[-1]
        # stand the flange on the case, square to the stem
        a = math.radians(clock)
        vv = []
        for (x, y, z) in v:
            # local frame: x -> radial, (y, z) -> (axial, tangential)
            rr = r_od + x
            vv.append((xm + y, rr * math.cos(a) - z * math.sin(a),
                       rr * math.sin(a) + z * math.cos(a)))
        stems[-1] = (vv, f)
    out["fuel_nozzles"] = mesh.join(*stems)
    return out


def _igniters():
    """Two igniter plugs, through the cases into ferrules in the outer liner,
    high on both sides where a wet start's fuel does not pool, and midway
    between two liner pins."""
    parts = []
    x = C["igniter_x"]
    r_od = spec.annulus(P["third"], x)[1] + spec.WALL["outer_case"]
    for clock in C["igniter_clock"]:
        parts.append(common.radial_pin(x, outer_liner(x) + 1.0, r_od + 30.0,
                                       7.0, clock, 16))
        parts.append(common.radial_pin(x, r_od, r_od + 22.0, 13.0, clock, 16))
    return mesh.join(*parts)
