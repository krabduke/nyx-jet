"""Environmental control and oxygen.

The pilot sat in a sealed cockpit with no air, and breathed nothing.

    bleed        each engine's customer bleed, off its compressor through the
                 fan frame's top strut, from the flange on top of its case
                 forward and inboard to the pack
    pack         an air-cycle machine between the centre tank and the PDU:
                 the bleed is cooled in a heat exchanger whose sink is fuel,
                 as in the F-35's thermal management system -- two lines to
                 the centre tank beside it -- then expanded through the
                 machine's turbine, which chills it, and dried in a water
                 separator
    duct         the conditioned air forward under the spine, in a channel
                 let into the tops of the fuel cells and through holes in
                 the frames, over the canopy's hinge beam and down to a
                 diffuser on the back of the seat bulkhead
    oxygen       an on-board oxygen generator beside the pack, fed off the
                 pack's output, with its hose forward beside the duct and
                 down through the seat bulkhead to the seat's connector
"""

import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec      # noqa: E402
import shapes    # noqa: E402
import mesh      # noqa: E402

PACK_X = (10205.0, 10375.0)
HX = {"y": 180.0, "z": (190.0, 300.0)}
ACM = {"x": 10290.0, "z": 390.0, "r": 70.0, "y": 135.0}
SEP = {"z": 505.0, "r": 34.0}
# low beside the exchanger, under the left bleed duct's run into the pack
OBOGS = {"x": (10222.0, 10358.0), "y": (-305.0, -215.0), "z": (150.0, 310.0)}
DUCT_R = 30.0
BLEED_R = 28.0
HOSE_R = 6.0
FUEL_R = 9.0

ROUTES = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      "ecs_routes.json")


def bleed_outlet(sy):
    """Each engine's bleed flange, in the aircraft (the engine's own
    accessories.bleed_outlet, moved to where the engine is installed)."""
    from parts import engines
    ox, oy, oz = engines.offset(sy)
    return (ox + 490.0, oy, oz + 528.0)


def _pack():
    x0, x1 = PACK_X
    xm = 0.5 * (x0 + x1)
    Y = HX["y"]
    z0, z1 = HX["z"]
    parts = [mesh.box(xm, 0.0, 0.5 * (z0 + z1), x1 - x0, 2 * Y, z1 - z0)]
    # the heat exchanger's fins, across it
    for k in range(12):
        y = -Y + 12.0 + (2 * Y - 24.0) * k / 11
        parts.append(mesh.box(xm, y, z1 + 6.0, x1 - x0 - 24.0, 4.0, 14.0))
    # the air-cycle machine: fan, compressor and turbine on one shaft, along y
    ay = ACM["y"]
    parts.append(mesh.pipe([(ACM["x"], -ay, ACM["z"]), (ACM["x"], ay, ACM["z"])],
                           ACM["r"], 36, bend=0.0))
    for y in (-ay, ay):
        parts.append(mesh.pipe([(ACM["x"], y - 14.0, ACM["z"]), (ACM["x"], y + 14.0, ACM["z"])],
                               ACM["r"] + 8.0, 36, bend=0.0))
    # standing on the exchanger
    for y in (-90.0, 90.0):
        parts.append(mesh.box(ACM["x"], y, 0.5 * (z1 + ACM["z"] - ACM["r"] + 6.0),
                              60.0, 16.0, ACM["z"] - ACM["r"] + 6.0 - z1 + 2.0))
    # the water separator on top, on its own stand
    parts.append(mesh.pipe([(x0 + 30.0, 0.0, SEP["z"]), (x1 - 30.0, 0.0, SEP["z"])],
                           SEP["r"], 24, bend=0.0))
    parts.append(mesh.box(ACM["x"], 0.0, 0.5 * (ACM["z"] + SEP["z"]), 30.0, 20.0,
                          SEP["z"] - ACM["z"]))
    return mesh.join(*parts)


def _mount():
    """Two struts from the pack's aft face over the PDU to the keel's
    forward end, which is the structure there."""
    parts = []
    for s in (-1.0, 1.0):
        parts.append(mesh.pipe([(10355.0, s * 120.0, 410.0),
                                (10420.0, s * 80.0, 440.0),
                                (10556.0, s * 12.0, 440.0)], 12.0, 16, bend=20.0))
    return mesh.join(*parts)


def _obogs():
    x0, x1 = OBOGS["x"]
    y0, y1 = OBOGS["y"]
    z0, z1 = OBOGS["z"]
    parts = [mesh.box(0.5 * (x0 + x1), 0.5 * (y0 + y1), 0.5 * (z0 + z1),
                      x1 - x0, y1 - y0, z1 - z0)]
    # its two molecular-sieve beds, as bottles on its outboard face
    for x in (x0 + 38.0, x1 - 38.0):
        parts.append(mesh.pipe([(x, y0 + 4.0, z0 + 20.0), (x, y0 + 4.0, z1 - 20.0)],
                               26.0, 24, bend=0.0))
    # bolted to the pack's exchanger by a bracket
    parts.append(mesh.box(0.5 * (x0 + x1), 0.5 * (y1 - HX["y"]) + 0.5 * (y1 + 2.0),
                          HX["z"][0] + 30.0, 60.0, abs(-HX["y"] - y1) + 6.0, 40.0))
    return mesh.join(*parts)


def diffuser_at():
    """The cockpit's air outlet, on the seat bulkhead's back face."""
    return (4955.3, 60.0, 845.7)


# the seat bulkhead's back face leans back 18 degrees: up it, and aft off it
LEAN_UP = (math.sin(math.radians(18.0)), math.cos(math.radians(18.0)))
LEAN_AFT = (math.cos(math.radians(18.0)), -math.sin(math.radians(18.0)))


def _leaning_box(c, depth, width, height):
    """A box on the bulkhead's back face, square to it: depth aft off the
    face, height up it."""
    v, f = mesh.box(0.0, 0.0, 0.0, depth, width, height)
    out = []
    for (a, y, h) in v:
        out.append((c[0] + a * LEAN_AFT[0] + h * LEAN_UP[0], c[1] + y,
                    c[2] + a * LEAN_AFT[1] + h * LEAN_UP[1]))
    return out, f


def _diffuser():
    """Between the bulkhead's two stiffeners, which are at y 120 either
    side, 3 mm into its back face."""
    c = diffuser_at()
    parts = [_leaning_box(c, 34.0, 60.0, 66.0)]
    # the grille the air comes out of, on the bulkhead's cockpit side, which
    # is 10 mm in front of its back face: a lip round it and six louvres
    # across it, which turn the air down over the pilot's shoulders
    fc = (c[0] - 26.0 * LEAN_AFT[0], c[1], c[2] - 26.0 * LEAN_AFT[1])
    for dy in (-31.0, 31.0):
        parts.append(_leaning_box((fc[0], c[1] + dy, fc[2]), 6.0, 4.0, 70.0))
    for dh in (-33.0, 33.0):
        parts.append(_leaning_box((fc[0] + dh * LEAN_UP[0], c[1], fc[2] + dh * LEAN_UP[1]),
                                  6.0, 66.0, 4.0))
    for k in range(6):
        dh = -25.0 + 10.0 * k
        parts.append(_leaning_box((fc[0] + dh * LEAN_UP[0], c[1], fc[2] + dh * LEAN_UP[1]),
                                  5.0, 58.0, 2.5))
    return mesh.join(*parts)


def build():
    out = {"ecs_pack": _pack(), "ecs_mount": _mount(), "ecs_obogs": _obogs(),
           "ecs_diffuser": _diffuser()}
    # the pack's fuel lines to the centre tank's aft wall, beside it
    out["ecs_fuel_lines"] = mesh.join(*[
        mesh.pipe([(PACK_X[0] + 4.0, y, 245.0), (10174.0, y, 245.0)], FUEL_R, 16,
                  bend=0.0) for y in (-100.0, 100.0)])
    if not os.path.exists(ROUTES):
        return out
    R = json.load(open(ROUTES))
    bleeds = []
    for tag, sy in (("r", 1.0), ("l", -1.0)):
        path = [tuple(p) for p in R["bleed_r"]]
        if sy < 0:
            path = [(x, -y, z) for (x, y, z) in path]
        bleeds.append(mesh.pipe(path, BLEED_R, 24, bend=60.0))
        out[f"ecs_bleed_{tag}"] = bleeds[-1]
    out["ecs_duct"] = mesh.pipe([tuple(p) for p in R["duct"]], DUCT_R, 24, bend=80.0)
    out["ecs_oxygen"] = mesh.join(
        mesh.pipe([tuple(p) for p in R["obogs_feed"]], HOSE_R + 2.0, 12, bend=20.0),
        mesh.pipe([tuple(p) for p in R["oxygen"]], HOSE_R, 12, bend=20.0))
    # the channel the duct and the hose lie in, let into the fuel cells'
    # tops, and the holes they pass through the frames by
    chan = mesh.join(
        mesh.pipe([tuple(p) for p in R["duct"]], DUCT_R + 6.0, 16, bend=80.0),
        mesh.pipe([tuple(p) for p in R["oxygen"]], HOSE_R + 5.0, 12, bend=20.0))
    for tank in ("fuel_tank_fwd_1", "fuel_tank_fwd_2", "fuel_tank_fwd_3",
                 "fuel_tank_centre"):
        out[f"cut:{tank}"] = chan
    for fr in ("frame_6800", "frame_7900", "frame_9000"):
        out[f"cut:{fr}"] = chan
    # and where each bleed duct leaves its engine, through the engine
    # frame's arch over the bleed flange
    out["cut:frame_engine_fwd"] = mesh.join(*[
        mesh.pipe([(x, y, z) for (x, y, z) in b], BLEED_R + 5.0, 16, bend=60.0)
        for b in ([tuple(p) for p in R["bleed_r"]],
                  [(p[0], -p[1], p[2]) for p in R["bleed_r"]])])
    return out
