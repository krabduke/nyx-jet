"""Fuel: the tanks, as the volumes that hold it.

The mass table carried 7,200 kg of fuel in three lines -- forward
fuselage, centre, wings -- and none of it was anywhere: no tank, no line to
an engine. These are the tanks, each the space the airframe has for it:

    forward cells  three bladder cells over the weapons bay, one between each
                   pair of frames, their ends let into the frames they hang
                   from, shaped round the intake ducts and the gun
    centre tank    between the bay's last frame and the engines' inlets,
                   between the ducts and inboard of the main gear's wells
    wing tanks     integral, between the front and rear spars (24 and 66 % of
                   the chord), outboard of the main gear's pivot -- the
                   stowed leg sweeps through the wing inboard of it

Each is the body's section inset by the bladder's clearance and cut round
everything that passes through it (the cutters below). How much each holds
is not written here: tools/measure_fuel.py measures the built tanks and
writes fuel_tanks.json, which the mass table reads, so the aircraft's
weight and balance are the tanks' own.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec      # noqa: E402
import shapes    # noqa: E402
import mesh      # noqa: E402

CLEAR = 20.0                 # a bladder stands this far off skin and structure
M = 72                       # points round a cell's section
FRAME_HALF = 20.0            # structure.FRAME_T / 2
CELLS = (("fuel_tank_fwd_1", 5680.0, 6800.0), ("fuel_tank_fwd_2", 6800.0, 7900.0),
         ("fuel_tank_fwd_3", 7900.0, 9000.0))
CENTRE = ("fuel_tank_centre", 9000.0, 10180.0)
Y_LIM_FWD = 1600.0           # inboard of the wing's root, embedded to 1720
Y_LIM_CENTRE = 1250.0        # inboard of the main gear's wells, from 1310
WING = {"u": (0.24, 0.66), "y": (3150.0, 5900.0), "skin": 8.0}


def _section(x, ylim, zmin):
    ring = shapes.ring(x, M, spec.SKIN_T + CLEAR)
    out = []
    for (_, y, z) in ring:
        y = max(-ylim, min(ylim, y))
        if zmin is not None:
            z = max(zmin, z)
        out.append((x, y, z))
    return out


def _cell(x0, x1, ylim, zmin, n=12):
    xs = [x0 + (x1 - x0) * i / n for i in range(n + 1)]
    return shapes.loft_rings([_section(x, ylim, zmin) for x in xs])


def _box(x0, x1, y0, y1, z0, z1):
    return mesh.box(0.5 * (x0 + x1), 0.5 * (y0 + y1), 0.5 * (z0 + z1),
                    x1 - x0, y1 - y0, z1 - z0)


def _cell_cutter(x0, x1):
    """What a cell is shaped round: the intake ducts and the gun, grown by
    the bladder's clearance."""
    from parts import intakes, details
    I = spec.INTAKE
    cuts = []
    n = max(2, int((x1 - x0) / 80.0))
    xs = [x0 - 20.0 + (x1 - x0 + 40.0) * i / n for i in range(n + 1)]
    for sgn in (1.0, -1.0):
        rings = [[(x, sgn * y, z) for (y, z) in
                  intakes._shape(min(max(x, I["x_mouth"]), I["x_end"]),
                                 I["wall"] + CLEAR)] for x in xs]
        cuts.append(shapes.loft_rings(rings))
    B = spec.BAY
    if x0 < B["x1"] + CLEAR:
        # the weapons bay's aft end runs past its last frame
        cuts.append(_box(x0 - 20.0, B["x1"] + CLEAR, -B["half_w"] - 8.0 - CLEAR,
                         B["half_w"] + 8.0 + CLEAR, -2000.0, B["z_roof"] + CLEAR))
    gx0, gx1 = details.GUN_X
    if x0 < gx1 and x1 > gx0:
        cuts.append(_box(gx0 - CLEAR - 60.0, gx1 + CLEAR,
                         details.GUN_Y - 110.0 - CLEAR, details.GUN_Y + 110.0 + CLEAR,
                         330.0 - CLEAR, 2000.0))
    return mesh.join(*cuts)


def _wing_loop(y, grow):
    """The tank's section in the wing at span y: the aerofoil between the
    spars, inset by the skin, grown by `grow` mm."""
    from parts import surfaces
    P = surfaces.WingPlace()
    c = spec.wing_chord(y)
    tc = P.tc(y)
    u0, u1 = WING["u"]
    cam = lambda u: 4.0 * P.camber * u * (1.0 - u)
    ht = lambda u: shapes.naca_t(u, tc) - (WING["skin"] - grow) / c
    us = [u0 + (u1 - u0) * i / 16 for i in range(17)]
    up = [P(y, u, cam(u) + ht(u)) for u in us]
    lo = [P(y, u, cam(u) - ht(u)) for u in reversed(us)]
    return up + lo


def _wing_tank(grow):
    y0, y1 = WING["y"]
    ys = [y0 + (y1 - y0) * i / 12 for i in range(13)]
    return shapes.loft_rings([_wing_loop(y, grow) for y in ys])


def _mirror(part):
    v, f = part
    return shapes.orient(([(x, -y, z) for (x, y, z) in v],
                          [tuple(reversed(c)) for c in f]))


def build():
    out = {}
    B = spec.BAY
    z_bay = B["z_roof"] + CLEAR
    for name, xf0, xf1 in CELLS:
        # the ends let 0.5 mm into the frames' arches, which it hangs from
        x0, x1 = xf0 + FRAME_HALF - 0.5, xf1 - FRAME_HALF + 0.5
        out[name] = _cell(x0, x1, Y_LIM_FWD, z_bay)
        out[f"cut:{name}"] = _cell_cutter(x0, x1)
    name, xf0, x1 = CENTRE
    x0 = xf0 + FRAME_HALF - 0.5
    out[name] = _cell(x0, x1, Y_LIM_CENTRE, None)
    out[f"cut:{name}"] = _cell_cutter(x0, x1)
    # the wing tanks: the cavity cut out of the wing, and the tank in it,
    # half a millimetre proud of the cavity, into the skin
    cav = _wing_tank(0.0)
    tank = _wing_tank(0.5)
    out["cut:wing_r"], out["cut:wing_l"] = cav, _mirror(cav)
    out["fuel_tank_wing_r"], out["fuel_tank_wing_l"] = tank, _mirror(tank)
    return out
