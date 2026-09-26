"""The main weapons bay, its doors and its four missiles.

The bay is on the centreline, between the two intake ducts and under the
cockpit's aft end and the centre fuel tank: 3.8 m long for a 3.65 m
medium-range missile, 810 mm wide, which is what the ducts leave. Four
missiles fit abreast with their tail fins clipped, each on a rail under the
roof that the launcher's trapeze swings down through the doors before
release.

The doors are the skin they replace: each is the patch of outer mould line
between the centreline and the bay's side wall, one skin thickness deep,
standing 0.15 mm off the cut edges. Closed they are flush; the viewer swings
them open about their outboard hinge lines.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec      # noqa: E402
import shapes    # noqa: E402
import mesh      # noqa: E402

B = spec.BAY
WALL = 8.0
SEAT = 0.15
WALL_SEAT = 1.0


def _wall_bottom(x, y):
    """Where a bay wall standing at (x, y) meets the skin's inner surface."""
    return shapes.z_dn(x, y, spec.SKIN_T) + WALL_SEAT


def structure():
    x0, x1, hw, zr = B["x0"], B["x1"], B["half_w"], B["z_roof"]
    parts = [mesh.box(0.5 * (x0 + x1), 0.0, zr + WALL / 2, x1 - x0 + 2 * WALL,
                      2 * (hw + WALL), WALL)]
    # side walls: from the roof down to the skin, following it along x
    xs = [x0 + (x1 - x0) * i / 24 for i in range(25)]
    for sy in (1.0, -1.0):
        y_in, y_out = sy * hw, sy * (hw + WALL)
        rings = []
        for x in xs:
            zb = max(_wall_bottom(x, y_in), _wall_bottom(x, y_out))
            rings.append([(x, y_in, zb), (x, y_out, zb), (x, y_out, zr),
                          (x, y_in, zr)])
        parts.append(shapes.loft_rings(rings))
    # end walls
    ys = [-hw + 2 * hw * i / 24 for i in range(25)]
    for x_a, x_b in ((x0 - WALL, x0), (x1, x1 + WALL)):
        rings = []
        for y in ys:
            zb = max(_wall_bottom(x_a, y), _wall_bottom(x_b, y))
            rings.append([(x_a, y, zb), (x_a, y, zr), (x_b, y, zr), (x_b, y, zb)])
        parts.append(shapes.loft_rings(rings))
    return mesh.join(*parts)


# The bay's fore and aft edges are serrated, like every edge that crosses
# the flow on this airframe: TEETH points across the bay, each TOOTH deep.
TEETH, TOOTH = 8, 70.0


def serration(y):
    """How far in from the bay's nominal end the edge is at y: a sawtooth
    across the bay's width, zero at the sides and the centreline."""
    hw = B["half_w"]
    u = (abs(y) / hw) * TEETH / 2.0
    f = u - math.floor(u)
    return TOOTH * (1.0 - abs(2.0 * f - 1.0))


def opening_cutter():
    x0, x1, hw = B["x0"], B["x1"], B["half_w"]
    ys = [-hw + 2 * hw * j / (4 * TEETH) for j in range(4 * TEETH + 1)]
    lo_z, hi_z = -3000.0, B["z_roof"]
    fore = [(x0 + serration(y), y) for y in ys]
    aft = [(x1 - serration(y), y) for y in reversed(ys)]
    outline = fore + aft
    return shapes.loft_rings([[(x, y, lo_z) for (x, y) in outline],
                              [(x, y, hi_z) for (x, y) in outline]])


def doors():
    """Two doors, each the patch of skin from the centreline split to the
    bay's side, one skin deep. Returns {name: part} and their hinge lines."""
    # a millimetre short of the opening at each end: the skin's cut faces
    # there are square across a curved surface, and a 0.15 mm seat is less
    # than the curve's own chord error between the skin's stations
    x0, x1, hw = B["x0"] + 1.0, B["x1"] - 1.0, B["half_w"] - SEAT
    out = {}
    for side, sy in (("r", 1.0), ("l", -1.0)):
        # stations across the door on every tooth's point and root, so the
        # serrated ends are sharp
        # -- the opening cutter's own stations, so the door's edge follows the
        # hole's chord for chord rather than cutting across its teeth
        ys = [sy * min(max(2.0, B["half_w"] * j / (2 * TEETH)), hw)
              for j in range(2 * TEETH + 1)]
        rings = []
        for i in range(41):
            t = i / 40.0

            def xat(y):
                a, b = x0 + serration(y), x1 - serration(y)
                return a + (b - a) * t
            outer = [(xat(y), y, shapes.z_dn(xat(y), y)) for y in ys]
            inner = [(xat(y), y, shapes.z_dn(xat(y), y, spec.SKIN_T))
                     for y in reversed(ys)]
            rings.append(outer + inner)
        # each end is a thin strip along the serrated edge, capped quad by
        # quad: one n-gon across a zigzag is triangulated straight over the
        # notches between the teeth, and the door then claims the skin that
        # stands in them
        v, f = shapes.loft_rings(rings, cap_start=False, cap_end=False)
        m, n = len(rings[0]), len(ys)
        f = list(f)
        for base, flip in ((0, True), ((len(rings) - 1) * m, False)):
            for k in range(n - 1):
                q = (base + k, base + k + 1, base + m - 2 - k, base + m - 1 - k)
                f.append(tuple(reversed(q)) if flip else q)
        from parts import gear
        horns = [gear._horn(bay_horn(x, sy), sy) for x in BAY_ACT_X]
        out[f"bay_door_{side}"] = mesh.join(shapes.orient((v, f)), *horns)
    return out


# Each bay door is opened by two actuators from the bay's side wall to horns
# on the door at its hinge edge, under the missiles. The doors hung on their
# hinges driven by nothing.
BAY_ACT_X = (6100.0, 8200.0)


def bay_horn(x, sy):
    """The eye of the horn on a door's inside at station x, door shut."""
    y = sy * (B["half_w"] - 10.0)
    return (x, y, shapes.z_dn(x, y, spec.SKIN_T) + 71.0)


def bay_anchor(x, sy):
    return (x, sy * (B["half_w"] - 16.0), bay_horn(x, sy)[2] + 170.0)


def bay_hinge(sy):
    """The door's hinge line, as the renders swing it: along x, through the
    bay's side at its mid-station."""
    xm = 0.5 * (B["x0"] + B["x1"])
    return (xm, sy * B["half_w"], shapes.z_dn(xm, sy * B["half_w"])), (1.0, 0.0, 0.0)


def bay_actuators():
    from parts import gear
    out = {}
    for side, sy in (("r", 1.0), ("l", -1.0)):
        for k, x in enumerate(BAY_ACT_X):
            b, r = gear._strut(bay_anchor(x, sy), bay_horn(x, sy), sy * (B["half_w"] + 2.0))
            out[f"bay_door_act_{k + 1}_{side}"] = b
            out[f"bay_door_act_rod_{k + 1}_{side}"] = r
    return out


def missile(x_nose, y, z):
    """A medium-range missile: ogive nose, body, four mid-body strakes and
    four clipped tail fins in an X, folded to fit a bay."""
    L, d = B["missile_len"], B["missile_d"]
    r = d / 2
    prof = [(0.0, 0.001)]
    for i in range(1, 13):
        t = i / 12
        prof.append((t * 480.0, r * math.sqrt(1 - (1 - t) ** 2)))
    prof += [(L, r), (L, 0.001)]
    v, f = mesh.revolve_open(prof, 24, cap_start=True, cap_end=True)
    parts = [(v, f)]
    for k in range(4):
        a = math.pi / 4 + k * math.pi / 2
        # clipped tail fins, as a missile carried internally has them
        for (xa, xb, span, chord_t) in ((1500.0, 2300.0, 30.0, 6.0),
                                        (L - 330.0, L - 20.0, 38.0, 6.0)):
            fv, ff = mesh.box(0.5 * (xa + xb), r + span / 2 - 2.0, 0.0,
                              xb - xa, span + 4.0, chord_t)
            parts.append((mesh.rot_x(fv, a), ff))
    v, f = mesh.join(*parts)
    return [(px + x_nose, py + y, pz + z) for (px, py, pz) in v], f


# four abreast, (y, z) of each missile's axis: 200 mm apart, which the
# clipped fins in an X clear by 20 mm
SLOTS = ((100.0, -320.0), (-100.0, -320.0), (300.0, -320.0), (-300.0, -320.0))


def missiles_and_launchers():
    x_nose = 0.5 * (B["x0"] + B["x1"]) - B["missile_len"] / 2
    ms, ls = [], []
    r = B["missile_d"] / 2
    for (y, z) in SLOTS:
        ms.append(missile(x_nose, y, z))
        # a launcher: a rail on the missile's back, hung from the roof on the
        # two arms of its trapeze
        top = B["z_roof"]
        rail = mesh.box(x_nose + 1800.0, y, z + r + 12.0, 1500.0, 50.0, 26.0)
        ls.append(rail)
        for xp in (x_nose + 1300.0, x_nose + 2300.0):
            ls.append(mesh.box(xp, y, 0.5 * (top + z + r + 22.0), 60.0, 36.0,
                               top - (z + r + 22.0) + 2.0))
    return mesh.join(*ms), mesh.join(*ls)


def build():
    m, l = missiles_and_launchers()
    out = {"bay_structure": structure(), "cut:fuselage_skin": opening_cutter(),
           "missiles": m, "launchers": l}
    out.update(doors())
    out.update(bay_actuators())
    return out
