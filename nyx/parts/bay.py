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
    x0, x1, hw = B["x0"] + SEAT, B["x1"] - SEAT, B["half_w"] - SEAT
    out = {}
    for side, sy in (("r", 1.0), ("l", -1.0)):
        # stations across the door on every tooth's point and root, so the
        # serrated ends are sharp
        ys = [sy * (2.0 + (hw - 2.0) * j / (2 * TEETH)) for j in range(2 * TEETH + 1)]
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
        out[f"bay_door_{side}"] = shapes.loft_rings(rings)
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
    return out
