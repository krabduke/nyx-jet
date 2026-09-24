"""Landing gear, drawn down: a twin-wheel nose leg and two single-wheel
main legs, each in a bay cut in the skin with its doors standing open.

The mains sit about 700 mm behind the aft-most centre of gravity -- an 18
degree tip-back angle, so the aircraft cannot sit on its tail when fully
fuelled -- and carry 89 % of the weight; the nose leg carries the other 11 %,
enough to steer with and little enough to rotate at take-off (verify.py
measures both). The track is 2.96 m, between the intake ducts and the wing
roots.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec      # noqa: E402
import shapes    # noqa: E402
import mesh      # noqa: E402

G = spec.GEAR
WALL = 8.0
SEAT = 0.15
WALL_SEAT = 1.0


def bay_box(x0, x1, y0, y1, z_roof):
    """An open-bottomed box: roof, and walls down to the skin's inner
    surface just outside the opening."""
    parts = [mesh.box(0.5 * (x0 + x1), 0.5 * (y0 + y1), z_roof + WALL / 2,
                      x1 - x0 + 2 * WALL, y1 - y0 + 2 * WALL, WALL)]
    xs = [x0 + (x1 - x0) * i / 12 for i in range(13)]
    for ya, yb in ((y0 - WALL, y0), (y1, y1 + WALL)):
        rings = []
        for x in xs:
            zb = max(shapes.z_dn(x, ya, spec.SKIN_T), shapes.z_dn(x, yb, spec.SKIN_T)) + WALL_SEAT
            rings.append([(x, ya, zb), (x, yb, zb), (x, yb, z_roof), (x, ya, z_roof)])
        parts.append(shapes.loft_rings(rings))
    ys = [y0 + (y1 - y0) * i / 12 for i in range(13)]
    for xa, xb in ((x0 - WALL, x0), (x1, x1 + WALL)):
        rings = []
        for y in ys:
            zb = max(shapes.z_dn(xa, y, spec.SKIN_T), shapes.z_dn(xb, y, spec.SKIN_T)) + WALL_SEAT
            rings.append([(xa, y, zb), (xa, y, z_roof), (xb, y, z_roof), (xb, y, zb)])
        parts.append(shapes.loft_rings(rings))
    return mesh.join(*parts)


def bay_cutter(x0, x1, y0, y1, z_roof):
    return mesh.box(0.5 * (x0 + x1), 0.5 * (y0 + y1), 0.5 * (-2000.0 + z_roof),
                    x1 - x0, y1 - y0, z_roof + 2000.0)


def door(x0, x1, y, t, height, tooth=110.0, depth=46.0):
    """A door standing open, hanging from the bay's edge at y: its top edge
    is its hinge, let 2 mm into the skin's outside, and its free edge is
    serrated, as every door on a low-observable airframe is -- a straight
    edge across the flow is a bright return, a sawtooth scatters it."""
    n_t = max(2, int(round((x1 - x0) / tooth)))
    xs = []
    for i in range(n_t):
        a = x0 + (x1 - x0) * i / n_t
        xs += [a, a + (x1 - x0) / n_t * 0.5]
    xs.append(x1)
    rings = []
    for k, x in enumerate(xs):
        # under the skin's outside across the door's whole thickness: the
        # skin slopes, and the door's top is its lowest point, not its middle
        zt = min(shapes.z_dn(x, y - t / 2), shapes.z_dn(x, y + t / 2)) + 2.0
        zb = zt - height + (depth if k % 2 == 0 else 0.0)
        rings.append([(x, y - t / 2, zt), (x, y + t / 2, zt),
                      (x, y + t / 2, zb), (x, y - t / 2, zb)])
    return shapes.loft_rings(rings)


def _about_z(part, cx, cy, cz):
    """A solid of revolution built about +X, stood upright about +Z at
    (cx, cy, cz)."""
    v, f = part
    return [(cx + y, cy + z, cz + x) for (x, y, z) in v], f


def _about_y(part, cx, cy, cz):
    """A solid of revolution built about +X, turned to lie about +Y and
    moved to (cx, cy, cz)."""
    v, f = part
    return [(cx + z, cy + x, cz + y) for (x, y, z) in v], f


def wheel(cx, cy, cz, r, w):
    """(tyre, hub) for one wheel on an axle along y."""
    rr = r * 0.55
    prof = []
    n = 10
    for i in range(n + 1):
        a = -0.5 * math.pi + math.pi * i / n
        prof.append((0.5 * w * math.sin(a), r - 0.18 * w + 0.18 * w * math.cos(a)))
    prof = [(-0.5 * w, rr)] + prof + [(0.5 * w, rr)]
    tyre = _about_y(mesh.revolve_ring(prof, 48), cx, cy, cz)
    hub = _about_y(mesh.revolve_ring([(-0.45 * w, 24.0), (0.45 * w, 24.0),
                                      (0.45 * w, rr + 3.0), (-0.45 * w, rr + 3.0)],
                                     36), cx, cy, cz)
    return tyre, hub


def nose():
    x0, x1, hw = G["nose_bay"]
    zr = -40.0
    xc = G["nose_x"]
    r, w = G["nose_wheel_r"], G["nose_wheel_w"]
    z_ax = spec.GROUND_Z + r
    x_ax = xc - 60.0                      # the leg rakes forward a little
    leg = mesh.join(
        mesh.pipe([(xc, 0.0, zr + 4.0), (xc - 20.0, 0.0, -1150.0)], G["strut_r_nose"], 20),
        mesh.pipe([(xc - 20.0, 0.0, -1100.0), (x_ax, 0.0, z_ax + 60.0)], 40.0, 18),
        mesh.box(x_ax, 0.0, z_ax + 30.0, 70.0, 60.0, 90.0),               # yoke
        mesh.pipe([(x_ax, -w - 60.0, z_ax), (x_ax, w + 60.0, z_ax)], 28.0, 14),  # axle
        # torque link
        mesh.pipe([(xc - 20.0, 0.0, -1040.0), (xc + 90.0, 0.0, -1250.0),
                   (x_ax, 0.0, z_ax + 90.0)], 12.0, 10, bend=0.0),
        # the gland nut where the piston enters the oleo, and the steering
        # collar above it
        _about_z(mesh.revolve_ring([(-12.0, 38.0), (12.0, 38.0), (12.0, 62.0),
                                    (-12.0, 62.0)], 32), xc - 18.0, 0.0, -1108.0),
        _about_z(mesh.revolve_ring([(-30.0, 50.0), (30.0, 50.0), (30.0, 70.0),
                                    (-30.0, 70.0)], 32), xc - 8.0, 0.0, -560.0),
        # taxi light on the collar, looking forward
        mesh.box(xc - 82.0, 0.0, -560.0, 34.0, 60.0, 40.0))
    tyres, hubs = [], []
    for sy in (1.0, -1.0):
        t, h = wheel(x_ax, sy * (0.5 * w + 45.0), z_ax, r, w)
        tyres.append(t); hubs.append(h)
    doors = [door(x0 + 10.0, x1 - 10.0, sy * (hw + 5.0), 8.0, 420.0) for sy in (1.0, -1.0)]
    return {"gear_nose": leg, "tyres_nose": mesh.join(*tyres),
            "wheels_nose": mesh.join(*hubs), "gear_door_nose": mesh.join(*doors),
            "gear_bay_nose": bay_box(x0, x1, -hw, hw, zr)}, bay_cutter(x0, x1, -hw, hw, zr)


def main(sy):
    x0, x1, y0, y1 = G["main_bay"]
    zr = -330.0
    xc, yc = G["main_x"], G["main_y"]
    r, w = G["main_wheel_r"], G["main_wheel_w"]
    z_ax = spec.GROUND_Z + r
    yw = yc + 0.5 * w + 60.0              # the wheel is outboard of its leg
    leg = mesh.join(
        mesh.pipe([(xc, yc, zr + 4.0), (xc, yc, -1050.0)], G["strut_r_main"], 22),
        mesh.pipe([(xc, yc, -1000.0), (xc, yc, z_ax)], 60.0, 20),
        mesh.pipe([(xc, yc - 40.0, z_ax), (xc, yw + 0.5 * w + 30.0, z_ax)], 42.0, 16),
        # drag brace from the bay's front to the leg
        mesh.pipe([(x0 + 60.0, yc, zr - 2.0), (xc - 60.0, yc, -900.0)], 30.0, 14),
        # side brace to the bay's inboard wall
        mesh.pipe([(xc, y0 + 40.0, zr - 2.0), (xc, yc - 50.0, -760.0)], 26.0, 14),
        # the gland nut where the chrome piston enters the oleo
        _about_z(mesh.revolve_ring([(-14.0, 58.0), (14.0, 58.0), (14.0, 88.0),
                                    (-14.0, 88.0)], 40), xc, yc, -1040.0),
        # the torque link: a scissor on the leg's front, cylinder to axle
        mesh.pipe([(xc - 70.0, yc, -1010.0), (xc - 170.0, yc, -1130.0),
                   (xc - 58.0, yc, z_ax + 70.0)], 13.0, 10, bend=0.0),
        # the fork the axle runs through
        mesh.box(xc, yc + 10.0, z_ax, 150.0, 110.0, 120.0))
    t, h = wheel(xc, yw, z_ax, r, w)
    doors = mesh.join(door(x0 + 10.0, x1 - 10.0, y1 + 5.0, 8.0, 520.0),
                      door(x0 + 10.0, x1 - 10.0, y0 - 5.0, 8.0, 300.0))
    parts = {"gear_main": leg, "tyre_main": t, "wheel_main": h,
             "gear_door_main": doors, "gear_bay_main": bay_box(x0, x1, y0, y1, zr)}
    cut = bay_cutter(x0, x1, y0, y1, zr)
    if sy < 0:
        mir = lambda p: shapes.orient(([(x, -y, z) for (x, y, z) in p[0]],
                                       [tuple(reversed(c)) for c in p[1]]))
        parts = {k: mir(v) for k, v in parts.items()}
        cut = mir(cut)
    return parts, cut


def build():
    out = {}
    n, cn = nose()
    out.update(n)
    cuts = [cn]
    for side, sy in (("r", 1.0), ("l", -1.0)):
        p, c = main(sy)
        cuts.append(c)
        for k, v in p.items():
            out[f"{k}_{side}"] = v
    out["cut:fuselage_skin"] = mesh.join(*cuts)
    return out
