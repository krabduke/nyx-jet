"""The avionics: what flies the aeroplane and what tells it where it is.

Relaxed stability is a flight control system that never stops working. The
aeroplane is 4.8 % of the chord unstable, so it is flown by computers, and
those computers were not in it. They are in the forward avionics bay now,
over the nose gear's bay between the radar bulkhead and the cockpit, where
a fighter keeps them: close to the sensors and reachable through the
forebody's access panels.

    rack                a tray on the nose gear bay's roof, which is the
                        deck the boxes stand on
    air data computers  two, one per pitot-static probe, at the front of the
                        rack; the probes' pitot and static lines run inside
                        the radome, over the radar and through the radar
                        bulkhead to them
    inertial unit       the ring-laser gyros and accelerometers
    flight control      three, voting: any one can fail and the other two
    computers           outvote it, which a relaxed-stability aeroplane
                        needs because it cannot fly on none
    mission computer    at the back of the rack, under the IRST
    loom                each box's pigtail onto a trunk down the rack's side,
                        which comes forward from the nose gear's run of the
                        starboard fuselage loom -- the PDU's supply
"""

import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec      # noqa: E402
import shapes    # noqa: E402
import mesh      # noqa: E402

TRAY_Z = (343.0, 351.0)          # on the nose gear bay's roof
TRAY_X = (1760.0, 3240.0)
TRAY_HW = 205.0
CLEAR = 14.0                     # under the skin's inside
FRAMES = ((2230.0, 2270.0), (3250.0, 3290.0))
FRAME_DEPTH = 70.0
IRST = ((2488.0, 3000.0), 113.0, 590.0)     # x range, half width, its floor
TRUNK_Y = 185.0
TRUNK_Z = 366.0
TRUNK_R = 7.0
PIG_R = 3.2
LINE_R = 3.0

# (name, x0, x1, y0, y1, tallest it needs to be)
LRUS = (
    ("avionics_adc_r", 1780.0, 1950.0, 25.0, 145.0, 110.0),
    ("avionics_adc_l", 1780.0, 1950.0, -145.0, -25.0, 110.0),
    ("avionics_ins", 2015.0, 2190.0, -95.0, 95.0, 165.0),
    ("avionics_fcc_1", 2310.0, 2570.0, -150.0, -70.0, 190.0),
    ("avionics_fcc_2", 2310.0, 2570.0, -40.0, 40.0, 190.0),
    ("avionics_fcc_3", 2310.0, 2570.0, 70.0, 150.0, 190.0),
    ("avionics_mission", 2645.0, 2920.0, -150.0, 150.0, 180.0),
)


def _headroom(x0, x1, y0, y1):
    """The highest a box over this footprint can stand: CLEAR under the
    skin's inside, under the frames' arches it spans, under the IRST."""
    top = min(shapes.z_up(x, y, spec.SKIN_T) for x in (x0, x1, 0.5 * (x0 + x1))
              for y in (y0, y1, 0.5 * (y0 + y1))) - CLEAR
    for (a, b) in FRAMES:
        if x0 < b and x1 > a:
            top = min(top, min(shapes.z_up(x, y, spec.SKIN_T) for x in (a, b)
                               for y in (y0, y1)) - FRAME_DEPTH - CLEAR)
    (a, b), hw, floor = IRST
    if x0 < b and x1 > a and y0 < hw and y1 > -hw:
        top = min(top, floor - CLEAR)
    return top


def box_of(name):
    for (n, x0, x1, y0, y1, h) in LRUS:
        if n == name:
            z0 = TRAY_Z[1]
            return x0, x1, y0, y1, z0, min(z0 + h, _headroom(x0, x1, y0, y1))
    raise KeyError(name)


def connector_of(name):
    """Where a box's pigtail leaves it: the plug on its aft face, high up."""
    x0, x1, y0, y1, z0, z1 = box_of(name)
    return (x1 + 12.0, 0.5 * (y0 + y1), z1 - 22.0)


def _lru(name):
    """A box with a faceplate, two handles on it, a plug on its back and
    cooling fins along its top."""
    x0, x1, y0, y1, z0, z1 = box_of(name)
    w, h = y1 - y0, z1 - z0
    yc, zc = 0.5 * (y0 + y1), 0.5 * (z0 + z1)
    parts = [mesh.box(0.5 * (x0 + 6.0 + x1), yc, zc, x1 - x0 - 6.0, w, h),
             # the faceplate, proud of the case all round
             mesh.box(x0 + 3.0, yc, zc + 1.0, 6.0, w + 4.0, h + 2.0)]
    for dy in (-0.32 * w, 0.32 * w):
        parts.append(mesh.pipe([(x0 + 1.0, yc + dy, zc - 0.3 * h),
                                (x0 - 18.0, yc + dy, zc - 0.25 * h),
                                (x0 - 18.0, yc + dy, zc + 0.25 * h),
                                (x0 + 1.0, yc + dy, zc + 0.3 * h)], 3.0, 10,
                               bend=6.0))
    nf = max(3, int(w // 14))
    for i in range(nf):
        fy = y0 + w * (i + 0.5) / nf
        parts.append(mesh.box(0.5 * (x0 + x1) + 3.0, fy, z1 + 3.0,
                              x1 - x0 - 30.0, 2.0, 7.0))
    cx, cy, cz = connector_of(name)
    parts.append(mesh.box(x1 + 5.0, cy, cz, 14.0, min(46.0, w - 10.0), 22.0))
    return mesh.join(*parts)


def _rack():
    """The tray and the hold-downs at each box's feet."""
    x0, x1 = TRAY_X
    parts = [mesh.box(0.5 * (x0 + x1), 0.0, 0.5 * sum(TRAY_Z), x1 - x0,
                      2 * TRAY_HW, TRAY_Z[1] - TRAY_Z[0])]
    # a hold-down each side of each box, front and back, against its case
    for (n, bx0, bx1, by0, by1, _h) in LRUS:
        for y in (by0 - 6.0, by1 + 6.0):
            parts.append(mesh.box(bx0 + 30.0, y, TRAY_Z[1] + 6.0, 16.0, 12.0, 12.0))
            parts.append(mesh.box(bx1 - 20.0, y, TRAY_Z[1] + 6.0, 16.0, 12.0, 12.0))
    return mesh.join(*parts)


ROUTES = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      "avionics_routes.json")


def _loom():
    """The trunk down the rack's starboard edge and each box's pigtail onto
    it; the trunk runs aft off the rack and down to the nose gear's loom."""
    parts = []
    xs = [box_of(n)[1] + 12.0 for (n, *_r) in LRUS]
    x_first = min(xs) + 8.0             # where the first pigtail joins it
    feed = json.load(open(ROUTES))["feed"] if os.path.exists(ROUTES) else []
    # off the tray's edge outboard before it drops to the nose gear's loom
    trunk = [(x_first, TRUNK_Y, TRUNK_Z), (TRAY_X[1] - 20.0, TRUNK_Y, TRUNK_Z),
             (TRAY_X[1] - 20.0, TRUNK_Y + 45.0, TRUNK_Z)]
    parts.append(mesh.pipe(trunk + [tuple(p) for p in feed], TRUNK_R, 12,
                           bend=3.0 * TRUNK_R))
    # boxes side by side in a row each drop their pigtail a little further
    # aft, so the ones from the far side pass behind the near ones
    row = {}
    for (n, *_r) in sorted(LRUS, key=lambda r: -r[4]):
        c = connector_of(n)
        k = row.setdefault(round(c[0]), [])
        x = c[0] + 8.0 + 9.0 * len(k)
        k.append(n)
        path = [(c[0], c[1], c[2]), (x, c[1], c[2]),
                (x, c[1], TRUNK_Z + 30.0), (x, TRUNK_Y - 12.0, TRUNK_Z + 30.0),
                (x, TRUNK_Y, TRUNK_Z)]
        parts.append(mesh.pipe(path, PIG_R, 10, bend=8.0))
    return mesh.join(*parts)


def _pitot_lines():
    """Each probe's pitot and static lines, side by side, from its strut's
    root inside the radome to the front of its air data computer, through a
    bulkhead union in the radar bulkhead."""
    if not os.path.exists(ROUTES):
        return {}
    routes = json.load(open(ROUTES))
    out, cuts = {}, {}
    for tag, s in (("r", 1.0), ("l", -1.0)):
        path = [tuple(p) for p in routes["pitot_r"]]
        if s < 0:
            path = [(x, -y, z) for (x, y, z) in path]
        lines = []
        for k in (-1.0, 1.0):
            off = k * (LINE_R + 0.6)
            lines.append(mesh.pipe([(x, y, z + off) for (x, y, z) in path], LINE_R,
                                   10, bend=12.0))
        # the unions through the bulkhead, one per line
        xb = 1506.0
        yb, zb = _at_x(path, xb)
        for k in (-1.0, 1.0):
            off = k * (LINE_R + 0.6)
            lines.append(mesh.pipe([(xb - 14.0, yb, zb + off), (xb + 14.0, yb, zb + off)],
                                   LINE_R + 2.5, 10, bend=0.0))
        out[f"avionics_pitot_lines_{tag}"] = mesh.join(*lines)
        # the holes they need: through the radar bulkhead where the unions
        # sit, and through the skin where they leave the probe's root
        holes = [mesh.pipe([(xb - 20.0, yb, zb), (xb + 20.0, yb, zb)],
                           2.0 * LINE_R + 5.0, 16, bend=0.0)]
        cuts.setdefault("cut:radar_bulkhead", []).extend(holes)
        a, b = path[0], path[2]
        d = [b[i] - a[i] for i in range(3)]
        cuts.setdefault("cut:fuselage_skin", []).append(
            mesh.pipe([tuple(a[i] - 0.3 * d[i] for i in range(3)), b], 2 * LINE_R + 3.0,
                      12, bend=0.0))
    for k, v in cuts.items():
        out[k] = mesh.join(*v)
    return out


def _at_x(path, x):
    """(y, z) where a polyline crosses the plane at x."""
    for a, b in zip(path, path[1:]):
        if (a[0] - x) * (b[0] - x) <= 0.0 and a[0] != b[0]:
            f = (x - a[0]) / (b[0] - a[0])
            return (a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f)
    raise ValueError("the line does not cross the bulkhead")


def build():
    out = {n: _lru(n) for (n, *_r) in LRUS}
    out["avionics_rack"] = _rack()
    out["avionics_loom"] = _loom()
    out.update(_pitot_lines())
    return out
