"""Landing gear: a twin-wheel nose leg and two single-wheel main legs, each
retracting into a bay closed by doors that lie flush with the skin.

The model is built with the gear down and the doors open, standing on the
ground; `pose("up")` gives the same parts retracted with the doors shut, and
`kinematics()` gives the viewer the pivots and angles between the two.

    nose gear   retracts forward about a pivot under the cockpit floor, the
                twin wheels ending up under the nose between the radar
                bulkhead and the cockpit; two doors hinged at the bay's
                sides close under it
    main gear   mounted in the wing roots, 5.8 m apart, each retracting
                inboard about a fore-and-aft pivot, so the wheel ends up
                lying flat in the thick wing-body blend beside the intake
                duct; a door on the leg closes the leg's slot in the wing,
                and a door hinged at the well's inboard edge closes over
                the wheel

Every stowed position is checked: tools/audit_stowage.py re-runs the
interference audit with the gear up and the doors shut.

The mains are about 700 mm behind the aft-most centre of gravity and carry
89 % of the weight; the nose leg carries the other 11 % (verify.py measures
both).
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
DOOR_T = spec.SKIN_T


# --------------------------------------------------------------------------
# geometry helpers

def rotate(verts, point, axis, ang):
    """Rodrigues: verts turned by ang about the line through point along
    the unit vector axis."""
    k = axis
    c, s = math.cos(ang), math.sin(ang)
    out = []
    for p in verts:
        v = [p[i] - point[i] for i in range(3)]
        dot = sum(k[i] * v[i] for i in range(3))
        cr = (k[1] * v[2] - k[2] * v[1], k[2] * v[0] - k[0] * v[2],
              k[0] * v[1] - k[1] * v[0])
        out.append(tuple(point[i] + v[i] * c + cr[i] * s + k[i] * dot * (1 - c)
                         for i in range(3)))
    return out


def _rot_part(part, point, axis, ang):
    v, f = part
    return rotate(v, point, axis, ang), f


def _mirror(part):
    v, f = part
    return shapes.orient(([(x, -y, z) for (x, y, z) in v],
                          [tuple(reversed(c)) for c in f]))


def wing_lower_z(x, y):
    """The wing's lower surface at (x, y), or None off the wing."""
    from parts import surfaces
    W = spec.WING
    ay = abs(y)
    if not W["y_root"] - 400.0 <= ay <= W["y_tip"]:
        return None
    c = spec.wing_chord(ay)
    u = (x - spec.wing_le_x(ay)) / c
    if not 0.0 <= u <= 1.0:
        return None
    P = surfaces.WingPlace()
    v = 4 * P.camber * u * (1 - u) - shapes.naca_t(u, P.tc(ay))
    return P(ay, u, v)[2]


def under_z(x, y):
    """The aircraft's underside at (x, y): the body's, or the wing's where
    the wing hangs below it."""
    zs = []
    if abs(y) < shapes.half_width(x) - 1.0:
        zs.append(shapes.z_dn(x, y))
    zw = wing_lower_z(x, y)
    if zw is not None:
        zs.append(zw)
    return min(zs) if zs else 0.0


def serration(u, teeth, tooth):
    """A sawtooth across a door's width: zero at its ends and at every
    tooth root, `tooth` deep at every tooth point. u runs 0..1."""
    if teeth <= 0:
        return 0.0
    f = u * teeth
    f -= math.floor(f)
    return tooth * (1.0 - abs(2.0 * f - 1.0))


def _outline(x0, x1, y0, y1, teeth, tooth, n, ends=(True, True), frame=None):
    """The fore and aft edges over y0..y1. The sawtooth is laid out over
    `frame` (default y0..y1), so a door can follow the teeth of an opening
    wider than itself tooth for tooth."""
    ys = [y0 + (y1 - y0) * j / n for j in range(n + 1)]
    f0, f1 = frame or (y0, y1)
    s = lambda y, on: serration((y - f0) / (f1 - f0), teeth, tooth) if on else 0.0
    fwd = [(x0 + s(y, ends[0]), y) for y in ys]
    aft = [(x1 - s(y, ends[1]), y) for y in ys]
    return fwd, aft


def panel(x0, x1, y0, y1, teeth=0, tooth=60.0, n=None, t=DOOR_T, gap=1.0,
          frame=None, ends=(True, True)):
    """A door lying flush in the underside: its outside on the underside,
    t thick inward, over x0..x1 (its fore and aft ends serrated with
    `teeth` teeth) and y0..y1, `gap` in from the opening all round. Each
    end is capped quad by quad along the teeth."""
    n = n or max(8, 4 * teeth)
    x0, x1 = x0 + gap, x1 - gap
    y0, y1 = (y0 + gap, y1 - gap) if y1 > y0 else (y0 - gap, y1 + gap)
    frame = frame or ((y0 - gap, y1 + gap) if y1 > y0 else (y0 + gap, y1 - gap))
    fwd, aft = _outline(x0, x1, y0, y1, teeth, tooth, n, ends=ends, frame=frame)
    rings = []
    for i in range(25):
        t_ = i / 24.0
        pts = [(fx + (ax - fx) * t_, y) for (fx, y), (ax, _) in zip(fwd, aft)]
        outer = [(x, y, under_z(x, y)) for (x, y) in pts]
        inner = [(x, y, under_z(x, y) + t) for (x, y) in reversed(pts)]
        rings.append(outer + inner)
    v, f = shapes.loft_rings(rings, cap_start=False, cap_end=False)
    m = len(rings[0])
    f = list(f)
    for base, flip in ((0, True), ((len(rings) - 1) * m, False)):
        for k in range(n):
            q = (base + k, base + k + 1, base + m - 2 - k, base + m - 1 - k)
            f.append(tuple(reversed(q)) if flip else q)
    return shapes.orient((v, f))


def prism(x0, x1, y0, y1, z_top, teeth=0, tooth=60.0, n=None, ends=(True, True),
          z_bot=-3000.0):
    """A cutter on a door's outline, from well below the aircraft up to
    z_top: the opening the door closes. `ends` says which of its fore and
    aft ends are serrated.

    Two cutters on one part must not share a vertex: the audits weld
    coincident vertices, and two overlapping solids welded into one read
    as neither. Nor share a plane: overlapping coplanar faces in one cutter
    are what the exact boolean gets wrong."""
    n = n or max(8, 4 * teeth)
    fwd, aft = _outline(x0, x1, y0, y1, teeth, tooth, n, ends)
    outline = fwd + list(reversed(aft))
    m = len(outline)
    v = [(x, y, z_bot) for (x, y) in outline] + [(x, y, z_top) for (x, y) in outline]
    f = [(k, (k + 1) % m, m + (k + 1) % m, m + k) for k in range(m)]
    # the ends capped strip by strip across the width, never as one n-gon:
    # a serrated outline is concave, and a fan over it closes over the
    # notches and leaves the solid's inside ambiguous
    for base in (0, m):
        for k in range(n):
            q = (base + k, base + k + 1, base + m - 2 - k, base + m - 1 - k)
            f.append(q if base else tuple(reversed(q)))
    return shapes.orient((v, f))


def walls(x0, x1, y0, y1, roof, skip=()):
    """A bay's roof and walls, standing on the skin's inside. `skip` names
    walls to leave out ('y1', 'x0', ...) where the bay opens into the next
    one or into the wing."""
    parts = [mesh.box(0.5 * (x0 + x1), 0.5 * (y0 + y1), roof + WALL / 2,
                      x1 - x0 + 2 * WALL, abs(y1 - y0) + 2 * WALL, WALL)]
    lo, hi = min(y0, y1), max(y0, y1)

    def foot(x, y):
        # on the skin's inside where the body is, on the wing's lower
        # surface where it is not
        if abs(y) < shapes.half_width(x, spec.SKIN_T) - 1.0:
            return shapes.z_dn(x, y, spec.SKIN_T) + 2.0
        return under_z(x, y) + 1.0
    xs = [x0 + (x1 - x0) * i / 16 for i in range(17)]
    for tag, ya, yb in (("ylo", lo - WALL, lo), ("yhi", hi, hi + WALL)):
        if tag in skip:
            continue
        parts.append(shapes.loft_rings(
            [[(x, ya, max(foot(x, ya), foot(x, yb))), (x, yb, max(foot(x, ya), foot(x, yb))),
              (x, yb, roof), (x, ya, roof)] for x in xs]))
    ys = [lo + (hi - lo) * i / 16 for i in range(17)]
    for tag, xa, xb in (("x0", x0 - WALL, x0), ("x1", x1, x1 + WALL)):
        if tag in skip:
            continue
        parts.append(shapes.loft_rings(
            [[(xa, y, max(foot(xa, y), foot(xb, y))), (xa, y, roof), (xb, y, roof),
              (xb, y, max(foot(xa, y), foot(xb, y)))] for y in ys]))
    return mesh.join(*parts)


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


def wheel(cx, cy, cz, r, w, out=1.0):
    """(tyre, hub) for one wheel on an axle along y; `out` is the side (+1
    or -1 in y) its dished outer face looks to.

    The tyre is a radial with rounded shoulders and three circumferential
    grooves in its crown; the hub a split rim with flanges either side of
    the bead, a dished face, a centre cap and a ring of ten bolts."""
    rr = r * 0.55                     # bead seat
    # tyre: crown with grooves, shoulders rounding into the sidewalls
    grooves = (-0.26 * w, 0.0, 0.26 * w)
    gw, gd = 0.055 * w, 0.022 * r
    crown = []
    n = 40
    for i in range(n + 1):
        u = -1.0 + 2.0 * i / n
        x = 0.5 * w * u
        shoulder = max(0.0, abs(u) - 0.62) / 0.38
        rad = r - 0.20 * w * (1.0 - math.sqrt(max(0.0, 1.0 - shoulder ** 2)))
        if any(abs(x - g) < gw / 2 for g in grooves):
            rad -= gd
        crown.append((x, rad))
    prof = [(-0.5 * w, rr), (-0.5 * w, crown[0][1] - 0.2 * w)] + crown[1:-1] + \
           [(0.5 * w, crown[-1][1] - 0.2 * w), (0.5 * w, rr)]
    tyre = _about_y(mesh.revolve_ring(prof, 64), cx, cy, cz)
    # hub: flanges at the bead, a barrel, the face dished in toward the axle
    h = 0.5 * w
    face = [(h * 0.95, rr + 10.0), (h * 0.95, rr - 6.0), (h * 0.55, rr - 10.0),
            (h * 0.35, rr * 0.55), (h * 0.62, rr * 0.30), (h * 0.66, 30.0),
            (h * 0.66, 12.0)]
    back = [(-h * 0.95, 12.0), (-h * 0.95, rr - 6.0), (-h * 0.95, rr + 10.0)]
    prof = [(x * out, y) for (x, y) in face] + [(x * out, y) for (x, y) in back]
    hub = mesh.revolve_ring(prof, 64)
    parts = [hub]
    # the bolt ring and the centre cap on the outer face
    for k in range(10):
        a = 2.0 * math.pi * k / 10
        yb, zb = rr * 0.42 * math.cos(a), rr * 0.42 * math.sin(a)
        xf = h * 0.50 * out
        parts.append(mesh.pipe([(xf - 2.0 * out, yb, zb), (xf + 12.0 * out, yb, zb)],
                               9.0, 8))
    parts.append(mesh.revolve_ring([(h * 0.60 * out, 12.0), (h * 0.80 * out, 12.0),
                                    (h * 0.80 * out, 34.0), (h * 0.60 * out, 34.0)], 32))
    return tyre, _about_y(mesh.join(*parts), cx, cy, cz)


def brake(cx, cy, cz, r, w, inboard):
    """A carbon brake stack inside the wheel's rim on its inboard half, and
    the piston housing on top of it."""
    y0 = cy + inboard * 0.04 * w
    y1 = cy + inboard * 0.40 * w
    disc = mesh.pipe([(cx, y0, cz), (cx, y1, cz)], r * 0.44, 48)
    housing = mesh.box(cx - r * 0.22, 0.5 * (y0 + y1), cz + r * 0.30, 80.0,
                       abs(y1 - y0), 50.0)
    return mesh.join(disc, housing)



# --------------------------------------------------------------------------
# nose gear

NX0, NX1, NHW = G["nose_bay"]
N_SPLIT = G["nose_bay_split"]           # tall forward of here, shallow aft
N_ROOF = G["nose_roofs"]


def nose_pivot():
    return (G["nose_x"], 0.0, G["nose_pivot_z"])


def nose_leg():
    """The nose leg, down: trunnion, oleo, yoke, axle, torque link, steering
    collar and taxi light, and an A-frame brace from the leg up to the
    trunnion's ends. (tyres, hubs, leg)."""
    px, _, pz = nose_pivot()
    r, w = G["nose_wheel_r"], G["nose_wheel_w"]
    z_ax = spec.GROUND_Z + r
    yw = 0.5 * w + 40.0
    leg = [
        # the trunnion, its ends in bearings let into the bay's side walls
        mesh.pipe([(px, -(NHW + 4.0), pz), (px, NHW + 4.0, pz)], 26.0, 20),
        mesh.pipe([(px, 0.0, pz), (px, 0.0, pz - 800.0)], G["strut_r_nose"], 24),
        mesh.pipe([(px, 0.0, pz - 790.0), (px, 0.0, z_ax + 110.0)], 36.0, 20),
        _about_z(mesh.revolve_ring([(-12.0, 34.0), (12.0, 34.0), (12.0, 58.0),
                                    (-12.0, 58.0)], 32), px, 0.0, pz - 792.0),
        _about_z(mesh.revolve_ring([(-28.0, 48.0), (28.0, 48.0), (28.0, 66.0),
                                    (-28.0, 66.0)], 32), px, 0.0, pz - 330.0),
        mesh.box(px - 70.0, 0.0, pz - 330.0, 34.0, 56.0, 38.0),          # taxi light
        mesh.box(px, 0.0, z_ax + 55.0, 80.0, 110.0, 150.0),              # yoke
        mesh.pipe([(px, -(yw + 0.5 * w + 18.0), z_ax),
                   (px, yw + 0.5 * w + 18.0, z_ax)], 24.0, 16),          # axle
        mesh.pipe([(px - 40.0, 0.0, pz - 760.0), (px - 150.0, 0.0, pz - 930.0),
                   (px - 40.0, 0.0, z_ax + 120.0)], 11.0, 10, bend=0.0),  # torque link
    ]
    for sy in (1.0, -1.0):                                               # A-frame
        leg.append(mesh.pipe([(px, sy * (NHW - 30.0), pz),
                              (px, sy * 44.0, pz - 430.0)], 16.0, 14))
    # the up-lock's roller on the yoke's front, which is up when the leg is
    # stowed forward
    leg.append(mesh.box(px - 40.0 - 6.0, 0.0, z_ax + 55.0, 14.0, 30.0, 26.0))
    leg.append(mesh.pipe([(px - 40.0 - 14.0, -25.0, z_ax + 55.0),
                          (px - 40.0 - 14.0, 25.0, z_ax + 55.0)], 10.0, 16, bend=0.0))
    # the steering's electro-hydrostatic actuator on the steering collar's aft
    # side, and its cable, with the taxi light's, up the leg into the
    # trunnion, which is hollow
    sz = pz - 330.0
    leg.append(mesh.box(px + 66.0 + 18.0, 0.0, sz, 40.0, 60.0, 70.0))
    leg.append(mesh.pipe([(px + 84.0, 0.0, sz + 30.0), (px + 84.0, 0.0, sz + 80.0),
                          (px + 50.0, 20.0, pz - 80.0), (px + 12.0, 30.0, pz - 8.0)],
                         5.0, 10, bend=20.0))
    leg.append(mesh.pipe([(px - 80.0, 0.0, sz + 15.0), (px - 80.0, 0.0, sz + 60.0),
                          (px - 50.0, -20.0, pz - 80.0), (px - 12.0, -30.0, pz - 8.0)],
                         4.0, 10, bend=20.0))
    tyres, hubs = [], []
    for sy in (1.0, -1.0):
        t, h = wheel(px, sy * yw, z_ax, r, w, out=sy)
        tyres.append(t); hubs.append(h)
    return mesh.join(*tyres), mesh.join(*hubs), mesh.join(*leg)


def nose_door(sy):
    """One of the two nose doors, shut: flush, from the centreline split to
    the bay's side, hinged along the side."""
    # laid out on the whole opening's teeth, so the two doors' teeth are the
    # opening's
    # (the aft end straight: the leg comes out through it)
    return mesh.join(panel(NX0, NX1, sy * 2.0, sy * NHW, teeth=8, tooth=70.0, n=64,
                           frame=(-NHW, NHW), ends=(True, False)),
                     _horn(nose_horn(sy), sy))


def nose_hinge(sy):
    a = (NX0, sy * NHW, under_z(NX0, sy * NHW) + DOOR_T / 2)
    b = (NX1, sy * NHW, under_z(NX1, sy * NHW) + DOOR_T / 2)
    L = math.dist(a, b)
    return a, tuple((b[i] - a[i]) / L for i in range(3))


NOSE_DOOR_OPEN = math.radians(95.0)


def nose_bay():
    """The bay: tall enough forward for the wheels, shallow aft under the
    cockpit floor where only the leg lies."""
    fwd = walls(NX0, N_SPLIT, -NHW, NHW, N_ROOF[0], skip=("x1",))
    aft = walls(N_SPLIT, NX1, -NHW, NHW, N_ROOF[1], skip=("x0",))
    # the step between the two roofs
    step = mesh.box(N_SPLIT - WALL / 2, 0.0, 0.5 * (N_ROOF[0] + N_ROOF[1]),
                    WALL, 2 * NHW + 2 * WALL, N_ROOF[0] - N_ROOF[1] + WALL)
    return mesh.join(fwd, aft, step)


def nose_cutter():
    # tall forward, shallow aft, the opening's ends serrated as the doors'
    # are (4 teeth a door, 8 across both)
    return mesh.join(
        prism(NX0, N_SPLIT + 2.0, -NHW, NHW, N_ROOF[0], teeth=8, tooth=70.0,
              ends=(True, False)),
        prism(N_SPLIT - 2.0, NX1, -NHW, NHW, N_ROOF[1], teeth=8, tooth=70.0,
              ends=(False, False), z_bot=-2990.0))


# --------------------------------------------------------------------------
# main gear (starboard; port is its mirror)

MX = G["main_x"]
MY, MZ = G["main_pivot"]
WELL = G["main_well"]           # x0, x1, y0, y1: the wheel well
SLOT_HW = G["main_slot_hw"]
ROOF = G["main_roofs"]          # well, slot


def main_pivot():
    return (MX, MY, MZ)


def main_stow_angle():
    """About +x, from down to stowed: the leg ends up pointing inboard and a
    little down, with the wheel lying in the well at main_stow_z."""
    r = G["main_wheel_r"]
    L = MZ - (spec.GROUND_Z + r)
    th = math.asin((MZ - G["main_stow_z"]) / L)
    # R_x(a) takes (0, 0, -1) to (0, sin a, -cos a): inboard is -y, so
    # sin a = -cos th and cos a = sin th
    return -(0.5 * math.pi - th)


def main_leg():
    """(tyre, hub, leg) for the starboard main gear, down."""
    r, w = G["main_wheel_r"], G["main_wheel_w"]
    z_ax = spec.GROUND_Z + r
    px, py, pz = main_pivot()
    crown = z_ax + r + 45.0
    arm_y = 0.5 * w + 22.0
    leg = [
        mesh.pipe([(px - 160.0, py, pz), (px + 160.0, py, pz)], 30.0, 24),   # trunnion
        mesh.pipe([(px, py, pz), (px, py, pz - 590.0)], G["strut_r_main"], 28),
        mesh.pipe([(px, py, pz - 580.0), (px, py, crown)], 46.0, 24),        # piston
        _about_z(mesh.revolve_ring([(-14.0, 44.0), (14.0, 44.0), (14.0, 72.0),
                                    (-14.0, 72.0)], 40), px, py, pz - 582.0),
        mesh.box(px, py, crown, 130.0, 2 * arm_y + 36.0, 60.0),               # fork crown
        # the torque link on the leg's inboard face, where it stows inside
        # the slot with the leg
        mesh.pipe([(px, py - 58.0, pz - 540.0), (px, py - 150.0, pz - 690.0),
                   (px, py - 50.0, crown + 20.0)], 12.0, 10, bend=0.0),
        mesh.pipe([(px, py - arm_y - 30.0, z_ax), (px, py + arm_y + 30.0, z_ax)],
                  30.0, 20),                                                  # axle
    ]
    for sy in (1.0, -1.0):                                                    # fork arms
        leg.append(mesh.box(px, py + sy * arm_y, 0.5 * (crown + z_ax),
                            96.0, 20.0, crown - z_ax + 50.0))
    # the up-lock's roller, on the crown's inboard face: that face is up
    # when the leg is stowed, and the roller rides into the hook in the
    # well's roof
    # (45 mm aft of the leg's middle: the torque link is down the middle)
    yc = py - (arm_y + 18.0)
    ux = px + UPLOCK_DX
    leg.append(mesh.box(ux, yc - 6.0, crown, 24.0, 14.0, 26.0))
    leg.append(mesh.pipe([(ux - 14.0, yc - 10.0 - 4.0, crown),
                          (ux + 14.0, yc - 10.0 - 4.0, crown)], 10.0, 16, bend=0.0))
    # the brake's own electro-hydrostatic unit on the inboard fork arm, and
    # its cable up the leg's front into the trunnion, which is hollow and
    # carries it to the rotary actuator on the pivot
    by = py - arm_y - 10.0 - 25.0
    leg.append(mesh.box(px + 10.0, by, z_ax + 170.0, 70.0, 50.0, 80.0))
    leg.append(mesh.pipe([(px + 10.0, by, z_ax + 206.0), (px + 10.0, by, crown + 50.0),
                          (px - 80.0, py - 40.0, crown + 90.0),
                          (px - 80.0, py - 40.0, pz - 90.0),
                          (px - 40.0, py - 12.0, pz - 12.0)], 5.0, 10, bend=20.0))
    t, h = wheel(px, py, z_ax, r, w, out=1.0)
    h = mesh.join(h, brake(px, py, z_ax, r, w, -1.0))
    return t, h, mesh.join(*leg)


LEG_DOOR_Y1 = MY - 200.0      # the leg's door ends here; the pivot door on
PIVOT_END = MY + 40.0         # from here out to the end of the slot


def leg_door_stowed():
    """The door on the leg, as it lies shut: flush in the wing's underside
    over the leg's slot, from the well to 200 mm short of the pivot. Its
    outboard end stops there because anything on the leg closer to the
    pivot than that swings up into the wing as the leg comes down."""
    door = panel(MX - SLOT_HW, MX + SLOT_HW, WELL[3], LEG_DOOR_Y1)
    # two brackets from its inside up to the leg, which carry it
    px, py, pz = main_pivot()
    cy = py - math.sqrt((pz - (spec.GROUND_Z + G["main_wheel_r"])) ** 2
                        - (pz - G["main_stow_z"]) ** 2)
    brackets = []
    for y in (WELL[3] + 180.0, LEG_DOOR_Y1 - 120.0):
        f = (py - y) / (py - cy)
        z_axis = pz + (G["main_stow_z"] - pz) * f
        z0 = under_z(MX, y) + DOOR_T - 2.0
        brackets.append(mesh.pipe([(MX, y, z0),
                                   (MX, y, z_axis - G["strut_r_main"] + 3.0)],
                                  11.0, 12))
    return mesh.join(door, *brackets)


def pivot_door():
    """The small door over the pivot end of the slot, shut. It is hinged
    along the slot's forward edge, on its outer face, and swings down to
    stand beside the leg, clear of it."""
    door = panel(MX - SLOT_HW, MX + SLOT_HW, LEG_DOOR_Y1, PIVOT_END, gap=3.0)
    # two hinge knuckles on the hinge line, between the door and the slot's
    # edge: on the axis, they stay where they are as the door swings
    a, ax = pivot_hinge()
    L = PIVOT_END - LEG_DOOR_Y1
    knuckles = [mesh.pipe([tuple(a[i] + ax[i] * s0 for i in range(3)),
                           tuple(a[i] + ax[i] * (s0 + 40.0) for i in range(3))],
                          7.0, 12)
                for s0 in (20.0, L - 60.0)]
    return mesh.join(door, *knuckles)


def pivot_hinge():
    x0 = MX - SLOT_HW
    a = (x0, LEG_DOOR_Y1, under_z(x0, LEG_DOOR_Y1))
    b = (x0, PIVOT_END, under_z(x0, PIVOT_END))
    L = math.dist(a, b)
    return a, tuple((b[i] - a[i]) / L for i in range(3))


PIVOT_DOOR_OPEN = math.radians(90.0)


def wheel_door():
    """The wheel-well door, shut: flush over the well, its fore and aft
    ends serrated, hinged along the well's inboard edge."""
    x0, x1, y0, y1 = WELL
    return mesh.join(panel(x0, x1, y0, y1, teeth=4, tooth=70.0),
                     _horn(MAIN_HORN, 1.0))


# --------------------------------------------------------------------------
# door actuators
#
# Each gear door is opened and shut by a linear actuator from its bay's wall
# to a horn on the door beside its hinge -- they swung on their hinges
# driven by nothing. The horn stands up into the bay when the door is shut;
# the actuator is drawn for whichever pose the aircraft is built in, and the
# viewer swings its body and runs its rod out to follow the horn.

# the horn's eye, door shut: (x, y, z) of the pin
MAIN_HORN = (MX, WELL[2] + 10.0, -555.0)
MAIN_ANCHOR = (MX, WELL[2] + 16.0, -390.0)
NOSE_HORN_X = 3000.0
STRUT_BODY = 140.0


def _horn(eye, side):
    """A lug on the door's inside, up from its hinge edge to its eye, and
    the eye: across it along x. `side` puts the lug on the eye's outboard
    face."""
    x, y, z = eye
    z_door = z - 71.0
    return mesh.join(mesh.box(x, y, 0.5 * (z_door + z), 30.0, 10.0, z - z_door),
                     mesh.pipe([(x - 15.0, y, z), (x + 15.0, y, z)], 8.0, 12, bend=0.0))


def _strut(A, L, bracket_to):
    """(body, rod) from anchor A to lug L, the body's bracket reaching to y
    bracket_to on the bay's wall."""
    d = [L[i] - A[i] for i in range(3)]
    n = math.sqrt(sum(c * c for c in d))
    u = [c / n for c in d]
    at = lambda t: tuple(A[i] + u[i] * t for i in range(3))
    body = mesh.join(
        mesh.pipe([A, at(STRUT_BODY)], 13.0, 16, bend=0.0),
        mesh.pipe([(A[0] - 13.0, A[1], A[2]), (A[0] + 13.0, A[1], A[2])], 9.0, 12,
                  bend=0.0),
        mesh.box(A[0], 0.5 * (A[1] + bracket_to), A[2], 30.0, abs(bracket_to - A[1]),
                 26.0))
    rod = mesh.join(
        mesh.pipe([at(STRUT_BODY - 20.0), L], 6.0, 16, bend=0.0),
        mesh.pipe([(L[0] - 22.0, L[1], L[2]), (L[0] - 15.0, L[1], L[2])], 9.0, 16,
                  bend=0.0))
    return body, rod


def main_door_strut(up):
    L = MAIN_HORN
    if not up:
        hp, hax = wheel_hinge()
        L = rotate([L], hp, hax, WHEEL_DOOR_OPEN)[0]
    return _strut(MAIN_ANCHOR, L, WELL[2] - 2.0)


def nose_horn(sy):
    hp, hax = nose_hinge(sy)
    t = (NOSE_HORN_X - hp[0]) / hax[0]
    x, y, z = (hp[i] + hax[i] * t for i in range(3))
    return (x, sy * (NHW - 10.0), z + 71.0)


def nose_door_strut(sy, up):
    L = nose_horn(sy)
    A = (NOSE_HORN_X, sy * (NHW - 16.0), L[2] + 170.0)
    if not up:
        hp, hax = nose_hinge(sy)
        L = rotate([L], hp, hax, sy * NOSE_DOOR_OPEN)[0]
    return _strut(A, L, sy * (NHW + 2.0))


def wheel_hinge():
    x0, x1, y0, _ = WELL
    a = (x0, y0, under_z(x0, y0) + DOOR_T / 2)
    b = (x1, y0, under_z(x1, y0) + DOOR_T / 2)
    L = math.dist(a, b)
    return a, tuple((b[i] - a[i]) / L for i in range(3))


WHEEL_DOOR_OPEN = math.radians(-92.0)


def main_bay():
    """The well's roof and walls where it is in the body. Outboard of the
    wing's root rib the wing's own cut faces are its walls."""
    x0, x1, y0, _ = WELL
    return walls(x0, x1, y0, spec.WING["y_root"] - 230.0, ROOF[0], skip=("yhi",))


def main_cutter():
    x0, x1, y0, y1 = WELL
    return mesh.join(prism(x0, x1, y0, y1 + 2.0, ROOF[0], teeth=4, tooth=70.0),
                     prism(MX - SLOT_HW, MX + SLOT_HW, y1 - 2.0, PIVOT_END, ROOF[1],
                           z_bot=-2990.0))


# --------------------------------------------------------------------------

# The gear's retraction actuators: a geared rotary actuator on each leg's
# trunnion, coaxial with it, bolted to the airframe -- the main legs' in a
# pocket in the wing ahead of each trunnion, the nose leg's outboard of the
# bay's starboard wall. The legs turned on their trunnions with nothing to
# turn them. Rotary, not a jack: the main leg's slot in the wing is as wide
# as the leg, and there is no room beside it for a jack's stroke.
ACT_R, ACT_L = 35.0, 150.0


def _rotary(p0, axis, flange_at_far_end=True):
    """A rotary actuator on an axis from p0, ACT_L long: its body, a
    mounting flange at its far end, and its output spline let 2 mm into the
    trunnion at p0."""
    m = math.sqrt(sum(c * c for c in axis))
    ax = [c / m for c in axis]
    q = lambda t: tuple(p0[k] + ax[k] * t for k in range(3))
    return mesh.join(
        mesh.pipe([q(-2.0), q(ACT_L)], ACT_R, 24, bend=0.0),
        mesh.pipe([q(ACT_L - 12.0), q(ACT_L)], ACT_R + 14.0, 24, bend=0.0),
        mesh.pipe([q(ACT_L * 0.35), q(ACT_L * 0.65)], ACT_R + 5.0, 24, bend=0.0))


MAIN_ACT_R = 28.0      # the wing's top skin is 40 mm over the pivot's axis
UPLOCK_DX = 45.0


def main_actuator():
    """The starboard main leg's, and the pocket it sits in. The pivot is high
    in the wing -- its axis 40 mm under the top skin -- so this one is
    slimmer than the nose leg's and has no round flange: it is bolted down
    through a foot under it into the wing's structure."""
    px, py, pz = main_pivot()
    x0 = px - 160.0
    r = MAIN_ACT_R
    act = mesh.join(
        mesh.pipe([(x0 + 2.0, py, pz), (x0 - ACT_L, py, pz)], r, 24, bend=0.0),
        mesh.pipe([(x0 - ACT_L * 0.35, py, pz), (x0 - ACT_L * 0.65, py, pz)],
                  r + 3.0, 24, bend=0.0),
        mesh.box(x0 - ACT_L / 2, py, pz - r - 14.0, ACT_L - 10.0, 2 * r + 20.0,
                 36.0))
    pocket = mesh.pipe([(x0 + 1.0, py, pz), (x0 - ACT_L - 1.0, py, pz)],
                       r + 4.0, 24, bend=0.0)
    return act, pocket


def nose_actuator():
    px, _, pz = nose_pivot()
    return _rotary((px, NHW + 4.0, pz), (0.0, 1.0, 0.0))


def main_uplock():
    """The starboard main leg's up-lock: a hook hung from the well's roof
    where the leg's roller arrives when it is stowed, its jaw under it."""
    px, py, pz = main_pivot()
    r, w = G["main_wheel_r"], G["main_wheel_w"]
    crown = spec.GROUND_Z + 2 * r + 45.0
    yc = py - (0.5 * w + 22.0 + 18.0) - 14.0
    x, y, z = rotate([(px + UPLOCK_DX, yc, crown)], (px, py, pz), (1.0, 0.0, 0.0),
                     main_stow_angle())[0]
    # bolted up into the well's roof, which is the wing's own inside here
    top = ROOF[0] + 2.0
    # the cheeks come down over the roller only as far as the crown's face,
    # 14 mm under its centre, and the jaw closes across its far side
    parts = [mesh.box(x + sx * 20.0, y, 0.5 * (top + z - 8.0), 8.0, 30.0, top - z + 8.0)
             for sx in (-1.0, 1.0)]
    parts.append(mesh.box(x, y, top - 10.0, 48.0, 30.0, 20.0))
    parts.append(mesh.box(x, y - 19.0, z + 2.0, 48.0, 10.0, 20.0))   # the jaw
    return mesh.join(*parts)


def nose_uplock():
    """The nose leg's up-lock: a hook on a hanger from the tall bay's roof
    where the yoke's roller arrives with the leg stowed forward."""
    px, _, pz = nose_pivot()
    z_ax = spec.GROUND_Z + G["nose_wheel_r"]
    x, y, z = rotate([(px - 54.0, 0.0, z_ax + 55.0)], nose_pivot(), (0.0, 1.0, 0.0),
                     0.5 * math.pi)[0]
    top = N_ROOF[0] + 2.0
    parts = [mesh.box(x, sy * 31.0, 0.5 * (top + z - 8.0), 30.0, 8.0, top - z + 8.0)
             for sy in (-1.0, 1.0)]
    parts.append(mesh.box(x, 0.0, top - 10.0, 30.0, 70.0, 20.0))
    parts.append(mesh.box(x - 19.0, 0.0, z + 8.0, 10.0, 70.0, 20.0))  # the jaw
    # the bolts holding it up to the roof, their heads under the block
    for dy in (-24.0, 24.0):
        parts.append(mesh.pipe([(x, dy, top - 20.0), (x, dy, top - 25.0)], 6.0, 12,
                               bend=0.0))
    return mesh.join(*parts)


def _main_parts(up):
    t, h, leg = main_leg()
    door = leg_door_stowed()
    P, ax, a = main_pivot(), (1.0, 0.0, 0.0), main_stow_angle()
    if up:
        t, h, leg = (_rot_part(p, P, ax, a) for p in (t, h, leg))
        wd, pd = wheel_door(), pivot_door()
    else:
        door = _rot_part(door, P, ax, -a)
        hp, hax = wheel_hinge()
        wd = _rot_part(wheel_door(), hp, hax, WHEEL_DOOR_OPEN)
        hp, hax = pivot_hinge()
        pd = _rot_part(pivot_door(), hp, hax, PIVOT_DOOR_OPEN)
    return {"gear_main": leg, "tyre_main": t, "wheel_main": h,
            "gear_leg_door_main": door, "gear_door_main": wd,
            # the up-lock is the well's structure: it is bolted up into the
            # well's roof, which out here is the wing's own inside
            "gear_pivot_door_main": pd, "gear_bay_main": mesh.join(main_bay(),
                                                                    main_uplock()),
            "gear_actuator_main": main_actuator()[0],
            "gear_door_act_main": main_door_strut(up)[0],
            "gear_door_act_rod_main": main_door_strut(up)[1]}


def pose():
    """'down' (the build) or 'up', from NYX_GEAR: tools/audit_stowage.py
    builds the aircraft with the gear up to check where it all goes."""
    return os.environ.get("NYX_GEAR", "down")


def build():
    up = pose() == "up"
    out = {}
    t, h, leg = nose_leg()
    if up:
        P, ax, a = nose_pivot(), (0.0, 1.0, 0.0), 0.5 * math.pi
        t, h, leg = (_rot_part(p, P, ax, a) for p in (t, h, leg))
    out.update({"gear_nose": leg, "tyres_nose": t, "wheels_nose": h,
                "gear_bay_nose": nose_bay(), "gear_actuator_nose": nose_actuator(),
                "gear_uplock_nose": nose_uplock()})
    for side, sy in (("r", 1.0), ("l", -1.0)):
        d = nose_door(sy)
        if not up:
            hp, hax = nose_hinge(sy)
            d = _rot_part(d, hp, hax, sy * NOSE_DOOR_OPEN)
        out[f"gear_door_nose_{side}"] = d
        body, rod = nose_door_strut(sy, up)
        out[f"gear_door_act_nose_{side}"] = body
        out[f"gear_door_act_rod_nose_{side}"] = rod
    mp = _main_parts(up)
    for k, v in mp.items():
        out[f"{k}_r"] = v
        out[f"{k}_l"] = _mirror(v)
    mc = main_cutter()
    out["cut:fuselage_skin"] = mesh.join(nose_cutter(), mc, _mirror(mc))
    pocket = main_actuator()[1]
    out["cut:wing_r"] = mesh.join(mc, pocket)
    out["cut:wing_l"] = mesh.join(_mirror(mc), _mirror(pocket))
    return out


def kinematics():
    """What the viewer needs to raise and lower the gear: each leg's pivot,
    axis and stowing angle, each door's hinge and closing angle, and the
    parts that move with each."""
    a = main_stow_angle()
    out = {"legs": [
        {"parts": ["gear_nose", "tyres_nose", "wheels_nose"],
         "pivot": list(nose_pivot()), "axis": [0.0, 1.0, 0.0],
         "stow": 0.5 * math.pi}],
        "doors": []}
    for side, sy in (("r", 1.0), ("l", -1.0)):
        px, py, pz = main_pivot()
        out["legs"].append({"parts": [f"gear_main_{side}", f"tyre_main_{side}",
                                      f"wheel_main_{side}",
                                      f"gear_leg_door_main_{side}"],
                            "pivot": [px, sy * py, pz], "axis": [1.0, 0.0, 0.0],
                            "stow": sy * a})
        hp, hax = wheel_hinge()
        out["doors"].append({"part": f"gear_door_main_{side}",
                             "hinge": [hp[0], sy * hp[1], hp[2]],
                             "axis": [hax[0], sy * hax[1], hax[2]],
                             "close": -sy * WHEEL_DOOR_OPEN})
        hp, hax = pivot_hinge()
        out["doors"].append({"part": f"gear_pivot_door_main_{side}",
                             "hinge": [hp[0], sy * hp[1], hp[2]],
                             "axis": [hax[0], sy * hax[1], hax[2]],
                             "close": -sy * PIVOT_DOOR_OPEN})
        hp, hax = nose_hinge(sy)
        out["doors"].append({"part": f"gear_door_nose_{side}",
                             "hinge": list(hp), "axis": list(hax),
                             "close": -sy * NOSE_DOOR_OPEN})
    # each door's actuator: its anchor, and its lug with the door open, which
    # turns with the door about the door's hinge
    out["struts"] = []
    for side, sy in (("r", 1.0), ("l", -1.0)):
        hp, hax = wheel_hinge()
        L = rotate([MAIN_HORN], hp, hax, WHEEL_DOOR_OPEN)[0]
        m = lambda p: (p[0], sy * p[1], p[2])
        out["struts"].append({"body": f"gear_door_act_main_{side}",
                              "rod": f"gear_door_act_rod_main_{side}",
                              "anchor": list(m(MAIN_ANCHOR)), "lug": list(m(L)),
                              "hinge": list(m(hp)), "axis": [hax[0], sy * hax[1], hax[2]],
                              "close": -sy * WHEEL_DOOR_OPEN})
        hp, hax = nose_hinge(sy)
        A = (NOSE_HORN_X, sy * (NHW - 16.0), nose_horn(sy)[2] + 170.0)
        L = rotate([nose_horn(sy)], hp, hax, sy * NOSE_DOOR_OPEN)[0]
        out["struts"].append({"body": f"gear_door_act_nose_{side}",
                              "rod": f"gear_door_act_rod_nose_{side}",
                              "anchor": list(A), "lug": list(L),
                              "hinge": list(hp), "axis": list(hax),
                              "close": -sy * NOSE_DOOR_OPEN})
    return out
