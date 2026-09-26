"""Canopy, cockpit tub, ejection seat, instrument panel, HUD and stick.

The canopy is a single frameless bubble: a one-piece canopy gives the pilot
an unbroken view over the nose and aft over the tails, which in a turning
fight is worth more than the weight of the thicker transparency. It sits on
the skin round the cockpit opening like a lid, its rim following the skin's
curve pressed 1 mm into its seal, and overlaps the opening by 25 mm all
round.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec      # noqa: E402
import shapes    # noqa: E402
import mesh      # noqa: E402

CK = spec.COCKPIT
OVERLAP = 25.0
GLASS_T = 14.0
SEAT = -1.0          # the canopy's rim sits 1 mm into the skin, on its seal


def opening_half_width(x):
    """Half-width of the cockpit opening at x: rounded at both ends."""
    xm = 0.5 * (CK["x0"] + CK["x1"])
    L = 0.5 * (CK["x1"] - CK["x0"])
    u = min(1.0, abs(x - xm) / L)
    # blunt at the windscreen, drawn out to a point aft where the canopy
    # runs into the spine, as a fighter's canopy does
    return CK["half_w"] * max(0.0, 1.0 - u ** (2.6 if x < xm else 1.5)) ** 0.5


def canopy_top(x):
    """Height of the canopy's crown: rising steeply off the windscreen to
    its highest over the pilot's head, then a long fall to the spine."""
    # measured from the canopy's own ends, 120 mm inside the opening's, so
    # it comes down to the skin there instead of ending in a tall open arch
    x0, x1, xe = CK["x0"] + 120.0, CK["x1"] - 120.0, CK["eye_x"] + 250.0
    base = shapes.z_up(x, 0.0)
    if x <= xe:
        t = (x - x0) / (xe - x0)
        f = math.sin(0.5 * math.pi * max(0.0, t)) ** 0.8
    else:
        # an eased fall, not a cosine held high and dropped at the end: the
        # crown lets down into the spine over the whole aft half
        # It falls to the spine's height where the canopy ends, not to the
        # skin under each station: the spine rises under the aft half, and
        # a crown measured from it sagged and rose again into it.
        t = (x - xe) / (x1 - xe)
        f = (0.5 + 0.5 * math.cos(math.pi * min(1.0, t))) ** 0.7
        z_end = shapes.z_up(x1, 0.0)
        return max(base + 2.0, z_end + (CK["canopy_top_z"] - z_end) * f)
    return base + (CK["canopy_top_z"] - base) * max(f, 0.015)


def _arch(u):
    return max(0.0, 1.0 - u ** 2.3) ** 0.55


def _canopy_ys(a, n):
    """n+1 stations across, from +a to -a, packed toward the sills where the
    arch turns fastest."""
    return [a * math.cos(math.pi * k / n) for k in range(n + 1)]


def _canopy_section(x, n=40):
    """A closed arch section: the outside from one sill over the top to the
    other, and back along the inside."""
    a = opening_half_width(x) + OVERLAP
    top = canopy_top(x)
    pts_o, pts_i = [], []
    for y in _canopy_ys(a, n):
        zb = shapes.z_up(x, y) + SEAT
        f = _arch(abs(y) / a)
        pts_o.append((x, y, zb + (top - zb) * f))
    ai = a - GLASS_T
    for y in reversed(_canopy_ys(ai, n)):
        zb = shapes.z_up(x, y) + SEAT
        f = _arch(abs(y) / ai)
        # never under the skin: at the ends the glass is solid to its seat
        pts_i.append((x, y, max(zb, zb + (top - GLASS_T - zb) * f)))
    return pts_o + pts_i


def canopy():
    x0, x1 = CK["x0"], CK["x1"]
    xs = [x0 + 120.0 + (x1 - x0 - 240.0) * (0.5 - 0.5 * math.cos(math.pi * i / 36))
          for i in range(37)]
    rings = [_canopy_section(x) for x in xs]
    return shapes.loft_rings(rings)


def sill_rail(sy):
    """The canopy's sill rail on one side: the seal carrier along the rim,
    half let into the skin, the glass's edge pressed into it."""
    x0, x1 = CK["x0"], CK["x1"]
    xs = [x0 + 150.0 + (x1 - x0 - 300.0) * (0.5 - 0.5 * math.cos(math.pi * i / 30))
          for i in range(31)]
    path = []
    for x in xs:
        y = sy * (opening_half_width(x) + OVERLAP + 3.0)
        path.append((x, y, shapes.z_up(x, y) + 2.0))
    return mesh.pipe(path, 9.0, 16, subdiv=2)


def opening_cutter():
    """A prism on the opening's footprint, from below the sill to above the
    canopy, so it cuts the skin's upper surface only. The footprint stops
    140 mm inside the canopy's ends, which are 120 mm inside the opening's
    nominal ends, so the canopy covers the hole everywhere."""
    x0, x1 = CK["x0"] + 140.0, CK["x1"] - 140.0
    xs = [x0 + (x1 - x0) * i / 48 for i in range(49)]
    outline = ([(x, opening_half_width(x)) for x in xs]
               + [(x, -opening_half_width(x)) for x in reversed(xs)])
    lo = [(x, y, CK["sill_z"] - 260.0) for (x, y) in outline]
    hi = [(x, y, 2000.0) for (x, y) in outline]
    return shapes.loft_rings([lo, hi])


# --------------------------------------------------------------------------
# How the canopy opens
#
# It was a lid with nothing to lift it and nothing to hold it shut. A
# one-piece canopy hinges at its aft end, as an F-16's does: the front lifts
# and the tail stays at the spine. It is lifted by two linear actuators
# behind the seat, and held down by three hooks a side under the opening's
# edge, which pins on the canopy's rim frame drop into.

HINGE = (CK["x1"] - 160.0, 0.0, shapes.z_up(CK["x1"] - 160.0, 0.0) - 23.0)
CANOPY_OPEN = math.radians(40.0)
LOCK_X = (3600.0, 4400.0, 5220.0)   # clear of frames 3270 and 5119
ACT_X = 5450.0                  # where the actuators push on the rim
ACT_ANCHOR_X = 5620.0           # on the tub's aft wall, whose face is at 5640
ACT_ANCHOR_Z = 530.0


def _rim(x, s):
    """The rim frame's centreline on side s: along the glass's inside at its
    foot, over the opening's edge."""
    y = s * (opening_half_width(x) + 4.0)
    return (x, y, shapes.z_up(x, y) + 9.0)


def act_lug(s):
    """Where each actuator's rod end is pinned to the rim frame, canopy shut:
    34 mm inboard of the opening's edge, 30 under it. (The hole in the skin
    is cut on straight runs between stations, which fall a few millimetres
    inside the opening's curve; anything through it keeps well in.)"""
    hw = opening_half_width(ACT_X)
    y = s * (hw - 34.0)
    return (ACT_X, y, shapes.z_up(ACT_X, s * hw) - 30.0)


def act_anchor(s):
    return (ACT_ANCHOR_X, act_lug(s)[1], ACT_ANCHOR_Z)


def canopy_rim():
    """The frame bonded round the glass's foot, which everything that moves
    the canopy or holds it is on: the lock pins, the actuators' lugs and
    the hinge lug at its aft end."""
    parts = []
    xs = [3100.0 + (5700.0 - 3100.0) * i / 40 for i in range(41)]
    for s in (-1.0, 1.0):
        parts.append(mesh.pipe([_rim(x, s) for x in xs], 6.0, 12, subdiv=2))
        # the lock pins, each on a tab down through the opening's edge
        for x in LOCK_X:
            hw = opening_half_width(x)
            zs = shapes.z_up(x, s * hw)
            zr = _rim(x, s)[2]
            yt = s * (hw - 18.0)
            parts.append(mesh.box(x, s * (hw - 7.0), zr + 5.0, 20.0, 30.0, 8.0))
            parts.append(mesh.box(x, yt, 0.5 * (zr + 7.0 + zs - 20.0), 20.0, 8.0,
                                  zr + 7.0 - (zs - 20.0)))
            # the pin, lying in its hook's throat over the whole of its length
            parts.append(mesh.pipe([(x - 13.0, yt, zs - 18.0),
                                    (x + 13.0, yt, zs - 18.0)], 4.0, 10, bend=0.0))
        # the actuator's lug: a tab in from the frame and two cheeks down to
        # the rod end's pin
        L = act_lug(s)
        zr = _rim(ACT_X, s)[2]
        hw = opening_half_width(ACT_X)
        y_in = L[1] - s * 22.0
        parts.append(mesh.box(ACT_X, 0.5 * (s * (hw + 4.0) + y_in), zr + 5.0,
                              30.0, abs(s * (hw + 4.0) - y_in), 8.0))
        for dy in (-16.0, 16.0):
            parts.append(mesh.box(ACT_X, L[1] + dy, 0.5 * (zr + 5.0 + L[2] - 14.0),
                                  30.0, 6.0, zr + 5.0 - (L[2] - 14.0)))
    # the hinge lug, down from the glass's solid aft end to the hinge pin
    hx, _hy, hz = HINGE
    top = shapes.z_up(hx, 0.0) + SEAT + 3.0
    parts.append(mesh.box(hx, 0.0, 0.5 * (hz - 12.0 + top), 30.0, 18.0,
                          top - (hz - 12.0)))
    return mesh.join(*parts)


def canopy_hinge():
    """A beam across the opening's aft end between frame 5680's cut ends,
    two clevis plates up from it and the pin the canopy's lug turns on."""
    hx, _hy, hz = HINGE
    parts = [mesh.box(5680.0, 0.0, 892.5, 36.0, 420.0, 25.0)]
    for y in (-14.0, 14.0):
        parts.append(mesh.box(0.5 * (5690.0 + hx + 18.0), y, 0.5 * (900.0 + hz + 8.0),
                              hx + 18.0 - 5690.0, 8.0, hz + 8.0 - 900.0))
    parts.append(mesh.pipe([(hx, -26.0, hz), (hx, 26.0, hz)], 5.5, 16, bend=0.0))
    return mesh.join(*parts)


def canopy_hinge_pocket():
    """What the hinge takes out of the top of the tank behind it."""
    hx, _hy, hz = HINGE
    return mesh.box(0.5 * (5695.0 + hx + 35.0), 0.0, 950.0, hx + 35.0 - 5695.0,
                    48.0, 120.0)


def canopy_locks():
    """The hooks under the opening's edge the rim's pins drop into, each
    hung from the skin's inside."""
    parts = []
    for s in (-1.0, 1.0):
        for x in LOCK_X:
            hw = opening_half_width(x)
            zs = shapes.z_up(x, s * hw)
            parts.append(mesh.box(x, s * (hw - 18.0), zs - 26.5, 32.0, 16.0, 7.0))
            # up to the skin's inside, 1 mm into it where it is lowest
            top = min(shapes.z_up(x, s * yy, spec.SKIN_T)
                      for yy in (hw, hw + 7.0, hw + 14.0)) + 1.0
            parts.append(mesh.box(x, s * (hw + 2.0), 0.5 * (zs - 30.0 + top), 32.0,
                                  24.0, top - (zs - 30.0)))
    return mesh.join(*parts)


def canopy_actuator(s):
    """(body, rod): the body pinned on a bracket on the tub's aft wall, the
    rod out of it to the rim's lug."""
    A, L = act_anchor(s), act_lug(s)
    d = [L[i] - A[i] for i in range(3)]
    n = math.sqrt(sum(c * c for c in d))
    u = [c / n for c in d]
    at = lambda t: tuple(A[i] + u[i] * t for i in range(3))
    body = mesh.join(
        mesh.pipe([A, at(190.0)], 16.0, 20, bend=0.0),
        mesh.pipe([(A[0], A[1] - 14.0, A[2]), (A[0], A[1] + 14.0, A[2])], 10.0, 14,
                  bend=0.0),
        # the bracket, on the wall's face
        mesh.box(0.5 * (A[0] - 8.0 + 5641.0), A[1], A[2], 5641.0 - (A[0] - 8.0),
                 36.0, 44.0))
    rod = mesh.join(
        mesh.pipe([at(170.0), L], 8.0, 14, bend=0.0),
        mesh.pipe([(L[0], L[1] - 12.0, L[2]), (L[0], L[1] + 12.0, L[2])], 10.0, 14,
                  bend=0.0))
    return body, rod


def canopy_kinematics():
    """What the viewer needs to open the canopy: its hinge, the angle, the
    parts that swing, and each actuator's anchor and lug."""
    struts = []
    for s, t in ((1.0, "r"), (-1.0, "l")):
        struts.append({"body": f"canopy_actuator_{t}", "rod": f"canopy_actuator_rod_{t}",
                       "anchor": list(act_anchor(s)), "lug": list(act_lug(s))})
    return {"hinge": list(HINGE), "axis": [0.0, 1.0, 0.0], "open": CANOPY_OPEN,
            "parts": ["canopy_glass", "canopy_rim"], "struts": struts}


def tub():
    """The cockpit tub: floor, sides and bulkheads, under the canopy. The
    walls stand up to 1 mm under the skin's inside, following it."""
    x0, x1 = 3300.0, 5650.0
    y = CK["half_w"] + 40.0
    zf = 120.0
    wall = 10.0
    top = lambda x, yy: min(CK["sill_z"], shapes.z_up(x, yy, spec.SKIN_T) - 1.0)
    parts = [mesh.box(0.5 * (x0 + x1), 0.0, zf, x1 - x0, 2 * y, wall)]
    xs = [x0 + (x1 - x0) * i / 20 for i in range(21)]
    for sy in (1.0, -1.0):
        ya, yb = sy * (y - wall), sy * y
        rings = [[(x, ya, zf), (x, yb, zf), (x, yb, top(x, yb)), (x, ya, top(x, yb))]
                 for x in xs]
        parts.append(shapes.loft_rings(rings))
    ys = [-y + 2 * y * i / 20 for i in range(21)]
    for xa in (x0, x1 - wall):
        rings = [[(xa, yy, zf), (xa, yy, top(xa, yy)), (xa + wall, yy, top(xa, yy)),
                  (xa + wall, yy, zf)] for yy in ys]
        parts.append(shapes.loft_rings(rings))
    return mesh.join(*parts)


def seat():
    """A zero-zero ejection seat, reclined 18 degrees, on its rails."""
    xs = CK["seat_x"]
    zf = 125.0
    r = math.radians(18.0)
    parts = []
    # the pan, on its base down to the floor
    parts.append(mesh.box(xs - 80.0, 0.0, 0.5 * (zf - 7.0 + zf + 215.0), 520.0, 520.0,
                          222.0))
    # the back and headbox, leaning aft
    for (h0, h1, w) in ((215.0, 820.0, 500.0), (820.0, 960.0, 380.0)):
        hc = 0.5 * (h0 + h1)
        v, f = mesh.box(0.0, 0.0, 0.0, 110.0, w, h1 - h0)
        v = [(xs + 190.0 + hc * math.sin(r) + x * math.cos(r) + z * math.sin(r),
              y, zf + hc * math.cos(r) - x * math.sin(r) + z * math.cos(r))
             for (x, y, z) in v]
        parts.append((v, f))
    # the rails the seat rides on, down to the floor
    for sy in (1.0, -1.0):
        parts.append(mesh.pipe([(xs + 230.0, sy * 200.0, zf - 4.0),
                                (xs + 230.0 + 900.0 * math.sin(r), sy * 200.0,
                                 zf + 900.0 * math.cos(r))], 16.0, 10))
    return mesh.join(*parts)


def seat_bulkhead():
    """The bulkhead behind the seat, which the seat's rails are bolted to.

    The rails used to stand up from the floor to z 981 and end there: a
    zero-zero seat's guide rails, which take the whole ejection load, held
    at their feet and nowhere else. The bulkhead leans back with them at
    the seat's 18 degrees, just behind them: full width inside the tub's
    walls up to the sill, set into the walls and the floor, and narrower
    above, inside the canopy's opening, up past the rails' tops behind the
    headbox."""
    xs = CK["seat_x"]
    zf = 125.0
    r = math.radians(18.0)
    d = (math.sin(r), math.cos(r))          # up the rails, in (x, z)
    n = (math.cos(r), -math.sin(r))         # aft, off their faces
    x0, z0 = xs + 230.0, zf - 4.0           # the rails' feet
    off = 16.0 - 2.0 + 5.0                  # rails 2 mm into its face
    wall = 460.0 + 1.0                      # the tub's walls' inner faces
    tiers = ((-8.0, (CK["sill_z"] - 4.0 - z0) / d[1], wall),
             ((CK["sill_z"] - 40.0 - z0) / d[1], 960.0, 240.0))
    parts = []
    for (t0, t1, hy) in tiers:
        ring = []
        for t in (t0, t1):
            cx = x0 + d[0] * t + n[0] * off
            cz = z0 + d[1] * t + n[1] * off
            for s in (-1.0, 1.0):
                ring.append((cx + n[0] * 5.0 * s, cz + n[1] * 5.0 * s))
        # a slab: (bottom front, bottom back, top back, top front) in x-z,
        # extruded across y
        q = [ring[0], ring[1], ring[3], ring[2]]
        verts = [(x, -hy, z) for (x, z) in q] + [(x, hy, z) for (x, z) in q]
        faces = [(0, 1, 2, 3), (7, 6, 5, 4), (0, 4, 5, 1), (1, 5, 6, 2),
                 (2, 6, 7, 3), (3, 7, 4, 0)]
        parts.append(shapes.orient((verts, faces)))
    # its stiffeners, up its back, and the rails' bolts through it, heads
    # on its back face
    for yy in (-120.0, 120.0):
        a0, a1 = 10.0, 900.0
        p0 = (x0 + d[0] * a0 + n[0] * (off + 5.0 + 14.0),
              z0 + d[1] * a0 + n[1] * (off + 5.0 + 14.0))
        p1 = (x0 + d[0] * a1 + n[0] * (off + 5.0 + 14.0),
              z0 + d[1] * a1 + n[1] * (off + 5.0 + 14.0))
        parts.append(mesh.pipe([(p0[0], yy, p0[1]), (p1[0], yy, p1[1])],
                               15.0, 4, bend=0.0))
    for yy in (-200.0, 200.0):
        for t in (150.0, 380.0, 610.0, 840.0):
            bx = x0 + d[0] * t
            bz = z0 + d[1] * t
            parts.append(mesh.pipe(
                [(bx + n[0] * 8.0, yy, bz + n[1] * 8.0),
                 (bx + n[0] * (off + 9.0), yy, bz + n[1] * (off + 9.0))],
                7.0, 6, bend=0.0))
    return mesh.join(*parts)


def _arc_loft(x0, x1, y_half, z_lo, z_hi, bulge, n=16):
    """A panel curved across the cockpit: rings at stations across y, each a
    rectangle in x-z, the whole thing bowed forward by `bulge` at the
    centreline -- the way a glare shield or a panel wraps round the pilot."""
    rings = []
    for i in range(n + 1):
        y = -y_half + 2 * y_half * i / n
        dx = -bulge * (1.0 - (y / y_half) ** 2)
        rings.append([(y, x0 + dx, z_lo), (y, x1 + dx, z_lo),
                      (y, x1 + dx, z_hi), (y, x0 + dx, z_hi)])
    return shapes.loft_rings([[(x, y, z) for (y, x, z) in r] for r in rings])


def panel():
    """Instrument panel: one wide touch display in a bezel, curved round the
    pilot, under a glare shield that wraps the same way, on a pedestal."""
    x = 3480.0
    parts = [_arc_loft(x - 30.0, x + 30.0, 280.0, 330.0, 630.0, 40.0),   # bezel
             _arc_loft(x + 30.0, x + 34.0, 262.0, 348.0, 612.0, 40.0),   # screen
             _arc_loft(x - 120.0, x + 60.0, 280.0, 635.0, 675.0, 60.0)]  # glare
    # pedestal down to the floor
    parts.append(mesh.box(x + 10.0, 0.0, 225.5, 60.0, 180.0, 213.0))
    return mesh.join(*parts)


def hud():
    """The head-up display: a projector box on the glare shield and the
    combiner glass above it, tilted back towards the pilot's eye, in a
    frame."""
    t = math.radians(22.0)
    w, h = 200.0, 150.0
    parts = [mesh.box(3480.0, 0.0, 700.0, 120.0, 160.0, 52.0)]           # projector
    loop = [(0.5 * w * math.cos(2 * math.pi * k / 24),
             0.5 * h * math.sin(2 * math.pi * k / 24)) for k in range(24)]
    rings = [[(3520.0 + off + (v + 0.5 * h) * math.sin(t), u,
               723.0 + (v + 0.5 * h) * math.cos(t)) for (u, v) in loop]
             for off in (-3.0, 3.0)]
    parts.append(shapes.loft_rings(rings))                               # combiner
    return mesh.join(*parts)


def stick():
    """Side-stick on the right console, and the throttles on the left."""
    parts = [mesh.pipe([(4250.0, 380.0, 121.0), (4250.0, 380.0, 420.0),
                        (4230.0, 380.0, 520.0)], 22.0, 12),
             mesh.box(4250.0, 380.0, 380.0, 260.0, 90.0, 40.0),
             mesh.box(4150.0, -380.0, 380.0, 300.0, 90.0, 40.0),
             mesh.pipe([(4150.0, -380.0, 398.0), (4150.0, -380.0, 500.0)], 20.0, 12),
             mesh.pipe([(4250.0, 380.0, 121.0), (4250.0, 380.0, 360.0)], 14.0, 10),
             mesh.pipe([(4150.0, -380.0, 121.0), (4150.0, -380.0, 360.0)], 14.0, 10)]
    return mesh.join(*parts)


def build():
    return {
        "canopy_glass": canopy(),
        "canopy_frame_r": sill_rail(1.0),
        "canopy_frame_l": sill_rail(-1.0),
        "cut:fuselage_skin": opening_cutter(),
        "cockpit_tub": tub(),
        "seat": seat(),
        "seat_bulkhead": seat_bulkhead(),
        "cockpit_panel": panel(),
        "cockpit_hud": hud(),
        "cockpit_controls": stick(),
        "canopy_rim": canopy_rim(),
        "canopy_hinge": canopy_hinge(),
        "cut:fuel_tank_fwd_1": canopy_hinge_pocket(),
        "canopy_locks": canopy_locks(),
        "canopy_actuator_r": canopy_actuator(1.0)[0],
        "canopy_actuator_rod_r": canopy_actuator(1.0)[1],
        "canopy_actuator_l": canopy_actuator(-1.0)[0],
        "canopy_actuator_rod_l": canopy_actuator(-1.0)[1],
    }
