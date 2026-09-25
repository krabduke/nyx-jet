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
        "cockpit_panel": panel(),
        "cockpit_hud": hud(),
        "cockpit_controls": stick(),
    }
