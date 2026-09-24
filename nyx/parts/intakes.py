"""Caret intakes and serpentine ducts.

Each mouth is a rounded rectangle tucked under the chine beside the
cockpit, with its lip swept in plan and in side view -- the caret, which
makes an oblique shock at supersonic speed that compresses the air before
the mouth and spills the boundary layer without a separate diverter. From
there the duct runs 5.8 m aft, turning inward 450 mm and upward 420 mm onto
the engine's axis while its section goes from the mouth's rectangle to the
fan's circle. The offset is more than the fan's radius, so from ahead there
is no straight line of sight into the duct that reaches the fan: the fan
face, the brightest radar reflector on a fighter, is hidden.

Where the duct runs outside the body it is the intake trunk you see under
the chine; the skin is cut to the duct's shape (plus 0.2 mm), so the skin
and the duct meet edge to face without either passing through the other.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec      # noqa: E402
import shapes    # noqa: E402
from parts import engines   # noqa: E402

I = spec.INTAKE
NP = 96


def smooth(t):
    t = min(max(t, 0.0), 1.0)
    return t * t * (3 - 2 * t)


def centre(x):
    """Duct centreline (y, z) at station x, for the starboard duct."""
    t = (x - I["x_mouth"]) / (I["x_end"] - I["x_mouth"])
    s = smooth(t)
    return (I["y_mouth"] + (spec.ENGINE_Y - I["y_mouth"]) * s,
            I["z_mouth"] + (spec.ENGINE_Z - I["z_mouth"]) * s)


def _shape(x, grow):
    """Points round the duct's air-side section at x, grown outward by
    `grow`: a superellipse at the mouth blended point by point into the
    engine's circle at the end."""
    t = (x - I["x_mouth"]) / (I["x_end"] - I["x_mouth"])
    e = smooth(min(1.0, max(0.0, t) * 1.15))
    R = engines.info()["inlet_bore"] + grow
    a, b = I["w_mouth"] / 2 + grow, I["h_mouth"] / 2 + grow
    yc, zc = centre(x)
    poly = _mouth_polygon(a, b)
    pts = []
    for k in range(NP):
        th = 2 * math.pi * k / NP
        c, s = math.cos(th), math.sin(th)
        # the corners eased over a few degrees: sharp, the samples cut
        # across them differently for the duct and for its hole in the skin
        r = sum(_reach(poly, math.cos(th + d), math.sin(th + d))
                for d in _EASE) / len(_EASE)
        pts.append((yc + (1 - e) * r * c + e * R * c,
                    zc + (1 - e) * r * s + e * R * s))
    return pts


_EASE = [math.radians(v) for v in (-4.0, -2.0, 0.0, 2.0, 4.0)]


def _mouth_polygon(a, b):
    """The mouth, about its centre (y outboard, z up): a trapezoid, full
    width under the chine and cut back underneath on the outboard side --
    straight edges and hard corners, which is what a caret is. It was a
    rounded superellipse, a soft scoop on a faceted airframe."""
    k = I["mouth_taper"]
    c = I["mouth_chamfer"]
    trap = [(-a, b), (a, b), (a - 2 * a * k, -b), (-a, -b)]
    # every corner cut back along both its edges: square, they stood out
    # through the skin under the chine and the belly as the duct turned in
    out = []
    n = len(trap)
    for i in range(n):
        p0, p1, p2 = trap[i - 1], trap[i], trap[(i + 1) % n]
        out.append((p1[0] + (p0[0] - p1[0]) * c, p1[1] + (p0[1] - p1[1]) * c))
        out.append((p1[0] + (p2[0] - p1[0]) * c, p1[1] + (p2[1] - p1[1]) * c))
    return out


def _reach(poly, c, s):
    """Distance from the centre to the polygon's edge along (c, s)."""
    best = 0.0
    n = len(poly)
    for i in range(n):
        y0, z0 = poly[i]
        y1, z1 = poly[(i + 1) % n]
        ey, ez = y1 - y0, z1 - z0
        den = c * ez - s * ey
        if abs(den) < 1e-12:
            continue
        t = (y0 * ez - z0 * ey) / den
        u = (y0 * s - z0 * c) / den
        if t > 0.0 and -1e-9 <= u <= 1.0 + 1e-9:
            best = max(best, t)
    return best


def lip_x(y, z):
    """The caret: the lip is swept back outboard and toward the bottom, so
    the mouth's inner top corner, under the chine, leads."""
    k = math.tan(math.radians(I["lip_sweep"]))
    return I["x_mouth"] + 0.55 * k * (y - I["y_mouth"]) + 0.45 * k * (I["z_mouth"] - z)


def lip_range():
    """(x_min, x_max) of the swept lip."""
    _, xs = _stations()
    xl = [lip_x(y, z) for (y, z) in _shape(xs[0], I["wall"])]
    return min(xl), max(xl)


def _stations():
    x_lead = I["x_mouth"] - 0.6 * I["h_mouth"]
    x_back = I["x_mouth"] + 0.6 * I["h_mouth"]
    xs = [x_back + (I["x_end"] - x_back) * (i / 40) ** 1.1 for i in range(41)]
    return x_lead, xs


def duct(grow_in=0.0, grow_out=None, end_trim=0.15):
    """The duct as a closed shell: air-side surface grown by grow_in, outer
    surface by grow_out (default: the wall), lip faces at both ends. With
    grow_out None it is the part; the skin cutter is the outer surface alone
    as a solid."""
    wall = I["wall"] if grow_out is None else grow_out
    _, xs = _stations()
    xs[-1] -= end_trim
    inner_first = [(lip_x(y, z), y, z) for (y, z) in _shape(xs[0], grow_in)]
    outer_first = [(lip_x(y, z), y, z) for (y, z) in _shape(xs[0], wall)]
    inner = [inner_first] + [[(x, y, z) for (y, z) in _shape(x, grow_in)] for x in xs[1:]]
    outer = [outer_first] + [[(x, y, z) for (y, z) in _shape(x, wall)] for x in xs[1:]]
    n = len(xs)
    verts = [p for r in outer for p in r] + [p for r in inner for p in r]
    O = lambda i, k: i * NP + k % NP
    Ii = lambda i, k: n * NP + i * NP + k % NP
    faces = []
    for i in range(n - 1):
        for k in range(NP):
            faces.append((O(i, k), O(i, k + 1), O(i + 1, k + 1), O(i + 1, k)))
            faces.append((Ii(i, k), Ii(i + 1, k), Ii(i + 1, k + 1), Ii(i, k + 1)))
    for k in range(NP):
        faces.append((O(0, k + 1), O(0, k), Ii(0, k), Ii(0, k + 1)))
        faces.append((O(n - 1, k), O(n - 1, k + 1), Ii(n - 1, k + 1), Ii(n - 1, k)))
    return shapes.orient((verts, faces))


def cutter():
    """The duct's outside, 0.2 mm proud, as a solid: what comes out of the
    skin where the duct passes through it."""
    _, xs = _stations()
    rings = [[(lip_x(y, z) - 30.0, y, z) for (y, z) in _shape(xs[0], I["wall"] + 0.2)]]
    rings += [[(x, y, z) for (y, z) in _shape(x, I["wall"] + 0.2)] for x in xs[1:-1]]
    return shapes.loft_rings(rings)


def _mirror(part):
    v, f = part
    return shapes.orient(([(x, -y, z) for (x, y, z) in v],
                          [tuple(reversed(c)) for c in f]))


def build():
    d = duct()
    c = cutter()
    import mesh
    return {"intake_duct_r": d, "intake_duct_l": _mirror(d),
            "cut:fuselage_skin": mesh.join(c, _mirror(c))}
