"""The body's outer mould line, and lofting helpers shared by the parts.
Pure Python, no bpy.

The body is defined by the section table in spec.BODY. Every other part that
has to sit on, in or through the body -- the wing root, the canopy, the
intake ducts, the bay doors, the frames -- asks this module where the skin
is, rather than keeping its own idea of it. That is what keeps a door flush
and a frame inside the skin when the body is reshaped.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import spec  # noqa: E402


# --------------------------------------------------------------------------
# monotone cubic interpolation (Fritsch-Carlson): fair lines, no overshoot
# --------------------------------------------------------------------------

def _pchip_slopes(xs, ys):
    n = len(xs)
    d = [(ys[i + 1] - ys[i]) / (xs[i + 1] - xs[i]) for i in range(n - 1)]
    m = [0.0] * n
    m[0], m[-1] = d[0], d[-1]
    for i in range(1, n - 1):
        if d[i - 1] * d[i] <= 0:
            m[i] = 0.0
        else:
            h0, h1 = xs[i] - xs[i - 1], xs[i + 1] - xs[i]
            w1, w2 = 2 * h1 + h0, h1 + 2 * h0
            m[i] = (w1 + w2) / (w1 / d[i - 1] + w2 / d[i])
    return m


class Pchip:
    def __init__(self, xs, ys):
        self.xs, self.ys = list(xs), list(ys)
        self.m = _pchip_slopes(self.xs, self.ys)

    def __call__(self, x):
        xs, ys, m = self.xs, self.ys, self.m
        if x <= xs[0]:
            return ys[0]
        if x >= xs[-1]:
            return ys[-1]
        i = max(k for k in range(len(xs) - 1) if xs[k] <= x)
        h = xs[i + 1] - xs[i]
        t = (x - xs[i]) / h
        h00 = 2 * t ** 3 - 3 * t ** 2 + 1
        h10 = t ** 3 - 2 * t ** 2 + t
        h01 = -2 * t ** 3 + 3 * t ** 2
        h11 = t ** 3 - t ** 2
        return h00 * ys[i] + h10 * h * m[i] + h01 * ys[i + 1] + h11 * h * m[i + 1]


_X = [r[0] for r in spec.BODY]
_W = Pchip(_X, [r[1] for r in spec.BODY])
_ZC = Pchip(_X, [r[2] for r in spec.BODY])
_ZT = Pchip(_X, [r[3] for r in spec.BODY])
_ZB = Pchip(_X, [r[4] for r in spec.BODY])


def section(x):
    """(half-width, chine z, crown z, keel z) of the outer mould line at x."""
    return _W(x), _ZC(x), _ZT(x), _ZB(x)


def _z_up_smooth(x, y, inset=0.0):
    w, zc, zt, zb = section(x)
    w -= inset * _edge_ratio()
    zt -= inset
    if w <= 0 or abs(y) >= w:
        return zc
    f = (1.0 - (abs(y) / w) ** spec.M_UP) ** (1.0 / spec.N_UP)
    return zc + (zt - zc) * f


def _z_dn_smooth(x, y, inset=0.0):
    w, zc, zt, zb = section(x)
    w -= inset * _edge_ratio()
    zb += inset
    if w <= 0 or abs(y) >= w:
        return zc
    f = (1.0 - (abs(y) / w) ** spec.M_DN) ** (1.0 / spec.N_DN)
    return zc - (zc - zb) * f


# The aft body is two nacelles and a valley, not a slab.
#
# Forward of NAC_X0 the section is the smooth superellipse above. Aft of it
# the section blends, over NAC_X0..NAC_X1, into two nacelles round the
# engines -- an ellipse over and under each engine's axis -- joined to a
# lowered body between and outside them by a smooth maximum, so the crown
# drops into a valley down the centreline and each engine stands proud in
# its own shoulder, the way the aft end of a real twin-engine fighter is
# shaped. The body used to carry its full crown and keel to the tail: a
# flat oval slab 4.5 m across, which with the faceting read as a carved box.
NAC_X0 = 9100.0
NAC_X1 = 11900.0
NAC_HW = 760.0        # nacelle half-width about the engine axis
NAC_UP = 650.0        # its height over the axis
NAC_DN = 800.0        # and depth under it: the gearbox is under the engine
NAC_LOW = 0.52        # the body between the nacelles, as a fraction of crown
NAC_K = 160.0         # the smooth-max blend width, mm


def _smax(a, b, k=NAC_K):
    h = max(k - abs(a - b), 0.0) / k
    return max(a, b) + h * h * k * 0.25


def _nac_weight(x):
    t = min(max((x - NAC_X0) / (NAC_X1 - NAC_X0), 0.0), 1.0)
    return t * t * (3.0 - 2.0 * t)


def _hump(y, h, inset):
    d = abs(abs(y) - spec.ENGINE_Y)
    a = NAC_HW - inset
    if d >= a:
        return -1e9
    return spec.ENGINE_Z + (h - inset) * math.sqrt(1.0 - (d / a) ** 2)


def z_up(x, y, inset=0.0):
    """Upper surface height at (x, y); with `inset` the surface moved in
    by that much (crown down, half-width in)."""
    base = _z_up_smooth(x, y, inset)
    a = _nac_weight(x)
    if a <= 0.0:
        return base
    zc = section(x)[1]
    low = zc + (base - zc) * NAC_LOW
    aft = _smax(low, _hump(y, NAC_UP, inset))
    return base + (aft - base) * a


def z_dn(x, y, inset=0.0):
    base = _z_dn_smooth(x, y, inset)
    a = _nac_weight(x)
    if a <= 0.0:
        return base
    zc = section(x)[1]
    low = zc + (base - zc) * NAC_LOW
    aft = -_smax(-low, _hump(y, NAC_DN, inset))
    return base + (aft - base) * a


def _edge_ratio():
    """How much a surface inset of t moves the chine edge in: the chine is a
    knife edge, and a skin of thickness t inside it is solid for
    SKIN_EDGE / SKIN_T times that."""
    return spec.SKIN_EDGE / spec.SKIN_T


def half_width(x, inset=0.0):
    return section(x)[0] - inset * _edge_ratio()


def ring(x, m, inset=0.0):
    """m points round the section at x, from the starboard chine over the
    top to the port chine and back underneath. Points are cosine-spaced in
    y so the chine edges, where the surface turns fastest, get the most."""
    w = half_width(x, inset)
    h = m // 2
    ys = [w * math.cos(math.pi * k / h) for k in range(h + 1)]
    pts = [(x, y, z_up(x, y, inset)) for y in ys]
    pts += [(x, -ys[k], z_dn(x, -ys[k], inset)) for k in range(1, h)]
    return pts


def body_stations(x0, x1, n):
    """Stations from x0 to x1, packed toward the nose where the section
    changes fastest."""
    out = []
    for i in range(n + 1):
        t = i / n
        out.append(x0 + (x1 - x0) * (t ** 1.35))
    return out


# --------------------------------------------------------------------------
# closed lofts and shells
# --------------------------------------------------------------------------

def loft_rings(rings, cap_start=True, cap_end=True):
    """Join equal-length rings into a closed solid (caps as n-gons, which is
    right for the convex sections everything here has)."""
    m = len(rings[0])
    verts = [p for r in rings for p in r]
    faces = []
    for i in range(len(rings) - 1):
        a, b = i * m, (i + 1) * m
        for k in range(m):
            k2 = (k + 1) % m
            faces.append((a + k, a + k2, b + k2, b + k))
    if cap_start:
        faces.append(tuple(range(m - 1, -1, -1)))
    if cap_end:
        base = (len(rings) - 1) * m
        faces.append(tuple(range(base, base + m)))
    return orient((verts, faces))


def loft_tip(tip, rings, cap_end=True):
    """A closed solid from a single tip point through the rings."""
    m = len(rings[0])
    verts = [tip] + [p for r in rings for p in r]
    faces = [(0, 1 + (k + 1) % m, 1 + k) for k in range(m)]
    for i in range(len(rings) - 1):
        a, b = 1 + i * m, 1 + (i + 1) * m
        for k in range(m):
            k2 = (k + 1) % m
            faces.append((a + k, a + k2, b + k2, b + k))
    if cap_end:
        base = 1 + (len(rings) - 1) * m
        faces.append(tuple(range(base, base + m)))
    return orient((verts, faces))


def orient(part):
    """Flip a closed mesh whose signed volume is negative, so every part
    faces outward whatever order its faces were written in."""
    verts, faces = part
    vol = 0.0
    for f in faces:
        a = verts[f[0]]
        for k in range(1, len(f) - 1):
            b, c = verts[f[k]], verts[f[k + 1]]
            vol += (a[0] * (b[1] * c[2] - b[2] * c[1])
                    - a[1] * (b[0] * c[2] - b[2] * c[0])
                    + a[2] * (b[0] * c[1] - b[1] * c[0]))
    if vol < 0:
        faces = [tuple(reversed(f)) for f in faces]
    return verts, faces


def outer_solid(x0=0.0, x1=None, n=None, m=None):
    """The body's outer mould line as one closed solid -- for the area
    ruling and for anything that needs to ask 'is this inside the body'."""
    x1 = spec.BODY_END_X if x1 is None else x1
    n = n or spec.RES["body_rings"]
    m = m or spec.RES["body_ring_pts"]
    xs = body_stations(max(x0, 1.0), x1, n)
    rings = [ring(x, m) for x in xs]
    if x0 <= 0.0:
        return loft_tip((0.0, 0.0, section(0.0)[1]), rings)
    return loft_rings(rings)


# --------------------------------------------------------------------------
# aerofoil sections for the lifting surfaces
# --------------------------------------------------------------------------

def naca_t(u, tc):
    u = max(0.0, min(1.0, u))
    return 5.0 * tc * (0.2969 * math.sqrt(u) - 0.1260 * u - 0.3516 * u * u
                       + 0.2843 * u ** 3 - 0.1036 * u ** 4)


def section_loop(u0, u1, tc, camber=0.0, n=40):
    """A closed aerofoil loop in chord units, (u, v), over chord fractions
    u0..u1: upper surface forward to aft, lower surface aft to forward. A
    cut end (u0 > 0 or u1 < 1) is a straight face; an end at the leading or
    trailing edge closes to a point."""
    us = [u0 + (u1 - u0) * 0.5 * (1 - math.cos(math.pi * i / n)) for i in range(n + 1)]

    def up(u):
        return 4 * camber * u * (1 - u) + naca_t(u, tc)

    def lo(u):
        return 4 * camber * u * (1 - u) - naca_t(u, tc)

    upper = [(u, up(u)) for u in us]
    lower = [(u, lo(u)) for u in reversed(us)]
    loop = upper[:]
    # drop a lower point that coincides with the upper one (a closed edge)
    for i, p in enumerate(lower):
        q_first = (i == 0 and abs(p[1] - upper[-1][1]) < 1e-9)
        q_last = (i == len(lower) - 1 and abs(p[1] - upper[0][1]) < 1e-9)
        if q_first or q_last:
            continue
        loop.append(p)
    return loop
