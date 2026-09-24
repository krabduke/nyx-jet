"""
Aerofoil sections and blade lofting. Pure Python, no bpy.

A blade here is lofted between two surfaces of revolution -- the one it roots
in and the one its tip runs against -- rather than between two radii. Every
point of every section finds its radius from the stream table at its OWN
axial position, so a blade in a converging annulus follows the hub up and
the casing down along its chord, and its root and tip stay the same distance
from the walls from leading edge to trailing edge. A blade lofted between
two fixed radii either digs into a rising hub at its trailing edge or stands
off it at its leading edge; in an HP compressor whose hub climbs 4 mm across
one chord that is the difference between a blade and a gap.

Sections are NACA four-digit thickness on a parabolic camber line, which is
not what a transonic fan or a turbine actually uses (those are custom
sections), but is the right family for the shape to read correctly.
"""

import math

STACK = 0.42     # sections stack about 42 % chord -- near the centroid


def naca_half_thickness(xc, tc):
    """Half-thickness at chord fraction xc. The last coefficient is -0.1036
    rather than -0.1015 so the trailing edge closes to a point."""
    xc = max(0.0, xc)
    return 5.0 * tc * (0.2969 * math.sqrt(xc) - 0.1260 * xc
                       - 0.3516 * xc ** 2 + 0.2843 * xc ** 3
                       - 0.1036 * xc ** 4)


def section(n_pts, tc, mc, nose=1.0):
    """Closed section, (u, v) in chord units, TE -> suction side -> LE ->
    pressure side. Cosine spacing packs points at the two edges, where the
    curvature is. `nose` > 1 fattens the leading edge -- turbine vanes carry
    a big blunt nose that a NACA section does not."""
    half = n_pts // 2 + 1
    xs = [0.5 * (1.0 - math.cos(math.pi * i / (half - 1))) for i in range(half)]
    up, lo = [], []
    for xc in xs:
        yt = naca_half_thickness(xc, tc)
        if nose != 1.0 and xc < 0.08:
            yt *= 1.0 + (nose - 1.0) * (1.0 - xc / 0.08)
        yc = 4.0 * mc * xc * (1.0 - xc)
        th = math.atan(4.0 * mc * (1.0 - 2.0 * xc))
        up.append((xc - yt * math.sin(th), yc + yt * math.cos(th)))
        lo.append((xc + yt * math.sin(th), yc - yt * math.cos(th)))
    return list(reversed(up)) + lo[1:-1]


def loft(row, r_root, r_top, n_chord, n_span, s_pts=None):
    """Loft one blade of `row` between two radius functions of x.

    r_root(x) and r_top(x) give the radius the blade starts and ends at, at
    axial station x. Returns (verts, faces), closed, the blade at clock 0.
    """
    nose = 1.8 if row.thickness >= 0.12 else 1.0
    sect = section(n_chord, row.thickness, row.camber, nose)
    ns = len(sect)
    s_pts = s_pts or [j / (n_span - 1) for j in range(n_span)]
    verts = []
    for s in s_pts:
        sc = min(max(s, 0.0), 1.0)
        g = math.radians(row.stagger_hub + (row.stagger_tip - row.stagger_hub) * sc)
        cg, sg = math.cos(g), math.sin(g)
        x_le, x_te = row.span_x(sc)
        c_ax = x_te - x_le
        tc = c_ax / max(abs(cg), 0.3)
        lean = math.radians(row.lean) * sc
        for (u, v) in sect:
            du, dv = (u - STACK) * tc, v * tc
            dx = du * cg - dv * sg
            dt = du * sg + dv * cg
            x = x_le + STACK * c_ax + dx
            r0, r1 = r_root(x), r_top(x)
            r = r0 + (r1 - r0) * s
            phi = dt / r + lean
            verts.append((x, r * math.cos(phi), r * math.sin(phi)))
    faces = []
    for j in range(len(s_pts) - 1):
        a, b = j * ns, (j + 1) * ns
        for i in range(ns):
            i2 = (i + 1) % ns
            faces.append((a + i, a + i2, b + i2, b + i))
    faces.append(tuple(range(ns - 1, -1, -1)))
    base = (len(s_pts) - 1) * ns
    faces.append(tuple(range(base, base + ns)))
    return verts, faces


def surface_point(row, r_root, r_top, u, s, side=+1, n=60):
    """A point on the blade surface at chord fraction u, span fraction s, on
    the pressure (+1) or suction (-1) side, and the outward surface normal
    there. Used to place film-cooling holes where the blade actually is."""
    nose = 1.8 if row.thickness >= 0.12 else 1.0
    sect = section(n, row.thickness, row.camber, nose)
    half = len(sect) // 2
    rng = range(half, len(sect)) if side > 0 else range(0, half)
    i = min(rng, key=lambda k: abs(sect[k][0] - u))
    (pu, pv) = sect[i]
    (au, av), (bu, bv) = sect[i - 1], sect[(i + 1) % len(sect)]
    tu, tv = bu - au, bv - av
    ln = math.hypot(tu, tv) or 1.0
    # the loop runs TE -> suction -> LE -> pressure, anticlockwise in (u, v)
    # when camber is up, so the outward normal is the tangent turned right
    nu, nv = tv / ln, -tu / ln

    g = math.radians(row.stagger_hub + (row.stagger_tip - row.stagger_hub) * s)
    cg, sg = math.cos(g), math.sin(g)
    x_le, x_te = row.span_x(s)
    c_ax = x_te - x_le
    tc = c_ax / max(abs(cg), 0.3)
    du, dv = (pu - STACK) * tc, pv * tc
    x = x_le + STACK * c_ax + du * cg - dv * sg
    t = du * sg + dv * cg
    r = r_root(x) + (r_top(x) - r_root(x)) * s
    phi = t / r + math.radians(row.lean) * s
    p = (x, r * math.cos(phi), r * math.sin(phi))
    ndx, ndt = nu * cg - nv * sg, nu * sg + nv * cg
    d = (ndx, -math.sin(phi) * ndt, math.cos(phi) * ndt)
    dl = math.sqrt(sum(c * c for c in d)) or 1.0
    return p, tuple(c / dl for c in d)
