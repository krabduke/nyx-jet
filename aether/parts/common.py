"""Helpers shared by the part modules: walls off the stream tables, blade
rows, rims and bands. Pure Python, no bpy."""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec      # noqa: E402
import mesh      # noqa: E402
import blades    # noqa: E402

SEG = spec.RES["revolve_segments"]


def knots(x0, x1, *tables, step=40.0):
    """Stations to sample a wall at between x0 and x1: both ends, every
    breakpoint of the given stream tables inside the range, and a station
    every `step` mm so a curved wall stays curved."""
    xs = {x0, x1}
    for t in tables:
        xs.update(p[0] for p in t if x0 < p[0] < x1)
    n = max(1, int((x1 - x0) // step))
    xs.update(x0 + (x1 - x0) * k / n for k in range(1, n))
    return sorted(xs)


def ring(x0, x1, r_in, r_out, tables=(), seg=SEG, step=40.0):
    """A solid of revolution between two radius functions of x.

    r_in and r_out are callables (or constants). The meridional loop runs
    forward along the bore and back along the outside, and revolve_ring fixes
    its winding, so the solid comes out closed and facing outward.
    """
    fi = r_in if callable(r_in) else (lambda x, v=r_in: v)
    fo = r_out if callable(r_out) else (lambda x, v=r_out: v)
    xs = knots(x0, x1, *tables, step=step)
    loop = [(x, fi(x)) for x in xs] + [(x, fo(x)) for x in reversed(xs)]
    return mesh.revolve_ring(loop, seg)


def hub(path, x):
    return spec.annulus(spec.PATHS[path], x)[0]


def tip(path, x):
    return spec.annulus(spec.PATHS[path], x)[1]


def neighbours(row):
    """The rows immediately ahead of and behind this one, in its own stream
    or the one it hands over to."""
    rows = sorted(spec.all_rows(), key=lambda r: r.x)
    i = rows.index(row)
    return (rows[i - 1] if i else None, rows[i + 1] if i + 1 < len(rows) else None)


def reach(row, frac=0.15, lo=None, hi=None):
    """Axial extent (x0, x1) of a row's rim or band.

    It reaches `frac` of a chord past the aerofoil each way, but never more
    than halfway to the next row less 1.5 mm, so a turning rim and a fixed
    band either side of a gap always keep 3 mm between them. `lo` / `hi`
    override an end where the row meets a wall rather than another row.
    """
    prev, nxt = neighbours(row)
    x_le = row.x
    x_te = row.x_te
    ahead = behind = frac * row.chord
    if prev is not None:
        ahead = min(ahead, (x_le - prev.x_te) / 2.0 - 1.5)
    if nxt is not None:
        behind = min(behind, (nxt.x - x_te) / 2.0 - 1.5)
    return (lo if lo is not None else x_le - ahead,
            hi if hi is not None else x_te + behind)


# --------------------------------------------------------------------------
# blade rows
# --------------------------------------------------------------------------

def rotor_blades(row):
    """A rotor row: every blade, from ROOT_EMBED inside the rim to the tip
    clearance under the casing."""
    clr = spec.TIP_CLEARANCE[row.path if row.path in spec.TIP_CLEARANCE else "core"]
    one = blades.loft(
        row,
        lambda x: hub(row.path, x) - spec.ROOT_EMBED,
        lambda x: tip(row.path, x) - clr,
        spec.RES["airfoil_chord_pts"], spec.RES["airfoil_span_pts"])
    return mesh.replicate(*one, row.count)


def stator_vanes(row, band=True, top_embed=None):
    """A stator row: every vane, sunk ROOT_EMBED into an inner band at the
    root and into the casing at the tip, plus the inner band itself.

    The band is a full ring BAND_DEPTH deep under the hub line: the rotor
    drum passes under it with the seal gap, which is how an inner shroud
    works."""
    emb = spec.ROOT_EMBED if top_embed is None else top_embed
    one = blades.loft(
        row,
        lambda x: hub(row.path, x) - spec.ROOT_EMBED,
        lambda x: tip(row.path, x) + emb,
        spec.RES["airfoil_chord_pts"], spec.RES["airfoil_span_pts"])
    parts = [mesh.replicate(*one, row.count)]
    if band:
        x0, x1 = reach(row)
        parts.append(ring(x0, x1,
                          lambda x: hub(row.path, x) - spec.BAND_DEPTH,
                          lambda x: hub(row.path, x),
                          (spec.PATHS[row.path],), step=10.0))
    return mesh.join(*parts)


def rim_profile(x0, x1, path, depth, web_x, web_t, bore_r, bore_w,
                bore_h=14.0):
    """Meridional loop of a disc: the rim's top is the hub line from x0 to
    x1, `depth` deep; a web of thickness web_t centred on web_x runs down to
    a bore ring bore_w wide and bore_h deep with its inside at bore_r."""
    xs = knots(x0, x1, spec.PATHS[path], step=8.0)
    top = [(x, hub(path, x)) for x in xs]
    r_in = min(r for _, r in top) - depth
    wa, wb = web_x - web_t / 2.0, web_x + web_t / 2.0
    ba, bb = web_x - bore_w / 2.0, web_x + bore_w / 2.0
    loop = top + [(x1, r_in), (wb, r_in), (wb, bore_r + bore_h),
                  (bb, bore_r + bore_h), (bb, bore_r), (ba, bore_r),
                  (ba, bore_r + bore_h), (wa, bore_r + bore_h), (wa, r_in),
                  (x0, r_in)]
    return loop


def revolve(loop, seg=SEG):
    return mesh.revolve_ring(loop, seg)


def sector_block(x0, x1, r0, r1, a0, a1, n=8):
    """A block that is a sector of an annulus: x0..x1, r0..r1, angles a0..a1
    (radians). Closed."""
    prof = [(x0, r0), (x1, r0), (x1, r1), (x0, r1)]
    verts, faces = [], []
    for k in range(n + 1):
        a = a0 + (a1 - a0) * k / n
        ca, sa = math.cos(a), math.sin(a)
        for (x, r) in prof:
            verts.append((x, r * ca, r * sa))
    for k in range(n):
        b0, b1 = k * 4, (k + 1) * 4
        for i in range(4):
            i2 = (i + 1) % 4
            faces.append((b0 + i, b0 + i2, b1 + i2, b1 + i))
    faces.append((3, 2, 1, 0))
    b = n * 4
    faces.append((b, b + 1, b + 2, b + 3))
    # a sector swept anticlockwise with this loop comes out inside-out;
    # flip if the first face's normal points toward the axis
    return _orient_out(verts, faces)


def _orient_out(verts, faces):
    """Flip a closed mesh whose signed volume is negative."""
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


def orient(part):
    return _orient_out(*part)


def airfoil_strut(x0, chord, t, r0, r1, clock_deg, n=24):
    """A symmetric-section strut standing radially from r0 to r1 at a clock
    angle, chord along x from x0. Closed."""
    sect = blades.section(n, t / chord, 0.0)
    verts = []
    a = math.radians(clock_deg)
    ca, sa = math.cos(a), math.sin(a)
    for r in (r0, r1):
        for (u, v) in sect:
            x = x0 + u * chord
            tt = v * chord
            # tangential offset perpendicular to the radial line
            verts.append((x, r * ca - tt * sa, r * sa + tt * ca))
    ns = len(sect)
    faces = [(i, (i + 1) % ns, ns + (i + 1) % ns, ns + i) for i in range(ns)]
    faces.append(tuple(range(ns - 1, -1, -1)))
    faces.append(tuple(range(ns, 2 * ns)))
    return _orient_out(verts, faces)


def radial_pin(x, r0, r1, rad, clock_deg, seg=12):
    """A cylinder standing radially at a clock angle."""
    a = math.radians(clock_deg)
    d = (0.0, math.cos(a), math.sin(a))
    p0 = (x, d[1] * r0, d[2] * r0)
    p1 = (x, d[1] * r1, d[2] * r1)
    return mesh.pipe([p0, p1], rad, seg)


def polar(x, r, clock_deg):
    a = math.radians(clock_deg)
    return (x, r * math.cos(a), r * math.sin(a))


def strip(x0, x1, r_fn, h, w, clock_deg, tables=(), embed=1.0, step=40.0):
    """A rectangular-section rib lying along x on a surface of revolution:
    from `embed` inside r_fn(x) to h above it, w wide, at a clock angle.
    Closed, with square ends."""
    xs = knots(x0, x1, *tables, step=step)
    a = math.radians(clock_deg)
    ca, sa = math.cos(a), math.sin(a)
    verts = []
    for x in xs:
        r0, r1 = r_fn(x) - embed, r_fn(x) + h
        for (r, t) in ((r0, -w / 2), (r1, -w / 2), (r1, w / 2), (r0, w / 2)):
            verts.append((x, r * ca - t * sa, r * sa + t * ca))
    n = len(xs)
    faces = []
    for i in range(n - 1):
        b0, b1 = 4 * i, 4 * (i + 1)
        for k in range(4):
            k2 = (k + 1) % 4
            faces.append((b0 + k, b1 + k, b1 + k2, b0 + k2))
    faces.append((0, 1, 2, 3))
    b = 4 * (n - 1)
    faces.append((b + 3, b + 2, b + 1, b))
    return _orient_out(verts, faces)
