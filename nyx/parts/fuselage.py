"""The body's skin, the closure round the nozzles at the tail, and the
tail stinger between them.

The skin is one shell: the outer mould line from the nose tip to the tail,
and inside it the same shape moved in by the skin thickness from the radome
bulkhead aft, joined by a lip round the open tail. Everything that goes
through it -- the canopy opening, the intake ducts, the weapons-bay and
gear-bay openings -- is cut by the module that owns the thing going through,
so the hole always matches the part.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec      # noqa: E402
import shapes    # noqa: E402
import mesh      # noqa: E402

M = spec.RES["body_ring_pts"]


def build():
    out = {}
    out["fuselage_skin"] = skin()
    out.update(_aft_closure())
    out["tail_stinger"] = _stinger()
    out["aft_fairing"] = _aft_fairing()
    return out


def skin():
    x_end = spec.BODY_END_X
    xs_o = shapes.body_stations(1.0, x_end, spec.RES["body_rings"])
    xs_i = [x for x in xs_o if x >= spec.RADOME_X]
    if xs_i[0] > spec.RADOME_X:
        xs_i.insert(0, spec.RADOME_X)
    outer = [shapes.ring(x, M) for x in xs_o]
    inner = [shapes.ring(x, M, spec.SKIN_T) for x in xs_i]
    tip = (0.0, 0.0, shapes.section(0.0)[1])
    verts = [tip] + [p for r in outer for p in r] + [p for r in inner for p in r]
    no, ni = len(outer), len(inner)
    O = lambda i, k: 1 + i * M + (k % M)
    I = lambda i, k: 1 + no * M + i * M + (k % M)
    faces = [(0, O(0, k + 1), O(0, k)) for k in range(M)]
    for i in range(no - 1):
        for k in range(M):
            faces.append((O(i, k), O(i, k + 1), O(i + 1, k + 1), O(i + 1, k)))
    for i in range(ni - 1):
        for k in range(M):
            faces.append((I(i, k), I(i + 1, k), I(i + 1, k + 1), I(i, k + 1)))
    for k in range(M):                       # the lip round the open tail
        faces.append((O(no - 1, k), O(no - 1, k + 1), I(ni - 1, k + 1),
                      I(ni - 1, k)))
    faces.append(tuple(I(0, k) for k in range(M)))   # radome bulkhead face
    return shapes.orient((verts, faces))


# The closure: a plate across the open tail, just inside the skin's lip, with
# an opening for each nozzle. The nozzles pass through it clear on every side
# -- they grow and move with the engine -- and the gap is closed on the real
# aircraft by a flexible seal, which is not modelled.
CLOSURE_T = 20.0
NOZZLE_CLEAR = 12.0


def nozzle_box():
    """(half-width, half-height) of an engine's nozzle box where it passes
    the closure, from the vendored engine's own dimensions."""
    from parts import engines
    return engines.nozzle_box_half()


def _aft_closure():
    x1 = spec.BODY_END_X - 2.0
    x0 = x1 - CLOSURE_T
    # its rim sits a seat inside the skin's inner surface
    inset = spec.SKIN_T + 0.15
    r0 = shapes.ring(x0, M, inset)
    r1 = shapes.ring(x1, M, inset)
    plate = shapes.loft_rings([r0, r1])
    hw, hh = nozzle_box()
    cut = []
    for sy in (1.0, -1.0):
        cut.append(mesh.box(0.5 * (x0 + x1), sy * spec.ENGINE_Y, spec.ENGINE_Z,
                            CLOSURE_T + 40.0, 2 * (hw + NOZZLE_CLEAR),
                            2 * (hh + NOZZLE_CLEAR)))
    return {"aft_closure": plate, "cut:aft_closure": mesh.join(*cut)}


def _stinger():
    """A flat tapered fairing between the nozzles, off the closure's aft
    face: it carries the tail warning receivers and the drag chute, and it
    fills the gap between the two jets that would otherwise be base drag."""
    x0 = spec.BODY_END_X - 2.0
    L = 780.0
    rings = []
    for i in range(9):
        t = i / 8.0
        x = x0 + L * t
        a = 260.0 * (1 - 0.55 * t)            # half-width
        b = 150.0 * (1 - 0.75 * t)            # half-height
        rings.append([(x, a * math.copysign(abs(math.cos(u)) ** 0.5, math.cos(u)),
                       b * math.copysign(abs(math.sin(u)) ** 0.8, math.sin(u)))
                      for u in (2 * math.pi * k / 40 for k in range(40))])
    return shapes.loft_rings(rings)


# The boat-tail: a shell from the body's rim at the tail to a pair of
# nacelles closing round the nozzles, joined by a web over the stinger. The
# body used to end square at the closure, a flat oval 3.1 m across with the
# nozzles in two holes in it. The nozzle boxes' outer corners are what stop
# the body itself tapering further, so the taper is carried out here, past
# them, round the nozzles.
FAIRING_X1 = 14360.0
FAIRING_T = 10.0
FAIRING_CLEAR = 14.0


def _polar_reach(pts, cy, cz, a):
    """How far a ray from (cy, cz) at angle a runs before it leaves the
    closed polygon pts (y, z): the furthest crossing."""
    ca, sa = math.cos(a), math.sin(a)
    best = 0.0
    n = len(pts)
    for i in range(n):
        y0, z0 = pts[i]
        y1, z1 = pts[(i + 1) % n]
        ey, ez = y1 - y0, z1 - z0
        den = ca * ez - sa * ey
        if abs(den) < 1e-12:
            continue
        t = ((y0 - cy) * ez - (z0 - cz) * ey) / den
        u = ((y0 - cy) * sa - (z0 - cz) * ca) / den
        if t > 0.0 and -1e-9 <= u <= 1.0 + 1e-9:
            best = max(best, t)
    return best


def _nacelles(hw, hh, web, r_c=70.0):
    """Outline of the aft end: two rounded rectangles round the nozzle boxes
    and a web between them, as polygons for _polar_reach."""
    def rrect(cy, cz, a, b, r):
        pts = []
        for (sy, sz, a0) in ((1, 1, 0), (-1, 1, 90), (-1, -1, 180), (1, -1, 270)):
            ox, oz = cy + sy * (a - r), cz + sz * (b - r)
            for k in range(7):
                t = math.radians(a0 + 90.0 * k / 6)
                pts.append((ox + r * math.cos(t), oz + r * math.sin(t)))
        return pts
    ey = spec.ENGINE_Y
    return [rrect(ey, spec.ENGINE_Z, hw, hh, r_c),
            rrect(-ey, spec.ENGINE_Z, hw, hh, r_c),
            rrect(0.0, 0.0, ey, web, 40.0)]


def _aft_fairing(n=288):
    x0 = spec.BODY_END_X
    x1 = FAIRING_X1
    bw, bh = nozzle_box()
    hw, hh = bw + FAIRING_CLEAR + FAIRING_T, bh + FAIRING_CLEAR + FAIRING_T
    # over the stinger where the fairing ends
    t_st = (x1 - (spec.BODY_END_X - 2.0)) / 780.0
    web = 150.0 * (1 - 0.75 * t_st) + FAIRING_CLEAR + FAIRING_T
    body = [(p[1], p[2]) for p in shapes.ring(x0, 400)]
    body_in = [(p[1], p[2]) for p in shapes.ring(x0, 400, spec.SKIN_T)]
    aft = _nacelles(hw, hh, web)
    aft_in = _nacelles(hw - FAIRING_T, hh - FAIRING_T, web - FAIRING_T)
    angles = [2 * math.pi * (k + 0.5) / n for k in range(n)]

    def reach_all(polys, a):
        return max(_polar_reach(p, 0.0, 0.0, a) for p in polys)
    rA = [_polar_reach(body, 0.0, 0.0, a) for a in angles]
    rAi = [_polar_reach(body_in, 0.0, 0.0, a) for a in angles]
    rB = [reach_all(aft, a) for a in angles]
    rBi = [reach_all(aft_in, a) for a in angles]
    stations = 9
    outer, inner = [], []
    for j in range(stations):
        f = j / (stations - 1)
        s_ = f * f * (3 - 2 * f)                 # eased in and out
        x = x0 + (x1 - x0) * f
        outer.append([(x, (ra + (rb - ra) * s_) * math.cos(a),
                       (ra + (rb - ra) * s_) * math.sin(a))
                      for ra, rb, a in zip(rA, rB, angles)])
        inner.append([(x, (ra + (rb - ra) * s_) * math.cos(a),
                       (ra + (rb - ra) * s_) * math.sin(a))
                      for ra, rb, a in zip(rAi, rBi, angles)])
    verts = [p for r in outer for p in r] + [p for r in inner for p in r]
    S = stations
    O = lambda i, k: i * n + (k % n)
    I = lambda i, k: S * n + i * n + (k % n)
    faces = []
    for i in range(S - 1):
        for k in range(n):
            faces.append((O(i, k), O(i, k + 1), O(i + 1, k + 1), O(i + 1, k)))
            faces.append((I(i, k), I(i + 1, k), I(i + 1, k + 1), I(i, k + 1)))
    for k in range(n):
        faces.append((O(0, k), I(0, k), I(0, k + 1), O(0, k + 1)))
        faces.append((O(S - 1, k), O(S - 1, k + 1), I(S - 1, k + 1),
                      I(S - 1, k)))
    return shapes.orient((verts, faces))
