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
