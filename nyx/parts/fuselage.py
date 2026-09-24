"""The body's skin.

The skin is one shell: the outer mould line from the nose tip to the tail,
and inside it the same shape moved in by the skin thickness from the radome
bulkhead aft, joined by a lip round the open tail. The tail is two nacelles
(shapes.py), and each ends just ahead of its engine's swivel bearing, with the
swivel duct and the nozzle out behind it in the air: everything aft of that
bearing turns when the jet is vectored, so nothing of the airframe can be. Everything that goes
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
