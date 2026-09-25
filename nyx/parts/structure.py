"""Primary structure the engines and the tail hang on, and the radar.

    keel          a vertical beam on the centreline between the engines,
                  from the bay's aft end to the tail
    frames        ring frames riveted inside the skin, each an upper and a
                  lower arch: seven forward (the forward fuselage joint,
                  either end of the cockpit, the canards' pivot, three along
                  the weapons bay), each relieved round what passes through
                  it; the forward engine frame at the engines' trunnions,
                  which the fins' front spars also pick up; and the aft
                  frame at the engines' thrust links
    engine mounts each engine hangs on two trunnions -- the inboard one on a
                  beam from the keel, the outboard one on a strut up to the
                  forward frame -- and is steadied at the back by a link from
                  its thrust lug up to the aft frame
    radar         an AESA array on a bulkhead behind the radome

The fuel tanks are in parts/fuel.py.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec      # noqa: E402
import shapes    # noqa: E402
import mesh      # noqa: E402
from parts import engines   # noqa: E402

M = 96
SEAT = 0.15
FRAME_DEPTH = 70.0
FRAME_T = 40.0


def loft_cyclic(rings):
    """Rings joined in a closed cycle (the last back to the first): a tube
    whose section is the polygon the rings trace, e.g. a frame whose
    section is outer-front, outer-back, inner-back, inner-front."""
    m = len(rings[0])
    n = len(rings)
    verts = [p for r in rings for p in r]
    faces = []
    for i in range(n):
        a, b = i * m, ((i + 1) % n) * m
        for k in range(m):
            k2 = (k + 1) % m
            faces.append((a + k, a + k2, b + k2, b + k))
    return shapes.orient((verts, faces))


ARCH_Y = 0.78      # an arch spans this fraction of the local half-width


def frame(x, y_cap=None):
    """A ring frame as two arches -- one under the upper skin, one over the
    lower -- each FRAME_T thick and FRAME_DEPTH deep, standing a seat in
    from the skin. They stop short of the chine, where the section is
    thinner than a frame is deep and the wing's spars take the load
    instead."""
    x0, x1 = x - FRAME_T / 2, x + FRAME_T / 2
    Y = ARCH_Y * shapes.half_width(x0, spec.SKIN_T)
    if y_cap is not None:
        Y = min(Y, y_cap)
    # finely: aft, the skin is a valley between the nacelles, and a coarse
    # chord across a valley stands proud of the skin in its bottom
    ys = [-Y + 2 * Y * i / 160 for i in range(161)]
    ins = spec.SKIN_T + SEAT
    parts = []
    for surf, sgn in ((shapes.z_up, -1.0), (shapes.z_dn, 1.0)):
        rings = []
        for y in ys:
            za = surf(x0, y, ins)
            zb = surf(x1, y, ins)
            # riveted to the skin: its outer face half a millimetre into the
            # skin's inside. (Over the spine's crest the skin's facets sag a
            # few millimetres under the surface they are cut from, and there
            # the frame is a few millimetres into the 10 mm skin.)
            z_out = (min(za, zb) + 0.5) if sgn < 0 else (max(za, zb) - 0.5)
            z_in = z_out + sgn * FRAME_DEPTH
            rings.append([(x0, y, z_in), (x1, y, z_in), (x1, y, z_out), (x0, y, z_out)])
        parts.append(shapes.loft_rings(rings))
    return mesh.join(*parts)


def arch_inner(x, y, upper=True):
    """The inner edge of a frame's arch at (x, y)."""
    ins = spec.SKIN_T + SEAT
    if upper:
        return min(shapes.z_up(x - FRAME_T / 2, y, ins),
                   shapes.z_up(x + FRAME_T / 2, y, ins)) - FRAME_DEPTH
    return max(shapes.z_dn(x - FRAME_T / 2, y, ins),
               shapes.z_dn(x + FRAME_T / 2, y, ins)) + FRAME_DEPTH


def keel():
    x0, x1 = spec.BAY["x1"] + 1500.0, spec.BODY_END_X - 40.0
    xs = [x0 + (x1 - x0) * i / 20 for i in range(21)]
    rings = []
    for x in xs:
        # the lowest of the skin over its faces and its middle: aft, the
        # crown is a valley between the nacelles, lowest on the centreline
        ins = spec.SKIN_T + SEAT
        zt = min(shapes.z_up(x, y, ins) for y in (-15.0, 0.0, 15.0)) - 4.0
        zb = max(shapes.z_dn(x, y, ins) for y in (-15.0, 0.0, 15.0)) + 4.0
        rings.append([(x, -15.0, zb), (x, 15.0, zb), (x, 15.0, zt), (x, -15.0, zt)])
    return shapes.loft_rings(rings)


def x_fwd_frame():
    return spec.ENGINE_FAN_FACE_X + engines.info()["trunnion_x"]


def x_aft_frame():
    return spec.ENGINE_FAN_FACE_X + engines.info()["aft_lug_x"]


def mounts():
    """Each engine hangs on its two trunnions -- the inboard one on a beam
    from the keel, the outboard one on a strut up and out to the forward
    frame's upper arch -- and is steadied by a link from its thrust lug up
    into the aft frame's upper arch."""
    e = engines.info()
    x = x_fwd_frame()
    top = e["trunnion_top"]
    parts = []
    for sy in (1.0, -1.0):
        yc = sy * spec.ENGINE_Y
        # inboard: keel face to the trunnion's end, let 2 mm into each
        ya, yb = sy * 13.0, yc - sy * (top - 2.0)
        parts.append(mesh.box(x, 0.5 * (ya + yb), spec.ENGINE_Z, 90.0, abs(yb - ya),
                              90.0))
        # outboard: a strut from the trunnion's end to the upper arch
        y_arch = yc + sy * (top + 260.0)
        p0 = (x, yc + sy * (top - 6.0), spec.ENGINE_Z)
        p1 = (x, y_arch, arch_inner(x, y_arch) + 12.0)
        parts.append(mesh.pipe([p0, p1], 34.0, 18))
        parts.append(mesh.box(x, yc + sy * (top + 8.0), spec.ENGINE_Z, 90.0, 28.0, 90.0))
        # thrust link: from the aft lug's pin up into the aft frame
        xa = x_aft_frame()
        z0 = spec.ENGINE_Z + e["od_aft"] + 38.0
        z1 = arch_inner(xa, yc) + 8.0
        parts.append(mesh.pipe([(xa, yc, z0 - 10.0), (xa, yc, z1)], 16.0, 14))
    return mesh.join(*parts)


def radar():
    """Bulkhead behind the radome, and the array on it, tilted back 20
    degrees so its broadside reflection goes up, away from a threat."""
    xb = 1500.0
    bulk = shapes.loft_rings([shapes.ring(xb, M, spec.SKIN_T + SEAT),
                              shapes.ring(xb + 12.0, M, spec.SKIN_T + SEAT)])
    xa, za, r = 1260.0, 80.0, 250.0
    t = math.radians(20.0)
    v, f = mesh.revolve_ring([(-18.0, 0.0 + 1.0), (18.0, 1.0), (18.0, r), (-18.0, r)], 48)
    v = [(xa + x * math.cos(t) - z * math.sin(t), y, za + x * math.sin(t) + z * math.cos(t))
         for (x, y, z) in v]
    stub = mesh.pipe([(xa + 10.0, 0.0, za), (xb + 2.0, 0.0, za)], 40.0, 16)
    return bulk, mesh.join((v, f), stub)


# The forward and centre fuselage's frames. The body used to have two
# frames in it, both at the engines: the cockpit, the canards, the gun, the
# weapons bay and the intakes hung in a skin with nothing inside it. Each of
# these stands where the airframe has a joint or a load to take -- the
# forward fuselage joint, either end of the cockpit tub, the canards'
# pivot, and three along the weapons bay -- and each is relieved where
# something passes through it: the canopy opening over the cockpit, the
# nose gear's bay and the weapons bay under it, the intake ducts, and the
# gun in the starboard shoulder.
FWD_FRAMES = (2250.0, 3270.0, 5119.0, 5680.0, 6800.0, 7900.0, 9000.0)
WING_ROOT_CLEAR = 1640.0      # the arches stop inboard of the wing's root


def _box(x0, x1, y0, y1, z0, z1):
    return mesh.box(0.5 * (x0 + x1), 0.5 * (y0 + y1), 0.5 * (z0 + z1),
                    x1 - x0, y1 - y0, z1 - z0)


def _frame_cutter(x):
    from parts import intakes, cockpit, details
    x0, x1 = x - FRAME_T / 2 - 10.0, x + FRAME_T / 2 + 10.0
    cuts = []
    CK = spec.COCKPIT
    if CK["x0"] < x < CK["x1"]:
        cuts.append(cockpit.opening_cutter())
    G = spec.GEAR
    nb0, nb1, _nw = G["nose_bay"]
    if nb0 - 30.0 < x < nb1 + 30.0:
        cuts.append(_box(x0, x1, -240.0, 240.0, -2000.0, 360.0))
    B = spec.BAY
    if B["x0"] - 30.0 < x < B["x1"] + 30.0:
        cuts.append(_box(x0, x1, -B["half_w"] - 20.0, B["half_w"] + 20.0,
                         -2000.0, B["z_roof"] + 20.0))
    I = spec.INTAKE
    if x > intakes.lip_range()[0] - 60.0:
        for sgn in (1.0, -1.0):
            rings = [[(xx, sgn * y, z) for (y, z) in
                      intakes._shape(max(xx, I["x_mouth"]), I["wall"] + 20.0)]
                     for xx in (x0, x1)]
            cuts.append(shapes.loft_rings(rings))
    gx0, gx1 = details.GUN_X
    if gx0 - 60.0 < x < gx1 + 60.0:
        cuts.append(_box(x0, x1, details.GUN_Y - 110.0, details.GUN_Y + 110.0,
                         330.0, 2000.0))
    return mesh.join(*cuts) if cuts else None


def build():
    bulk, array = radar()
    out = {}
    for x in FWD_FRAMES:
        name = f"frame_{x:.0f}"
        out[name] = frame(x, WING_ROOT_CLEAR)
        c = _frame_cutter(x)
        if c is not None:
            out[f"cut:{name}"] = c
    return {**out,
        "keel": keel(),
        "frame_engine_fwd": frame(x_fwd_frame()),
        "frame_engine_aft": frame(x_aft_frame()),
        "engine_mounts": mounts(),
        "radar_bulkhead": bulk,
        "radar_array": array,
    }
