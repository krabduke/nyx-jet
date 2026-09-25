"""Everything inside the airframe stays inside it.

The interference audit has to let an actuator share material with the wing
it sits in, and a frame with the skin it is riveted to -- and the same rule
would let either stand out through the outside of the aircraft without a
word. A main gear actuator's flange did, 10 mm proud of the wing's top.

This checks every vertex of the airframe's internal parts against the
outer mould line -- the body, wings, flaps and fins as lofted, before any
opening is cut in them: each must be inside one of them.

    python3 tools/audit_oml.py
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import _interfere  # noqa: E402

if not _interfere.in_blender():
    import subprocess
    out = subprocess.run([_interfere.blender_exe(), "-b",
                          os.path.join(ROOT, "build", "nyx.blend"), "-P",
                          os.path.abspath(__file__)],
                         capture_output=True, text=True).stdout
    lines = [l for l in out.splitlines()
             if l.startswith(("PASS", "FAIL", "  ", "Traceback", "Error"))]
    sys.stdout.write("".join(l + "\n" for l in lines))
    sys.exit(0 if any(l.startswith("PASS") for l in lines) else 1)

import bpy  # noqa: E402
from mathutils import Vector  # noqa: E402
from mathutils.bvhtree import BVHTree  # noqa: E402

sys.path.insert(0, os.path.join(ROOT, "nyx"))
import spec    # noqa: E402
import shapes  # noqa: E402
from parts import surfaces  # noqa: E402

MM = 0.001
INTERNAL = ("frame_", "fuel_tank_", "canard_drive_", "flaperon_act_",
            "le_flap_act_", "rudder_act_", "gear_actuator_", "seat_bulkhead",
            "cockpit_tub", "radar_", "keel", "bay_structure", "gear_bay_")
TOL = 0.5          # mm


def _oml_solids():
    xs = shapes.body_stations(1.0, spec.BODY_END_X, spec.RES["body_rings"])
    body = shapes.loft_rings([shapes.ring(x, spec.RES["body_ring_pts"]) for x in xs])
    out = [body]
    for mirror in (False, True):
        for part in (surfaces.wing_parts(), surfaces.fin_parts()):
            for k, (v, f) in part.items():
                if k.startswith("cut:") or "act" in k:
                    continue
                if mirror:
                    v = [(x, -y, z) for (x, y, z) in v]
                    f = [tuple(reversed(c)) for c in f]
                out.append((v, f))
    # and the canopy's envelope: its outer surface, closed along the skin
    from parts import cockpit
    CK = spec.COCKPIT
    rings = []
    for i in range(25):
        x = CK["x0"] + 125.0 + (CK["x1"] - CK["x0"] - 250.0) * i / 24
        sec = cockpit._canopy_section(x)
        outer = sec[:len(sec) // 2]
        a = abs(outer[0][1])
        skin = [(x, y, shapes.z_up(x, y) - 60.0)
                for y in [-a + 2 * a * k / 20 for k in range(21)]]
        rings.append(outer + skin)
    out.append(shapes.loft_rings(rings))
    return out


trees = []
for (v, f) in _oml_solids():
    vv = [Vector((x * MM, y * MM, z * MM)) for (x, y, z) in v]
    ff = [list(c) for c in f]
    trees.append(BVHTree.FromPolygons(vv, ff))


def inside(p):
    for t in trees:
        n = 0
        q = p.copy()
        d = Vector((0.0123, 0.0071, 1.0)).normalized()
        for _ in range(64):
            hit = t.ray_cast(q, d)
            if hit[0] is None:
                break
            n += 1
            q = hit[0] + d * 1e-6
        if n % 2 == 1:
            return True
        # or within tolerance of the surface
        loc = t.find_nearest(p)
        if loc[0] is not None and loc[3] < TOL * MM:
            return True
    return False


bad = []
n_parts = 0
for o in bpy.data.objects:
    if o.type != "MESH" or not o.name.startswith(INTERNAL):
        continue
    n_parts += 1
    mw = o.matrix_world
    worst = None
    for vtx in o.data.vertices:
        p = mw @ vtx.co
        if not inside(p):
            worst = p
            break
    if worst is not None:
        bad.append(f"  {o.name:28s} out at ({worst.x / MM:.0f}, {worst.y / MM:.0f}, "
                   f"{worst.z / MM:.0f})")
if bad:
    print(f"FAIL  {len(bad)} internal parts come out through the airframe's skin")
    for b in bad:
        print(b)
else:
    print(f"PASS  all {n_parts} internal parts are inside the airframe")
