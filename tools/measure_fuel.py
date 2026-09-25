"""Measure the fuel tanks as built, for the mass table.

    python3 tools/measure_fuel.py          write nyx/fuel_tanks.json
    python3 tools/measure_fuel.py --check  fail if it no longer matches

The mass table's fuel is what the tanks in build/nyx.blend hold: each
fuel_tank_* object's volume, after every cutter, times the fuel's density
and the usable fraction, at the tank's own centroid. It used to be three
numbers typed into spec.py, one of them -- 2,800 kg in the wings -- more
than twice what the wings between their spars can hold.
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import _interfere  # noqa: E402

OUT = os.path.join(ROOT, "nyx", "fuel_tanks.json")
DENSITY = 0.80          # kg/L, JP-8 at 15 C
USABLE = 0.97           # the rest is below the pickups
TOL = 0.005             # the file and the build agree to half a percent

if not _interfere.in_blender():
    import subprocess
    args = [_interfere.blender_exe(), "-b", os.path.join(ROOT, "build", "nyx.blend"),
            "-P", os.path.abspath(__file__), "--"] + sys.argv[1:]
    out = subprocess.run(args, capture_output=True, text=True).stdout
    lines = [l for l in out.splitlines() if l.startswith(("PASS", "FAIL", "  ", "wrote", "Traceback"))]
    sys.stdout.write("".join(l + "\n" for l in lines))
    sys.exit(0 if not any(l.startswith(("FAIL", "Traceback")) for l in lines) else 1)

import bmesh  # noqa: E402
import bpy    # noqa: E402

argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
dg = bpy.context.evaluated_depsgraph_get()
tanks = {}
for o in bpy.data.objects:
    if o.type != "MESH" or not o.name.startswith("fuel_tank_"):
        continue
    bm = bmesh.new()
    bm.from_object(o, dg)
    bm.transform(o.matrix_world)
    vol = bm.calc_volume(signed=False)            # m^3
    # centroid of the solid, by signed tetrahedra from the origin
    cx = cy = cz = 0.0
    tot = 0.0
    bmesh.ops.triangulate(bm, faces=bm.faces[:])
    for f in bm.faces:
        a, b, c = (v.co for v in f.verts)
        v6 = a.dot(b.cross(c))
        tot += v6
        cx += v6 * (a.x + b.x + c.x)
        cy += v6 * (a.y + b.y + c.y)
        cz += v6 * (a.z + b.z + c.z)
    bm.free()
    litres = vol * 1000.0
    tanks[o.name] = {"litres": round(litres, 1),
                     "kg": round(litres * DENSITY * USABLE, 1),
                     "x_cg_mm": round(cx / (4.0 * tot) * 1000.0, 1),
                     "y_cg_mm": round(cy / (4.0 * tot) * 1000.0, 1),
                     "z_cg_mm": round(cz / (4.0 * tot) * 1000.0, 1)}
tanks = dict(sorted(tanks.items()))
if "--check" in argv:
    old = json.load(open(OUT)) if os.path.exists(OUT) else {}
    bad = []
    for n, t in tanks.items():
        o = old.get(n)
        if o is None or abs(o["kg"] - t["kg"]) > TOL * max(t["kg"], 1.0) \
                or abs(o["x_cg_mm"] - t["x_cg_mm"]) > 5.0:
            bad.append(f"  {n}: built {t['kg']:.0f} kg at x {t['x_cg_mm']:.0f}, "
                       f"file says {o and o['kg']} kg")
    for n in old:
        if n not in tanks:
            bad.append(f"  {n}: in the file, not in the build")
    if bad:
        print("FAIL  the mass table's fuel is not what the tanks hold -- "
              "run tools/measure_fuel.py")
        for b in bad:
            print(b)
    else:
        tot = sum(t["kg"] for t in tanks.values())
        print(f"PASS  the mass table's fuel is what the {len(tanks)} tanks hold, "
              f"{tot:,.0f} kg")
else:
    json.dump(tanks, open(OUT, "w"), indent=1)
    for n, t in tanks.items():
        print(f"  {n:22s} {t['litres']:8.0f} L  {t['kg']:7.0f} kg  x {t['x_cg_mm']:7.0f}")
    print(f"wrote {OUT}")
