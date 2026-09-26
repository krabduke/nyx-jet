"""Build the aircraft in Blender. Run under `blender --background`.

Same pipeline as the engine's: pure-Python (verts, faces) in mm -> a bpy mesh
in metres -> normals outward -> boolean cutters (the skin's openings, the
engines' cooling holes) -> material -> collection -> shading. Writes
build/parts.csv for verify.py and the viewer's manifest.
"""

import csv
import math
import os
import sys
import time

import bpy

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import spec              # noqa: E402
import mesh as meshlib   # noqa: E402
import materials         # noqa: E402
from parts import (fuselage, surfaces, intakes, cockpit, bay, gear,  # noqa: E402
                   structure, engines, details, fuel, electrical, avionics)

MM = 0.001

MODULES = [
    ("fuselage", fuselage), ("surfaces", surfaces), ("intakes", intakes),
    ("cockpit", cockpit), ("bay", bay), ("gear", gear),
    ("structure", structure), ("details", details), ("engines", engines),
    ("fuel", fuel), ("electrical", electrical), ("avionics", avionics),
]

COLLECTIONS = ["01 Airframe", "02 Flying surfaces", "03 Intakes",
               "04 Cockpit", "05 Weapons bay", "06 Landing gear",
               "07 Structure", "08 Engine port", "09 Engine starboard",
               "10 Systems"]


def collection_for(name):
    if name.startswith("engine_l_"):
        return "08 Engine port"
    if name.startswith("engine_r_"):
        return "09 Engine starboard"
    if name.startswith(("fuselage", "aft_closure", "irst_", "air_data_",
                        "aoa_vane", "antenna_", "nav_light", "tail_light",
                        "static_wicks", "panel_seams")):
        return "01 Airframe"
    if name.startswith(("wing", "le_flap", "flaperon", "canard", "fin", "rudder")):
        return "02 Flying surfaces"
    if name.startswith("intake"):
        return "03 Intakes"
    if name.startswith(("canopy", "cockpit", "seat")):
        return "04 Cockpit"
    if name.startswith(("bay", "missile", "launcher", "gun")):
        return "05 Weapons bay"
    if name.startswith(("gear", "tyre", "wheel")):
        return "06 Landing gear"
    if name.startswith(("avionics_", "loom_", "pdu")):
        return "10 Systems"
    return "07 Structure"


def material_for(name):
    m = engines.part_material(name)
    if m:
        return m
    best, best_len = spec.DEFAULT_MATERIAL, -1
    for key, mat in spec.MATERIAL_MAP.items():
        if name.startswith(key) and len(key) > best_len:
            best, best_len = mat, len(key)
    return best


def palette():
    p = dict(engines.info()["palette"])
    p.update(spec.PALETTE)
    return p


def clear_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    s = bpy.context.scene
    s.unit_settings.system = "METRIC"
    s.unit_settings.length_unit = "METERS"


def make_object(name, verts, faces, collection):
    me = bpy.data.meshes.new(name)
    me.from_pydata([(x * MM, y * MM, z * MM) for (x, y, z) in verts], [],
                   [list(f) for f in faces])
    me.validate(verbose=False)
    me.update()
    obj = bpy.data.objects.new(name, me)
    collection.objects.link(obj)
    return obj


def recalc_normals(obj):
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.mesh.normals_make_consistent(inside=False)
    bpy.ops.object.mode_set(mode="OBJECT")
    obj.select_set(False)


def apply_cutters(obj, cv, cf):
    cutter = make_object(obj.name + "__cutter", cv, cf, bpy.context.scene.collection)
    recalc_normals(cutter)
    m = obj.modifiers.new("holes", "BOOLEAN")
    m.operation = "DIFFERENCE"
    m.solver = "EXACT"
    m.use_self = True
    m.object = cutter
    bpy.context.view_layer.objects.active = obj
    try:
        bpy.ops.object.modifier_apply(modifier=m.name)
        ok = True
    except RuntimeError as exc:
        print(f"    ! boolean failed on {obj.name}: {exc}")
        obj.modifiers.remove(m)
        ok = False
    bpy.data.objects.remove(cutter, do_unlink=True)
    return ok


def build_proto(name, proto, collection):
    """An engine's cooled aerofoil row: holes cut in one aerofoil, arrayed
    about the engine's own axis, then moved to where the engine sits."""
    one, holes, count, rest, off = proto
    obj = make_object(name, *one, collection)
    recalc_normals(obj)
    ok = apply_cutters(obj, *holes)
    v = [tuple(c / MM for c in p.co) for p in obj.data.vertices]
    f = [tuple(p.vertices) for p in obj.data.polygons]
    v, f = meshlib.replicate(v, f, count)
    if rest is not None:
        v, f = meshlib.join((v, f), rest)
    v = [(x + off[0], y + off[1], z + off[2]) for (x, y, z) in v]
    me = bpy.data.meshes.new(name + "_row")
    me.from_pydata([(x * MM, y * MM, z * MM) for (x, y, z) in v], [],
                   [list(c) for c in f])
    me.validate(verbose=False)
    me.update()
    old = obj.data
    obj.data = me
    bpy.data.meshes.remove(old)
    return obj, ok


FACETED = ()


def shade(obj, angle_deg=32.0):
    me = obj.data
    for p in me.polygons:
        p.use_smooth = True
    lim = math.cos(math.radians(angle_deg))
    normals = [tuple(p.normal) for p in me.polygons]
    foe = {}
    for pi, poly in enumerate(me.polygons):
        for ek in poly.edge_keys:
            foe.setdefault(ek, []).append(pi)
    ebk = {e.key: e for e in me.edges}
    for ek, fs in foe.items():
        e = ebk.get(ek)
        if e is not None and (len(fs) != 2 or sum(a * b for a, b in zip(
                normals[fs[0]], normals[fs[1]])) < lim):
            e.use_edge_sharp = True


def main():
    t0 = time.time()
    clear_scene()
    mats = materials.build_all(palette())
    cols = {}
    for c in COLLECTIONS:
        col = bpy.data.collections.new(c)
        bpy.context.scene.collection.children.link(col)
        cols[c] = col
    built, cutters = [], {}
    for modname, mod in MODULES:
        b = mod.build()
        for k, g in b.items():
            if k.startswith("cut:"):
                cutters.setdefault(k[4:], []).append(g)
        built.append((modname, mod, b))
    rows, n_bool, n_ok = [], 0, 0
    for modname, mod, b in built:
        t1 = time.time()
        protos = getattr(mod, "PROTO", {})
        objs = {k: v for k, v in b.items() if not k.startswith("cut:")}
        for name, (verts, faces) in sorted(objs.items()):
            col = cols[collection_for(name)]
            if name in protos:
                obj, ok = build_proto(name, protos[name], col)
                n_bool += 1
                n_ok += int(ok)
            else:
                obj = make_object(name, verts, faces, col)
                recalc_normals(obj)
                for cut in cutters.get(name, ()):
                    n_bool += 1
                    n_ok += int(apply_cutters(obj, *cut))
            recalc_normals(obj)
            mat = material_for(name)
            # A boolean leaves the cutter's (empty) material slot on the
            # part, and every face points at it -- so every part with a hole
            # cut in it rendered in Blender's default white. One slot, ours.
            obj.data.materials.clear()
            for poly in obj.data.polygons:
                poly.material_index = 0
            obj.data.materials.append(mats[mat])
            # the faceted airframe shades its creases hard: they are 10 to
            # 25 degrees, and smoothed at the default 32 the panels melted
            # back into a blob
            shade(obj, 8.0 if name.startswith(FACETED) else 32.0)
            co = [tuple(v.co) for v in obj.data.vertices]
            bb = meshlib.bbox(co)
            sp = engines.part_spool(name)
            rows.append({
                "name": name, "collection": collection_for(name), "module": modname,
                "material": mat, "verts": len(obj.data.vertices),
                "faces": len(obj.data.polygons),
                "x_min_mm": round(bb[0] / MM, 1), "x_max_mm": round(bb[3] / MM, 1),
                "y_min_mm": round(bb[1] / MM, 1), "y_max_mm": round(bb[4] / MM, 1),
                "z_min_mm": round(bb[2] / MM, 1), "z_max_mm": round(bb[5] / MM, 1),
                "spool": sp,
                "spin": 1.0 if sp == "lp" else (-1.0 if sp == "hp" else ""),
            })
        print(f"  [{modname}] {len(objs)} objects in {time.time() - t1:.1f}s")
    os.makedirs(os.path.join(ROOT, "build"), exist_ok=True)
    rows.sort(key=lambda r: r["name"])
    with open(os.path.join(ROOT, "build", "parts.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"\n{len(rows)} objects | {sum(r['verts'] for r in rows):,} verts | "
          f"{sum(r['faces'] for r in rows):,} faces")
    print(f"booleans: {n_ok}/{n_bool} applied")
    if n_ok < n_bool:
        raise SystemExit(f"{n_bool - n_ok} booleans failed")
    blend = os.path.join(ROOT, "build", "nyx.blend")
    bpy.ops.wm.save_as_mainfile(filepath=blend)
    print(f"blend -> {blend}\ntotal {time.time() - t0:.1f}s")


if __name__ == "__main__":
    try:
        main()
    except BaseException:
        import traceback
        traceback.print_exc()
        sys.stdout.flush()
        os._exit(1)
