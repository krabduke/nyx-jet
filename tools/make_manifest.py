"""Generate viewer/parts.json from build/parts.csv, nyx/spec.py and aero/.

    python3 tools/make_manifest.py [out.json]

The viewer holds no engineering of its own: the modules, the palette, the
figures in its panel -- mass, wing loading, thrust-to-weight, stability and
turn rates -- all come from here, computed by the same code `make aero` runs,
so the page cannot quote a number the model does not have.
"""

import csv
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "nyx"))
sys.path.insert(0, os.path.join(ROOT, "aero"))
import spec      # noqa: E402
import agility   # noqa: E402
import vlm       # noqa: E402
from parts import engines   # noqa: E402
from parts import gear      # noqa: E402
from parts import cockpit   # noqa: E402
from parts import bay       # noqa: E402
from parts import fuel      # noqa: E402
from parts import ecs       # noqa: E402
from parts import intakes   # noqa: E402

MODULES = [
    ("01 Airframe", "Airframe", "#C9CDD1"),
    ("02 Flying surfaces", "Flying surfaces", "#8E979E"),
    ("03 Intakes", "Intakes", "#6F7B84"),
    ("04 Cockpit", "Cockpit", "#B58A3C"),
    ("05 Weapons bay", "Weapons bay", "#7D8570"),
    ("06 Landing gear", "Landing gear", "#5C6166"),
    ("07 Structure", "Structure", "#9A8F7E"),
    ("08 Engine port", "Engine, port", "#B8562E"),
    ("09 Engine starboard", "Engine, starboard", "#B8562E"),
    ("10 Systems", "Systems", "#4F8A8B"),
]


def r1(v, n=1):
    return round(float(v), n)


def figures(rows):
    j = agility.Jet()
    sm, x_np, x_cg, cla = vlm.static_margin()
    itr0, vc0 = j.instantaneous(0.0)
    str0, vs0, ns0 = j.best_sustained(0.0)
    itr15, _ = j.instantaneous(4572.0)
    str15, _, _ = j.best_sustained(4572.0)
    span = max(float(r["y_max_mm"]) for r in rows) - \
        min(float(r["y_min_mm"]) for r in rows)
    length = max(float(r["x_max_mm"]) for r in rows) - \
        min(float(r["x_min_mm"]) for r in rows)
    return {
        "length_mm": r1(length, 0), "span_mm": r1(span, 0),
        "combat_mass_kg": r1(j.m, 0), "empty_mass_kg": r1(spec.empty_mass(), 0),
        "wing_area_m2": r1(j.S), "aspect_ratio": r1(j.AR, 2),
        "wing_loading": r1(j.m / j.S, 0), "tw": r1(j.T0 / j.W, 2),
        "static_margin_pct": r1(100 * sm), "g_limit": spec.G_LIMIT,
        "itr_sl": r1(itr0), "corner_ms": r1(vc0, 0),
        "str_sl": r1(str0), "str_sl_g": r1(ns0),
        "itr_15k": r1(itr15), "str_15k": r1(str15),
        "engine_thrust_kn": r1(agility.engine_thrust(True) / 1000.0),
    }


def palette():
    """The materials by name, as the build assigns them: the engine's alloys
    from the vendored Aether, the airframe's own over them."""
    p = dict(engines.info()["palette"])
    p.update(spec.PALETTE)
    return {k: {"rgb": list(v[0]), "metal": v[1], "rough": v[2]}
            for k, v in p.items()}


def nozzles():
    """Each engine's swivel, in the aircraft's frame: every bearing's centre
    and axis, the installed parts that turn on it, and the fold angle
    against the middle bearing's turn, for the viewer to invert."""
    n = engines.info()["nozzle"]
    out = []
    for side, sy in engines.SIDES:
        off = engines.offset(sy)
        brg = []
        for (c, axis), key in zip(n["bearings"], ("fwd", "mid", "aft")):
            brg.append({"c": [r1(c[i] + off[i], 3) for i in range(3)],
                        "n": [r1(v, 6) for v in axis],
                        "parts": [f"engine_{side}_{p}" for p in n["groups"][key]]})
        out.append({"side": side, "bearings": brg,
                    "exit": [r1(off[0] + n["x_exit"]), r1(off[1]), r1(off[2])]})
    return {"engines": out, "exit_r": r1(n["r_exit"]),
            "fold_table": [[r1(a, 2), r1(b, 3)] for a, b in n["fold_table"]]}


def gear_kinematics():
    """Each gear leg's pivot, axis and stowing angle, and each door's hinge
    and closing angle, rounded for the page."""
    k = gear.kinematics()
    rd = lambda v: [r1(c, 4) for c in v]
    return {"legs": [{"parts": L["parts"], "pivot": rd(L["pivot"]),
                      "axis": rd(L["axis"]), "stow": r1(L["stow"], 5)}
                     for L in k["legs"]],
            "doors": [{"part": d["part"], "hinge": rd(d["hinge"]),
                       "axis": rd(d["axis"]), "close": r1(d["close"], 5)}
                      for d in k["doors"]],
            "struts": [{"body": t["body"], "rod": t["rod"], "anchor": rd(t["anchor"]),
                        "lug": rd(t["lug"]), "hinge": rd(t["hinge"]),
                        "axis": rd(t["axis"]), "close": r1(t["close"], 5)}
                       for t in k.get("struts", [])]}


def canopy_kinematics():
    """The canopy's hinge and opening angle, the parts that swing with it,
    and each actuator's anchor and lug, rounded for the page."""
    k = cockpit.canopy_kinematics()
    rd = lambda v: [r1(c, 4) for c in v]
    return {"hinge": rd(k["hinge"]), "axis": rd(k["axis"]),
            "open": r1(k["open"], 5), "parts": k["parts"],
            "struts": [{"body": t["body"], "rod": t["rod"],
                        "anchor": rd(t["anchor"]), "lug": rd(t["lug"])}
                       for t in k["struts"]]}


def bay_kinematics():
    k = bay.kinematics()
    rd = lambda v: [r1(c, 4) for c in v]
    return {"doors": [{"part": d["part"], "hinge": rd(d["hinge"]), "axis": rd(d["axis"]),
                       "open": r1(d["open"], 5)} for d in k["doors"]],
            "struts": [{"body": t["body"], "rod": t["rod"], "anchor": rd(t["anchor"]),
                        "lug": rd(t["lug"]), "hinge": rd(t["hinge"]), "axis": rd(t["axis"]),
                        "open": r1(t["open"], 5)} for t in k["struts"]]}


def flows():
    """What moves through the aircraft, each as a centreline from where it
    comes from to where it goes, for the viewer to run along."""
    I = spec.INTAKE
    R = json.load(open(ecs.ROUTES))
    mirror = lambda pts: [(x, -y, z) for (x, y, z) in pts]
    NOZZLE_START = engines.info()["nozzle"]["bearings"][0][0][0]
    out = []

    def add(kind, label, note, pts, r):
        out.append({"kind": kind, "label": label, "note": note, "r": r1(r),
                    "pts": [[r1(c) for c in p] for p in pts]})

    air = []
    for k in range(25):
        x = I["x_mouth"] + (I["x_end"] - I["x_mouth"]) * k / 24
        air.append((x, *intakes.centre(x)))
    air += [(spec.ENGINE_FAN_FACE_X, spec.ENGINE_Y, spec.ENGINE_Z),
            (spec.ENGINE_FAN_FACE_X + NOZZLE_START, spec.ENGINE_Y, spec.ENGINE_Z)]
    note = ("In at the caret intake, round the S-duct to the fan face, and through the "
            "engine to its swivel nozzle.")
    add("air", "Intake air, starboard engine", note, air, 45.0)
    add("air", "Intake air, port engine", note, mirror(air), 45.0)

    add("fuel", "Refuelling", "From the boom's receptacle on the spine down into the "
        "gallery, and aft through the three forward cells to the collector tank.",
        fuel.refuel_path(), fuel.GALLERY_R)
    note = "From the boost pump on the collector tank's back to the fuel pump on the engine's gearbox."
    add("fuel", "Engine feed, starboard", note, fuel.feed_path(), 15.0)
    add("fuel", "Engine feed, port", note, mirror(fuel.feed_path()), 15.0)
    for y, lab, a, b in ((-100.0, "Fuel to the air-conditioning pack", 10174.0, ecs.PACK_X[0] + 4.0),
                         (100.0, "Fuel back from the air-conditioning pack", ecs.PACK_X[0] + 4.0, 10174.0)):
        add("fuel", lab, "The pack's heat exchanger dumps its heat into the fuel, which "
            "the engines then burn.", [(a, y, 245.0), (b, y, 245.0)], ecs.FUEL_R)

    note = "Hot air off each engine's compressor, forward and inboard to the air-cycle pack."
    add("bleed", "Bleed air, starboard engine", note, R["bleed_r"], ecs.BLEED_R)
    add("bleed", "Bleed air, port engine", note, mirror(R["bleed_r"]), ecs.BLEED_R)
    add("cool", "Cockpit air", "Cooled and dried in the pack, forward under the spine to "
        "the outlet behind the pilot's seat.", R["duct"], ecs.DUCT_R)
    add("cool", "Oxygen generator's supply", "Off the pack's outlet, down to the oxygen "
        "generator beside it.", R["obogs_feed"], ecs.HOSE_R + 2.0)
    add("oxygen", "Oxygen", "From the generator's sieve beds forward beside the cockpit "
        "air duct to the seat's connector.", R["oxygen"], ecs.HOSE_R)
    return out


def main():
    rows = list(csv.DictReader(open(os.path.join(ROOT, "build", "parts.csv"))))
    mods = []
    for key, label, color in MODULES:
        mine = [r for r in rows if r["collection"] == key]
        if not mine:
            continue
        mods.append({"key": key, "label": label, "color": color,
                     "parts": len(mine),
                     "faces": sum(int(r["faces"]) for r in mine)})
    parts = {r["name"]: {"m": r["collection"], "mat": r["material"],
                         "f": int(r["faces"])} for r in rows}
    out = {
        "name": spec.NAME, "role": spec.ROLE,
        "figures": figures(rows),
        "palette": palette(),
        "nozzles": nozzles(),
        "gear": gear_kinematics(),
        "canopy": canopy_kinematics(),
        "bay": bay_kinematics(),
        "flows": flows(),
        "modules": mods,
        "parts": parts,
    }
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "viewer",
                                                              "parts.json")
    json.dump(out, open(path, "w"), indent=1)
    print(f"  -> {path}  ({len(parts)} parts, {len(mods)} modules)")


if __name__ == "__main__":
    main()
