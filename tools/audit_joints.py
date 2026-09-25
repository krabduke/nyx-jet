"""Does the aircraft hold together, and is every load path and circuit joined?

    python3 tools/audit_joints.py

The obligations the other audits cannot state: every part attached to the
airframe, and each declared chain continuous link by link -- the engines
hung on their mounts from the structure, the ducts delivering to the fans,
the flying surfaces on their hinges and spindles, the gear standing in its
bays. Contact is within 6 mm; see tools/_joints.py. The engine and the car
use 1-2 mm, which on a 14.6 m airframe is a contact grid of tens of
millions of cells; at 6 mm it is nine times smaller, and a joint on an
aircraft this size is not a 6 mm question.
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import _joints  # noqa: E402

CONTACT_MM = 6.0
ROOT_PART = "fuselage_skin"
PKG = "nyx/parts"
UNIT = "mm"


def mirrored(label, chain):
    return [(label + " (starboard)", [c.replace("{s}", "r") for c in chain]),
            (label + " (port)", [c.replace("{s}", "l") for c in chain])]


CIRCUITS = []
CIRCUITS += mirrored("the wing is joined to the body", ["fuselage_skin", "wing_{s}"])
CIRCUITS += mirrored("its flaps and flaperons hang on it",
                     ["le_flap_{s}", "wing_{s}", "flaperon_in_{s}"])
CIRCUITS += mirrored("", ["wing_{s}", "flaperon_out_{s}"])
CIRCUITS += mirrored("each canard turns on its spindle in the body",
                     ["fuselage_skin", "canard_spindle_{s}", "canard_{s}"])
CIRCUITS += mirrored("each fin stands on the body and carries its rudder",
                     ["fuselage_skin", "fin_{s}", "rudder_{s}"])
CIRCUITS += mirrored("each intake duct runs from its mouth to its engine's face",
                     ["fuselage_skin", "intake_duct_{s}", "engine_{s}_inlet_case"])
CIRCUITS += mirrored("each engine hangs on the keel and the forward frame",
                     ["keel", "engine_mounts", "engine_{s}_mount_trunnions",
                      "engine_{s}_case_outer_fwd"])
CIRCUITS += mirrored("", ["frame_engine_fwd", "engine_mounts"])
CIRCUITS += mirrored("and is steadied by its thrust link into the aft frame",
                     ["engine_{s}_mount_aft_lug", "engine_mounts", "frame_engine_aft"])
CIRCUITS += mirrored("the frames are in the skin", ["fuselage_skin", "frame_engine_fwd"])
CIRCUITS += mirrored("", ["fuselage_skin", "frame_engine_aft", "keel"])
CIRCUITS += [
    ("the canopy sits on the skin", ["fuselage_skin", "canopy_glass"]),
    ("the cockpit is built into the body",
     ["fuselage_skin", "cockpit_tub", "seat"]),
    ("", ["cockpit_tub", "cockpit_panel", "cockpit_hud"]),
    ("", ["cockpit_tub", "cockpit_controls"]),
    ("the weapons bay is built into the body and holds its missiles",
     ["fuselage_skin", "bay_structure", "launchers", "missiles"]),
    ("its doors close on the skin", ["fuselage_skin", "bay_door_r"]),
    ("", ["fuselage_skin", "bay_door_l"]),
    ("the nose gear stands in its bay",
     ["fuselage_skin", "gear_bay_nose", "gear_nose", "wheels_nose", "tyres_nose"]),
    ("and closes behind two doors", ["fuselage_skin", "gear_door_nose_r"]),
    ("", ["fuselage_skin", "gear_door_nose_l"]),
    ("the radar is on its bulkhead",
     ["fuselage_skin", "radar_bulkhead", "radar_array"]),
]
CIRCUITS += mirrored("each main gear hangs from its wing root",
                     ["wing_{s}", "gear_main_{s}", "wheel_main_{s}",
                      "tyre_main_{s}"])
CIRCUITS += mirrored("and its doors: on the leg, over the pivot, over the well",
                     ["gear_main_{s}", "gear_leg_door_main_{s}"])
CIRCUITS += mirrored("", ["wing_{s}", "gear_pivot_door_main_{s}"])
CIRCUITS += mirrored("", ["fuselage_skin", "gear_door_main_{s}"])
CIRCUITS += mirrored("the well is built into the body",
                     ["fuselage_skin", "gear_bay_main_{s}"])


def main():
    parts, collisions, cut_owner, built_by, failures = _joints.load(ROOT, PKG)
    bad = []
    print(f"\n{len(parts)} parts from {len(set(built_by.values()))} modules")
    print("\nMODULES")
    for name, why in failures:
        print(f"  x   {name:26s} did not build: {why}")
        bad.append(name)
    for key, first, second in collisions:
        print(f"  x   {key:26s} built by both {first} and {second}")
        bad.append(key)
    for target, owners in sorted(cut_owner.items()):
        if target not in built_by:
            print(f"  x   {target:26s} cut declared by {owners}, nothing builds it")
            bad.append(target)
    if not bad:
        print("  ok  every part name is built once and every cutter has a target")
    print(f"\nASSEMBLY  (contact within {CONTACT_MM:g} {UNIT})")
    graph = _joints.contact_graph(parts, CONTACT_MM)
    groups = _joints.components(graph)
    main_group = next((g for g in groups if ROOT_PART in g), set())
    loose = [g for g in groups if g is not main_group]
    if not loose:
        print(f"  ok  all {len(main_group)} parts hang together off {ROOT_PART}")
    for g in loose:
        names = ", ".join(sorted(g))
        print(f"  x   detached: {names}")
        bad.append("detached")
    print("\nCIRCUITS")
    last = ""
    for label, chain in CIRCUITS:
        label = label or last
        last = label
        br = _joints.broken_links(parts, graph, chain)
        if not br:
            print(f"  ok  {label}")
            continue
        for (_i, a, b, why) in br:
            extra = ""
            if why == "no contact":
                A, B = _joints.match(parts, a), _joints.match(parts, b)
                d = min(_joints.gap(parts[x], parts[y]) for x in A for y in B)
                extra = f" ({d:.1f} {UNIT} apart)"
            print(f"  x   {label}: {a} -> {b}, {why}{extra}")
            bad.append(label)
    print()
    if not bad:
        print("PASS  it is one assembly and every circuit is joined")
        return 0
    print(f"FAIL  {len(bad)} joints are not made")
    return 1


if __name__ == "__main__":
    sys.exit(main())
