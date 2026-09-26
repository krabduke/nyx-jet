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
CIRCUITS += mirrored("and each flaperon is driven from the wing",
                     ["wing_{s}", "flaperon_act_in_{s}", "flaperon_in_{s}"])
CIRCUITS += mirrored("", ["wing_{s}", "flaperon_act_out_{s}", "flaperon_out_{s}"])
CIRCUITS += mirrored("and its leading-edge flap",
                     ["wing_{s}", "le_flap_act_{s}", "le_flap_{s}"])
CIRCUITS += mirrored("each rudder is driven from its fin",
                     ["fin_{s}", "rudder_act_{s}", "rudder_{s}"])
CIRCUITS += mirrored("each canard turns on its spindle in the body",
                     ["fuselage_skin", "canard_spindle_{s}", "canard_{s}"])
CIRCUITS += [("the fuel cells hang between the frames",
              ["frame_5680", "fuel_tank_fwd_1", "frame_6800", "fuel_tank_fwd_2",
               "frame_7900", "fuel_tank_fwd_3", "frame_9000", "fuel_tank_centre"])]
CIRCUITS += mirrored("and each wing's tank is in it", ["wing_{s}", "fuel_tank_wing_{s}"])
CIRCUITS += [("the nose leg is turned by its actuator on the bay's wall",
              ["gear_bay_nose", "gear_actuator_nose", "gear_nose"])]
CIRCUITS += mirrored("each main leg by its actuator in the wing",
                     ["wing_{s}", "gear_actuator_main_{s}", "gear_main_{s}"])
CIRCUITS += [("refuelling: receptacle, gallery, every tank",
              ["refuel_receptacle", "fuel_gallery", "fuel_tank_fwd_1"]),
             ("", ["fuel_gallery", "fuel_tank_fwd_2"]),
             ("", ["fuel_gallery", "fuel_tank_fwd_3"]),
             ("", ["fuel_gallery", "fuel_tank_centre"])]
CIRCUITS += mirrored("power: each engine's generator feeds the PDU",
                     ["engine_{s}_generator_{s}", "loom_fuselage_{s}", "pdu", "keel"])
CIRCUITS += mirrored("and the PDU every actuator: the wing's",
                     ["pdu", "loom_fuselage_{s}", "loom_wing_{s}", "flaperon_act_in_{s}"])
CIRCUITS += mirrored("", ["loom_wing_{s}", "flaperon_act_out_{s}"])
CIRCUITS += mirrored("", ["loom_wing_{s}", "le_flap_act_{s}"])
CIRCUITS += mirrored("", ["loom_wing_{s}", "gear_actuator_main_{s}"])
CIRCUITS += mirrored("the fin's", ["loom_fuselage_{s}", "loom_fin_{s}", "rudder_act_{s}"])
CIRCUITS += mirrored("the canard's", ["loom_fuselage_{s}", "canard_drive_{s}"])
CIRCUITS += [("and the nose gear's",
              ["loom_fuselage_r", "loom_nose_gear", "gear_actuator_nose"])]
CIRCUITS += mirrored("each engine is fed from the collector",
                     ["fuel_tank_centre", "fuel_feed_{s}", "engine_{s}_fuel_pump"])
CIRCUITS += [("the seat's rails are bolted to the bulkhead in the tub",
              ["seat", "seat_bulkhead", "cockpit_tub"])]
CIRCUITS += [("the canopy turns on its hinge, pushed by its actuators",
              ["canopy_glass", "canopy_rim", "canopy_hinge", "frame_5680"]),
             ("", ["cockpit_tub", "canopy_actuator_r", "canopy_actuator_rod_r",
                   "canopy_rim"]),
             ("", ["cockpit_tub", "canopy_actuator_l", "canopy_actuator_rod_l",
                   "canopy_rim"]),
             ("and is held shut by its locks", ["canopy_rim", "canopy_locks",
                                                "fuselage_skin"])]
CIRCUITS += [("the flight computers are on the PDU",
              ["pdu", "loom_fuselage_r", "loom_nose_gear", "avionics_loom",
               "avionics_fcc_1"]),
             ("", ["avionics_loom", "avionics_fcc_2"]),
             ("", ["avionics_loom", "avionics_fcc_3"]),
             ("", ["avionics_loom", "avionics_ins"]),
             ("", ["avionics_loom", "avionics_mission"])]
CIRCUITS += mirrored("each air data computer reads its probe, and is powered",
                     ["air_data_probe_{s}", "avionics_pitot_lines_{s}",
                      "avionics_adc_{s}", "avionics_loom"])
CIRCUITS += mirrored("air: each engine's bleed to the pack",
                     ["engine_{s}_bleed_valve", "ecs_bleed_{s}", "ecs_pack"])
CIRCUITS += [("and the pack's air to the cockpit, the pack on the keel",
              ["keel", "ecs_mount", "ecs_pack", "ecs_duct", "ecs_diffuser",
               "seat_bulkhead"]),
             ("its heat into the fuel", ["ecs_pack", "ecs_fuel_lines",
                                         "fuel_tank_centre"]),
             ("oxygen: the generator off the pack, to the seat",
              ["ecs_pack", "ecs_oxygen", "ecs_obogs"]),
             ("", ["ecs_oxygen", "seat"])]
CIRCUITS += mirrored("each wheel door is driven by its actuator off the bay",
                     ["gear_bay_main_{s}", "gear_door_act_main_{s}",
                      "gear_door_act_rod_main_{s}", "gear_door_main_{s}"])
CIRCUITS += [("and each nose door", ["gear_bay_nose", "gear_door_act_nose_r",
                                     "gear_door_act_rod_nose_r", "gear_door_nose_r"]),
             ("", ["gear_bay_nose", "gear_door_act_nose_l", "gear_door_act_rod_nose_l",
                   "gear_door_nose_l"]),
             ("the nose leg's up-lock hangs in its bay", ["gear_bay_nose", "gear_uplock_nose"])]
# (each main leg's up-lock is part of its well's structure, gear_bay_main)
CIRCUITS += mirrored("the lights are wired: each wing's navigation light",
                     ["loom_wing_{s}", "nav_light_{s}"])
CIRCUITS += mirrored("each fin's tip light and formation strip",
                     ["loom_fin_{s}", "tail_light_{s}"])
CIRCUITS += mirrored("", ["loom_fin_{s}", "formation_light_{s}"])
CIRCUITS += [("and the forebody's strips",
              ["loom_nose_gear", "loom_formation", "formation_light_r"]),
             ("", ["loom_fuselage_l", "loom_formation", "formation_light_l"]),
             ("ground power to the PDU", ["fuselage_skin", "ground_power", "pdu"])]
CIRCUITS += mirrored("each bay door is driven by its actuators off the bay's wall",
                     ["bay_structure", "bay_door_act_1_{s}", "bay_door_act_rod_1_{s}",
                      "bay_door_{s}"])
CIRCUITS += mirrored("", ["bay_structure", "bay_door_act_2_{s}", "bay_door_act_rod_2_{s}",
                          "bay_door_{s}"])
CIRCUITS += [("the boxes stand on the rack on the nose gear bay's roof",
              ["gear_bay_nose", "avionics_rack", "avionics_fcc_2"])]
CIRCUITS += mirrored("and is carried and driven off the cockpit tub",
                     ["cockpit_tub", "canard_drive_{s}", "canard_spindle_{s}"])
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
for _x in (2250, 3270, 5119, 5680, 6800, 7900, 9000):
    CIRCUITS += mirrored("" if _x != 2250 else "and the forward frames",
                         ["fuselage_skin", f"frame_{_x}"])
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
CIRCUITS += mirrored("and each pivot door is driven by a link off its leg",
                     ["gear_main_{s}", "gear_pivot_link_main_{s}", "gear_pivot_door_main_{s}"])
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
