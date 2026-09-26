"""No part of the aircraft may occupy another part's space.

    python3 tools/audit_intersect.py            (runs itself under Blender)
    python3 tools/audit_intersect.py --shrink   (after a fix: drop what is fixed)

Every pair of parts whose material overlaps by TOL or more -- measured
exactly, both ways; see tools/_interfere.py -- must be either declared in
EXPECTED (a joint, with the reason) or on the shrink-only KNOWN list.

Each installed engine is held to the engine's own list of joints, vendored
with it (aether/engine_joints.py), with every name prefixed engine_l_ or
engine_r_: a blade root in its disc is a joint in the aircraft exactly as it
is in the engine. Anything between the two engines, or between an engine
and the airframe, has to be declared here.
"""
import importlib.util
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def _engine_rules():
    p = os.path.join(ROOT, "aether", "engine_joints.py")
    spec = importlib.util.spec_from_file_location("engine_joints", p)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    out = []
    for side in ("l", "r"):
        for a, b in m.EXPECTED:
            out.append((f"engine_{side}_{a}", f"engine_{side}_{b}"))
    return out


AIRFRAME = [
    # the wing runs into the body: its root panel is the carry-through
    ("wing_", "fuselage_skin"),
    # control surfaces hang on hinge fittings let into the surface they move on
    ("le_flap_", "wing_"), ("flaperon_", "wing_"), ("rudder_", "fin_"),
    # each flaperon's and rudder's actuator: its trunnion is let into its
    # pocket's front wall, and its rod end's eye is on the horn lug
    ("flaperon_act_in_", "flaperon_in_"), ("flaperon_act_out_", "flaperon_out_"),
    ("rudder_act_", "rudder_"), ("le_flap_act_", "le_flap_"),
    # fin roots are let into the skin, and their front spars into the frames
    ("fin_", "fuselage_skin"), ("fin_", "frame_"),
    # each canard turns on a spindle through the skin into its root
    ("canard_spindle_", "canard_"), ("canard_spindle_", "fuselage_skin"),
    # its drive: the pivot beam is let into the tub wall and the skin, the
    # bearings and the crank are on the spindle, and the actuator's anchor
    # bracket is let into the tub wall
    ("canard_drive_", "canard_spindle_"), ("canard_drive_", "cockpit_tub"),
    ("canard_drive_", "fuselage_skin"),
    # and the canards' pivot beams are bolted through the canard frame
    ("canard_drive_", "frame_5119"),
    # every frame is riveted to the skin: its outer face is let into it
    ("frame_", "fuselage_skin"),
    # the fuselage's fuel cells hang from the frames at their ends, let into
    # them; the wing tanks are integral, half a millimetre into the skins
    # that close them
    ("fuel_tank_fwd_", "frame_"), ("fuel_tank_centre", "frame_9000"),
    ("fuel_tank_wing_", "wing_"),
    # each engine's feed: the boost pump's housing is in the collector
    # tank's aft wall, and the line is pushed onto the engine's inlet union
    ("fuel_feed_", "fuel_tank_centre"), ("fuel_feed_r", "engine_r_fuel_pump"),
    ("fuel_feed_l", "engine_l_fuel_pump"),
    # the gallery runs inside the cells and through the collector's wall;
    # the receptacle's line comes down through the forward cell into it
    # electrical power: the PDU is bolted to the keel's face; every loom
    # is plugged into what it connects -- the PDU, each engine's inboard
    # generator, each actuator -- and the fuselage's looms into the wings'
    # and fins' at their roots
    ("pdu", "keel"), ("loom_fuselage_", "pdu"),
    ("loom_fuselage_r", "engine_r_generator_r"),
    ("loom_fuselage_l", "engine_l_generator_l"),
    ("loom_fuselage_", "canard_drive_"), ("loom_nose_gear", "gear_actuator_nose"),
    ("loom_nose_gear", "loom_fuselage_r"),
    ("loom_fuselage_", "loom_fin_"),
    # the canopy's mechanism: the rim frame is bonded into the glass's foot;
    # the hinge beam bolts across frame 5680's cut ends and its pin is
    # through the rim's lug; each actuator's rod runs in its body and its
    # bracket is on the tub's aft wall; the lock hooks hang from the skin
    # the pitot and static lines start in their probe's root and end in the
    # front of their air data computer
    ("air_data_probe_", "avionics_pitot_lines_"),
    ("avionics_adc_", "avionics_pitot_lines_"),
    # the avionics loom comes off the nose gear's run of the starboard loom
    ("avionics_loom", "loom_nose_gear"),
    # the air conditioning: each bleed duct on its engine's bleed flange and
    # into the pack's air-cycle machine; the duct out of the pack's
    # separator and into the diffuser on the seat bulkhead's back; the
    # pack's fuel lines into the centre tank's aft wall; its mount on the
    # keel; the oxygen generator's feed off the separator and its hose from
    # the generator into the seat's connector
    ("ecs_bleed_", "engine_"),
    ("ecs_duct", "ecs_pack"), ("ecs_duct", "ecs_diffuser"),
    ("ecs_diffuser", "seat_bulkhead"), ("ecs_fuel_lines", "fuel_tank_centre"),
    ("ecs_fuel_lines", "ecs_pack"), ("ecs_mount", "keel"), ("ecs_mount", "ecs_pack"),
    ("ecs_bleed_", "ecs_pack"),
    ("ecs_obogs", "ecs_oxygen"), ("ecs_oxygen", "ecs_pack"), ("ecs_oxygen", "seat"),
    # each gear door's actuator: bracketed to its bay's wall, its rod running
    # in its body and pinned through the horn on the door
    ("gear_bay_", "gear_door_act_"), ("gear_door_act_", "gear_door_act_rod_"),
    ("gear_bay_nose", "gear_uplock_nose"),
    ("gear_door_act_rod_", "gear_door_"),
    # and each weapons-bay door's two, off the bay's side walls
    ("bay_door_act_", "bay_structure"), ("bay_door_act_", "bay_door_act_rod_"),
    ("bay_door_act_rod_", "bay_door_"),
    # the looms' ends in the lights: each wing's in its tip's navigation
    # light, each fin's in its tip light (the formation strips' feeds end
    # just under the skin beneath them)
    ("loom_wing_", "nav_light_"), ("loom_fin_", "tail_light_"),
    ("loom_formation", "loom_nose_gear"), ("loom_formation", "loom_fuselage_l"),
    # the ground power receptacle's cable into the PDU, its feet on the skin
    ("ground_power", "pdu"), ("fuselage_skin", "ground_power"),
    ("canopy_glass", "canopy_rim"), ("canopy_hinge", "canopy_rim"),
    ("canopy_hinge", "frame_5680"), ("canopy_actuator_", "canopy_actuator_rod_"),
    ("canopy_actuator_", "cockpit_tub"), ("canopy_locks", "fuselage_skin"),
    ("loom_wing_", "flaperon_act_"), ("loom_wing_", "le_flap_act_"),
    ("loom_wing_", "gear_actuator_main_"), ("loom_fin_", "rudder_act_"),
    ("fuel_gallery", "fuel_tank_"), ("refuel_receptacle", "fuel_tank_fwd_1"),
    ("refuel_receptacle", "fuel_gallery"),
    # the seat's rails are bolted to the bulkhead behind it, which is set
    # into the tub's walls and floor
    ("seat", "seat_bulkhead"), ("seat_bulkhead", "cockpit_tub"),
    # the keel passes through the frames' arches
    ("frame_", "keel"),
    # engine mounts: beams from the keel and the forward frame to the
    # trunnions, and thrust links from the aft lugs up into the aft frame
    ("engine_mounts", "keel"), ("engine_mounts", "frame_"),
    ("engine_mounts", "engine_l_mount_"), ("engine_mounts", "engine_r_mount_"),
    # the radar array stands on its bulkhead
    ("radar_array", "radar_bulkhead"),
    # launchers hang from the bay roof and hold the missiles on their rails
    ("launchers", "missiles"), ("launchers", "bay_structure"),
    # landing gear: legs into their bay roofs, axles through the hubs, tyres
    # on the hubs
    ("gear_nose", "gear_bay_nose"),
    # each leg's rotary actuator: its output spline is in the trunnion's
    # end, its body in the bay's wall or the wing's pocket
    ("gear_actuator_nose", "gear_nose"), ("gear_actuator_nose", "gear_bay_nose"),
    ("gear_actuator_main_", "gear_main_"), ("gear_actuator_main_", "wing_"),
    ("wheels_nose", "gear_nose"), ("wheel_main_", "gear_main_"),
    ("tyres_nose", "wheels_nose"), ("tyre_main_", "wheel_main_"),
    # the canopy's rim is pressed into its seal on the skin
    ("canopy_glass", "fuselage_skin"),
    # and the sill rails either side carry that seal: let into the skin,
    # the glass's edge pressed into them
    ("canopy_frame_", "fuselage_skin"), ("canopy_frame_", "canopy_glass"),
    # the gear doors' hinges are let into the skin at the bay edges
    ("gear_door_", "fuselage_skin"),
    # each main leg's trunnion runs in fittings let into the wing's root,
    # the well's walls meet the wing's root rib, and the leg's door hangs on
    # brackets from the leg
    ("gear_main_", "wing_"), ("gear_bay_main_", "wing_"),
    ("gear_leg_door_main_", "gear_main_"),
    # the pivot doors' hinge knuckles are let into the slot's edge
    ("gear_pivot_door_main_", "wing_"),
    # cockpit furniture is bolted to the tub
    ("seat", "cockpit_tub"), ("cockpit_panel", "cockpit_tub"),
    ("cockpit_controls", "cockpit_tub"), ("cockpit_hud", "cockpit_panel"),
    # the details: the aft closure is fastened inside the skin's lip; the
    # IRST housing, probes, vanes and blade antennas stand on bases let into
    # the skin, and the IRST's ball window is set into its housing's nose;
    # the lights are set into the wing and fin tips, and the wicks are
    # rooted in the trailing edges they bleed static from
    ("aft_closure", "fuselage_skin"),
    ("irst_", "fuselage_skin"), ("irst_window", "irst_fairing"),
    ("air_data_probe_", "fuselage_skin"), ("aoa_vane_", "fuselage_skin"),
    ("antenna_", "fuselage_skin"),
    ("nav_light_", "wing_"), ("tail_light_", "fin_"),
    ("static_wicks_", "wing_"), ("static_wicks_", "fin_"),
    # the gun hangs from the skin on two posts let into it
    ("gun", "fuselage_skin"),
    # the panel seams are the sealant in the gaps between skin panels
    ("panel_seams", "fuselage_skin"), ("formation_light_", "fuselage_skin"),
    ("formation_light_", "fin_"),
]

EXPECTED = AIRFRAME + _engine_rules()

PKG = "nyx/parts"
UNIT = 1.0
TOL = 0.3

# --- KNOWN: rewritten by --shrink, never by hand to add ---
KNOWN = {
}
# --- end KNOWN ---

if __name__ == "__main__":
    import _interfere
    sys.exit(_interfere.intersect_main(__file__, ROOT, PKG, EXPECTED, KNOWN,
                                       TOL, UNIT))
