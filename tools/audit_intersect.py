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
