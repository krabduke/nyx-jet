"""No part of the engine may occupy another part's space.

    python3 tools/audit_intersect.py            (runs itself under Blender)
    python3 tools/audit_intersect.py --shrink   (after a fix: drop what is fixed)

Every pair of parts whose material overlaps by TOL or more -- measured
exactly, both ways, buried parts included; see tools/_interfere.py -- must be
one of two things:

*   Declared in EXPECTED: meant to be that way, a blade root in its disc, a
    vane let into its casing. A rule that excuses nothing, or names a part
    that does not exist, fails the audit.
*   On the KNOWN list: a real defect, written down with its depth and where
    it is. The list only gets shorter. A pair not on it fails, a pair that
    gets deeper fails, and a pair that has been fixed fails until --shrink
    takes it off. --shrink never adds anything.

This engine started with KNOWN empty, and the aim is to keep it that way:
every overlap in it is either a joint, declared below with the reason it is
one, or it is fixed.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Pairs that share material on purpose, by name prefix. Each is a joint, and
# the comment says which.
EXPECTED = [
    # blades are dovetailed or welded into what carries them: 2 mm of root
    # into the drum, disc or rim
    ("blades_hpc_r", "hpc_drum"),
    ("blades_hpt_r", "hpt_disc"),
    ("blades_lpt_r", "lpt_disc"),
    # stator vanes are let 2 mm into the casing they hang from
    ("vanes_fan_", "case_fan"),
    ("vanes_cdfs", "intermediate_case"),
    ("vanes_hpc_", "case_hpc"),
    ("vanes_hpt_ngv", "case_turbine"),
    ("vanes_mtf", "case_turbine"),
    # the compressor exit guide vanes root in the inner case and tip in the
    # combustor case
    ("diffuser", "combustor_inner_case"), ("diffuser", "case_combustor"),
    # frame struts run from their hub ring out through the splitter wall to
    # the outer case, and are welded into all three
    ("fan_frame_struts", "fan_frame_hub"),
    ("fan_frame_struts", "intermediate_case"),
    ("fan_frame_struts", "case_outer_fwd"),
    ("service_struts", "case_turbine"),
    ("service_struts", "intermediate_case"),
    ("service_struts", "case_outer_aft"),
    # a bolted flange collar sits over the two cases it joins, bored to the
    # smaller; the larger is spigoted into it
    ("flange_", "case_"), ("flange_", "inlet_case"),
    # the liner mount pins are let into the liners and the cases
    ("combustor_mount_pins", "combustor_liner_"),
    ("combustor_mount_pins", "case_combustor"),
    ("combustor_mount_pins", "combustor_inner_case"),
    # a fuel nozzle is fitted from outside: its stem passes through every
    # case between the manifold and the dome, and tees off the manifold
    ("fuel_nozzles", "fuel_manifold"),
    ("fuel_nozzles", "case_outer_aft"),
    ("fuel_nozzles", "intermediate_case"),
    ("fuel_nozzles", "case_combustor"),
    # igniters likewise, into a ferrule in the outer liner
    ("igniters", "case_outer_aft"), ("igniters", "intermediate_case"),
    ("igniters", "case_combustor"), ("igniters", "combustor_liner_outer"),
    # reheat fuel: a feed from the manifold through the case and liner into
    # the gas path to zone 1's spray ring
    ("ab_fuel_manifold", "case_outer_aft"),
    ("ab_fuel_manifold", "augmentor_case"),
    ("ab_fuel_manifold", "augmentor_liner"),
    ("ab_fuel_manifold", "ab_spray_rings"),
    # zones 2 and 3: the spraybars come off their manifolds and pass through
    # the outer case, the augmentor case and the liner into the stream
    ("ab_spraybars", "ab_fuel_manifold"), ("ab_spraybars", "case_outer_aft"),
    ("ab_spraybars", "augmentor_case"), ("ab_spraybars", "augmentor_liner"),
    # and end in the spray rings of their zones
    ("ab_spraybars", "ab_spray_rings"),
    # the igniter goes in through a boss on the case to a gutter's wake
    ("ab_igniter", "case_outer_aft"), ("ab_igniter", "augmentor_case"),
    ("ab_igniter", "augmentor_liner"),
    # the reheat control stands on the case and its feeds go into the rings
    ("ab_fuel_control", "case_outer_aft"), ("ab_fuel_control", "ab_fuel_manifold"),
    # the radial gutters are let into the tail cone and the liner
    ("flameholder", "tailcone"), ("flameholder", "augmentor_liner"),
    # the variable-vane spindles pass through the case into each vane
    ("vsv_actuation", "case_hpc"), ("vsv_actuation", "vanes_hpc_"),
    # the mode valve's hinge ring is let into the splitter wall
    ("mode_valve", "intermediate_case"),
    # the tower shaft: its pinion meshes with the bevel on the HP shaft, and
    # it runs out through the hub ring, the hollow bottom strut (solid here),
    # the splitter wall and the outer case into the gearbox
    ("towershaft", "shaft_hp"), ("towershaft", "fan_frame_hub"),
    ("towershaft", "fan_frame_struts"), ("towershaft", "intermediate_case"),
    ("towershaft", "case_outer_fwd"), ("towershaft", "gearbox"),
    # oil: the feed runs up the same strut to the front sump; the scavenge
    # comes in through the bottom service strut and the MTF vane behind it,
    # down the frame's web into the rear sump; both leave from the tank
    ("oil_lines", "gearbox"), ("oil_lines", "oil_tank"),
    ("oil_lines", "fan_frame_hub"), ("oil_lines", "fan_frame_struts"),
    ("oil_lines", "intermediate_case"), ("oil_lines", "case_outer_"),
    ("oil_lines", "sump_front"), ("oil_lines", "sump_rear"),
    ("oil_lines", "service_struts"), ("oil_lines", "case_turbine"),
    ("oil_lines", "vanes_mtf"),
    # everything on the gearbox is bolted to a pad on it
    ("gearbox_mounts", "gearbox"), ("gearbox_mounts", "case_"),
    ("generator_", "gearbox"), ("fuel_pump", "gearbox"),
    ("fuel_metering_unit", "gearbox"), ("oil_tank", "gearbox"),
    # fuel lines leave the metering unit and end in the manifolds
    ("fuel_lines", "fuel_metering_unit"), ("fuel_lines", "fuel_manifold"),
    # the FADECs stand on posts let into the case; the looms plug into
    # the FADECs and the gearbox
    ("fadec_", "case_outer_fwd"), ("harnesses", "fadec_"),
    ("harnesses", "gearbox"),
    # each channel's instrumentation loom plugs into its FADEC's aft face,
    # and each probe's pigtail into the probe's connector; the probes'
    # bosses are let into the case
    ("harness_looms", "fadec_"), ("harness_looms", "sensor_probes"),
    ("sensor_probes", "case_outer_"),
    # mounts are let into the case, the trunnions over the frame's struts
    ("mount_trunnions", "case_outer_fwd"),
    ("mount_trunnions", "fan_frame_struts"),
    ("mount_aft_lug", "case_outer_aft"),
    # the heat exchanger's coolant lines come out through the outer case
    ("coolant_lines", "tms_hx"), ("coolant_lines", "case_outer_aft"),
    # the orthogrid's ribs are machined out of the case they stiffen
    ("case_ribs", "case_outer_"),
    # the mode valve's actuators stand on lugs let into the case, and their
    # rods pass into it through a boss
    ("mode_valve_actuators", "case_outer_fwd"),
    # the hydraulic pump is on a gearbox pad; its lines screw into the pump
    # and into the front swivel motor's ports and the rotary union's
    ("hydraulic_pump", "gearbox"), ("hydraulic_lines", "hydraulic_pump"),
    ("hydraulic_lines", "swivel_drive_1"), ("hydraulic_lines", "swivel_rotary_union"),
    # the swivel: the fixed ring is bolted into the outer case's aft collar;
    # each bearing's race sits between the two flanges it joins, and each
    # motor's pinion is in mesh with its ring gear
    ("flange_outer_aft", "swivel_fixed_ring"),
    ("swivel_bearing_1", "swivel_fixed_ring"),
    ("swivel_bearing_", "swivel_duct_"),
    ("swivel_drive_", "swivel_bearing_"),
    # every line on the case is held in P-clamps: a band round the line on a
    # foot let into the case
    ("line_clamps", "case_outer_"),
    ("line_clamps", "hydraulic_lines"), ("line_clamps", "fuel_lines"),
    ("line_clamps", "oil_lines"), ("line_clamps", "harness_looms"),
    ("swivel_drive_1", "case_outer_aft"),
    # the oblique bearings' motors stand on pads welded to the sloping shell
    # of the duct in front, let into it at their low end
    ("swivel_drive_", "swivel_duct_"),
    ("swivel_rotary_union", "case_outer_aft"),
    # the nozzle: hinge knuckles run through the flaps and seat on the
    # static ring; seals and external flaps are pressed onto the flaps
    ("nozzle_hinges", "nozzle_"),
    ("nozzle_conv_seals", "nozzle_conv_flaps"),
    ("nozzle_div_seals", "nozzle_div_flaps"),
    ("nozzle_ext_flaps", "nozzle_div_flaps"),
    ("nozzle_div_links", "nozzle_div_flaps"), ("nozzle_div_links", "nozzle_ext_flaps"),
    ("nozzle_unison_ring", "nozzle_conv_flaps"),
    # the actuators' head lugs are let into the aft duct; the bellcranks go
    # in through the static ring and their arms into the unison ring
    ("nozzle_actuators", "swivel_duct_aft"),
    ("nozzle_actuators", "nozzle_static_ring"),
    ("nozzle_actuators", "nozzle_unison_ring"),
]

PKG = "engine/parts"
UNIT = 1.0            # mm of real part per model unit
TOL = 0.3             # mm: deeper than this is sharing material

# Real defects, in mm of overlap, deepest first. Fix them and --shrink.
# --- KNOWN: rewritten by --shrink, never by hand to add ---
KNOWN = {
}
# --- end KNOWN ---

if __name__ == "__main__":
    import _interfere
    sys.exit(_interfere.intersect_main(__file__, ROOT, PKG, EXPECTED, KNOWN,
                                       TOL, UNIT))
