"""
Nyx -- a twin-engine, canard-delta, blended-body agile fighter.
Master specification.

ALL dimensions are millimetres, full size. Axes: +X aft from the nose tip,
+Y to starboard, +Z up. The body's datum (z = 0) is the chine plane, which is
also the wing's chord plane at the root. Masses in kg.

WHAT IT IS FOR
--------------
The brief is agility first: turn rate, not top speed. Everything below follows
from that one requirement.

  * Low wing loading. Turn rate at a given speed is lift over mass, so the
    wing is big for the aircraft's weight: 68 m^2 of reference area on a
    17.2 t combat mass, about 250 kg/m^2 -- lower than any current fighter.
  * Thrust. Two Aether AX-1 engines at 135 kN each on reheat give a combat
    thrust-to-weight of about 1.6, so a hard turn can be held without
    bleeding speed.
  * A short, wide airframe. Pitch and yaw inertia go with length squared;
    a 14.6 m aircraft with its mass close in turns faster than a 19 m one
    for the same control power. The width goes into a lifting body -- the
    fuselage between the wings makes lift -- rather than into length.
  * Relaxed stability. The centre of gravity sits behind the neutral point
    subsonically, so the aircraft wants to pitch up on its own and the
    control system only has to let it. That is what gives instant pitch
    response; supersonic the neutral point moves aft and it is stable again.
  * Close-coupled canards. All-moving foreplanes just ahead of and above the
    wing trim the unstable airframe with a lifting force (a tail would trim
    with a download), and their vortex energises the wing at high angle of
    attack.
  * Thrust vectoring. Each engine ends in a three-bearing swivel duct and a
    round C-D nozzle behind the airframe: the jet points anywhere in a cone
    round the axis, together or differentially, which gives control where
    the surfaces have none -- past the stall.
  * Stealth shaping that does not cost agility: chined nose, caret intakes
    with serpentine ducts hiding the fans, internal weapons, canted fins,
    and sawtooth edges.

Everything aerodynamic here is checked by aero/: area rule, wing loading,
static margin from a vortex-lattice solve, thrust-to-weight and turn rates.
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

NAME = "Nyx"
ROLE = "twin-engine canard-delta agile fighter"

# --------------------------------------------------------------------------
# Engines (vendored Aether AX-1) -- where they sit
# --------------------------------------------------------------------------

# The engine's own frame has its fan face at x = 0 on its axis. It is placed
# so the body ends just ahead of the swivel duct's front bearing: everything
# behind that bearing turns when the jet is vectored, so the airframe has to
# stop there and leave the swivel and the nozzle out in the air behind it.
ENGINE_FAN_FACE_X = 10390.0
ENGINE_BEARING_1 = 2880.0     # the engine's front swivel bearing, from the fan
ENGINE_Y = 800.0              # each engine's axis is this far off centre
ENGINE_Z = 0.0
# the vendored engine's key figures, read from its own spec at import time
# by nyx/parts/engines.py; repeated here only for the mass table
ENGINE_MASS = 1590.0
ENGINE_CG_FROM_FAN = 1650.0   # mm aft of the fan face

# --------------------------------------------------------------------------
# Body (outer mould line)
#
# The body is lofted through cross-sections. Each section has a sharp chine
# at half-width w and height zc -- the edge that runs from the nose into the
# wing leading edge and is the aircraft's radar signature control and its
# vortex-lift strake -- and upper and lower surfaces that fall from the
# crown zt and the keel zb to the chine:
#
#     z_up(y) = zc + (zt - zc) (1 - |y/w|^M_UP)^(1/N_UP)
#     z_dn(y) = zc - (zc - zb) (1 - |y/w|^M_DN)^(1/N_DN)
#
# The table is (x, w, zc, zt, zb). Between stations the values are joined by
# a monotone cubic, so the lines are fair: no station leaves a crease.
# --------------------------------------------------------------------------

BODY = [
    # x        w      zc     zt     zb
    (0.0,      0.0,   60.0,  60.0,  60.0),
    (150.0,  150.0,   64.0, 160.0,  -20.0),
    (500.0,  300.0,   70.0, 290.0, -110.0),
    (1200.0, 480.0,   70.0, 420.0, -200.0),
    (2200.0, 680.0,   60.0, 560.0, -300.0),
    (3200.0, 880.0,   50.0, 680.0, -390.0),
    (4200.0, 1160.0,  40.0, 770.0, -500.0),
    (5200.0, 1700.0,  30.0, 800.0, -610.0),
    (6200.0, 2180.0,  20.0, 770.0, -690.0),
    (7200.0, 2280.0,  10.0, 730.0, -740.0),
    (8400.0, 2320.0,   0.0, 680.0, -760.0),
    (9800.0, 2360.0,   0.0, 700.0, -780.0),
    (11200.0, 2200.0,  0.0, 730.0, -800.0),
    # Aft of the wing the body closes in on the two nacelles (shapes.py):
    # 1,560 mm from the centreline is the outside of a nacelle, which is as
    # far as the chine can come in.
    (12200.0, 1840.0,  0.0, 720.0, -790.0),
    (13240.0, 1560.0,  0.0, 690.0, -800.0),
]
BODY_END_X = BODY[-1][0]
M_UP, N_UP = 2.2, 1.6
M_DN, N_DN = 2.4, 1.5
SKIN_T = 10.0                 # skin thickness, mm (outer mould line inward)
SKIN_EDGE = 70.0              # the chine is solid this far in from its edge
RADOME_X = 420.0              # the skin's inner surface starts at the
                              # radome bulkhead; the nose ahead is solid

# --------------------------------------------------------------------------
# Wing -- a clipped delta, mid-set on the chine plane
# --------------------------------------------------------------------------

WING = {
    "y_root": 1950.0,         # where the exposed panel starts (inside body)
    "x_le_root": 6100.0,
    "c_root": 6750.0,
    "y_tip": 6600.0,
    "c_tip": 1300.0,
    "le_sweep": 48.0,         # deg
    "tc_root": 0.050,
    "tc_tip": 0.035,
    "twist_tip": -2.0,        # deg, washout at the tip
    "dihedral": -1.5,         # deg -- slight anhedral: a delta is already
                              # stable enough in roll, and anhedral quickens it
    "embed": 260.0,           # root section runs this far into the body
    # trailing-edge flaperons (inboard, outboard): y range, chord fraction
    "flaperons": ((2300.0, 4300.0), (4300.0, 6300.0)),
    "flaperon_cf": 0.22,
    # leading-edge flap, full exposed span
    "le_flap": (2300.0, 6400.0),
    "le_flap_cf": 0.12,
    "hinge_gap": 0.15,        # the seat between a surface and its wing
}


def wing_le_x(y):
    return WING["x_le_root"] + (y - WING["y_root"]) * math.tan(math.radians(WING["le_sweep"]))


def wing_chord(y):
    t = (y - WING["y_root"]) / (WING["y_tip"] - WING["y_root"])
    return WING["c_root"] + (WING["c_tip"] - WING["c_root"]) * t


# --------------------------------------------------------------------------
# Canards -- close-coupled, all-moving, above the wing plane
# --------------------------------------------------------------------------

CANARD = {
    "y_root": 1000.0,         # inside the body side
    "x_le_root": 4300.0,
    "c_root": 1950.0,
    "y_tip": 3100.0,
    "c_tip": 560.0,
    "le_sweep": 50.0,
    "z": 330.0,               # above the chine plane
    "tc": 0.045,
    "pivot_frac": 0.42,       # spindle at 42 % of the root chord
    "dihedral": 0.0,
}

# --------------------------------------------------------------------------
# Fins -- twin, canted out, on the upper body over the engines
# --------------------------------------------------------------------------

FIN = {
    "y_root": 1380.0,
    "x_le_root": 10550.0,
    "c_root": 2450.0,
    "span": 2050.0,           # along the cant
    "c_tip": 820.0,
    "le_sweep": 45.0,
    "cant": 32.0,             # deg out from vertical
    "tc": 0.045,
    "rudder_cf": 0.30,
    "rudder_span": (0.12, 0.92),
    "embed": 40.0,
}

# --------------------------------------------------------------------------
# Intakes -- caret mouths under the chine, serpentine ducts to the fans
# --------------------------------------------------------------------------

INTAKE = {
    "x_mouth": 5000.0,        # lip station at the mouth's centre
    "y_mouth": 1300.0,        # mouth centre, off the centreline
    "z_mouth": -330.0,
    "w_mouth": 760.0,         # mouth width
    "h_mouth": 820.0,         # mouth height
    "lip_sweep": 38.0,        # deg: the caret lip is swept in plan and side
    "mouth_taper": 0.20,      # the outboard lower corner is cut in by this
                              # fraction of the width: a trapezoid mouth
    "mouth_chamfer": 0.18,
    "wall": 10.0,
    # the duct ends on the engine's inlet case flange
    "x_end": ENGINE_FAN_FACE_X - 190.0,
}

# --------------------------------------------------------------------------
# Cockpit and canopy
# --------------------------------------------------------------------------

COCKPIT = {
    "x0": 2900.0, "x1": 5900.0,       # canopy opening in the skin
    "half_w": 430.0,
    "sill_z": 560.0,                  # canopy sill height
    "canopy_top_z": 1180.0,
    "eye_x": 4150.0, "eye_z": 1010.0,
    "seat_x": 4450.0,
}

# --------------------------------------------------------------------------
# Weapons bay -- on the centreline between the intake ducts
# --------------------------------------------------------------------------

BAY = {
    # the ducts close to 485 mm off the centreline by the bay's aft end, so
    # the bay stops there and is as wide as they leave
    "x0": 5250.0, "x1": 9050.0,
    "half_w": 405.0,
    "z_roof": -150.0,
    "n_missiles": 4,
    "missile_len": 3650.0,
    "missile_d": 178.0,
    "door_t": 12.0,
}

# --------------------------------------------------------------------------
# Landing gear -- built down, on the ground; it retracts (parts/gear.py)
# --------------------------------------------------------------------------

# The stance is set by where the gear can go when it is up. At 2.05 m to the
# chine plane the legs were too long to stow anywhere in the airframe; at
# 1.54 m the main legs fold into the wing roots and the nose leg under the
# nose, the belly is still 0.74 m off the ground, and the tail clears it by
# 12 degrees of rotation at take-off.
GROUND_Z = -1540.0
GEAR = {
    "nose_x": 3350.0,
    # about 700 mm behind the aft-most CG and 11 % of the weight on the nose
    # wheel, both inside the usual bands (verify.py measures both)
    "main_x": 9500.0,
    # the nose leg pivots under the cockpit floor and folds forward; its
    # bay is tall forward of the split, where the wheels go, and shallow
    # aft of it, under the floor, where only the leg lies
    "nose_pivot_z": 40.0,
    "nose_bay": (1750.0, 3440.0, 205.0),      # x0, x1, half-width
    "nose_bay_split": 2330.0,
    "nose_roofs": (335.0, 100.0),       # the cockpit floor is at 115
    # each main leg pivots in the wing root and folds inboard; its wheel
    # lies in a well in the wing-body blend, between the intake duct and
    # the wing's root, the leg in a slot in the wing
    # the pivot high in the wing, so the stowed leg stays inside the wing's
    # thickness all the way in, and far enough out that the wheel lands in
    # its well clear of the duct
    "main_pivot": (2968.0, 128.0),            # y, z
    "main_stow_z": -150.0,                    # the wheel's centre, stowed
    # (its fore and aft edges are serrated 70 deep, and the teeth must clear
    # the 700 mm wheel lying in it)
    "main_well": (9070.0, 9930.0, 1318.0, 2050.0),   # x0, x1, y0, y1
    "main_slot_hw": 118.0,
    "main_roofs": (172.0, 152.0),             # well, slot
    "nose_wheel_r": 260.0, "nose_wheel_w": 150.0,
    "main_wheel_r": 350.0, "main_wheel_w": 230.0,
    "strut_r_nose": 50.0, "strut_r_main": 62.0,
}

# --------------------------------------------------------------------------
# Mass and balance (kg, CG x mm). Combat condition: half internal fuel and
# four medium-range missiles in the bay. Items are estimated from the
# component-weight fractions of current fighters; the fuel tanks are in this
# table but are not modelled as parts.
# --------------------------------------------------------------------------

MASS_EMPTY = [
    # item, kg, x_cg
    ("fuselage structure", 3700.0, 8000.0),
    ("wing", 1950.0, 9250.0),
    ("canards", 190.0, 6150.0),
    ("fins", 270.0, 12550.0),
    ("landing gear", 610.0, 9450.0),
    ("engines (2)", 2 * ENGINE_MASS, ENGINE_FAN_FACE_X + ENGINE_CG_FROM_FAN),
    ("engine installation, ducts", 650.0, 8200.0),
    ("fuel system", 380.0, 8800.0),
    ("hydraulics, electrics, ECS", 1100.0, 7600.0),
    ("radar and avionics", 870.0, 2100.0),
    ("cockpit and ejection seat", 260.0, 4300.0),
]
def _fuel_tanks():
    """[(tank, full kg, x_cg)] -- what the built tanks hold, as measured by
    tools/measure_fuel.py into fuel_tanks.json. This was three typed-in
    numbers, and the wings between their spars hold a third of the 2,800 kg
    they were given, while the three cells over the bay hold more than twice
    the 1,700 kg the forward fuselage was."""
    import json
    t = json.load(open(os.path.join(HERE, "fuel_tanks.json")))
    return [(n, v["kg"], v["x_cg_mm"]) for n, v in sorted(t.items())]


FUEL_TANKS = _fuel_tanks()

# The order the tanks are used in, which is how the CG is kept where the
# control laws want it: the forwardmost cell first, then the wings, then the
# other cells, and last the centre tank, which is the collector the engines
# feed from. Burned evenly, the CG ran from -1.5 % of the mean chord full to
# -4 % nearly empty and was -2.7 % at combat weight; in this order it is
# -4.7 to -6.1 % from three-quarters full down, and -4.8 % at combat weight.
BURN_ORDER = ["fuel_tank_fwd_1", "fuel_tank_wing_l", "fuel_tank_wing_r",
              "fuel_tank_fwd_2", "fuel_tank_fwd_3", "fuel_tank_centre"]
PAYLOAD = [
    ("pilot", 110.0, 4200.0),
    ("4 medium-range missiles", 4 * 160.0, 7850.0),
]
COMBAT_FUEL_FRACTION = 0.5
G_LIMIT = 9.5


def mass_table(fuel_fraction=COMBAT_FUEL_FRACTION):
    """The aircraft's masses with this fraction of its fuel left, the fuel
    burned in BURN_ORDER: the tanks used last are the ones still full."""
    rows = list(MASS_EMPTY) + list(PAYLOAD)
    tanks = {n: (m, x) for (n, m, x) in FUEL_TANKS}
    left = fuel_fraction * sum(m for (m, _) in tanks.values())
    for n in reversed(BURN_ORDER):
        m, x = tanks[n]
        kg = min(left, m)
        left -= kg
        rows.append((n, kg, x))
    return rows


def mass(fuel_fraction=COMBAT_FUEL_FRACTION):
    return sum(m for _, m, _ in mass_table(fuel_fraction))


def cg_x(fuel_fraction=COMBAT_FUEL_FRACTION):
    rows = mass_table(fuel_fraction)
    return sum(m * x for _, m, x in rows) / sum(m for _, m, _ in rows)


def empty_mass():
    return sum(m for _, m, _ in MASS_EMPTY)


# --------------------------------------------------------------------------
# Reference wing (the planform continued to the centreline)
# --------------------------------------------------------------------------

def ref_planform():
    """Root chord at the centreline, tip chord, semi-span, LE x at y = 0."""
    y0 = WING["y_root"]
    t = math.tan(math.radians(WING["le_sweep"]))
    x_le0 = WING["x_le_root"] - y0 * t
    # the trailing edge is continued straight in to the centreline
    te_root = WING["x_le_root"] + WING["c_root"]
    te_tip = wing_le_x(WING["y_tip"]) + WING["c_tip"]
    slope = (te_tip - te_root) / (WING["y_tip"] - y0)
    te0 = te_root - y0 * slope
    return {"c0": te0 - x_le0, "ct": WING["c_tip"], "b2": WING["y_tip"],
            "x_le0": x_le0, "le_tan": t, "te_slope": slope}


def s_ref_m2():
    p = ref_planform()
    return (p["c0"] + p["ct"]) * p["b2"] * 1e-6


def mac():
    """(MAC length, x of its leading edge, its spanwise station), mm."""
    p = ref_planform()
    lam = p["ct"] / p["c0"]
    c = 2.0 / 3.0 * p["c0"] * (1 + lam + lam * lam) / (1 + lam)
    y = p["b2"] / 3.0 * (1 + 2 * lam) / (1 + lam)
    return c, p["x_le0"] + y * p["le_tan"], y


# --------------------------------------------------------------------------
# Materials
# --------------------------------------------------------------------------

MATERIAL_MAP = {
    "fuselage": "skin",
    "aft_closure": "skin_dark",
    "irst_fairing": "skin",
    "irst_window": "sensor_glass",
    "air_data_probe": "steel",
    "aoa_vane": "steel",
    "antenna": "skin_dark",
    "nav_light_l": "lens_red",
    "nav_light_r": "lens_green",
    "tail_light": "lens_white",
    "static_wicks": "rubber",
    "panel_seams": "seam",
    "formation_light": "lens_formation",
    "wing": "skin",
    "flaperon": "skin_dark",
    "le_flap": "skin_dark",
    "canard": "skin",
    "fin": "skin",
    "rudder": "skin_dark",
    "intake": "skin",
    "duct": "duct",
    "bay": "structure",
    "bay_door": "skin_dark",
    "missile": "missile",
    "launcher": "steel",
    "canopy_glass": "canopy",
    "canopy_frame": "skin_dark",
    "cockpit": "cockpit",
    "seat": "seat",
    "seat_bulkhead": "structure",
    "gear": "steel",
    "tyre": "rubber",
    "wheel": "wheel",
    "gear_door": "skin_dark",
    "gear_leg_door": "skin_dark",
    "gear_pivot_door": "skin_dark",
    "frame": "structure",
    "longeron": "structure",
    "keel": "structure",
    "spar": "structure",
    "engine_mount": "steel",
    "fuel_tank": "tank",
    "radar": "radar",
    "radome": "skin",
    "pitot": "steel",
    "gun": "steel",
    "canard_spindle": "steel",
    "canard_drive": "steel",
    "flaperon_act": "steel",
    "le_flap_act": "steel",
    "fuel_feed": "steel",
    "rudder_act": "steel",
}
DEFAULT_MATERIAL = "structure"

PALETTE = {
    # a matt grey-blue radar-absorbent finish, and a darker one for the
    # moving surfaces and door edges
    "skin":      ((0.105, 0.115, 0.128), 0.10, 0.62),
    "skin_dark": ((0.070, 0.076, 0.086), 0.10, 0.58),
    "duct":      ((0.72, 0.72, 0.70), 0.00, 0.60),
    "structure": ((0.42, 0.43, 0.45), 1.00, 0.40),
    "steel":     ((0.52, 0.53, 0.55), 1.00, 0.20),
    "missile":   ((0.62, 0.63, 0.62), 0.00, 0.45),
    "canopy":    ((0.60, 0.45, 0.20), 0.60, 0.06),
    "cockpit":   ((0.040, 0.042, 0.046), 0.00, 0.70),
    "seat":      ((0.10, 0.10, 0.09), 0.00, 0.75),
    "rubber":    ((0.030, 0.030, 0.032), 0.00, 0.90),
    "wheel":     ((0.40, 0.41, 0.42), 1.00, 0.35),
    "tank":      ((0.30, 0.30, 0.20), 0.00, 0.60),
    "radar":     ((0.35, 0.33, 0.30), 0.80, 0.35),
    # lenses, and the IRST's glass: sapphire-dark, with a gold coat
    "lens_red":     ((0.70, 0.04, 0.03), 0.00, 0.10),
    "lens_green":   ((0.03, 0.55, 0.12), 0.00, 0.10),
    "lens_white":   ((0.85, 0.85, 0.82), 0.00, 0.08),
    # electroluminescent strip, unlit: a pale green-yellow panel
    "lens_formation": ((0.42, 0.50, 0.24), 0.00, 0.35),
    "sensor_glass": ((0.22, 0.17, 0.08), 0.70, 0.05),
    # the gaps between panels, filled with sealant a shade darker than skin
    "seam":         ((0.055, 0.060, 0.068), 0.10, 0.70),
}

RES = {"body_rings": 150, "body_ring_pts": 128, "wing_chord_pts": 48,
       "wing_span_pts": 14, "pipe": 16, "revolve": 48}
