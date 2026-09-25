"""
Aether AX-1 -- adaptive-cycle three-stream afterburning turbofan.
Master dimensional specification.

ALL dimensions are millimetres. ALL angles are degrees. The engine axis is +X,
station 0.0 at the leading edge of the first fan rotor at its hub, looking
downstream. Radius is measured from the axis. +Z is up: the gearbox hangs underneath,
at -Z, and clock angles are measured from +Y towards +Z.

Every other module consumes this file. No geometry module may contain a
literal dimension; if a number describes the engine, it belongs here.

THIS IS AN ORIGINAL DESIGN
--------------------------
It is not a copy of any production engine. The architecture is chosen for one
job: powering a twin-engine fighter whose design goal is agility rather than
top speed (see the Nyx repo). That asks for

  * a lot of thrust for its size and weight -- so a hot, high-pressure core
    (T4 2,050 K, OPR ~37) behind a compact two-stage fan;
  * good part-power fuel burn, because an agile fighter spends its life at
    part throttle between hard manoeuvres -- so an ADAPTIVE CYCLE: a
    core-driven fan stage (CDFS) on the HP spool and a third bypass stream
    behind a mode valve, so the bypass ratio can be moved without moving the
    fan;
  * a heat sink and electrical power for sensors -- the third stream carries
    a heat exchanger and the HP spool drives two generators;
  * thrust vectoring for post-stall control, and a vertical landing -- so
    the exhaust ends in a three-bearing swivel duct that folds the jet up
    to 95 degrees down, with a round convergent-divergent nozzle on it.

WHERE THE NUMBERS COME FROM
---------------------------
`cycle.py` is the thermodynamic design point. It fixes airflow, the pressure
ratios, the (solved) bypass ratio, thrust and the nozzle throat area. This
file draws a flowpath to carry those flows, and `verify.py` recomputes the
axial Mach number the flowpath implies at every station from the cycle's mass
flows, and each blade row's solidity from its count and chord, and fails the
build if any is outside its band. Stage counts, blade counts, chords and twist
follow ordinary turbomachinery practice. They are engineering-plausible, not
the data of any real engine.
"""

import math
from dataclasses import dataclass

import cycle

ENGINE_NAME = "Aether AX-1"
ENGINE_TAGLINE = ("adaptive-cycle three-stream reheated turbofan "
                  "with a three-bearing swivel nozzle")

# --------------------------------------------------------------------------
# Headline figures -- taken from the cycle, not typed in
# --------------------------------------------------------------------------

CYCLE = cycle.design()

AIRFLOW_KG_S = CYCLE["w"]
THRUST_DRY_N = round(CYCLE["thrust_dry"], -2)
THRUST_AB_N = round(CYCLE["thrust_wet"], -2)
BYPASS_RATIO = round(CYCLE["bpr2"], 3)          # second stream / core
BYPASS_RATIO_TOTAL = round(CYCLE["bpr_total"], 3)
OVERALL_PRESSURE_RATIO = round(CYCLE["opr"], 1)
FAN_PRESSURE_RATIO = cycle.DESIGN["fpr"]
T4_K = cycle.DESIGN["t4"]

N_FAN_STAGES = 2
N_CDFS_STAGES = 1
N_HPC_STAGES = 6
N_HPT_STAGES = 1
N_LPT_STAGES = 1

# Dry mass, estimated rather than measured, from the component-weight
# fractions of a modern reheated turbofan scaled to 112 kg/s: 1,420 kg for
# the engine to the augmentor's end, the CMC hot section and the blisks
# buying the top of the current class. The three-bearing swivel duct and its
# round C-D nozzle add 170 kg over the 2D nozzle they replaced -- three
# bearings with ring gears and three motors, and a metre of double-walled
# duct -- which is the price of landing vertically.
DRY_WEIGHT_KG = 1590.0

# How far the swivel duct folds the jet off the axis: 4 * beta.
SWIVEL_DEG = 95.0

# --------------------------------------------------------------------------
# Resolution
# --------------------------------------------------------------------------

RES = {
    "revolve_segments": 160,
    "small_revolve": 36,
    "airfoil_chord_pts": 34,
    "airfoil_span_pts": 11,
    "pipe_segments": 18,
}

# --------------------------------------------------------------------------
# Flowpath (DERIVED from the cycle -- see module docstring)
#
# Each stream is a table of (x, inner radius, outer radius). Everything that
# bounds the gas -- blade tips, platforms, casings, splitters, cowls -- is
# built off these tables, so moving one number here moves the wall, the
# blades against it and the Mach check together.
# --------------------------------------------------------------------------

# Fan, face to exit. Hub/tip 0.30 at the face; tip radius from a specific
# flow of ~205 kg/s per m^2 of annulus, which is what a modern fighter fan
# swallows at take-off: 112 kg/s / 205 = 0.546 m^2 -> r_tip 437 mm. The
# annulus contracts hard across the two stages because the air leaving is
# 2.6 times denser than the air arriving.
FAN_PATH = [
    (-8.0,   128.0, 437.0),
    (0.0,    131.0, 437.0),
    (135.0,  196.0, 431.0),
    (157.0,  204.0, 429.0),
    (221.0,  222.0, 426.0),
    (243.0,  230.0, 424.0),
    (328.0,  262.0, 417.0),
    (348.0,  272.0, 414.0),
    (408.0,  292.0, 406.0),
    (425.0,  294.0, 405.0),
]

# Where the third stream is peeled off the top of the fan exit, and at what
# radius. The splitter is sized for the third stream's CRUISE share (about a
# fifth of the fan's annulus) -- at maximum thrust the mode valve behind it
# closes down to the 10 % the cycle carries.
SPLITTER3_X = 425.0
SPLITTER3_R = 384.0

# Inner flow (core + second stream) from the third-stream splitter, under the
# fan frame and through the core-driven fan stage to the core splitter.
CDFS_PATH = [
    (425.0,  294.0, 384.0),
    (555.0,  294.0, 384.0),
    (614.0,  298.0, 381.0),
    (634.0,  300.0, 380.0),
    (678.0,  302.0, 377.0),
    (695.0,  302.0, 376.0),
]

# Core splitter. Its radius divides the CDFS exit annulus in the cycle's
# core : second-stream ratio (62 : 39 kg/s) at equal flux.
SPLITTER2_X = 695.0
SPLITTER2_R = 349.0

# Core, splitter to LP turbine exit. A swan neck drops it 34 mm onto the HP
# compressor, whose annulus closes from 48 mm of span to 14 mm as the air is
# squeezed 7.4 to 1.
CORE_PATH = [
    (695.0,  302.0, 349.0),
    (740.0,  280.0, 330.0),
    (780.0,  268.0, 320.0),
    (832.0,  270.0, 318.0),
    (1100.0, 288.0, 312.0),
    (1340.0, 295.0, 309.0),
    (1390.0, 295.0, 309.0),
    # combustor: the gas path is the space between the liners (see COMBUSTOR)
    (1690.0, 262.0, 326.0),
    (1745.0, 262.0, 328.0),
    (1760.0, 262.0, 329.0),
    (1808.0, 260.0, 333.0),
    (1830.0, 259.0, 336.0),
    (1930.0, 252.0, 356.0),
    (1950.0, 250.0, 358.0),
    (2008.0, 248.0, 366.0),
    (2020.0, 247.0, 367.0),
]

# Second stream: the bypass duct between the core cowl and the intermediate
# case, splitter to mixer. It diffuses as it goes, which is where it wants to
# be slow; it arrives at the mixer to meet the core at equal pressure.
BYPASS_PATH = [
    (695.0,  349.0, 376.0),
    (780.0,  344.0, 382.0),
    (1360.0, 344.0, 384.0),
    (1410.0, 382.0, 418.0),
    (2020.0, 382.0, 420.0),
]

# Third stream: from its splitter round the outside of everything to the
# nozzle, where it cools the flaps and leaves as a film.
THIRD_PATH = [
    (425.0,  384.0, 405.0),
    (490.0,  392.0, 418.0),
    (610.0,  402.0, 440.0),
    (1360.0, 402.0, 440.0),
    (1410.0, 426.0, 460.0),
    (2020.0, 426.0, 460.0),
    (2200.0, 434.0, 464.0),
    (2850.0, 434.0, 464.0),
]


def _interp(table, x, col):
    if x <= table[0][0]:
        return table[0][col]
    for a, b in zip(table, table[1:]):
        if a[0] <= x <= b[0]:
            t = 0.0 if b[0] == a[0] else (x - a[0]) / (b[0] - a[0])
            return a[col] + (b[col] - a[col]) * t
    return table[-1][col]


def annulus(table, x):
    """(inner, outer) radius of a stream table at station x."""
    return _interp(table, x, 1), _interp(table, x, 2)


PATHS = {"fan": FAN_PATH, "cdfs": CDFS_PATH, "core": CORE_PATH,
         "bypass": BYPASS_PATH, "third": THIRD_PATH}

# --------------------------------------------------------------------------
# Blade rows (DERIVED)
# --------------------------------------------------------------------------


@dataclass
class BladeRow:
    """One rotor or stator row. Its hub and tip are read off the stream it
    sits in, at every point along the chord, so a blade follows the annulus
    flare rather than sitting on a cylinder."""
    name: str
    path: str               # which stream table it lives in
    x: float                # leading edge at the hub, mm
    chord: float            # axial chord at the hub, mm
    count: int
    stagger_hub: float      # deg from axial
    stagger_tip: float
    thickness: float = 0.07     # t/c
    camber: float = 0.06        # max camber / chord
    rotor: bool = True
    spool: str = ""             # "lp" / "hp" for rotors
    variable: bool = False
    cooled: bool = False
    chord_tip: float = 0.0      # axial chord at the tip; 0 = same as hub
    sweep: float = 0.0          # axial offset of the tip LE, mm (+ aft)
    lean: float = 0.0           # tangential lean at the tip, deg

    def hub(self, x):
        return annulus(PATHS[self.path], x)[0]

    def tip(self, x):
        return annulus(PATHS[self.path], x)[1]

    def ctip(self):
        return self.chord_tip or self.chord

    def span_x(self, s):
        """(x_le, x_te) at span fraction s."""
        x0 = self.x + self.sweep * s * s
        return x0, x0 + self.chord + (self.ctip() - self.chord) * s

    @property
    def x_te(self):
        return max(self.span_x(0.0)[1], self.span_x(1.0)[1])


# Fan. The first rotor is a wide-chord, swept blisk of 18 hollow titanium
# blades: few, big blades are what survive a bird, and the swept tip keeps
# the passage shock off the leading edge at 1.5 times the speed of sound.
FAN_ROWS = [
    BladeRow("fan_r1", "fan", 0.0, 100.0, 18, 25.0, 60.0,
             thickness=0.055, camber=0.05, spool="lp", chord_tip=110.0,
             sweep=25.0),
    BladeRow("fan_s1", "fan", 157.0, 64.0, 40, 24.0, 20.0,
             thickness=0.07, camber=0.09, rotor=False, lean=5.0),
    BladeRow("fan_r2", "fan", 243.0, 85.0, 30, 35.0, 58.0,
             thickness=0.06, camber=0.055, spool="lp", chord_tip=70.0,
             sweep=12.0),
    BladeRow("fan_s2", "fan", 348.0, 60.0, 52, 18.0, 12.0,
             thickness=0.07, camber=0.08, rotor=False, lean=4.0),
]

# The core-driven fan stage: on the HP spool, in the inner flow only (core +
# second stream), downstream of the third-stream splitter and the fan frame.
# Its stator is variable, and it is the vane angle here that moves air between
# the core and the bypass.
CDFS_ROWS = [
    BladeRow("cdfs_r", "cdfs", 555.0, 55.0, 36, 40.0, 56.0,
             thickness=0.06, camber=0.05, spool="hp", sweep=4.0),
    BladeRow("cdfs_s", "cdfs", 634.0, 44.0, 60, 26.0, 18.0,
             thickness=0.07, camber=0.09, rotor=False, variable=True),
]

# Six-stage HP compressor. The inlet guide vane and first two stators are
# variable: at part speed they close to keep the front stages off stall,
# which is what lets an agile fighter's engine be slammed from idle to reheat
# mid-manoeuvre.
HPC_ROWS = [
    BladeRow("hpc_igv", "core", 780.0, 34.0, 44, 12.0, 6.0,
             thickness=0.06, camber=0.04, rotor=False, variable=True),
    BladeRow("hpc_r1", "core", 832.0, 48.0, 36, 42.0, 58.0, spool="hp"),
    BladeRow("hpc_s1", "core", 896.0, 42.0, 54, 26.0, 18.0,
             rotor=False, variable=True),
    BladeRow("hpc_r2", "core", 954.0, 42.0, 42, 42.0, 56.0, spool="hp"),
    BladeRow("hpc_s2", "core", 1011.0, 38.0, 60, 26.0, 18.0,
             rotor=False, variable=True),
    BladeRow("hpc_r3", "core", 1064.0, 36.0, 48, 41.0, 55.0, spool="hp"),
    BladeRow("hpc_s3", "core", 1114.0, 34.0, 66, 25.0, 17.0, rotor=False),
    BladeRow("hpc_r4", "core", 1161.0, 31.0, 54, 41.0, 54.0, spool="hp"),
    BladeRow("hpc_s4", "core", 1204.0, 30.0, 74, 25.0, 17.0, rotor=False),
    BladeRow("hpc_r5", "core", 1245.0, 25.0, 64, 40.0, 53.0, spool="hp"),
    BladeRow("hpc_s5", "core", 1282.0, 24.0, 88, 24.0, 16.0, rotor=False),
    BladeRow("hpc_r6", "core", 1318.0, 20.0, 76, 40.0, 52.0, spool="hp"),
]

# Turbines. One HP stage, one LP stage, counter-rotating: the LP rotor takes
# the swirl the HP rotor leaves, and a vane row between them would only take
# it out to put it back. The mid-turbine frame between them is a ring of
# sixteen fat, barely-turning vanes that carry the rear bearings and the oil
# lines -- structure first, aerodynamics second.
TURBINE_ROWS = [
    # its leading edge is 8 mm behind the liners' exit: a vane staggered 45
    # degrees with a fat nose reaches 5 mm ahead of its nominal edge
    BladeRow("hpt_ngv", "core", 1694.0, 51.0, 36, 45.0, 50.0,
             thickness=0.20, camber=0.18, rotor=False, cooled=True),
    BladeRow("hpt_r", "core", 1760.0, 48.0, 56, -25.0, -40.0,
             thickness=0.16, camber=0.17, spool="hp", cooled=True),
    BladeRow("mtf", "core", 1830.0, 100.0, 16, 8.0, 4.0,
             thickness=0.22, camber=0.03, rotor=False),
    BladeRow("lpt_r", "core", 1950.0, 58.0, 46, 30.0, 45.0,
             thickness=0.12, camber=0.15, spool="lp"),
]


def all_rows():
    return FAN_ROWS + CDFS_ROWS + HPC_ROWS + TURBINE_ROWS


def row(name):
    return next(r for r in all_rows() if r.name == name)


# Solidity bands a row has to sit in, checked by verify.py. The mid-turbine
# frame is a structural strut ring and is allowed to be sparse.
SOLIDITY_BAND = (1.0, 3.0)
SOLIDITY_EXEMPT = {"mtf": "a ring of sixteen structural struts",
                   "hpc_igv": "an inlet guide vane barely turns the flow"}

# Running clearance from a rotor tip to its casing, cold.
TIP_CLEARANCE = {"fan": 1.6, "cdfs": 1.4, "core": 1.0}

# A part that sits ON a surface -- a wrap on a case, a dome between liners --
# stands this far off it. Two coincident surfaces are neither in contact nor
# apart as far as an exact crossing test is concerned, and it reports them as
# a crossing; 0.15 mm is well inside the audits' 0.3 mm contact tolerance, so
# the part still reads as held.
SEAT = 0.15

# How far a blade root is sunk into the disc rim that holds it, and a vane's
# ends into its bands. Blades are dovetailed or welded (blisk) into their
# hubs; nothing in an engine is held on at a tangent.
ROOT_EMBED = 2.0

# A stator's inner band is a ring under the vanes, this deep, and the rotor
# drum under it runs this far below the band (the labyrinth-seal gap).
BAND_DEPTH = 5.0
DRUM_RECESS = 3.0

# --------------------------------------------------------------------------
# Walls, casings and frames (DERIVED)
# --------------------------------------------------------------------------

WALL = {
    "fan_case": 9.0,          # titanium
    "containment": 13.0,      # aramid wrap, over the fan rotors only
    "outer_case": 6.0,        # the engine's outside, over the third stream
    "core_cowl": 4.0,         # between bypass and the HP compressor case
    "core_case": 7.0,         # compressor pressure casing
    "hub_ring": 8.0,          # static inner walls
}

# Inlet case the aircraft duct bolts to, ahead of the fan.
INLET = {"x0": -190.0, "x1": -8.0, "wall": 8.0}

# Spinner: turns with the fan, an ogive of fineness 1.3. There is no inlet
# guide vane row in this engine -- the first thing the air meets is the fan
# -- so the nose is a turning spinner and not a static centre-body, and it
# sheds ice by spinning rather than by being heated.
SPINNER = {"x_nose": -236.0, "x_base": -8.0, "wall": 4.0}

# Fan frame: eight struts from the inner hub ring out through the splitter and
# the third stream to the outer case. It carries bearings 1, 2 and 3 and the
# forward mounts, and the bottom strut, fatter than the rest, is the tower
# shaft's conduit to the gearbox.
FAN_FRAME = {"x0": 440.0, "x1": 540.0, "n_struts": 8, "t_c": 0.18,
             "t_c_king": 0.42,
             # the hub ring stops 4 mm short of the CDFS rim, which turns
             "hub_x1": 542.0, "cdfs_rim_x0": 546.0}

# Mid-turbine frame: the MTF vanes' inner band hangs a web down to the rear
# sump, and four service struts carry it out across the bypass and third
# streams to the outer case, with the oil lines inside two of them.
MTF = {"n_service": 4, "service_chord": 90.0, "service_t": 22.0}

# Combustor module: a short annular burner with ceramic-matrix-composite
# liners. CMC takes 1,600 K+ without film cooling, so the liners are plain
# skins with one row of dilution holes -- no cooling rings -- which is what
# lets the combustor be this short.
COMBUSTOR = {
    "x_ogv": 1350.0,          # compressor exit guide vanes, in the diffuser
    "ogv_chord": 28.0,
    "ogv_count": 90,
    "x_dump": 1400.0,         # pre-diffuser ends, dump into the case
    "x_dome": 1430.0,         # dome face
    "dome_t": 10.0,
    "x_exit": 1686.0,         # liner exit, at the NGV band
    "outer_liner_r0": 346.0,
    "outer_liner_r1": 328.0,
    "inner_liner_r0": 242.0,
    "inner_liner_r1": 262.0,
    "liner_t": 4.0,
    "case_bore": 374.0,       # combustor outer case bore
    "inner_case_r": 214.0,    # inner case top
    "case_t": 7.0,
    "n_nozzles": 18,
    "swirler_r": 24.0,
    "swirler_len": 26.0,
    "nozzle_pitch_r": 294.0,  # swirler / fuel-nozzle pitch radius
    "stem_r": 9.0,
    "manifold_x": 1400.0,     # fuel manifold ring on the outer case
    "manifold_tube_r": 10.0,
    "n_igniters": 2,
    # just behind the dome, in the primary zone, high on both sides and
    # clear of the heat exchanger that starts 18 mm further aft
    "igniter_clock": (30.0, 150.0),
    "igniter_x": 1462.0,
    "n_dilution": 36,
    "dilution_r": 7.0,
    "x_dilution": 1560.0,
}

# Mixer, augmentor and the tail cone behind the LP turbine.
AUGMENTOR = {
    "x_mixer0": 2020.0,
    "x_mixer1": 2200.0,
    "n_lobes": 16,
    "lobe_depth": 32.0,
    "mixer_t": 3.0,
    "x_liner0": 2200.0,
    "x_liner1": 2850.0,
    "liner_r": 418.0,         # augmentor duct bore: the bypass duct's outer line
    "liner_t": 3.0,
    "case_t": 6.0,            # its outside is the third stream's inner wall
    "tailcone_x0": 2016.0,
    "tailcone_x1": 2560.0,
    # the flameholder: V-gutters open aft, in whose wake the flame holds --
    # three concentric rings tied by sixteen radial gutters from the tail
    # cone to the liner, the spider web you see looking up a reheat nozzle
    "n_vanes": 16,                      # radial gutters
    "vane_x0": 2400.0,                  # the gutters' apex
    "vane_chord": 45.0,                 # apex to trailing edge
    "gutter_w": 44.0,                   # radial gutters, across the mouth
    "gutter_t": 3.0,
    "ring_r": (215.0, 290.0, 365.0),    # the concentric gutters
    "ring_w": 38.0,
    "ring_depth": 40.0,
    # the liner is corrugated for cooling film: pitch divides its length
    "liner_wave": 3.0,
    "liner_pitch": 65.0,
    "screech_rows": 20,
    "screech_per_row": 60,
    "screech_hole_r": 3.5,
    "ab_manifold_x": 2340.0,
    # staged reheat: three zones, each off its own manifold and lit in turn
    # as reheat comes in -- zone 1 outer, zones 2 and 3 on radial spraybars
    # reaching into the middle and the inner stream
    "zone_x": (2340.0, 2250.0, 2285.0),
    "spraybar_r": (300.0, 226.0),       # how far in zones 2 and 3 reach
    # each zone sprays from a ring in the stream: zone 1 outer, ahead of
    # the gutters; zones 2 and 3 on their spraybars' tips
    "spray_ring_r": (360.0, 300.0, 226.0),
    "spray_tube_r": 6.0,
    "igniter_x": 2462.0,
    "fuel_control_x": (2080.0, 2200.0),
}

# The nozzle: a three-bearing swivel duct and an axisymmetric convergent-
# divergent nozzle on the end of it, drawn at maximum reheat and stowed.
#
# The swivel is three short round ducts in a row joined by three bearings.
# The first bearing is square to the engine axis; the other two are cut
# obliquely at `beta`, leaning opposite ways, so the middle duct is a wedge.
# Turning the middle duct half a turn one way and the aft duct half a turn
# the other folds the jet down through 4 * beta -- 95 degrees, straight down
# and a little forward, for a vertical landing -- and the front bearing
# steers the plane the fold happens in, so every direction in a 95 degree
# cone round the axis is reachable, including yaw for hover control.
#
# The throat and exit radii are not chosen: they are the cycle's choked
# throat area at max reheat and its fully-expanded exit/throat area ratio.
_A8 = CYCLE["a8_wet"] * 1e6            # mm^2
_A9A8 = CYCLE["a9_a8_wet"]
NOZZLE = {
    "beta_deg": 23.75,        # oblique cut: 4 * beta = 95 degrees
    "x_fixed0": 2850.0,       # fixed ring bolted to the outer case's end
    "x_brg1": 2880.0,         # front bearing, square to the axis
    "x_cut1": 3180.0,         # the oblique bearings' centres on the axis
    "x_cut2": 3700.0,
    "x_aft": 4000.0,          # aft duct ends square at the nozzle's static ring
    "duct_r_in": 418.0,       # liner bore: the augmentor liner's
    "liner_t": 4.0,
    "duct_r_out": 470.0,      # structural shell outside: the outer case's
    "shell_t": 8.0,
    "flange_t": 14.0,         # each half of a bearing housing
    "flange_r": 490.0,
    "race_r": 500.0,          # the bearing race and the ring gear on it
    "gear_r": 510.0,
    "n_gear_teeth": 150,
    # axisymmetric C-D nozzle
    "x_static1": 4100.0,      # static ring ends: convergent flap hinges
    "x_throat": 4280.0,
    "x_exit": 4500.0,
    "r_throat": math.sqrt(_A8 / math.pi),
    "r_exit": math.sqrt(_A8 * _A9A8 / math.pi),
    "n_flaps": 16,
    "flap_w": 94.0,           # every convergent and divergent flap, mm
    "seal_w": 92.0,
    "seal_t": 6.0,
    "conv_t": 14.0,
    "div_t": 12.0,
    "ext_t": 8.0,
    "saw": 60.0,              # depth of the external flaps' serrations
    "actuator_r": 18.0,
    "n_actuators": 4,
}
NOZZLE_EXIT_X = NOZZLE["x_exit"]

# The outer case over the third stream, in three pieces. The bore follows
# the fan tip line over the fan and the third stream's outer line after it.
OUTER_CASES = [
    ("case_fan", -8.0, 425.0),
    ("case_outer_fwd", 425.0, 1360.0),
    ("case_outer_aft", 1360.0, 2850.0),
]

# Bolted flanges: (name, x, outer radius, thickness, bolt count). Each is a
# collar on the joint between two cases, bored to sit on them.
FLANGES = [
    ("flange_inlet", -8.0, 478.0, 16.0, 36),
    ("flange_fan_aft", 425.0, 440.0, 16.0, 36),
    ("flange_outer_mid", 1360.0, 482.0, 16.0, 40),
    ("flange_outer_aft", 2850.0, 492.0, 16.0, 40),
]

# Shafts: LP inside HP, counter-rotating.
SHAFTS = {
    "lp_r_in": 48.0, "lp_r_out": 62.0, "lp_x0": 10.0, "lp_x1": 2012.0,
    "hp_r_in": 104.0, "hp_r_out": 116.0, "hp_x0": 470.0, "hp_x1": 1858.0,
}

# Bearings: (name, x centre, inner race bore, outer race outside radius,
# kind, spool). Five bearings in two sumps -- the fan frame's and the
# mid-turbine frame's. The fan is overhung ahead of 1 and 2; the HP spool
# runs between 3 in front and 4 behind its turbine.
BEARINGS = [
    ("brg_1_lp_roller", 230.0, 62.0, 100.0, "roller", "lp"),
    ("brg_2_lp_ball", 455.0, 62.0, 100.0, "ball", "lp"),
    ("brg_3_hp_ball", 530.0, 116.0, 158.0, "ball", "hp"),
    ("brg_4_hp_roller", 1845.0, 116.0, 156.0, "roller", "hp"),
    ("brg_5_lp_roller", 1905.0, 62.0, 98.0, "roller", "lp"),
]
BEARING_WIDTH = {"ball": 24.0, "roller": 22.0}
SUMP_T = 8.0

# Accessory gearbox, under the engine, driven by a tower shaft down the bottom
# fan-frame strut off a bevel on the HP shaft's front stub.
GEARBOX = {
    "x0": 380.0, "x1": 880.0,
    "r_top": 452.0,           # its top face, clear of the flanges and wrap
    "depth": 120.0,
    "width": 230.0,
    "towershaft_x": 490.0,    # the fan frame's mid-chord
    "towershaft_r": 13.0,
    "bevel_r": 150.0,         # outside of the bevel on the HP stub
}

# Mounts: trunnions on the fan frame's outer ring at 3 and 9 o'clock, and a
# thrust link lug on top of the rear case.
MOUNTS = {"x_fwd": 490.0, "trunnion_r": 36.0, "trunnion_len": 56.0,
          # forward of the reheat zone manifolds, which ring the case at
          # 2250-2340 and cannot pass through a thrust lug
          "x_aft": 2150.0}

# Third-stream heat exchanger: the aircraft's heat load goes into the third
# stream here. Twelve plate-fin segments filling the duct.
TMS_HX = {"x0": 1480.0, "x1": 1760.0, "n": 12, "gap_deg": 3.0}

# Orthogrid on the outer cases: circumferential rings and axial stringers
# standing on the skin -- how a thin case is made stiff against buckling and
# the pressure of the third stream. Each run stops short of anything bolted
# through or on to the case: (x0, x1) gaps to leave.
CASE_RIBS = {
    "h": 6.0, "w": 8.0, "n_stringers": 16, "phase_deg": 11.25,
    "rings": (700.0, 1100.0, 1620.0, 2050.0, 2500.0, 2720.0),
    "x0": 446.0, "x1": 2836.0,
    "gaps": ((1344.0, 1376.0),     # the mid flange
             (1380.0, 1422.0),     # fuel manifold and nozzle flanges
             (2232.0, 2303.0),     # reheat zone 2 and 3 manifolds
             (2322.0, 2358.0)),    # reheat zone 1 manifold
}

# Mode valve: petals round the third stream's entrance that set how much air
# it takes -- open at cruise, closed down at max thrust.
MODE_VALVE = {"x_hinge": 620.0, "n": 24, "len": 70.0, "t": 5.0,
              "deflect": 14.0}

# --------------------------------------------------------------------------
# Materials and palette
# --------------------------------------------------------------------------

# name prefix -> material. Longest matching key wins.
MATERIAL_MAP = {
    "spinner": "titanium",
    "fan_blisk": "titanium",
    "fan_drum": "titanium",
    "vanes_fan": "titanium",
    "cdfs_blisk": "titanium",
    "vanes_cdfs": "titanium",
    "hp_front_drum": "titanium",
    "blades_hpc_r1": "titanium",
    "blades_hpc_r2": "titanium",
    "blades_hpc_r3": "titanium",
    "blades_hpc": "nickel",
    "vanes_hpc": "nickel",
    "hpc_drum": "titanium",
    "diffuser": "nickel",
    "combustor_liner": "cmc",
    "combustor_dome": "nickel_hot",
    "combustor_inner_case": "nickel",
    "case_combustor": "casing",
    "swirler": "nickel_hot",
    "vanes_hpt": "cmc",
    "blades_hpt": "nickel_hot",
    "hpt_disc": "nickel",
    "vanes_mtf": "nickel_hot",
    "mtf_hub": "nickel",
    "blades_lpt": "nickel_hot",
    "lpt_disc": "nickel",
    "shaft": "steel",
    "brg": "steel",
    "sump": "steel",
    "case": "casing",
    "case_outer_aft": "inconel",
    "case_ribs": "casing",
    "flange": "casing",
    "flange_outer_aft": "inconel",
    "inlet": "casing",
    "splitter": "titanium",
    "core_cowl": "titanium",
    "intermediate": "titanium",
    "fan_frame": "casing",
    "service_struts": "casing",
    "containment": "composite",
    "mixer": "nickel_hot",
    "tailcone": "cmc",
    "augmentor": "cmc",
    "flameholder": "cmc",
    "nozzle": "cmc",
    "nozzle_static_ring": "inconel",
    "nozzle_ext": "inconel",
    "nozzle_actuator": "steel",
    "line_clamps": "steel",
    "nozzle_hinge": "steel",
    "nozzle_div_link": "steel",
    "nozzle_unison": "inconel",
    "swivel_": "inconel",
    "swivel_bearing": "steel",
    "swivel_drive": "steel",
    "swivel_rotary_union": "steel",
    "mode_valve": "titanium",
    "mode_valve_actuators": "steel",
    "hydraulic": "steel",
    "tms_hx": "copper",
    "coolant": "copper",
    "gearbox": "casing",
    "towershaft": "steel",
    "generator": "casing",
    "fuel": "steel",
    "ab_fuel": "steel",
    "ab_spray": "inconel",
    "ab_igniter": "steel",
    "oil": "steel",
    "fadec": "composite",
    "harness": "rubber",
    "mount": "steel",
    "igniter": "steel",
    "vsv": "steel",
}
DEFAULT_MATERIAL = "casing"

# base colour (linear RGB), metallic, roughness. glTF cannot carry the
# procedural roughness used in the renders, so the viewer reapplies this table
# by material name -- it ships in viewer/parts.json.
PALETTE = {
    "titanium":   ((0.380, 0.390, 0.410), 1.00, 0.30),
    "nickel":     ((0.470, 0.460, 0.440), 1.00, 0.30),
    "nickel_hot": ((0.330, 0.240, 0.180), 1.00, 0.46),
    "steel":      ((0.520, 0.530, 0.550), 1.00, 0.16),
    "casing":     ((0.300, 0.310, 0.325), 1.00, 0.42),
    "inconel":    ((0.300, 0.265, 0.230), 1.00, 0.40),
    "cmc":        ((0.130, 0.120, 0.110), 0.00, 0.62),
    "composite":  ((0.050, 0.055, 0.062), 0.00, 0.42),
    "copper":     ((0.600, 0.330, 0.200), 1.00, 0.32),
    "rubber":     ((0.050, 0.050, 0.055), 0.00, 0.85),
}


# --------------------------------------------------------------------------
# What turns
# --------------------------------------------------------------------------

# The LP spool is the fan and its turbine; the HP spool is the CDFS, the
# compressor and their turbine, and turns the other way. One list, read by
# the assembly (for the viewer's manifest) and by the rotor-clearance audit.
SPOOLS = {
    "lp": ["spinner", "fan_blisk_1", "fan_blisk_2", "fan_drum", "shaft_lp",
           "blades_lpt_r", "lpt_disc"],
    "hp": ["cdfs_blisk", "hp_front_drum", "hpc_drum", "shaft_hp",
           "blades_hpt_r", "hpt_disc"] + [f"blades_{r.name}" for r in HPC_ROWS
                                           if r.rotor],
}


def spool_of(name):
    for k, names in SPOOLS.items():
        if name in names:
            return k
    return ""


def total_airfoils():
    return sum(r.count for r in all_rows()) + COMBUSTOR["ogv_count"]
