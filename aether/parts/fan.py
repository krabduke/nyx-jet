"""Inlet case, spinner, the two-stage blisk fan and its case.

There are no inlet guide vanes. The first thing the air meets is the fan, so
the nose is a spinner that turns with it, and the fan case is the first
casing. Both fan rotors are blisks -- blades and disc machined from one
forging -- which is why the blades and their rim are one part here: there is
no dovetail to model because there is no dovetail.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec          # noqa: E402
import mesh          # noqa: E402
from parts import common   # noqa: E402

SEG = common.SEG
R1, S1, R2, S2 = spec.FAN_ROWS
FAN = spec.FAN_PATH
CLR = spec.TIP_CLEARANCE["fan"]


def bore(x):
    """The fan case bore IS the tip line: the rotor tips are cut the running
    clearance short of it. (It was the tip line plus the clearance, on top of
    tips already cut short -- a fan running 3.2 mm clear, not 1.6, which the
    rotor-clearance audit found.)"""
    return common.tip("fan", x)


def case_od(x):
    return bore(x) + spec.WALL["fan_case"]


def build():
    out = {}
    out["spinner"] = _spinner()
    out.update(_rotors())
    out["vanes_fan_s1"] = common.stator_vanes(S1)
    x0, _ = common.reach(S2)
    out["vanes_fan_s2"] = mesh.join(
        common.stator_vanes(S2, band=False),
        # the exit vanes' band runs on aft to meet the fan frame's hub ring:
        # between them they are the static inner wall from here to the CDFS
        common.ring(x0, spec.SPLITTER3_X,
                    lambda x: common.hub("fan", x) - spec.BAND_DEPTH,
                    lambda x: common.hub("fan", x), (FAN,), step=10.0))
    out.update(_cases())
    return out


def _spinner():
    """A solid ogive from the nose to the first blisk's front face, where its
    base radius is the hub line's. r = R (t (2 - t))^0.62 gives a fine,
    slightly blunted point rather than a cone's sharp one, which is what
    sheds ice."""
    s = spec.SPINNER
    x0, x1 = s["x_nose"], s["x_base"]
    rb = common.hub("fan", x1)
    prof = [(x0, 0.001)]
    n = 28
    for i in range(1, n + 1):
        t = i / n
        prof.append((x0 + (x1 - x0) * t, rb * (t * (2.0 - t)) ** 0.62))
    prof.append((x1, 0.001))
    return mesh.revolve_open(prof, SEG, cap_start=True, cap_end=True)


def _rotors():
    out = {}
    sh = spec.SHAFTS
    # First blisk: the rim is the hub line from the spinner's base to half way
    # to the first stator; the web comes down to a bore ring shrunk onto the
    # LP shaft, which is the fan's only connection to its spool.
    x0, x1 = common.reach(R1, lo=spec.SPINNER["x_base"])
    loop = common.rim_profile(x0, x1, "fan", 22.0, 52.0, 34.0,
                              sh["lp_r_out"], 80.0, bore_h=34.0)
    out["fan_blisk_1"] = mesh.join(common.revolve(loop),
                                   common.rotor_blades(R1))
    # Second blisk: carried off the first by the drum under the stator, its
    # bore well clear of the No.1 bearing housing that runs under it.
    y0, y1 = common.reach(R2)
    loop = common.rim_profile(y0, y1, "fan", 18.0, 0.5 * (y0 + y1), 26.0,
                              124.0, 50.0)
    out["fan_blisk_2"] = mesh.join(common.revolve(loop),
                                   common.rotor_blades(R2))
    # The drum between them runs under the first stator's inner band with
    # the labyrinth gap, and butts onto each rim.
    off = spec.BAND_DEPTH + spec.DRUM_RECESS
    out["fan_drum"] = common.ring(
        x1, y0,
        lambda x: common.hub("fan", x) - off - 8.0,
        lambda x: common.hub("fan", x) - off, (FAN,), step=10.0)
    return out


def _cases():
    out = {}
    w = spec.WALL["fan_case"]
    i = spec.INLET
    x_fan = spec.SPINNER["x_base"]
    b0 = bore(x_fan)
    # the inlet case is straight: the aircraft's duct delivers the air square
    # to the fan, and its flange bolts to this one
    out["inlet_case"] = common.ring(i["x0"], i["x1"], b0, b0 + w)
    out["case_fan"] = common.ring(x_fan, spec.SPLITTER3_X, bore, case_od,
                                  (FAN,), step=20.0)
    # Aramid containment wrap over both rotors: what catches a released
    # blade. It sits on the case from just behind the inlet flange's bolt
    # heads to past the second rotor.
    fl = next(f for f in spec.FLANGES if f[0] == "flange_inlet")
    c0 = fl[1] + fl[3] / 2.0 + 8.0
    c1 = R2.x_te + 10.0
    out["containment_wrap"] = common.ring(
        c0, c1, lambda x: case_od(x) + spec.SEAT,
        lambda x: case_od(x) + spec.WALL["containment"],
        (FAN,), step=20.0)
    return out
