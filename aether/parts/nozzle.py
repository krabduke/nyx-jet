"""The three-bearing swivel duct and the axisymmetric C-D nozzle on it.

Front to back:

    fixed ring      bolted to the outer case's aft flange; it carries the
                    front bearing and the motor that drives it
    swivel ducts    three round ducts, each a structural shell outside a
                    cooling liner, with the third stream running between
                    the two. The front duct turns on a bearing square to
                    the axis; the middle and aft ducts on bearings cut at
                    beta to it, leaning opposite ways, so the middle one is
                    a wedge. Each bearing is a pair of flanges round a
                    race with a ring gear cut in it, driven by a hydraulic
                    motor on the duct in front
    static ring     the nozzle's fixed structure on the aft duct: it carries
                    the flap hinges and the four actuators
    convergent      sixteen flaps and sixteen seals, hinged at the static
      flaps         ring and closing to the throat. A unison ring behind
                    them, turned by the actuators through bellcranks, sets
                    the throat area
    divergent       sixteen flaps and seals, hinged at the throat, opening to
      flaps         the exit
    external        sixteen outer flaps from the static ring to the exit,
      flaps         serrated at the trailing edge, each held on its
                    divergent flap by a compression link

Stowed, the three ducts are one straight tube. Turning the middle duct half
a turn and the aft duct half a turn back folds the jet 4 * beta = 95 degrees
down; the front bearing's angle puts the fold in whatever plane is asked
for. The oblique cuts lean in plan, not in elevation, which is what lets
the front bearing's schedule start from zero: see `swivel_pose`.

The throat and exit radii are the cycle's at max reheat, which is how the
nozzle is drawn.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec          # noqa: E402
import mesh          # noqa: E402
from parts import common   # noqa: E402

N = spec.NOZZLE
SEG = common.SEG

BETA = math.radians(N["beta_deg"])
R_IN = N["duct_r_in"]
R_OUT = N["duct_r_out"]
LINER_OUT = R_IN + N["liner_t"]
SHELL_IN = R_OUT - N["shell_t"]
FT = N["flange_t"]
R8 = N["r_throat"]
R9 = N["r_exit"]
X_S1 = N["x_static1"]
X8 = N["x_throat"]
X9 = N["x_exit"]


# --------------------------------------------------------------------------
# frames: a joint plane is (centre, normal, e1, e2); local (a, u, v) maps to
# centre + a * normal + u * e1 + v * e2

def square(x):
    return ((x, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def oblique(k):
    """The k-th oblique bearing's plane (k = 1, 2). The first leans toward
    +y and the second toward -y, in plan."""
    s = 1.0 if k == 1 else -1.0
    n = (math.cos(BETA), s * math.sin(BETA), 0.0)
    e1 = (-n[1], n[0], 0.0)
    return ((N["x_cut%d" % k], 0.0, 0.0), n, e1, (0.0, 0.0, 1.0))


def frame_pt(fr, a, u, v):
    c, n, e1, e2 = fr
    return tuple(c[i] + a * n[i] + u * e1[i] + v * e2[i] for i in range(3))


def to_frame(part, fr):
    """Place a part built about the x axis (x as a, y as u, z as v) on a
    joint plane."""
    v, f = part
    return [frame_pt(fr, x, y, z) for (x, y, z) in v], f


def _flip_a(part):
    v, f = part
    return [(-x, y, z) for (x, y, z) in v], [tuple(reversed(fc)) for fc in f]


def _ring_local(a0, a1, r0, r1, seg=SEG):
    return mesh.revolve_ring([(a0, r0), (a1, r0), (a1, r1), (a0, r1)], seg)


# --------------------------------------------------------------------------
# ducts

def _loft(fa, fb, r_in, r_out, nst=12, m=SEG):
    """A closed annular solid from joint plane fa to joint plane fb: both
    ends are circles in their planes, so the duct turns on either one."""
    rings = []
    for r in (r_out, r_in):
        for i in range(nst + 1):
            s = i / nst
            ring = []
            for k in range(m):
                t = 2.0 * math.pi * k / m
                pa = frame_pt(fa, 0.0, r * math.cos(t), r * math.sin(t))
                pb = frame_pt(fb, 0.0, r * math.cos(t), r * math.sin(t))
                ring.append(tuple(pa[j] + (pb[j] - pa[j]) * s for j in range(3)))
            rings.append(ring)
    verts = [p for rg in rings for p in rg]
    no = (nst + 1) * m
    faces = []
    for i in range(nst):
        for k in range(m):
            k2 = (k + 1) % m
            a, b = i * m + k, i * m + k2
            c, d = (i + 1) * m + k2, (i + 1) * m + k
            faces.append((a, b, c, d))
            faces.append((no + d, no + c, no + b, no + a))
    for k in range(m):
        k2 = (k + 1) % m
        faces.append((k2, k, no + k, no + k2))
        o0, o1 = nst * m + k, nst * m + k2
        faces.append((o0, o1, no + o1, no + o0))
    return common.orient((verts, faces))


def _flange(fr, aft):
    """One half of a bearing housing on a joint plane: a flange from the
    shell out to flange_r, FT thick on the duct's own side of the plane,
    with a bolt circle on its outer face."""
    ring = _ring_local(-FT, 0.0, SHELL_IN + 1.0, N["flange_r"])
    bv, bf = mesh.bolt_ring(0.0, N["flange_r"] - 11.0, 36, 6.0, 5.0)
    # bolt heads stand on the face away from the joint: flip them to point
    # -a and sit them on the -FT face
    bv = [(-x - FT + 0.5, y, z) for (x, y, z) in bv]
    bf = [tuple(reversed(f)) for f in bf]
    part = mesh.join(ring, (bv, bf))
    if not aft:
        part = _flip_a(part)
    return to_frame(part, fr)


def _hangers(fr, a):
    """Eight radial hangers carrying the liner off the shell, a mm along the
    duct from a joint plane."""
    parts = []
    for k in range(8):
        t = 2.0 * math.pi * (k + 0.5) / 8
        p0 = frame_pt(fr, a, (LINER_OUT - 1.0) * math.cos(t),
                      (LINER_OUT - 1.0) * math.sin(t))
        p1 = frame_pt(fr, a, (SHELL_IN + 1.0) * math.cos(t),
                      (SHELL_IN + 1.0) * math.sin(t))
        parts.append(mesh.pipe([p0, p1], 7.0, 10))
    return mesh.join(*parts)


def _mid_frame(fa, fb, s):
    """A plane part way from fa to fb, its normal blended between theirs."""
    c = tuple(fa[0][j] + (fb[0][j] - fa[0][j]) * s for j in range(3))
    nn = tuple(fa[1][j] + (fb[1][j] - fa[1][j]) * s for j in range(3))
    L = math.sqrt(sum(q * q for q in nn))
    nn = tuple(q / L for q in nn)
    e1 = (-nn[1], nn[0], 0.0)
    L1 = math.sqrt(sum(q * q for q in e1))
    return (c, nn, tuple(q / L1 for q in e1), (0.0, 0.0, 1.0))


def _stiffener(fa, fb):
    """A hoop stiffener round a duct's shell, half way between its ends."""
    return to_frame(_ring_local(-6.0, 6.0, R_OUT - 1.0, R_OUT + 9.0),
                    _mid_frame(fa, fb, 0.5))


def _duct(fa, fb, front_flange=True, aft_flange=True, stiff=True, hang=40.0):
    parts = [_loft(fa, fb, SHELL_IN, R_OUT),
             _loft(fa, fb, R_IN, LINER_OUT),
             _hangers(fa, hang), _hangers(fb, -hang)]
    if front_flange:
        parts.append(_flange(fa, aft=False))
    if aft_flange:
        parts.append(_flange(fb, aft=True))
    if stiff:
        parts.append(_stiffener(fa, fb))
    return mesh.join(*parts)


def _race(fr):
    """A bearing's race and the ring gear cut in its outside, straddling the
    joint plane between the two flanges."""
    race = _ring_local(-FT + 2.0, FT - 2.0, N["flange_r"] - 1.0, N["race_r"])
    n = N["n_gear_teeth"]
    tv, tf = mesh.box(0.0, (N["race_r"] + N["gear_r"]) / 2.0 - 0.5, 0.0,
                      2.0 * FT - 8.0, N["gear_r"] - N["race_r"] + 1.0,
                      math.pi * N["race_r"] / n)
    teeth = mesh.replicate(tv, tf, n)
    return to_frame(mesh.join(race, teeth), fr)


def _drive(fr, clock_deg):
    """A hydraulic motor on the duct in front of a bearing: a pinion in mesh
    with the ring gear, the motor behind it along the bearing's axis, its two
    ports facing out, and a bracket down to the flange's rim."""
    t = math.radians(clock_deg)
    rp = 26.0
    rc = N["gear_r"] + rp - 1.0
    pin = mesh.revolve_ring([(-FT + 3.0, 6.0), (FT - 3.0, 6.0),
                             (FT - 3.0, rp), (-FT + 3.0, rp)], 30)
    motor = mesh.revolve_ring([(-FT - 4.0, 6.0), (-FT - 110.0, 6.0),
                               (-FT - 110.0, 34.0), (-FT - 4.0, 34.0)], 32)
    shaft = mesh.revolve_ring([(-FT - 6.0, 2.0), (FT - 4.0, 2.0),
                               (FT - 4.0, 7.0), (-FT - 6.0, 7.0)], 16)
    ports = [mesh.pipe([(-FT - 60.0 + dx, 0.0, 30.0), (-FT - 60.0 + dx, 0.0, 48.0)],
                       7.0, 12) for dx in (-22.0, 22.0)]
    lv, lf = mesh.join(pin, motor, shaft, *ports)
    # ports (+z in the motor's own frame) turned to face radially out, and
    # the whole motor moved out to its place round the gear
    ca, sa = math.cos(t - math.pi / 2), math.sin(t - math.pi / 2)
    u, v = rc * math.cos(t), rc * math.sin(t)
    lv = [(x, y * ca - z * sa + u, y * sa + z * ca + v) for (x, y, z) in lv]
    # the bracket stands on the duct's shell (the case, for the front
    # bearing's motor), stopping short of anything bolted round the joint
    # -- and a duct cut obliquely meets its bearing as an ellipse, so at the
    # motor's clock, square to the cut, its shell is R cos(beta) out
    oblique_cut = abs(fr[1][0] - 1.0) > 1e-9
    rb0 = (R_OUT * math.cos(BETA) + 8.0) if oblique_cut else R_OUT - 1.0
    bracket = common.sector_block(-FT - 100.0, -FT - 40.0, rb0, rc - 30.0,
                                  t - 0.05, t + 0.05, 4)
    return to_frame(mesh.join((lv, lf), bracket), fr)


# --------------------------------------------------------------------------
# the C-D nozzle

def gas_r(x):
    """The gas-side line: the liner bore to the static ring's end, then
    the convergent flaps to the throat and the divergent ones to the exit."""
    if x <= X_S1:
        return R_IN
    if x <= X8:
        return R_IN + (R8 - R_IN) * (x - X_S1) / (X8 - X_S1)
    return R8 + (R9 - R8) * (x - X8) / (X9 - X8)


def _petal(x0, x1, r_fn, t, clock, w, saw=0.0, nu=6, nv=10, angular=False):
    """A curved plate on a cone: its inner face on r_fn(x), t thick, centred
    on a clock angle. w is its width in mm, or with angular=True its angular
    half-width in radians. saw points the trailing edge: the corners stop
    `saw` mm short of the middle."""
    verts = []
    for layer in (0.0, t):
        for i in range(nv + 1):
            s = i / nv
            for j in range(nu + 1):
                q = -1.0 + 2.0 * j / nu
                x = x0 + (x1 - saw * abs(q) - x0) * s
                r = r_fn(x) + layer
                ang = w * q if angular else (w / 2.0) * q / r_fn(x)
                a = math.radians(clock) + ang
                verts.append((x, r * math.cos(a), r * math.sin(a)))
    n = (nv + 1) * (nu + 1)

    def idx(L, i, j):
        return L * n + i * (nu + 1) + j
    faces = []
    for i in range(nv):
        for j in range(nu):
            faces.append((idx(0, i, j), idx(0, i + 1, j), idx(0, i + 1, j + 1),
                          idx(0, i, j + 1)))
            faces.append((idx(1, i, j), idx(1, i, j + 1), idx(1, i + 1, j + 1),
                          idx(1, i + 1, j)))
    for i in range(nv):
        for j in (0, nu):
            a, b = idx(0, i, j), idx(0, i + 1, j)
            c, d = idx(1, i + 1, j), idx(1, i, j)
            faces.append((a, d, c, b) if j == 0 else (a, b, c, d))
    for i in (0, nv):
        for j in range(nu):
            a, b = idx(0, i, j), idx(0, i, j + 1)
            c, d = idx(1, i, j + 1), idx(1, i, j)
            faces.append((a, b, c, d) if i == 0 else (a, d, c, b))
    return common.orient((verts, faces))


def _clocks(half=False):
    n = N["n_flaps"]
    return [360.0 * (k + (0.5 if half else 0.0)) / n for k in range(n)]


def flap_r(x):
    """A flap's inside: on the seals, which are on the gas line."""
    return gas_r(x) + N["seal_t"] - 0.5


def _flap_set(x0, x1, t, back_h, back_in):
    flaps, seals = [], []
    for c in _clocks():
        flaps.append(_petal(x0, x1, flap_r, t, c, N["flap_w"]))
        # a backbone down the middle of each flap
        flaps.append(_petal(x0 + back_in, x1 - back_in,
                            lambda x, t=t: flap_r(x) + t - 1.0, back_h, c, 14.0,
                            nu=2, nv=6))
    for c in _clocks(half=True):
        seals.append(_petal(x0 + 1.0, x1, gas_r, N["seal_t"], c, N["seal_w"]))
    return mesh.join(*flaps), mesh.join(*seals)


def ext_inner_r(x):
    """The external flaps' inside: from the static ring's shell to resting
    on the divergent flaps' backs at the exit."""
    r0 = SHELL_IN
    r1 = flap_r(X9) + N["div_t"] - 0.5
    return r0 + (r1 - r0) * (x - X_S1) / (X9 - X_S1)


def _ext():
    half = math.pi / N["n_flaps"] - 0.0035
    return mesh.join(*[_petal(X_S1, X9, ext_inner_r, N["ext_t"], c, half,
                              saw=N["saw"], nu=8, nv=14, angular=True)
                       for c in _clocks()])


def _knuckles(x, r, length, rad):
    """A hinge knuckle across every flap: a short pin on the tangent."""
    parts = []
    for c in _clocks():
        a = math.radians(c)
        ca, sa = math.cos(a), math.sin(a)
        p = (x, r * ca, r * sa)
        tv = (0.0, -sa, ca)
        h = length / 2.0
        parts.append(mesh.pipe([tuple(p[i] - tv[i] * h for i in range(3)),
                                tuple(p[i] + tv[i] * h for i in range(3))],
                               rad, 14))
    return mesh.join(*parts)


def _hinges():
    tc, td, te = N["conv_t"], N["div_t"], N["ext_t"]
    return mesh.join(
        _knuckles(X_S1, flap_r(X_S1) + tc / 2.0, 60.0, tc / 2.0 + 3.0),
        _knuckles(X8, flap_r(X8) + tc / 2.0, 56.0, tc / 2.0 + 3.0),
        _knuckles(X_S1, ext_inner_r(X_S1) + te / 2.0, 70.0, te / 2.0 + 3.0))


def _div_links():
    """A compression link from every divergent flap's backbone up to its
    external flap: it holds the outer skin on the flap under it."""
    parts = []
    xa, xb = X8 + 120.0, X8 + 80.0
    for c in _clocks():
        ra = flap_r(xa) + N["div_t"] + 10.0 - 3.0
        rb = ext_inner_r(xb) + 1.0
        parts.append(mesh.pipe([common.polar(xa, ra, c), common.polar(xb, rb, c)],
                               6.0, 12))
        for (x, r) in ((xa, ra), (xb, rb - 2.0)):
            parts.append(mesh.pipe([common.polar(x - 10.0, r, c),
                                    common.polar(x + 10.0, r, c)], 8.0, 12))
    return mesh.join(*parts)


UNISON_X = (X_S1 + 18.0, X_S1 + 34.0)
UNISON_R = (438.0, 450.0)


def _unison():
    """The unison ring behind the static ring, between the convergent and
    external flaps, and a link from it down to every convergent flap."""
    x0, x1 = UNISON_X
    xm = (x0 + x1) / 2.0
    parts = [mesh.tube(x0, x1, UNISON_R[0], UNISON_R[1], SEG)]
    for c in _clocks():
        r0 = flap_r(xm) + N["conv_t"] + 12.0 - 3.0
        parts.append(common.radial_pin(xm, r0, UNISON_R[0] + 2.0, 5.0, c, 12))
    return mesh.join(*parts)


def _static_ring():
    x0, x1 = N["x_aft"], X_S1
    return mesh.join(mesh.tube(x0, x1 - 15.0, SHELL_IN, R_OUT, SEG),
                     mesh.tube(x0, x1, R_IN, LINER_OUT, SEG),
                     mesh.tube(x1 - 15.0, x1, R_IN, R_OUT, SEG),
                     _hangers(square(x0), 40.0),
                     _flange(square(x0), aft=False))


# between flaps, where the bellcrank arms pass clear of the hinge knuckles
ACT_CLOCKS = tuple(45.0 + 11.25 + 90.0 * k for k in range(4))
ACT_R = N["flange_r"] + 8.0 + N["actuator_r"]


def _actuators():
    """Four actuators on the aft duct, each turning a bellcrank that reaches
    in through the static ring to the unison ring."""
    ra = N["actuator_r"]
    x_head = N["x_aft"] - 70.0
    x_end = X_S1 - 12.0             # clear of the external flaps' hinges
    parts = []
    for c in ACT_CLOCKS:
        parts += [
            mesh.pipe([common.polar(x_head, ACT_R, c),
                       common.polar(x_head + 115.0, ACT_R, c)], ra, 20),
            mesh.pipe([common.polar(x_head + 113.0, ACT_R, c),
                       common.polar(x_head + 125.0, ACT_R, c)], ra * 0.7, 16),
            mesh.pipe([common.polar(x_head + 120.0, ACT_R, c),
                       common.polar(x_end, ACT_R, c)], ra * 0.42, 14),
            # head clevis on the aft duct's shell
            common.radial_pin(x_head + 8.0, R_OUT - 1.0, ACT_R, 12.0, c, 14),
            # the bellcrank: a shaft from the rod end down through the static
            # ring into the cavity, and an arm aft to the unison ring
            common.radial_pin(x_end, ACT_R + 4.0, 444.0, 9.0, c, 14),
            mesh.pipe([common.polar(x_end, 444.0, c),
                       common.polar(UNISON_X[0] + 8.0, 444.0, c)], 5.0, 12),
        ]
    return mesh.join(*parts)


# --------------------------------------------------------------------------

DRIVE_CLOCKS = {1: -135.0, 2: 180.0, 3: 0.0}
UNION_CLOCK = -45.0
UNION_X = (2770.0, 2836.0)
UNION_TOP = R_OUT + 44.0


def hyd_port(unit, k):
    """(x, radius of the port's tip, clock) of hydraulic port k (0, 1) on
    unit 0 -- the front bearing's motor -- or unit 1, the rotary union."""
    if unit == 0:
        rc = N["gear_r"] + 26.0 - 1.0
        return (N["x_brg1"] - FT - 60.0 + (-22.0, 22.0)[k], rc + 48.0,
                DRIVE_CLOCKS[1])
    return (UNION_X[0] + (16.0, 50.0)[k], UNION_TOP + 16.0, UNION_CLOCK)


def _union():
    """The rotary union on the outer case ahead of the swivel: the fixed
    lines come into it, and it hands pressure and return across the
    bearings to the motors and actuators that turn."""
    t = math.radians(UNION_CLOCK)
    body = common.sector_block(UNION_X[0], UNION_X[1], R_OUT - 1.0, UNION_TOP,
                               t - 0.075, t + 0.075, 4)
    ports = [common.radial_pin(hyd_port(1, k)[0], UNION_TOP - 1.0,
                               hyd_port(1, k)[1], 7.0, UNION_CLOCK, 12)
             for k in (0, 1)]
    return mesh.join(body, *ports)


def build():
    out = {}
    fb1 = square(N["x_brg1"])
    f1, f2 = oblique(1), oblique(2)
    fa = square(N["x_aft"])
    # 30 mm long: one set of hangers, in its middle
    out["swivel_fixed_ring"] = _duct(square(N["x_fixed0"]), fb1,
                                     front_flange=False, stiff=False, hang=15.0)
    out["swivel_duct_fwd"] = _duct(fb1, f1)
    out["swivel_duct_mid"] = _duct(f1, f2)
    out["swivel_duct_aft"] = _duct(f2, fa)
    out["swivel_bearing_1"] = _race(fb1)
    out["swivel_bearing_2"] = _race(f1)
    out["swivel_bearing_3"] = _race(f2)
    out["swivel_drive_1"] = _drive(fb1, DRIVE_CLOCKS[1])
    out["swivel_drive_2"] = _drive(f1, DRIVE_CLOCKS[2])
    out["swivel_drive_3"] = _drive(f2, DRIVE_CLOCKS[3])
    out["swivel_rotary_union"] = _union()
    out["nozzle_static_ring"] = _static_ring()
    out["nozzle_conv_flaps"], out["nozzle_conv_seals"] = _flap_set(
        X_S1, X8, N["conv_t"], 12.0, 10.0)
    out["nozzle_div_flaps"], out["nozzle_div_seals"] = _flap_set(
        X8, X9, N["div_t"], 10.0, 14.0)
    out["nozzle_ext_flaps"] = _ext()
    out["nozzle_hinges"] = _hinges()
    out["nozzle_div_links"] = _div_links()
    out["nozzle_unison_ring"] = _unison()
    out["nozzle_actuators"] = _actuators()
    return out


# which parts turn with which bearing, front to back
GROUPS = {
    "fwd": ["swivel_duct_fwd", "swivel_bearing_1", "swivel_drive_2"],
    "mid": ["swivel_duct_mid", "swivel_bearing_2", "swivel_drive_3"],
    "aft": ["swivel_duct_aft", "swivel_bearing_3", "nozzle_static_ring",
            "nozzle_conv_flaps", "nozzle_conv_seals", "nozzle_div_flaps",
            "nozzle_div_seals", "nozzle_ext_flaps", "nozzle_hinges",
            "nozzle_div_links", "nozzle_unison_ring", "nozzle_actuators"],
}


def _rot(k, ang, p):
    """Rodrigues: p turned by ang about the unit axis k through the origin."""
    c, s = math.cos(ang), math.sin(ang)
    dot = sum(k[i] * p[i] for i in range(3))
    cr = (k[1] * p[2] - k[2] * p[1], k[2] * p[0] - k[0] * p[2],
          k[0] * p[1] - k[1] * p[0])
    return tuple(p[i] * c + cr[i] * s + k[i] * dot * (1 - c) for i in range(3))


def fold(psi):
    """The jet's direction with the middle duct turned psi and the aft duct
    -psi on their bearings, the front bearing at zero."""
    d = _rot(oblique(2)[1], -psi, (1.0, 0.0, 0.0))
    return _rot(oblique(1)[1], psi, d)


def swivel_pose(pitch_deg, yaw_deg=0.0):
    """Bearing angles (front, middle, aft), radians, that point the jet
    pitch_deg down and tilted yaw_deg to the side -- in the hover, with the
    jet straight down, yaw is a sideways tilt of it. The middle and aft ducts set how
    far the jet folds off the axis; the front bearing turns the fold into
    the plane asked for."""
    p, y = math.radians(pitch_deg), math.radians(yaw_deg)
    t = (math.cos(p) * math.cos(y), math.sin(y), -math.sin(p) * math.cos(y))
    want = math.acos(max(-1.0, min(1.0, t[0])))
    if want < 1e-9:
        return 0.0, 0.0, 0.0
    lo, hi = 0.0, math.pi
    for _ in range(60):
        mid = (lo + hi) / 2.0
        if math.acos(max(-1.0, min(1.0, fold(mid)[0]))) < want:
            lo = mid
        else:
            hi = mid
    psi = (lo + hi) / 2.0
    d = fold(psi)
    phi = math.atan2(t[2], t[1]) - math.atan2(d[2], d[1])
    phi = (phi + math.pi) % (2.0 * math.pi) - math.pi
    return phi, psi, -psi


def max_fold_deg():
    return math.degrees(math.acos(max(-1.0, min(1.0, fold(math.pi)[0]))))
