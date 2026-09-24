"""Externals: the accessory gearbox and what hangs on it, the tower shaft,
the fuel, oil and coolant lines, the two FADEC channels and their looms, and
the mounts.

Everything is under the engine or low on its flanks. The Nyx carries two of
these side by side with a weapons bay between and above them, so the top of
the engine and its inboard side have to stay clean; the gearbox hangs at six
o'clock and the controls sit at four and eight.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec          # noqa: E402
import mesh          # noqa: E402
from parts import common   # noqa: E402

G = spec.GEARBOX
P = spec.PATHS
SEG_S = spec.RES["small_revolve"]
PIPE = spec.RES["pipe_segments"]


def outer_od(x):
    from parts import fan
    if x < spec.SPLITTER3_X:
        return fan.case_od(x)
    return spec.annulus(P["third"], x)[1] + spec.WALL["outer_case"]


def gb_centre_z():
    return -(G["r_top"] + G["depth"] / 2.0)


def _rounded_box(x0, x1, yc, zc, a, b, n_exp=5.0, m=48, taper=18.0):
    """A solid with a superellipse section, rounded off at both ends -- the
    shape of a cast housing rather than a brick."""
    rings = []
    xs = [x0, x0 + taper * 0.35, x0 + taper, x1 - taper, x1 - taper * 0.35, x1]
    sc = [0.82, 0.95, 1.0, 1.0, 0.95, 0.82]
    for x, s in zip(xs, sc):
        ring = []
        for k in range(m):
            t = 2.0 * math.pi * k / m
            c, sn = math.cos(t), math.sin(t)
            ring.append((x, yc + a * s * math.copysign(abs(c) ** (2 / n_exp), c),
                         zc + b * s * math.copysign(abs(sn) ** (2 / n_exp), sn)))
        rings.append(ring)
    verts = [p for r in rings for p in r]
    faces = []
    for i in range(len(rings) - 1):
        for k in range(m):
            k2 = (k + 1) % m
            faces.append((i * m + k, i * m + k2, (i + 1) * m + k2, (i + 1) * m + k))
    faces.append(tuple(range(m - 1, -1, -1)))
    base = (len(rings) - 1) * m
    faces.append(tuple(range(base, base + m)))
    return common.orient((verts, faces))


def build():
    out = {}
    out["gearbox"] = _gearbox()
    out["gearbox_mounts"] = _gearbox_mounts()
    out["towershaft"] = _towershaft()
    out.update(_gearbox_units())
    out["oil_tank"] = _oil_tank()
    out["oil_lines"] = _oil_lines()
    out["fuel_lines"] = _fuel_lines()
    out.update(_fadec())
    out["mount_trunnions"] = _trunnions()
    out["mount_aft_lug"] = _aft_lug()
    out["coolant_lines"] = _coolant()
    out["mode_valve_actuators"] = _mode_valve_actuators()
    out.update(_hydraulics())
    return out


def _mode_valve_actuators():
    """Two actuators high on the forward case, at 45 and 135 degrees, that
    drive the mode valve's unison ring through the case. Each stands on two
    lugs, and its rod enters the case through a boss."""
    parts = []
    x0, x1 = 530.0, 660.0
    rb = 20.0
    for clock in (45.0, 135.0):
        r_c = max(outer_od(x0), outer_od(x1)) + rb + 8.0
        body = mesh.pipe([common.polar(x0, r_c, clock),
                          common.polar(x1 - 30.0, r_c, clock)], rb, 20)
        rod = mesh.pipe([common.polar(x1 - 34.0, r_c, clock),
                         common.polar(x1, r_c, clock),
                         common.polar(x1 + 20.0, outer_od(x1 + 20.0) + 8.0, clock),
                         common.polar(x1 + 20.0, outer_od(x1 + 20.0) - 2.0, clock)],
                        7.0, 12, bend=10.0)
        boss = common.radial_pin(x1 + 20.0, outer_od(x1 + 20.0) - 1.0,
                                 outer_od(x1 + 20.0) + 6.0, 14.0, clock, 20)
        lugs = [common.radial_pin(x, outer_od(x) - 1.0, r_c - rb + 3.0, 9.0,
                                  clock, 12) for x in (x0 + 20.0, x1 - 50.0)]
        parts += [body, rod, boss] + lugs
    return mesh.join(*parts)


def _hydraulics():
    """A hydraulic pump under the gearbox's front and the lines from it aft
    along the lower flanks at 45 degrees below the horizontal: one pair to
    the front swivel bearing's motor, the other to the rotary union that
    carries pressure across the bearings to the two motors and the four
    nozzle actuators that turn with the ducts."""
    out = {}
    zc = gb_centre_z()
    zp = zc - G["depth"] / 2.0 - 32.0
    pv, pf = mesh.revolve_ring([(430.0, 6.0), (520.0, 6.0), (520.0, 38.0),
                                (430.0, 38.0)], 32)
    out["hydraulic_pump"] = ([(x, y, z + zp) for (x, y, z) in pv], pf)
    from parts import nozzle as nzl
    lines = []
    # Laid along the case, just over its ribs and lifted over the mid
    # flange, the way a line clipped to a case runs. At a fixed 530 mm they
    # stood 50-80 mm off the case the whole way aft, like wires strung past
    # the engine.
    cr = spec.CASE_RIBS

    def over_case(x, extra):
        r = outer_od(x) + cr["h"] + 6.0 + 2.0 + extra
        # and over what rings the case: the FADEC harness, the mid flange
        # with the fuel manifold and nozzles on it, the reheat manifold
        for (a, b, top) in ((830.0, 970.0, 496.0), (1320.0, 1440.0, 485.0),
                            (2210.0, 2380.0, 491.0)):
            if a <= x <= b:
                r = max(r, top + 6.0 + 2.5 + extra)
        return r
    for sy, clock in ((-1.0, -135.0), (1.0, -45.0)):
        for sz, dz in ((-1.0, 0.0), (1.0, 14.0)):
            # the pair on each side run side by side, 14 mm apart round
            # the case rather than one over the other
            ck = clock + sy * (dz / 470.0) * 57.3
            run = [common.polar(x, over_case(x, 0.0), ck)
                   for x in (620.0, 780.0, 840.0, 960.0, 1060.0, 1260.0,
                             1330.0, 1430.0, 1520.0, 2000.0, 2240.0, 2310.0,
                             2370.0, 2460.0, 2800.0)]
            path = [(470.0, sy * 20.0, zp),
                    (470.0, sy * 200.0, zp - 6.0 - dz)] + run
            # down onto a port: the motor's two on the left, the union's
            # on the right
            xp, rp, cp = nzl.hyd_port(0 if sy < 0 else 1, 0 if dz == 0.0 else 1)
            path += [common.polar(xp, rp + 30.0, cp),
                     common.polar(xp, rp - 1.0, cp)]
            lines.append(mesh.pipe(path, 6.0, 12, bend=30.0))
    out["hydraulic_lines"] = mesh.join(*lines)
    return out


# --------------------------------------------------------------------------

def _gearbox():
    """The accessory gearbox: a cast housing along the bottom of the engine,
    with a drive pad for every unit on it."""
    zc = gb_centre_z()
    body = _rounded_box(G["x0"], G["x1"], 0.0, zc, G["width"] / 2.0,
                        G["depth"] / 2.0)
    # stiffening ribs round the housing
    ribs = []
    for k in range(1, 6):
        x = G["x0"] + (G["x1"] - G["x0"]) * k / 6.0
        ribs.append(_rounded_box(x - 4.0, x + 4.0, 0.0, zc,
                                 G["width"] / 2.0 + 4.0, G["depth"] / 2.0 + 4.0,
                                 taper=2.0))
    return mesh.join(body, *ribs)


def _gearbox_mounts():
    """Two hanger links from the gearbox's top to the case, fore and aft."""
    parts = []
    zc = gb_centre_z()
    top = -zc - G["depth"] / 2.0          # radius of the gearbox's top face
    for x in (G["x0"] + 15.0, G["x1"] - 40.0):
        r0 = outer_od(x) - 5.0
        for y in (-70.0, 70.0):
            parts.append(mesh.pipe([(x, y, -r0 * math.cos(math.asin(y / r0))),
                                    (x, y, -(top + 6.0))], 8.0, 12))
    return mesh.join(*parts)


def _towershaft():
    """From the bevel on the HP shaft's front stub, down the inside of the
    fat bottom strut of the fan frame, to the gearbox. On the engine the
    strut is hollow and is the shaft's tunnel; here the struts are solid, so
    the shaft is inside one."""
    x = G["towershaft_x"]
    rb = G["bevel_r"]
    zc = gb_centre_z()
    r_end = -zc - 10.0
    shaft = mesh.pipe([(x, 0.0, -(rb + 4.0)), (x, 0.0, -r_end)],
                      G["towershaft_r"], 20)
    # the pinion on its inner end, meshing with the bevel's teeth
    pinion = mesh.revolve_ring([(-6.0, 6.0), (10.0, 6.0), (10.0, 22.0),
                                (-6.0, 16.0)], 24)
    v, f = pinion
    # turn the pinion's axis from +X to -Z and seat it on the bevel
    v = [(x + y, z, -(rb - 2.0) - px) for (px, y, z) in v]
    return mesh.join(shaft, common.orient((v, f)))


def _gearbox_units():
    """The two generators on the gearbox's flanks, the fuel pump on its
    front face and the fuel metering unit under it."""
    out = {}
    zc = gb_centre_z()
    a = G["width"] / 2.0
    rg = 62.0
    for side, sy in (("l", 1.0), ("r", -1.0)):
        y = sy * (a + rg - 5.0)
        body = mesh.revolve_ring([(600.0, 0.0 + 8.0), (820.0, 8.0),
                                  (820.0, rg), (600.0, rg)], 40)
        v, f = body
        v = [(px, py + y, pz + zc) for (px, py, pz) in v]
        # cooling fins round the body
        fins = []
        for k in range(8):
            xf = 620.0 + 24.0 * k
            fv, ff = mesh.revolve_ring([(xf, rg - 1.0), (xf + 6.0, rg - 1.0),
                                        (xf + 6.0, rg + 5.0), (xf, rg + 5.0)], 40)
            fins.append(([(px, py + y, pz + zc) for (px, py, pz) in fv], ff))
        out[f"generator_{side}"] = mesh.join((v, f), *fins)
    # fuel pump on the front face
    pv, pf = mesh.revolve_ring([(G["x0"] - 52.0, 6.0), (G["x0"] + 6.0, 6.0),
                                (G["x0"] + 6.0, 40.0), (G["x0"] - 52.0, 40.0)], 32)
    out["fuel_pump"] = ([(px, py, pz + zc - 16.0) for (px, py, pz) in pv], pf)
    # fuel metering unit under the gearbox
    out["fuel_metering_unit"] = _rounded_box(
        700.0, 860.0, 0.0, zc - G["depth"] / 2.0 - 26.0, 70.0, 30.0, 4.0, 32,
        taper=10.0)
    return out


def _oil_tank():
    """The oil tank on the gearbox's aft face: a pill-shaped tank on the
    engine centreline."""
    zc = gb_centre_z()
    r = 56.0
    x0, x1 = G["x1"] - 4.0, G["x1"] + 180.0
    prof = [(x0, 0.001), (x0, r)]
    for i in range(1, 9):
        a = 0.5 * math.pi * i / 8
        prof.append((x1 - r + r * math.sin(a), r * math.cos(a) + 0.001))
    v, f = mesh.revolve_open(prof, SEG_S, cap_start=True, cap_end=True)
    return ([(px, py, pz + zc) for (px, py, pz) in v], f)


def _oil_lines():
    """Oil feed up the fan frame's bottom strut to the front sump, and the
    rear sump's scavenge along the bottom of the case, in through the bottom
    service strut and the mid-turbine-frame vane behind it, down the frame's
    web to the sump."""
    zc = gb_centre_z()
    top = -zc - G["depth"] / 2.0
    parts = []
    x_f = spec.FAN_FRAME["x0"] + 15.0
    r_sump = spec.BEARINGS[0][3] + spec.SUMP_T - 1.0
    parts.append(mesh.pipe([(x_f, 0.0, -(top + 8.0)), (x_f, 0.0, -r_sump)],
                           6.0, PIPE))
    xw = spec.BEARINGS[3][1] + spec.BEARING_WIDTH["roller"] / 2 + 5.0 + spec.SUMP_T / 2
    r_run = 495.0
    xt = G["x1"] + 180.0
    parts.append(mesh.pipe([(xt - 10.0, 0.0, zc), (xt + 60.0, 0.0, zc),
                            (xt + 140.0, 0.0, -r_run), (xw, 0.0, -r_run),
                            (xw, 0.0, -(spec.BEARINGS[3][3] - 6.0))],
                           6.0, PIPE, bend=40.0))
    return mesh.join(*parts)


def _fuel_lines():
    """Two lines from the fuel metering unit: the main burner feed to the
    manifold ring, and the reheat feed on aft to the augmentor's manifold.
    They run at seven o'clock, clear of the oil line at six."""
    zc = gb_centre_z()
    z_fmu = zc - G["depth"] / 2.0 - 26.0
    clock = -112.0
    c = spec.COMBUSTOR
    rm = outer_od(c["manifold_x"]) + c["manifold_tube_r"] + 3.0
    from parts import augmentor
    fc = augmentor.fuel_control_inlet()
    r_run = 500.0
    # out of the metering unit's side, under the generator, then up to the
    # run at seven o'clock
    main = mesh.pipe([(800.0, -40.0, z_fmu), (800.0, -195.0, z_fmu - 30.0),
                      (960.0, *common.polar(0.0, r_run, clock)[1:]),
                      common.polar(c["manifold_x"] - 60.0, r_run, clock),
                      common.polar(c["manifold_x"], rm + 8.0, clock)],
                     8.0, PIPE, bend=40.0)
    # the reheat feed runs on aft to the reheat fuel control on the case
    reheat = mesh.pipe([(770.0, -40.0, z_fmu), (770.0, -212.0, z_fmu - 52.0),
                        (950.0, *common.polar(0.0, r_run + 18.0, clock)[1:]),
                        common.polar(fc[0] - 90.0, r_run + 18.0, clock),
                        (fc[0] - 30.0, fc[1], fc[2]), fc],
                       9.0, PIPE, bend=40.0)
    return mesh.join(main, reheat)


def _fadec():
    """Two FADEC channels, one each side low on the case, on four stand-off
    posts each, with a loom from each down to the gearbox's sensors."""
    out = {}
    x0, x1 = 920.0, 1260.0
    for name, clock in (("fadec_a", -32.0), ("fadec_b", -148.0)):
        r0 = outer_od(x0) + 12.0
        rc = r0 + 30.0
        a = math.radians(clock)
        # the box, built at clock 0 and turned
        v, f = _rounded_box(x0, x1, rc, 0.0, 30.0, 85.0, 6.0, 40, taper=12.0)
        box = (mesh.rot_x(v, a), f)
        posts = []
        for xp in (x0 + 40.0, x1 - 40.0):
            for off in (-55.0, 55.0):
                p0 = (xp, outer_od(xp) - 8.0, off)
                p1 = (xp, r0 + 3.0, off)
                pv, pf = mesh.pipe([p0, p1], 7.0, 12)
                posts.append((mesh.rot_x(pv, a), pf))
        out[name] = mesh.join(box, *posts)
    zc = gb_centre_z()
    looms = []
    # each loom leaves its FADEC's front end, runs down the case under the
    # hydraulic lines (which are at 45 degrees and 530 mm out) and plugs
    # into the gearbox's aft sensor pad
    for clock, c_mid, sy in ((-32.0, -60.0, 1.0), (-148.0, -120.0, -1.0)):
        r = outer_od(x0) + 12.0 + 30.0
        p0 = common.polar(x0 + 10.0, r, clock)
        p1 = common.polar(x0 - 20.0, 480.0, c_mid)
        p2 = (G["x1"] - 12.0, sy * (G["width"] / 2.0 - 30.0), zc + 30.0)
        looms.append(mesh.pipe([p0, p1, p2], 7.0, 14, bend=30.0))
    out["harnesses"] = mesh.join(*looms)
    return out


def _trunnions():
    """The forward mounts: a trunnion each side at 3 and 9 o'clock on the fan
    frame's outer ring, right over the struts that carry the load in."""
    m = spec.MOUNTS
    x = m["x_fwd"]
    r0 = spec.annulus(P["third"], x)[1]
    parts = []
    for clock in (0.0, 180.0):
        parts.append(common.radial_pin(x, r0 + 1.0, r0 + 6.0 + m["trunnion_len"],
                                       m["trunnion_r"], clock, 28))
        # the pad it stands on
        parts.append(common.radial_pin(x, r0 + 1.0, r0 + 16.0,
                                       m["trunnion_r"] + 8.0, clock, 28))
    return mesh.join(*parts)


def _aft_lug():
    """The thrust link lug on top of the rear case: a clevis the aircraft's
    aft mount link pins into."""
    x = spec.MOUNTS["x_aft"]
    r0 = spec.annulus(P["third"], x)[1] + spec.WALL["outer_case"]
    parts = []
    for dy in (-22.0, 22.0):
        v, f = mesh.box(x, dy, r0 + 26.0, 70.0, 12.0, 56.0)
        parts.append((v, f))
    parts.append(mesh.box(x, 0.0, r0 + 3.0, 90.0, 70.0, 10.0))
    parts.append(mesh.pipe([(x, -34.0, r0 + 38.0), (x, 34.0, r0 + 38.0)], 9.0, 16))
    return mesh.join(*parts)


def _coolant():
    """Supply and return lines between the third-stream heat exchanger and
    the aircraft's coolant loop, out through the top of the case."""
    h = spec.TMS_HX
    clock = 75.0
    parts = []
    for x in (h["x0"] + 50.0, h["x1"] - 50.0):
        r_hx = spec.annulus(P["third"], x)[0] + 20.0
        r_up = outer_od(x) + 40.0
        path = [common.polar(x, r_hx, clock), common.polar(x, r_up, clock),
                common.polar(x - 160.0, r_up + 10.0, clock)]
        parts.append(mesh.pipe(path, 10.0, PIPE, bend=30.0))
        # the interface flange at the end
        e = common.polar(x - 160.0, r_up + 10.0, clock)
        fv, ff = mesh.revolve_ring([(-4.0, 10.0), (4.0, 10.0), (4.0, 20.0),
                                    (-4.0, 20.0)], 24)
        parts.append(([(px + e[0], py + e[1], pz + e[2]) for (px, py, pz) in fv], ff))
    return mesh.join(*parts)
