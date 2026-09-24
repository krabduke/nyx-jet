"""The small things that make it an aircraft rather than a shape.

    aft closure       the plate that closes each nacelle round its engine's
                      swivel, just inside the skin's lip: a round hole for
                      the fixed ring and a notch for the front bearing's
                      motor. With the nacelles open you looked straight
                      into the engine bay.
    IRST              the infrared search-and-track sensor ahead of the
                      windscreen: a faired housing with a glass ball in its
                      nose
    air data probes   two pitot-static probes on the nose, well ahead of
                      anything that disturbs the flow, and two angle-of-
                      attack vanes on the forebody's sides
    antennas          blade antennas: a UHF/IFF blade on the spine, one
                      under the forebody, and a datalink blade under the
                      valley between the nacelles
    lights            navigation lights on the wingtips -- red to port,
                      green to starboard -- and white tail lights on the
                      fin tips
    static wicks      discharge wicks on the trailing edges of the wing tips
                      and fin tips, where static bleeds off in flight

Everything here is placed on the surface it stands on by asking shapes (the
body) or surfaces (the wing and fins) where that surface is.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec      # noqa: E402
import shapes    # noqa: E402
import mesh      # noqa: E402
from parts import engines, surfaces   # noqa: E402

M = spec.RES["body_ring_pts"]


def _mirror(part):
    v, f = part
    return shapes.orient(([(x, -y, z) for (x, y, z) in v],
                          [tuple(reversed(c)) for c in f]))


# --------------------------------------------------------------------------
# aft closure

CLOSURE_T = 11.0
HOLE_R = 500.0        # the fixed ring's flange is 492 across, the race 510 aft


def _aft_closure():
    x1 = spec.BODY_END_X - 2.0
    x0 = x1 - CLOSURE_T
    inset = spec.SKIN_T + 0.15
    plate = shapes.loft_rings([shapes.ring(x0, M, inset), shapes.ring(x1, M, inset)])
    n = engines.info()["nozzle"]
    cut = []
    for sy in (1.0, -1.0):
        ey, ez = sy * spec.ENGINE_Y, spec.ENGINE_Z
        cut.append(mesh.pipe([(x0 - 20.0, ey, ez), (x1 + 20.0, ey, ez)], HOLE_R, 96))
        # the front swivel bearing's motor stands out through the plane of
        # the closure at its clock: a round notch for it
        t = math.radians(n["drive1_clock"])
        rc = n["drive1_r"]
        cy, cz = ey + rc * math.cos(t), ez + rc * math.sin(t)
        cut.append(mesh.pipe([(x0 - 20.0, cy, cz), (x1 + 20.0, cy, cz)],
                             n["drive1_rad"] + 8.0, 32))
    return {"aft_closure": plate, "cut:aft_closure": mesh.join(*cut)}


# --------------------------------------------------------------------------
# IRST

IRST_X = (2540.0, 2960.0)
IRST_H = 118.0
IRST_A = 125.0


def _irst():
    x0, x1 = IRST_X
    rings = []
    n_ring = 32
    for i in range(1, 24):
        t = i / 24.0
        x = x0 + (x1 - x0) * t
        # blunt at the front, where the sensor ball is, long and fine aft
        f = (min(1.0, t / 0.22) ** 0.5) * (1.0 - max(0.0, (t - 0.22) / 0.78) ** 1.6)
        a, h = IRST_A * f + 2.0, IRST_H * f + 2.0
        ring = []
        for k in range(n_ring + 1):
            ph = math.pi * k / n_ring
            y = a * math.cos(ph)
            ring.append((x, y, shapes.z_up(x, y) - 3.0 + h * math.sin(ph)))
        # back along the skin underneath, 3 mm into it
        for k in range(n_ring - 1, 0, -1):
            y = a * math.cos(math.pi * k / n_ring)
            ring.append((x, y, shapes.z_up(x, y) - 3.0))
        rings.append(ring)
    tip0 = (x0, 0.0, shapes.z_up(x0, 0.0) - 1.0)
    tip1 = (x1, 0.0, shapes.z_up(x1, 0.0) - 1.0)
    v, f = shapes.loft_tip(tip0, rings, cap_end=False)
    m = len(rings[0])
    base = 1 + (len(rings) - 1) * m
    iv = len(v)
    v = list(v) + [tip1]
    f = list(f) + [(base + (k + 1) % m, base + k, iv) for k in range(m)]
    fairing = shapes.orient((v, f))
    # the window: a glass ball in the housing's nose, turned to look ahead
    xw = x0 + 0.20 * (x1 - x0)
    zw = shapes.z_up(xw, 0.0) - 3.0 + IRST_H * 0.62
    wv, wf = mesh.revolve_closed([(xw - 84.0 + 84.0 * (1 - math.cos(math.pi * k / 24)),
                                   max(0.5, 84.0 * math.sin(math.pi * k / 24)))
                                  for k in range(25)], 48)
    window = ([(x, y, z + zw) for (x, y, z) in wv], wf)
    return {"irst_fairing": fairing, "irst_window": window}


# --------------------------------------------------------------------------
# air data

def _pitot(x, frac):
    """A pitot-static probe standing off the nose's upper side at x."""
    w = shapes.half_width(x)
    y = frac * w
    z = shapes.z_up(x, y)
    p0 = (x, y - 4.0, z - 6.0)
    p1 = (x, y + 38.0, z + 42.0)
    p2 = (x - 240.0, y + 38.0, z + 42.0)
    strut = mesh.pipe([p0, p1], 9.0, 16)
    tube = mesh.pipe([p1, p2], [8.0, 6.0], 16, bend=0.0)
    tipv, tipf = mesh.revolve_closed([(p2[0] + 0.1, 0.5), (p2[0], 4.0),
                                      (p2[0] - 30.0, 3.0), (p2[0] - 34.0, 0.5)], 16)
    tip = ([(xx, yy + p2[1], zz + p2[2]) for (xx, yy, zz) in tipv], tipf)
    return mesh.join(strut, tube, tip)


def _aoa_vane(x):
    """An angle-of-attack vane on the forebody's side, under the chine: a
    swept blade on a round base plate."""
    w = shapes.half_width(x)
    y = 0.86 * w
    z = shapes.z_dn(x, y)
    base = mesh.pipe([(x, y - 6.0, z + 8.0), (x, y + 4.0, z - 3.0)], 26.0, 24)
    blade = shapes.loft_rings([
        [(x - 30.0, y - 2.0, z - 2.0), (x + 30.0, y - 2.0, z - 2.0),
         (x + 30.0, y + 2.0, z - 2.0), (x - 30.0, y + 2.0, z - 2.0)],
        [(x - 4.0, y + 20.0, z - 70.0), (x + 34.0, y + 20.0, z - 70.0),
         (x + 34.0, y + 24.0, z - 70.0), (x - 4.0, y + 24.0, z - 70.0)]])
    return mesh.join(base, blade)


# --------------------------------------------------------------------------
# antennas

def _blade(x, y, z_of, down=False, h=170.0, c0=260.0, c1=110.0, t=12.0):
    """A swept blade antenna on the surface z_of(x, y), standing up (or
    hanging down) from it, rooted 3 mm into the skin."""
    s = -1.0 if down else 1.0
    rings = []
    for j in range(7):
        f = j / 6.0
        c = c0 + (c1 - c0) * f
        xl = x + 0.62 * h * f              # swept back
        tt = t * (1.0 - 0.45 * f)
        ring = []
        for k in range(12):
            u = k / 11.0
            xx = xl + c * u
            half = tt * 0.5 * math.sin(math.pi * min(1.0, u * 1.25) ** 0.7) + 0.6
            ring.append((xx, y + half, 0.0))
        for k in range(11, -1, -1):
            u = k / 11.0
            xx = xl + c * u
            half = tt * 0.5 * math.sin(math.pi * min(1.0, u * 1.25) ** 0.7) + 0.6
            ring.append((xx, y - half, 0.0))
        rings.append([(px, py, z_of(px, y) - s * 3.0 + s * h * f)
                      for (px, py, _) in ring])
    return shapes.loft_rings(rings)


def _antennas():
    return {
        "antenna_dorsal": _blade(7700.0, 0.0, lambda x, y: shapes.z_up(x, y)),
        "antenna_ventral": _blade(1900.0, 0.0, lambda x, y: shapes.z_dn(x, y),
                                  down=True, h=130.0, c0=200.0, c1=90.0),
        "antenna_datalink": _blade(11300.0, 0.0, lambda x, y: shapes.z_dn(x, y),
                                   down=True, h=150.0, c0=240.0, c1=100.0),
    }


# --------------------------------------------------------------------------
# lights and wicks

def _wing_tip_light():
    P = surfaces.WingPlace()
    y = spec.WING["y_tip"]
    x, _, z = P(y, 0.30, 0.0)
    return mesh.pipe([(x - 50.0, y + 6.0, z), (x + 50.0, y + 6.0, z)],
                     [12.0, 12.0], 32)


def _fin_tip_light():
    P = surfaces.FinPlace()
    s = spec.FIN["span"] - 6.0
    x, y, z = P(s, 0.80, 0.0)
    return mesh.pipe([(x - 28.0, y, z), (x + 28.0, y, z)], 10.0, 32)


def _wicks():
    """Discharge wicks off the wing tip's and the fin tip's trailing edges:
    a stiff rod with a fine brush at its end."""
    parts = []
    P = surfaces.WingPlace()
    for y in (6330.0, 6450.0, 6560.0):
        x, _, z = P(y, 1.0, 0.0)
        parts.append(mesh.pipe([(x - 10.0, y, z), (x + 95.0, y, z - 4.0)],
                               [3.2, 2.2], 10))
    F = surfaces.FinPlace()
    for s in (1950.0, 2020.0):
        x, y, z = F(s, 1.0, 0.0)
        parts.append(mesh.pipe([(x - 10.0, y, z), (x + 85.0, y, z)], [3.0, 2.0], 10))
    return mesh.join(*parts)


def build():
    out = {}
    out.update(_aft_closure())
    out.update(_irst())
    probe = _pitot(620.0, 0.55)
    vane = _aoa_vane(1450.0)
    out["air_data_probe_r"], out["air_data_probe_l"] = probe, _mirror(probe)
    out["aoa_vane_r"], out["aoa_vane_l"] = vane, _mirror(vane)
    out.update(_antennas())
    light = _wing_tip_light()
    out["nav_light_r"], out["nav_light_l"] = light, _mirror(light)
    tail = _fin_tip_light()
    out["tail_light_r"], out["tail_light_l"] = tail, _mirror(tail)
    wicks = _wicks()
    out["static_wicks_r"], out["static_wicks_l"] = wicks, _mirror(wicks)
    return out
