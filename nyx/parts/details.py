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
    formation lights  electroluminescent strips on the forebody's shoulders
                      and up the fins, for a wingman at night
    refuelling door   the air-refuelling receptacle's door on the spine
    static wicks      discharge wicks on the trailing edges of the wing tips
                      and fin tips, where static bleeds off in flight
    gun               a six-barrel rotary cannon in the starboard shoulder,
                      above the intake duct, firing through a trough in the
                      skin ahead of the wing root, hung from the skin on two
                      posts

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

IRST_X = (2560.0, 3000.0)    # ball centre to the housing's tail
IRST_R = 72.0                # the sensor ball
IRST_A = 112.0               # the housing's half-width and height at its
IRST_H = 168.0               # fullest, just behind the ball


def _irst():
    """A teardrop housing that starts at the sensor ball's centre and
    swallows its back half -- the ball looks out ahead and to the sides --
    then fines away along the spine to the windscreen."""
    x0, x1 = IRST_X
    z_ball = shapes.z_up(x0, 0.0) - 3.0 + IRST_R + 6.0
    rings = []
    n_ring = 32
    for i in range(0, 25):
        t = i / 24.0
        x = x0 + (x1 - x0) * t
        # from just inside the ball's silhouette to its fullest, then a
        # long fine taper to a point on the skin
        grow = min(1.0, t / 0.18)
        a = (IRST_R * 0.92) + (IRST_A - IRST_R * 0.92) * (grow ** 0.5)
        h = (IRST_R * 2.0 + 2.0) + (IRST_H - IRST_R * 2.0 - 2.0) * (grow ** 0.5)
        fade = 1.0 - max(0.0, (t - 0.18) / 0.82) ** 1.5
        a, h = a * fade + 2.0, h * fade + 2.0
        ring = []
        for k in range(n_ring + 1):
            ph = math.pi * k / n_ring
            y = a * math.cos(ph)
            ring.append((x, y, shapes.z_up(x, y) - 3.0 + h * math.sin(ph)))
        for k in range(n_ring - 1, 0, -1):
            y = a * math.cos(math.pi * k / n_ring)
            ring.append((x, y, shapes.z_up(x, y) - 3.0))
        rings.append(ring)
    fairing = shapes.loft_rings(rings)
    wv, wf = mesh.revolve_closed([(x0 - IRST_R * math.cos(math.pi * k / 24),
                                   max(0.5, IRST_R * math.sin(math.pi * k / 24)))
                                  for k in range(25)], 48)
    window = ([(x, y, z + z_ball) for (x, y, z) in wv], wf)
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
        # under the cockpit, aft of the nose gear's bay
        "antenna_ventral": _blade(3700.0, 0.0, lambda x, y: shapes.z_dn(x, y),
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


# --------------------------------------------------------------------------
# gun

GUN_Y = 1050.0
GUN_X = (6420.0, 8150.0)     # muzzles to the back of the drive
GUN_DEPTH = (100.0, 120.0)   # the axis this far under the skin at each end


def gun_axis_z(x):
    """The gun's axis: straight, from GUN_DEPTH[0] under the skin at the
    muzzles to GUN_DEPTH[1] under it at the back -- the skin falls away
    aft, so it is pitched about two degrees nose-up to stay inside."""
    x0, x1 = GUN_X
    z0 = shapes.z_up(x0, GUN_Y) - GUN_DEPTH[0]
    z1 = shapes.z_up(x1, GUN_Y) - GUN_DEPTH[1]
    return z0 + (z1 - z0) * (x - x0) / (x1 - x0)


def _gun():
    x0, x1 = GUN_X
    zc = gun_axis_z
    parts = []
    # six barrels round the axis, held by two clamp rings
    for k in range(6):
        a = 2.0 * math.pi * k / 6
        dy, dz = 34.0 * math.cos(a), 34.0 * math.sin(a)
        xb = x0 + 1500.0
        parts.append(mesh.pipe([(x0, GUN_Y + dy, zc(x0) + dz),
                                (xb, GUN_Y + dy, zc(xb) + dz)], 12.0, 16))
    for xc in (x0 + 60.0, x0 + 620.0):
        parts.append(mesh.pipe([(xc - 14.0, GUN_Y, zc(xc - 14.0)),
                                (xc + 14.0, GUN_Y, zc(xc + 14.0))], 52.0, 32))
    # the rotor housing and the drive and feed at the back
    xa, xb = x0 + 1460.0, x1 - 160.0
    parts.append(mesh.pipe([(xa, GUN_Y, zc(xa)), (xb, GUN_Y, zc(xb))], 64.0, 40))
    parts.append(mesh.box(x1 - 80.0, GUN_Y, zc(x1 - 80.0), 160.0, 150.0, 120.0))
    # two mounting posts up to the skin's inside
    for xp in (x0 + 900.0, x1 - 250.0):
        top = shapes.z_up(xp, GUN_Y, spec.SKIN_T) + 2.0
        parts.append(mesh.pipe([(xp, GUN_Y, zc(xp) + 40.0), (xp, GUN_Y, top)], 14.0, 16))
    return mesh.join(*parts)


def _gun_port():
    """The trough the gun fires along: open from the muzzles forward, cut
    through the skin down to the barrels' axis."""
    x0 = GUN_X[0]
    zc = gun_axis_z(x0)
    return mesh.join(
        mesh.pipe([(x0 - 420.0, GUN_Y, zc), (x0 + 40.0, GUN_Y, zc)], 62.0, 32),
        mesh.box(x0 - 190.0, GUN_Y, zc + 200.0, 460.0, 124.0, 400.0))


# --------------------------------------------------------------------------
# panel seams

SEAM_W = 5.0          # the gap between two panels, as it reads on the skin
SEAM_H = 0.8          # standing this proud of the skin, and 0.6 into it


def _surface(x, y, upper):
    return shapes.z_up(x, y) if upper else shapes.z_dn(x, y)


def _ribbon(pts, upper, width=SEAM_W, proud=SEAM_H):
    """A seam along a polyline of (x, y) on the upper or lower skin: a thin
    strip following the surface, `width` wide."""
    s = 1.0 if upper else -1.0
    rings = []
    n = len(pts)
    for i, (x, y) in enumerate(pts):
        xa, ya = pts[max(0, i - 1)]
        xb, yb = pts[min(n - 1, i + 1)]
        dx, dy = xb - xa, yb - ya
        L = math.hypot(dx, dy) or 1.0
        # across the seam, in plan
        cx, cy = -dy / L * width / 2, dx / L * width / 2
        # across the strip in steps, so a wide one follows the skin's
        # curvature instead of spanning it flat -- a 50 mm strip laid flat
        # across the forebody's crown was under the skin in its middle and
        # showed as two lines
        k = max(1, int(width / 8.0))
        across = [-1.0 + 2.0 * i / k for i in range(k + 1)]
        ring = []
        for f in across:
            px, py = x + cx * f, y + cy * f
            ring.append((px, py, _surface(px, py, upper) - s * 0.6))
        for f in reversed(across):
            px, py = x + cx * f, y + cy * f
            ring.append((px, py, _surface(px, py, upper) + s * proud))
        rings.append(ring)
    return shapes.loft_rings(rings)


def _across(x, upper, frac=0.97, n=60):
    """A seam straight across the skin at station x, chine to chine."""
    w = shapes.half_width(x) * frac
    return _ribbon([(x, -w + 2 * w * i / n) for i in range(n + 1)], upper)


def _panel(x0, x1, y0, y1, upper, teeth=0, tooth=60.0, n=24):
    """An access panel's outline, its fore and aft edges serrated with
    `teeth` teeth as the airframe's edges across the flow are."""
    def edge(x, sgn):
        pts = []
        m = max(2, teeth * 2) if teeth else n
        for i in range(m + 1):
            y = y0 + (y1 - y0) * i / m
            dx = (tooth if (teeth and i % 2 == 1) else 0.0) * sgn
            pts.append((x + dx, y))
        return pts
    fwd = edge(x0, 1.0)
    aft = edge(x1, -1.0)
    side = lambda y: [(x0 + (x1 - x0) * i / n, y) for i in range(n + 1)]
    return [_ribbon(fwd, upper), _ribbon(aft, upper),
            _ribbon(side(y0), upper), _ribbon(side(y1), upper)]


def _seams():
    # (the forward-fuselage joint only over the top: underneath, the nose
    # gear's doors are the joints)
    parts = [_across(1300.0, True), _across(1300.0, False),     # radome
             _across(2250.0, True)]                              # fwd fuselage
    # the air-refuelling receptacle's door, on the spine just aft of the
    # canopy where the boom operator can see it and the pilot need not, and
    # the door's hinge line across its aft edge
    parts += _panel(5880.0, 6230.0, -135.0, 135.0, True, teeth=3, tooth=45.0)
    parts.append(_ribbon([(6190.0, -120.0 + 240.0 * i / 20) for i in range(21)],
                         True))
    # spine access panels
    parts += _panel(6350.0, 7450.0, -260.0, 260.0, True, teeth=4)
    parts += _panel(8150.0, 9450.0, -300.0, 300.0, True, teeth=4)
    # engine bay doors over and under each nacelle
    for sy in (1.0, -1.0):
        y0, y1 = sorted((sy * 380.0, sy * 1180.0))
        parts += _panel(10300.0, 12700.0, y0, y1, True, teeth=5)
        parts += _panel(10000.0, 12700.0, y0, y1, False, teeth=5)
    return mesh.join(*parts)


# --------------------------------------------------------------------------
# formation lights

def _formation_light():
    """The starboard set of low-voltage electroluminescent strips a wingman
    keeps station on at night: one on the forebody's shoulder under the
    canopy, one up the fin's outboard face. Flush strips, 50 mm wide,
    standing a millimetre proud."""
    parts = []
    x0, x1 = 2750.0, 3250.0
    pts = [(x0 + (x1 - x0) * i / 20, 0.0) for i in range(21)]
    pts = [(x, shapes.half_width(x) * 0.80) for (x, _) in pts]
    parts.append(_ribbon(pts, True, width=50.0, proud=1.0))
    # and one up the fin's outboard face, ahead of the rudder
    P = surfaces.FinPlace()
    F = spec.FIN
    rings = []
    for j in range(13):
        sp = F["span"] * (0.28 + 0.34 * j / 12)
        c = P.chord(sp)
        u0, u1 = 0.42 - 25.0 / c, 0.42 + 25.0 / c
        ring = []
        for (u, off) in ((u0, -0.6), (u1, -0.6), (u1, 1.0), (u0, 1.0)):
            v = shapes.naca_t(u, F["tc"]) + off / c
            ring.append(P(sp, u, v))
        rings.append(ring)
    parts.append(shapes.loft_rings(rings))
    return mesh.join(*parts)


def build():
    out = {}
    fl = _formation_light()
    out["formation_light_r"], out["formation_light_l"] = fl, _mirror(fl)
    out["panel_seams"] = _seams()
    out["gun"] = _gun()
    out["cut:fuselage_skin"] = _gun_port()
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
