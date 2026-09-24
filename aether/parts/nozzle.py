"""The two-dimensional convergent-divergent vectoring nozzle.

Why 2D and not axisymmetric: a rectangular nozzle vectors in pitch by
swinging two flaps rather than a ring of twelve, it blends into a flat
aft fuselage between two engines with no boat-tail gap to drag, and its
flat sides hide the hot parts from the side. It costs weight -- flat walls
carrying pressure have to be stiff -- which is the price of the agility.

Drawn at maximum reheat and zero vector. From front to back:

    transition   round augmentor duct to a 720 mm wide rectangle
    shroud       the outer structure over it, round to rectangular; the third
                 stream runs between the two and leaves through the flap
                 cavities as a cooling film
    sidewalls    the fixed flat sides the flaps run between
    convergent   upper and lower, hinged at the transition, closing to the
      flaps      throat
    divergent    upper and lower, hinged at the throat. Swinging both the
      flaps      same way vectors the jet +/-20 degrees in pitch
    external     the outside skin over each pair, from the shroud to the
      flaps      trailing edge
    actuators    two a side on the sidewalls

The throat and exit heights are the cycle's: the choked throat area at max
reheat and the fully-expanded area ratio at the reheat nozzle pressure ratio,
over the fixed width.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec          # noqa: E402
import mesh          # noqa: E402
from parts import common   # noqa: E402

N = spec.NOZZLE
A = spec.AUGMENTOR
P = spec.PATHS

HW = N["width"] / 2.0              # half-width of the gas path
H_TRANS = N["h_trans"] / 2.0       # half-height at the convergent hinges
H_THROAT = N["h_throat"] / 2.0
H_EXIT = N["h_exit"] / 2.0
SW_OUT = HW + N["sidewall_t"]      # outside face of a sidewall
SAW = 70.0                         # depth of the trailing-edge sawtooth


def _blend_ring(x, r, a, b, n_exp, e, m=96):
    """Points round a section part way from a circle of radius r (e = 0) to
    a superellipse |y/a|^n + |z/b|^n = 1 (e = 1), blended point by point at
    the same parameter. Blending the circle and the superellipse themselves
    rather than their half-widths and exponent is what keeps the corners
    from bulging out: interpolating the exponent alone makes a section whose
    diagonal is longer than either end's."""
    pts = []
    for k in range(m):
        t = 2.0 * math.pi * k / m
        c, s = math.cos(t), math.sin(t)
        ys = a * math.copysign(abs(c) ** (2.0 / n_exp), c)
        zs = b * math.copysign(abs(s) ** (2.0 / n_exp), s)
        pts.append((x, (1.0 - e) * r * c + e * ys, (1.0 - e) * r * s + e * zs))
    return pts


def _shell(stations, m=96):
    """A closed thin-walled duct lofted through blended stations.

    stations: [(x, r_in, a_in, b_in, r_out, a_out, b_out, n_exp, e)], front
    to back. The inside and outside surfaces are joined by a lip at each
    end."""
    rings_in = [_blend_ring(st[0], st[1], st[2], st[3], st[7], st[8], m)
                for st in stations]
    rings_out = [_blend_ring(st[0], st[4], st[5], st[6], st[7], st[8], m)
                 for st in stations]
    verts = [p for r in rings_out for p in r] + [p for r in rings_in for p in r]
    no = len(stations) * m
    faces = []
    for i in range(len(stations) - 1):
        for k in range(m):
            k2 = (k + 1) % m
            a, b = i * m + k, i * m + k2
            c, d = (i + 1) * m + k2, (i + 1) * m + k
            faces.append((a, b, c, d))
            faces.append((no + d, no + c, no + b, no + a))
    last = len(stations) - 1
    for k in range(m):
        k2 = (k + 1) % m
        faces.append((k2, k, no + k, no + k2))
        o0, o1 = last * m + k, last * m + k2
        faces.append((o0, o1, no + o1, no + o0))
    return common.orient((verts, faces))


def _ease(s, hold=0.0):
    """Smoothstep from 0 to 1, held at 0 for the first `hold` of the run."""
    u = min(max((s - hold) / (1.0 - hold), 0.0), 1.0)
    return u * u * (3.0 - 2.0 * u)


def _lerp(a, b, t):
    return a + (b - a) * t


def build():
    out = {}
    out["nozzle_transition"] = _transition()
    out["nozzle_shroud"] = _shroud()
    out["nozzle_sidewalls"] = _sidewalls()
    out.update(_flaps())
    out["nozzle_hinges"] = _hinges()
    out["nozzle_actuators"] = _actuators()
    return out


def _transition():
    x0, x1 = N["x_trans0"], N["x_trans1"]
    r0 = A["liner_r"]
    t = N["trans_t"]
    st = []
    for i in range(13):
        s = i / 12.0
        st.append((_lerp(x0, x1, s), r0, HW, H_TRANS, r0 + t, HW + t,
                   H_TRANS + t, 12.0, _ease(s)))
    return _shell(st)


# The shroud stays round for its first 12 mm, under the flange collar that
# bolts it to the outer case, before it starts to square off.
SHROUD_HOLD = 12.0


def shroud_outer():
    """Radius at the front, and half-width and half-height at the back, of
    the shroud's outside."""
    x0 = N["x_trans0"]
    r0 = spec.annulus(P["third"], x0)[1] + spec.WALL["outer_case"] - spec.SEAT
    return r0, SW_OUT + N["shroud_t"], H_TRANS + N["flap_t"] + 44.0


def _shroud():
    x0, x1 = N["x_trans0"], N["x_trans1"]
    t = N["shroud_t"]
    r0, a1, b1 = shroud_outer()
    hold = SHROUD_HOLD / (x1 - x0)
    st = []
    for i in range(15):
        s = i / 14.0
        st.append((_lerp(x0, x1, s), r0 - t, a1 - t, b1 - t, r0, a1, b1, 10.0,
                   _ease(s, hold)))
    return _shell(st)


def _plate_xz(poly, y0, y1):
    """Extrude a polygon in the (x, z) plane between y0 and y1. Closed."""
    n = len(poly)
    verts = [(x, y0, z) for (x, z) in poly] + [(x, y1, z) for (x, z) in poly]
    faces = [tuple(range(n - 1, -1, -1)), tuple(range(n, 2 * n))]
    for i in range(n):
        i2 = (i + 1) % n
        faces.append((i, i2, n + i2, n + i))
    return common.orient((verts, faces))


def serrated_plate(p0, p1, half_w, t, teeth=0, depth=0.0, up=1.0,
                   front_vertical=False):
    """A plate lying on the line from p0 = (x0, z0) to p1 = (x1, z1) in the
    (x, z) plane, half_w either side of y = 0, t thick on the `up` side.

    With teeth > 0 its trailing edge is a sawtooth `depth` deep: the saw
    edge every current low-observable nozzle carries, which scatters the
    radar return off the trailing edge away from the direction it came. The
    plate is built from convex quads, one column per half-tooth, because a
    concave n-gon would be triangulated across its own notches by anything
    that reads it.

    front_vertical cuts the leading end square to the engine axis instead of
    to the plate, so it meets a plate ahead of it that is cut the same way
    face to face.
    """
    (x0, z0), (x1, z1) = p0, p1
    L = math.hypot(x1 - x0, z1 - z0)
    ux, uz = (x1 - x0) / L, (z1 - z0) / L
    nx, nz = -uz * up, ux * up
    if nz * up < 0:
        nx, nz = -nx, -nz
    cols = max(1, 2 * teeth)
    ys = [-half_w + 2.0 * half_w * i / cols for i in range(cols + 1)]
    aft = [L - (depth if (teeth and i % 2 == 0) else 0.0) for i in range(cols + 1)]

    def pt(u, y, h):
        return (x0 + ux * u + nx * h, y, z0 + uz * u + nz * h)

    verts = []
    for h in (0.0, t):
        u_f = -nx * h / ux if front_vertical else 0.0
        verts += [pt(u_f, y, h) for y in ys]
        verts += [pt(a, y, h) for a, y in zip(aft, ys)]
    n = cols + 1
    F0, A0, F1, A1 = 0, n, 2 * n, 3 * n
    faces = []
    for i in range(cols):
        faces.append((F0 + i, A0 + i, A0 + i + 1, F0 + i + 1))      # gas side
        faces.append((F1 + i, F1 + i + 1, A1 + i + 1, A1 + i))      # back side
        faces.append((F0 + i, F0 + i + 1, F1 + i + 1, F1 + i))      # front edge
        faces.append((A0 + i, A1 + i, A1 + i + 1, A0 + i + 1))      # saw edge
    faces.append((F0, F1, A1, A0))                                   # y = -w end
    faces.append((F0 + cols, A0 + cols, A1 + cols, F1 + cols))       # y = +w end
    return common.orient((verts, faces))


def ribs(p0, p1, half_w, t, n, h=26.0, w=10.0, up=1.0, inset=0.08,
         inset_aft=None):
    """Stiffening ribs standing on a plate's back face, running its length:
    a flat plate carrying 4 bar of nozzle pressure needs them."""
    (x0, z0), (x1, z1) = p0, p1
    L = math.hypot(x1 - x0, z1 - z0)
    ia = inset if inset_aft is None else inset_aft
    parts = []
    for i in range(n):
        y = -half_w + 2.0 * half_w * (i + 0.5) / n
        a = (x0 + (x1 - x0) * inset, z0 + (z1 - z0) * inset)
        b = (x0 + (x1 - x0) * (1 - ia), z0 + (z1 - z0) * (1 - ia))
        # a rib is a narrow plate lifted onto the back face, 1 mm into it
        v, f = serrated_plate(a, b, w / 2.0, h + 1.0, up=up)
        dn = t - 1.0
        (ux, uz) = ((x1 - x0) / L, (z1 - z0) / L)
        nx, nz = -uz * up, ux * up
        if nz * up < 0:
            nx, nz = -nx, -nz
        parts.append(([(px + nx * dn, py + y, pz + nz * dn) for (px, py, pz) in v],
                      f))
    return mesh.join(*parts)


def _flap_poly(x0, z0, x1, z1, t):
    """A flap's section in (x, z) for the upper flap: its gas-side face from
    (x0, z0) to (x1, z1), thickened outward (+z) with square-cut vertical
    ends, so flaps that meet at a hinge meet face to face."""
    ang = math.atan2(z1 - z0, x1 - x0)
    tv = t / math.cos(ang)
    return [(x0, z0), (x1, z1), (x1, z1 + tv), (x0, z0 + tv)]


def flap_lines():
    """Hinge-line geometry the viewer needs to swing the flaps: the
    convergent flaps from the transition to the throat, the divergent ones
    from the throat to the exit."""
    return {
        "conv": (N["x_trans1"], H_TRANS, N["x_throat"], H_THROAT),
        "div": (N["x_throat"], H_THROAT, N["x_exit"], H_EXIT),
    }


def _mirror_z(part):
    v, f = part
    return [(x, y, -z) for (x, y, z) in v], [tuple(reversed(fc)) for fc in f]


def _flaps():
    out = {}
    t = N["flap_t"]
    L = flap_lines()
    conv = mesh.join(_plate_xz(_flap_poly(*L["conv"], t), -HW, HW),
                     ribs(L["conv"][:2], L["conv"][2:], HW, t, 6, h=18.0))
    # the divergent flap carries the same sawtooth as the external flap over
    # it, tooth for tooth; its front end is cut square to the axis, like the
    # convergent flap's back end, so the two meet face to face at the throat
    Ld = math.hypot(L["div"][2] - L["div"][0], L["div"][3] - L["div"][1])
    div = mesh.join(serrated_plate(L["div"][:2], L["div"][2:], HW, t,
                                   teeth=6, depth=SAW, up=1.0,
                                   front_vertical=True),
                    ribs(L["div"][:2], L["div"][2:], HW, t, 6, h=14.0,
                         inset=0.10, inset_aft=(SAW + 12.0) / Ld))
    out["nozzle_conv_flap_upper"] = conv
    out["nozzle_conv_flap_lower"] = _mirror_z(conv)
    out["nozzle_div_flap_upper"] = div
    out["nozzle_div_flap_lower"] = _mirror_z(div)
    # External flaps: from the shroud's trailing edge to the divergent flap's
    # trailing edge, over the ribs, with the sawtooth edge that closes the
    # outside lines. Full width, over the sidewalls' tops.
    _, _, b_sh = shroud_outer()
    x_te = N["x_exit"]
    ang = math.atan2(H_EXIT - H_THROAT, x_te - N["x_throat"])
    z_te = H_EXIT + t / math.cos(ang)
    te = N["ext_t"]
    ext = serrated_plate((N["x_trans1"], b_sh - te), (x_te, z_te), SW_OUT, te,
                         teeth=6, depth=SAW, up=1.0)
    out["nozzle_ext_flap_upper"] = ext
    out["nozzle_ext_flap_lower"] = _mirror_z(ext)
    return out


def _sidewalls():
    """Two flat sidewalls, the gas path's sides from the transition to the
    exit. Their top and bottom edges follow the external flaps down."""
    _, _, b_sh = shroud_outer()
    x0, x1 = N["x_trans1"], N["x_exit"]
    t = N["flap_t"]
    ang = math.atan2(H_EXIT - H_THROAT, x1 - N["x_throat"])
    z_te = H_EXIT + t / math.cos(ang)
    te = N["ext_t"]
    poly = [(x0, -(b_sh - te)), (x1, -z_te), (x1, z_te), (x0, b_sh - te)]
    left = _plate_xz(poly, HW, SW_OUT)
    right = _plate_xz(poly, -SW_OUT, -HW)
    return mesh.join(left, right)


def _hinges():
    """Hinge pins across the full width: the convergent flaps' at the
    transition, and the throat hinge between the convergent and divergent
    flaps. Their ends are let into the sidewalls."""
    parts = []
    r = N["hinge_r"]
    t = N["flap_t"]
    for (x, z) in ((N["x_trans1"], H_TRANS + t / 2.0),
                   (N["x_throat"], H_THROAT + t / 2.0)):
        for sgn in (1.0, -1.0):
            parts.append(mesh.pipe([(x, -SW_OUT + 4.0, sgn * z),
                                    (x, SW_OUT - 4.0, sgn * z)], r, 16))
    return mesh.join(*parts)


def _actuators():
    """Four hydraulic actuators, two on each sidewall's outside face: the
    upper pair swings the upper divergent flap and the lower pair the lower,
    each through a crank on the throat hinge. Moving the pairs together opens
    and closes the exit; moving them opposite ways vectors the jet."""
    parts = []
    ra = N["actuator_r"]
    y = SW_OUT + ra - 0.2
    x0 = N["x_trans1"] + 30.0
    x1 = N["x_throat"] + 20.0
    for sy in (1.0, -1.0):
        for sz in (1.0, -1.0):
            za = sz * (H_TRANS * 0.62)
            zb = sz * (H_THROAT + N["flap_t"] + 10.0)
            body = mesh.pipe([(x0, sy * y, za), (x0 + 0.62 * (x1 - x0), sy * y,
                                                 za + 0.62 * (zb - za))], ra, 16)
            rod = mesh.pipe([(x0 + 0.6 * (x1 - x0), sy * y, za + 0.6 * (zb - za)),
                             (x1, sy * y, zb)], ra * 0.45, 12)
            # clevis lugs welded to the sidewall at each end
            lug0 = mesh.box(x0, sy * (SW_OUT + ra * 0.5), za, 24.0, ra + 1.0, 18.0)
            lug1 = mesh.box(x1, sy * (SW_OUT + ra * 0.5), zb, 24.0, ra + 1.0, 18.0)
            parts += [body, rod, lug0, lug1]
    return mesh.join(*parts)
