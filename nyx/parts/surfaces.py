"""The lifting and control surfaces: wing, leading-edge flaps, flaperons,
canards and fins with rudders.

Every surface is lofted through aerofoil sections placed on its planform.
Control surfaces are cut out of their parent by chord fraction with a 4 mm
hinge gap, and each hangs on hinge fittings that bridge the gap -- so it is
held by the surface it moves on, and is a separate part the viewer can
swing.

The wing's root section is not at the body side: the planform is continued
into the body to y = WING_ROOT_IN, where the body is thicker than the wing
all along the chord. The wing panel therefore runs into the body rather than
stopping at its surface, which is what a wing-body join is; the audits
declare that overlap as the joint.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec      # noqa: E402
import shapes    # noqa: E402
import mesh      # noqa: E402

W = spec.WING
GAP = 4.0
WING_ROOT_IN = 1720.0
NC = spec.RES["wing_chord_pts"]


def mirror(part):
    v, f = part
    return shapes.orient(([(x, -y, z) for (x, y, z) in v],
                          [tuple(reversed(fc)) for fc in f]))


# --------------------------------------------------------------------------
# generic lofting on a planform
# --------------------------------------------------------------------------

def loft_surface(stations, u_fn, place, n=NC, ladder_root=False):
    """A closed loft through aerofoil sections.

    stations: list of span parameters s. u_fn(s) -> (u0, u1) chord-fraction
    range at s. place(s, u, v) -> (x, y, z): where chord fraction u and
    thickness coordinate v (in chord units) land at span s.

    ladder_root closes the root with a quad across the thickness at every
    chord station instead of one n-gon: a root that follows a curved body is
    far from planar, and an n-gon over it is triangulated into fans that cut
    corners straight through the body's curvature.
    """
    rings = []
    for s in stations:
        u0, u1 = u_fn(s)
        loop = shapes.section_loop(u0, u1, place.tc(s), place.camber, n)
        rings.append([place(s, u, v) for (u, v) in loop])
    m = min(len(r) for r in rings)
    if any(len(r) != m for r in rings):
        raise ValueError("sections of one loft must have the same point count")
    if not ladder_root:
        return shapes.loft_rings(rings)
    v, f = shapes.loft_rings(rings, cap_start=False)
    # the root loop is upper u0..u1 (n + 1 points) then lower u1..u0 without
    # the closed edges' duplicates; with both edges closed that is n - 1
    if m != 2 * n:
        raise ValueError("ladder_root needs a section closed at both edges")
    low = lambda k: 2 * n - k           # index of the lower point at chord k
    f = list(f)
    f.append((0, 1, low(1)))
    for k in range(1, n - 1):
        f.append((k, k + 1, low(k + 1), low(k)))
    f.append((n - 1, n, low(n - 1)))
    return shapes.orient((v, f))


class WingPlace:
    """Where points of the wing's sections go. s is spanwise y."""
    camber = 0.012

    def tc(self, y):
        t = (y - W["y_root"]) / (W["y_tip"] - W["y_root"])
        t = min(max(t, 0.0), 1.0)
        return W["tc_root"] + (W["tc_tip"] - W["tc_root"]) * t

    def __call__(self, y, u, v):
        c = spec.wing_chord(y)
        x_le = spec.wing_le_x(y)
        t = (y - W["y_root"]) / (W["y_tip"] - W["y_root"])
        th = math.radians(W["twist_tip"] * max(0.0, t))
        z0 = (y - W["y_root"]) * math.tan(math.radians(W["dihedral"]))
        # twist about the quarter chord
        dx, dz = (u - 0.25) * c, v * c
        x = x_le + 0.25 * c + dx * math.cos(th) + dz * math.sin(th)
        z = z0 - dx * math.sin(th) + dz * math.cos(th)
        return (x, y, z)


def _span(a, b, n):
    return [a + (b - a) * i / n for i in range(n + 1)]


def wing_parts():
    """Starboard wing, its leading-edge flap and two flaperons."""
    P = WingPlace()
    le0, le1 = W["le_flap"]
    (fa0, fa1), (fb0, fb1) = W["flaperons"]
    cf, lf = W["flaperon_cf"], W["le_flap_cf"]
    g = lambda y: GAP / spec.wing_chord(y)
    out = {}
    segs = []
    # inboard of the moving surfaces: full chord, from inside the body
    segs.append(loft_surface(_span(WING_ROOT_IN, le0 - GAP / 2, 4),
                             lambda y: (0.0, 1.0), P))
    # under the leading-edge flap and ahead of the flaperons
    segs.append(loft_surface(_span(le0 - GAP / 2, fb1 + GAP / 2, 16),
                             lambda y: (lf + g(y) / 2, 1.0 - cf - g(y) / 2), P))
    # outboard of the flaperons, still under the LE flap
    segs.append(loft_surface(_span(fb1 + GAP / 2, le1 + GAP / 2, 2),
                             lambda y: (lf + g(y) / 2, 1.0), P))
    # the tip
    segs.append(loft_surface(_span(le1 + GAP / 2, W["y_tip"], 3),
                             lambda y: (0.0, 1.0), P))
    out["wing"] = mesh.join(*segs)

    # leading-edge flap, on four hinge fittings
    flap = loft_surface(_span(le0 + GAP / 2, le1 - GAP / 2, 16),
                        lambda y: (0.0, lf - g(y) / 2), P)
    out["le_flap"] = mesh.join(flap, *_fittings(P, le0, le1, lf, 4))
    # flaperons
    for name, (a, b) in (("flaperon_in", (fa0, fa1)), ("flaperon_out", (fb0, fb1))):
        surf = loft_surface(_span(a + GAP / 2, b - GAP / 2, 8),
                            lambda y: (1.0 - cf + g(y) / 2, 1.0), P)
        out[name] = mesh.join(surf, *_fittings(P, a, b, 1.0 - cf, 3))
    return out


def _fittings(P, y0, y1, u_hinge, n):
    """Hinge fittings across the gap at chord fraction u_hinge: a small block
    either side of the gap, let into both surfaces."""
    out = []
    for i in range(n):
        y = y0 + (y1 - y0) * (i + 0.5) / n
        c = spec.wing_chord(y)
        x, _, z = P(y, u_hinge, 0.0)
        t = P.tc(y) * c * 0.5
        out.append(mesh.box(x, y, z, GAP + 24.0, 30.0, max(8.0, t)))
    return out


# --------------------------------------------------------------------------
# canards
# --------------------------------------------------------------------------

C = spec.CANARD


ROOT_CLEAR = 45.0


def body_side_y(x, z):
    """How far out the body's upper surface is at station x and height z --
    where a surface standing out of the body at that height meets it."""
    lo, hi = 0.0, shapes.half_width(x)
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if shapes.z_up(x, mid) > z:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


class CanardPlace:
    """s runs 0 at the root to 1 at the tip. An all-moving surface cannot be
    let into the body -- it turns -- so its root edge follows the body's
    side at ROOT_GAP off it, point by point along the chord, and the section
    is blended out to the straight planform by the tip."""
    camber = 0.0

    def tc(self, s):
        return C["tc"]

    def _planform(self, y, u):
        t = (y - C["y_root"]) / (C["y_tip"] - C["y_root"])
        c = C["c_root"] + (C["c_tip"] - C["c_root"]) * t
        return C["x_le_root"] + (y - C["y_root"]) * math.tan(math.radians(C["le_sweep"])) + u * c, c

    def __call__(self, s, u, v):
        # each point stands off the body far enough that the body's top is
        # ROOT_CLEAR below it: where the body's top is nearly flat, that is a
        # long way out, and it is what lets the surface turn +/-30 degrees
        # without its root edge touching the skin
        # a fixed point: the root moves out as the body widens aft, and the
        # sweep moves it aft as it moves out, so each step closes only ~40 %
        # of the gap -- five steps left it 80 mm short, inside the body
        y = C["y_root"]
        for _ in range(40):
            x, c = self._planform(y, u)
            yb = body_side_y(x, C["z"] + v * c - ROOT_CLEAR) + 5.0
            y = yb + s * (C["y_tip"] - yb)
        x, c = self._planform(y, u)
        return (x, y, C["z"] + v * c)


def canard_parts():
    P = CanardPlace()
    surf = loft_surface(_span(0.0, 1.0, 12), lambda s: (0.0, 1.0), P,
                        ladder_root=True)
    xp = C["x_le_root"] + C["pivot_frac"] * C["c_root"]
    # the spindle: from inside the body out into the canard's root, on which
    # the whole surface turns
    # 30 mm: it has to stay inside a root section 88 mm thick
    spindle = mesh.pipe([(xp, C["y_root"] - 380.0, C["z"]),
                         (xp, C["y_root"] + 520.0, C["z"])], 30.0, 24,
                        subdiv=4)
    return {"canard": surf, "canard_spindle": spindle}


# --------------------------------------------------------------------------
# fins
# --------------------------------------------------------------------------

F = spec.FIN


class FinPlace:
    """s is distance along the canted span from the root line; the root
    line follows the body's upper surface, let into it by `embed`."""
    camber = 0.0

    def __init__(self):
        a = math.radians(F["cant"])
        self.d = (0.0, math.sin(a), math.cos(a))       # span direction
        self.n = (0.0, math.cos(a), -math.sin(a))      # thickness direction

    def tc(self, s):
        return F["tc"]

    def chord(self, s):
        return F["c_root"] + (F["c_tip"] - F["c_root"]) * s / F["span"]

    def __call__(self, s, u, v):
        c = self.chord(s)
        x_le = F["x_le_root"] + s * math.tan(math.radians(F["le_sweep"]))
        x = x_le + u * c
        z_root = shapes.z_up(x, F["y_root"]) - F["embed"]
        d, n = self.d, self.n
        return (x, F["y_root"] + d[1] * s + n[1] * v * c,
                z_root + d[2] * s + n[2] * v * c)


def fin_parts():
    P = FinPlace()
    cf = F["rudder_cf"]
    s0, s1 = F["rudder_span"][0] * F["span"], F["rudder_span"][1] * F["span"]
    g = lambda s: GAP / P.chord(s)
    segs = [
        loft_surface(_span(0.0, s0 - GAP / 2, 3), lambda s: (0.0, 1.0), P),
        loft_surface(_span(s0 - GAP / 2, s1 + GAP / 2, 10),
                     lambda s: (0.0, 1.0 - cf - g(s) / 2), P),
        loft_surface(_span(s1 + GAP / 2, F["span"], 2), lambda s: (0.0, 1.0), P),
    ]
    rudder = loft_surface(_span(s0 + GAP / 2, s1 - GAP / 2, 10),
                          lambda s: (1.0 - cf + g(s) / 2, 1.0), P)
    fits = []
    for i in range(3):
        s = s0 + (s1 - s0) * (i + 0.5) / 3
        x, y, z = P(s, 1.0 - cf, 0.0)
        c = P.chord(s)
        fits.append(mesh.box(x, y, z, GAP + 24.0, max(10.0, F["tc"] * c * 0.45),
                             26.0))
    return {"fin": mesh.join(*segs), "rudder": mesh.join(rudder, *fits)}


def build():
    out = {}
    for name, geom in wing_parts().items():
        out[f"{name}_r"] = geom
        out[f"{name}_l"] = mirror(geom)
    for name, geom in canard_parts().items():
        out[f"{name}_r"] = geom
        out[f"{name}_l"] = mirror(geom)
    for name, geom in fin_parts().items():
        out[f"{name}_r"] = geom
        out[f"{name}_l"] = mirror(geom)
    return out
