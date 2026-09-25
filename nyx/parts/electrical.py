"""Electrical power: the distribution unit, the feeders from the engines'
generators, and the looms to every actuator.

Every flight control and gear actuator on this aircraft is electro-
hydrostatic, as on the F-35: a sealed unit with its own motor, pump and
reservoir, driven by power alone. So there are no hydraulic lines to them
-- but they were not wired to anything either, and a gear leg or a canard
with no power to its actuator does not move.

    PDU          the power distribution unit, on the keel's forward end
                 between the engines' inlets
    feeders      one heavy cable from each engine's inboard generator
    looms        from the PDU through the fuselage to each wing's root, each
                 fin's root, and forward to the canards' actuators and the
                 nose gear's -- paths tools/route_solve found clear, kept in
                 electrical_routes.json -- and on inside the wings and fins
                 in conduits along their spars to every actuator there

    python3 nyx/parts/electrical.py nets.json    the nets to solve
"""

import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec      # noqa: E402
import shapes    # noqa: E402
import mesh      # noqa: E402

PDU = {"x": (10400.0, 10553.0), "y": 140.0, "z": (120.0, 360.0)}
ROUTES = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      "electrical_routes.json")


def pdu():
    """The box, finned on top for its cooling, 3 mm into the keel's face
    it is bolted to."""
    x0, x1 = PDU["x"]
    z0, z1 = PDU["z"]
    Y = PDU["y"]
    parts = [shapes.rounded_box(0.5 * (x0 + x1), 0.0, 0.5 * (z0 + z1), x1 - x0,
                                2 * Y, z1 - z0, 10.0)] if hasattr(shapes, "rounded_box") \
        else [mesh.box(0.5 * (x0 + x1), 0.0, 0.5 * (z0 + z1), x1 - x0, 2 * Y, z1 - z0)]
    for k in range(9):
        y = -Y + 25.0 + (2 * Y - 50.0) * k / 8
        parts.append(mesh.box(0.5 * (x0 + x1 - 20.0), y, z1 + 10.0, x1 - x0 - 40.0,
                              6.0, 22.0))
    return mesh.join(*parts)


LOOM_R = {"feeder": 10.0, "wing": 7.0, "wingf": 6.0, "fin": 5.0, "fwd": 6.0}
BRANCH_R = 4.0
CONDUIT = 3.0                 # a conduit is this much larger than its loom
RS_U, FS_U = 0.705, 0.228     # the wing's rear and front spar lines


def _wing_pt(y, u, v=None):
    from parts import surfaces
    P = surfaces.WingPlace()
    if v is None:
        v = 4.0 * P.camber * u * (1.0 - u)
    return P(y, u, v)


def _act_mount(P, chord, s, cf, g, fwd=False, u_hinge=None):
    """Where a pocket actuator's trunnion block is (see
    surfaces.pocket_actuator): the point a loom plugs into."""
    c = chord(s)
    tc = P.tc(s)
    cam = 4.0 * P.camber
    if not fwd:
        u_gap = 1.0 - cf - g / 2.0
        L = min(300.0, 0.3 * c)
        u0 = u_gap - L / c
        h = shapes.naca_t(u_gap, tc) - 6.0 / c
        vc = cam * u_gap * (1.0 - u_gap)
        return P(s, u0 + 4.0 / c, vc - 0.35 * h), u0
    u_gap = u_hinge + g / 2.0
    L = min(260.0, 0.2 * c)
    u1 = u_gap + L / c
    h = min(shapes.naca_t(u, tc) for u in (u_gap, u1)) - 6.0 / c
    vc = cam * u_gap * (1.0 - u_gap)
    return P(s, u1 - 4.0 / c, vc - 0.35 * h), u1


def wing_looms():
    """The starboard wing's looms, in conduits cut inside it: one along the
    rear spar to both flaperons' actuators with a branch forward to the main
    gear's, and one along the front spar to the leading-edge flap's two.
    Returns (looms, conduits)."""
    from parts import surfaces, gear
    P = surfaces.WingPlace()
    W = spec.WING
    (fa0, fa1), (fb0, fb1) = W["flaperons"]
    cf, lf = W["flaperon_cf"], W["le_flap_cf"]
    g = lambda y: surfaces.GAP / spec.wing_chord(y)
    runs = []                                   # (path, loom radius)
    y_in = fa0 + (fa1 - fa0) / 3.0
    y_out = fb0 + (fb1 - fb0) / 3.0
    # the rear spar: root to the outboard flaperon's actuator
    ys = [1690.0 + (y_out - 1690.0) * i / 16 for i in range(17)]
    m_out, _ = _act_mount(P, spec.wing_chord, y_out, cf, g(y_out))
    runs.append(([_wing_pt(y, RS_U) for y in ys], LOOM_R["wing"]))
    runs.append(([_wing_pt(y_out, RS_U), m_out], BRANCH_R))
    # its branches: aft to the inboard flaperon's actuator
    m_in, _ = _act_mount(P, spec.wing_chord, y_in, cf, g(y_in))
    runs.append(([_wing_pt(y_in, RS_U), m_in], BRANCH_R))
    # and forward, outboard of the gear's slot, to the gear's actuator
    px, py, pz = gear.main_pivot()
    yb = 3040.0
    xg = px - 160.0 - 80.0                     # the actuator's middle
    ug = (xg - spec.wing_le_x(yb)) / spec.wing_chord(yb)
    runs.append(([_wing_pt(yb, RS_U), _wing_pt(yb, ug + 0.02), (xg, yb, pz - 20.0),
                  (xg, py + gear.MAIN_ACT_R - 4.0, pz)], BRANCH_R))
    # the front spar: root to the outer leading-edge flap actuator, with a
    # branch into the inner one's
    le0, le1 = W["le_flap"]
    y1, y3 = le0 + (le1 - le0) / 4.0, le0 + (le1 - le0) * 3.0 / 4.0
    ys = [1690.0 + (y3 - 1690.0) * i / 16 for i in range(17)]
    m3, _ = _act_mount(P, spec.wing_chord, y3, None, g(y3), fwd=True, u_hinge=lf)
    runs.append(([_wing_pt(y, FS_U) for y in ys], LOOM_R["wingf"]))
    runs.append(([_wing_pt(y3, FS_U), m3], BRANCH_R))
    m1, _ = _act_mount(P, spec.wing_chord, y1, None, g(y1), fwd=True, u_hinge=lf)
    runs.append(([_wing_pt(y1, FS_U), m1], BRANCH_R))
    looms = mesh.join(*[mesh.pipe(p, r, 12, bend=3.0 * r) for (p, r) in runs])
    # each conduit runs the loom's whole length, into its actuator's pocket,
    # and 6 mm past both its ends: a conduit ending flush with its loom
    # leaves the loom's end cap on the conduit's boundary
    conduits = mesh.join(*[mesh.pipe(_extend(p, 6.0), r + CONDUIT, 12, bend=3.0 * r)
                           for (p, r) in runs])
    return looms, conduits


def wing_roots():
    """Where the starboard wing's two looms start, at its root, and the
    direction each runs in: the fuselage's looms end on them."""
    out = {}
    for kind, u in (("wing", RS_U), ("wingf", FS_U)):
        a, b = _wing_pt(1690.0, u), _wing_pt(1740.0, u)
        out[kind] = (a, b)
    return out


def _extend(p, d):
    """A polyline lengthened by d at both ends, along its end segments."""
    def out(a, b):
        L = math.dist(a, b) or 1.0
        return tuple(a[k] + (a[k] - b[k]) / L * d for k in range(3))
    return [out(p[0], p[1])] + list(p[1:-1]) + [out(p[-1], p[-2])] if len(p) > 2 \
        else [out(p[0], p[1]), out(p[1], p[0])]


def _toward(a, b, f):
    return tuple(a[k] + (b[k] - a[k]) * f for k in range(3))


def fin_loom():
    """Up the starboard fin, inside it, from under its root to its rudder's
    actuator. Returns (loom, conduit)."""
    from parts import surfaces
    P = surfaces.FinPlace()
    F = spec.FIN
    s0 = F["rudder_span"][0] * F["span"]
    sa = s0 + 95.0
    g = surfaces.GAP / P.chord(sa)
    m, u0 = _act_mount(P, P.chord, sa, F["rudder_cf"], g)
    path = [P(-80.0, 0.62, 0.0), P(0.0, 0.62, 0.0), P(sa, 0.62, 0.0),
            P(sa, u0 + 20.0 / P.chord(sa), 0.0), m]
    loom = mesh.pipe(path, LOOM_R["fin"], 12, bend=15.0)
    conduit = mesh.pipe(path[1:], LOOM_R["fin"] + CONDUIT, 12, bend=15.0)
    # and where it passes out of the body into the fin, a grommeted hole in
    # the skin -- the conduit, from inside the body
    hole = mesh.pipe(path, LOOM_R["fin"] + CONDUIT, 12, bend=15.0)
    return loom, conduit, hole


def fuselage_looms():
    """The looms through the fuselage, on the solved paths, each plugged into
    what it connects at both ends; and each's end pieces into them."""
    if not os.path.exists(ROUTES):
        return {}
    routes = json.load(open(ROUTES))
    out = {}
    for tag in ("r", "l"):
        sgn = 1.0 if tag == "r" else -1.0
        parts = []
        for name, pts in routes.items():
            if not name.endswith("_" + tag):
                continue
            kind = name.rsplit("_", 1)[0]
            pts = [tuple(p) for p in pts]
            r = LOOM_R[kind]
            # into what each end is plugged into
            first, last = pts[0], pts[-1]
            if kind == "feeder":
                pts = [(10994.0, first[1], first[2])] + pts + [(last[0], last[1], PDU["z"][0] + 4.0)]
            else:
                pts = [(first[0], sgn * (PDU["y"] - 4.0), first[2])] + pts
                if kind in ("wing", "wingf"):
                    a0, a1 = wing_roots()[kind]
                    mir = lambda q: (q[0], sgn * q[1], q[2])
                    pts = pts[:-1] + [mir(a0), mir(_toward(a0, a1, 0.4))]
                elif kind == "fwd":
                    # it ends on the canard's actuator; on the starboard side
                    # the nose gear's run goes on from there, its own part
                    k = min(range(len(pts)),
                            key=lambda i: math.dist(pts[i], (4800.0, sgn * 640.0, 255.0)))
                    if k < len(pts) - 1:
                        nose = pts[k:] + [(pts[-1][0], sgn * 340.0, pts[-1][2])]
                        out["loom_nose_gear"] = mesh.pipe(nose, BRANCH_R + 1.0, 12,
                                                          bend=15.0)
                    pts = pts[:k + 1] + [(pts[k][0], pts[k][1], 236.0)]
            parts.append(mesh.pipe(pts, r, 12, bend=2.5 * r))
        out[f"loom_fuselage_{tag}"] = mesh.join(*parts)
    return out


def _mirror(part):
    v, f = part
    return shapes.orient(([(x, -y, z) for (x, y, z) in v],
                          [tuple(reversed(c)) for c in f]))


def build():
    out = {"pdu": pdu()}
    out.update(fuselage_looms())
    looms, conduits = wing_looms()
    out["loom_wing_r"], out["loom_wing_l"] = looms, _mirror(looms)
    out["cut:wing_r"], out["cut:wing_l"] = conduits, _mirror(conduits)
    loom, conduit, hole = fin_loom()
    out["loom_fin_r"], out["loom_fin_l"] = loom, _mirror(loom)
    out["cut:fin_r"], out["cut:fin_l"] = conduit, _mirror(conduit)
    # where the looms pass through the skin: the wings' chine edges, where
    # the wing's root is let into the body's solid edge, and the fins' roots
    out["cut:fuselage_skin"] = mesh.join(hole, _mirror(hole), conduits,
                                         _mirror(conduits))
    return out


if __name__ == "__main__":
    pass
