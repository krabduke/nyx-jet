"""A vortex-lattice solver for the wing and canards, and the neutral point
it gives.

    python3 aero/vlm.py

Each lifting surface is laid out as a lattice of horseshoe vortices: bound
leg on the panel's quarter chord, trailing legs straight aft to infinity,
flow tangency enforced at the three-quarter-chord control point (Katz &
Plotkin, "Low-Speed Aerodynamics", ch. 12). Solving for the circulations
at a small angle of attack gives the lift and pitching moment, and their
ratio gives the neutral point:

    x_np = x_ref - (dCm/da / dCL/da) * c_ref

What the solve represents, and what it does not:

  * The wing is continued straight in to the centreline through the body.
    For a blended lifting body that is the usual first-order model -- the
    body between the wings carries lift at about the wing's loading -- and
    it is what the reference area is defined on.
  * The canards are their exposed panels at their own height above the
    wing plane, so their downwash on the wing and the wing's upwash on them
    are both in the solve.
  * It is linear, inviscid and incompressible (Prandtl-Glauert is applied
    to the lift slope for Mach number). It knows nothing of the leading-edge
    vortex, which adds non-linear lift at high angle of attack (aero/agility
    handles that with Polhamus's analogy), and nothing of stall.

The solver is checked against lifting-line theory before its neutral point
is used: `validate()` runs a rectangular AR 8 wing, whose lift slope is
known to about 1 %.
"""

import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(HERE, "nyx"))
import spec  # noqa: E402


def surface(x_le_fn, chord_fn, y0, y1, z, ns, nc, mirror=True):
    """Panels of a lifting surface as arrays of (A, B, C): bound-leg ends
    and control point, for the starboard side y0..y1 and its mirror. Span
    stations are cosine-spaced to pack panels at the tip."""
    ys = [y0 + (y1 - y0) * 0.5 * (1 - math.cos(math.pi * i / ns)) for i in range(ns + 1)]
    A, B, C = [], [], []
    for side in ((1.0, -1.0) if mirror else (1.0,)):
        for i in range(ns):
            ya, yb = ys[i], ys[i + 1]
            for j in range(nc):
                def pt(y, f):
                    return (x_le_fn(y) + chord_fn(y) * f, side * y, z)
                fa, fb = j / nc, (j + 1) / nc
                q = fa + 0.25 * (fb - fa)
                cp = fa + 0.75 * (fb - fa)
                a, b = pt(ya, q), pt(yb, q)
                if side < 0:
                    a, b = b, a            # keep the bound leg running +y
                ym = 0.5 * (ya + yb)
                A.append(a); B.append(b); C.append(pt(ym, cp))
    return np.array(A), np.array(B), np.array(C)


def _segment(P, A, B):
    """Velocity induced at points P by unit-strength segments A->B
    (Biot-Savart), broadcast over panels. P: (n,3), A,B: (m,3) -> (n,m,3)."""
    r1 = P[:, None, :] - A[None, :, :]
    r2 = P[:, None, :] - B[None, :, :]
    r0 = B - A
    c = np.cross(r1, r2)
    c2 = np.sum(c * c, axis=2)
    n1 = np.linalg.norm(r1, axis=2)
    n2 = np.linalg.norm(r2, axis=2)
    k = (np.sum(r0[None] * r1, axis=2) / np.maximum(n1, 1e-12)
         - np.sum(r0[None] * r2, axis=2) / np.maximum(n2, 1e-12))
    k = np.where(c2 > 1e-12, k / np.maximum(c2, 1e-12) / (4 * math.pi), 0.0)
    return c * k[:, :, None]


def _trailing(P, A, sign):
    """A semi-infinite leg from A to +x infinity (sign +1) or from +x
    infinity to A (sign -1)."""
    far = A + np.array([1e9, 0.0, 0.0])
    return _segment(P, A, far) if sign > 0 else _segment(P, far, A)


def solve(panels, alpha_deg, mach=0.0):
    """Circulations and the forces they give, for all surfaces together.
    Returns (CL, Cm about x_ref, per-panel lift)."""
    A = np.concatenate([p[0] for p in panels])
    B = np.concatenate([p[1] for p in panels])
    C = np.concatenate([p[2] for p in panels])
    V = (_segment(C, A, B) + _trailing(C, B, +1) + _trailing(C, A, -1))
    n = np.array([0.0, 0.0, 1.0])
    AIC = V @ n
    a = math.radians(alpha_deg)
    rhs = -np.full(len(C), math.sin(a))
    gam = np.linalg.solve(AIC, rhs)
    dy = B[:, 1] - A[:, 1]
    lift = gam * dy * math.cos(a)           # per unit rho V^2 / ... (Kutta-Joukowski)
    beta = math.sqrt(max(1e-6, 1.0 - mach * mach))
    return gam, lift / beta, 0.5 * (A + B)


def aircraft_panels(ns_w=22, nc_w=10, ns_c=10, nc_c=6):
    p = spec.ref_planform()
    w = surface(lambda y: p["x_le0"] + y * p["le_tan"],
                lambda y: p["c0"] + (p["ct"] - p["c0"]) * y / p["b2"],
                0.0, p["b2"], 0.0, ns_w, nc_w)
    c = spec.CANARD
    t = math.tan(math.radians(c["le_sweep"]))
    cn = surface(lambda y: c["x_le_root"] + (y - c["y_root"]) * t,
                 lambda y: c["c_root"] + (c["c_tip"] - c["c_root"])
                 * (y - c["y_root"]) / (c["y_tip"] - c["y_root"]),
                 c["y_root"], c["y_tip"], c["z"], ns_c, nc_c)
    return [w, cn]


def neutral_point(panels=None, mach=0.0):
    """(x_np mm, CL_alpha per rad) from two solves either side of zero."""
    panels = panels or aircraft_panels()
    S = spec.s_ref_m2() * 1e6
    res = []
    for a in (-1.0, 1.0):
        _, L, mid = solve(panels, a, mach)
        # L is lift / (rho V) per unit V: normalise by S/2 (q = rho V^2 / 2)
        CL = 2.0 * L.sum() / S
        M = -(2.0 * (L * mid[:, 0]).sum() / S)      # nose-up positive about x = 0
        res.append((CL, M))
    dCL = (res[1][0] - res[0][0]) / math.radians(2.0)
    dM = (res[1][1] - res[0][1]) / math.radians(2.0)
    # M about x=0 is -CL * x_cp; the neutral point is where dM/da about it
    # vanishes: x_np = -dM / dCL
    return -dM / dCL, dCL


def validate():
    """Rectangular AR 8 wing against lifting-line theory, and an elliptic
    reference. Returns the ratio of the VLM lift slope to the theory's."""
    b, c = 8000.0, 1000.0
    w = surface(lambda y: 0.0, lambda y: c, 0.0, b / 2, 0.0, 30, 8)
    S = b * c
    _, L, _ = solve([w], 1.0)
    cla = 2.0 * L.sum() / S / math.radians(1.0)
    AR = b * b / S
    # Helmbold's lifting-surface formula, good to a few per cent at AR 8
    theory = 2 * math.pi * AR / (2 + math.sqrt(AR * AR + 4))
    return cla, theory


def static_margin(mach=0.0, fuel_fraction=None):
    x_np, cla = neutral_point(mach=mach)
    c, x_mac, _ = spec.mac()
    x_cg = spec.cg_x() if fuel_fraction is None else spec.cg_x(fuel_fraction)
    return (x_np - x_cg) / c, x_np, x_cg, cla


if __name__ == "__main__":
    cla, th = validate()
    print(f"validation: rectangular AR 8, CL_alpha {cla:.3f}/rad vs lifting-surface "
          f"theory {th:.3f} ({100 * (cla / th - 1):+.1f} %)")
    sm, xnp, xcg, cla = static_margin()
    c, xm, _ = spec.mac()
    print(f"aircraft: CL_alpha {cla:.3f}/rad, neutral point x {xnp:.0f} mm "
          f"({100 * (xnp - xm) / c:.1f} % MAC)")
    print(f"combat CG x {xcg:.0f} mm ({100 * (xcg - xm) / c:.1f} % MAC); "
          f"static margin {100 * sm:+.1f} % MAC")
