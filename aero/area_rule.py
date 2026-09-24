"""The transonic area rule: the aircraft's cross-sectional area distribution.

    python3 aero/area_rule.py [--plot out.svg]

Near Mach 1 the wave drag of an aircraft is, to first order, the wave drag
of a body of revolution with the same distribution of cross-sectional area
along its length (Whitcomb, 1952). The least wave drag for a given length
and volume comes from the Sears-Haack body,

    A_SH(x) = (16 V / 3 pi L) [4 t (1 - t)]^(3/2),   t = x / L,

so an aircraft's area curve is judged by how smooth it is and how close to
that shape: one hump, its peak near the middle, no bumps where a canopy or
a wing root adds area abruptly.

The area is measured off the parts themselves, not from the spec: every
external part is sliced at each station, the slices are rasterised on a
10 mm grid and their UNION measured (the wing roots run inside the body and
must not be counted twice). The air the intakes swallow is not body -- the
capture streamtube is subtracted aft of the mouths, which is the usual
treatment. The engines' nozzle boxes behind the body are counted.

This is the area rule, not a wave-drag calculation: it says whether the
shape is well arranged, not what its drag coefficient is.
"""

import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(HERE, "nyx"))
import spec     # noqa: E402
import shapes   # noqa: E402

GRID = 10.0          # mm


def _tris(part):
    v, f = part
    V = np.asarray(v, dtype=float)
    T = []
    for fc in f:
        for k in range(1, len(fc) - 1):
            T.append((fc[0], fc[k], fc[k + 1]))
    T = np.asarray(T)
    return V[T[:, 0]], V[T[:, 1]], V[T[:, 2]]


def _canopy_solid():
    from parts import cockpit
    CK = spec.COCKPIT
    xs = [CK["x0"] + 120.0 + (CK["x1"] - CK["x0"] - 240.0) * i / 30 for i in range(31)]
    rings = []
    for x in xs:
        sec = cockpit._canopy_section(x)
        rings.append(sec[: len(sec) // 2])        # the outside only, closed by its base
    return shapes.loft_rings(rings)


def _nozzle_boxes():
    """The engines' nozzle boxes behind the body, as two tapering boxes."""
    from parts import engines
    import mesh
    n = engines.info()["nozzle"]
    x0 = spec.ENGINE_FAN_FACE_X + n["x_trans1"]
    x1 = spec.ENGINE_FAN_FACE_X + n["x_exit"]
    hw = n["sw_out"]
    h0 = n["b_shroud"]
    h1 = n["h_exit"] + n["flap_t"]
    out = []
    for sy in (1.0, -1.0):
        rings = []
        for x, h in ((x0, h0), (x1, h1)):
            yc = sy * spec.ENGINE_Y
            rings.append([(x, yc - hw, -h), (x, yc + hw, -h), (x, yc + hw, h),
                          (x, yc - hw, h)])
        out.append(shapes.loft_rings(rings))
    return out


def external_parts():
    """Closed solids for everything the outside air sees."""
    from parts import surfaces, intakes
    parts = [shapes.outer_solid(n=120, m=96), _canopy_solid()]
    s = surfaces.build()
    for k, g in s.items():
        if not k.startswith("canard_spindle"):
            parts.append(g)
    c = intakes.cutter()
    parts += [c, intakes._mirror(c)]
    parts += _nozzle_boxes()
    return parts


def capture_area():
    """The two mouths' capture area, mm^2."""
    I = spec.INTAKE
    # a superellipse of exponent 5 fills 0.94 of its bounding box
    return 2 * 0.94 * I["w_mouth"] * I["h_mouth"]


def areas(xs=None, parts=None):
    parts = parts or external_parts()
    tri = [_tris(p) for p in parts]
    L = spec.ENGINE_FAN_FACE_X + 3560.0
    xs = xs if xs is not None else np.linspace(5.0, L - 5.0, 160)
    out = []
    for x in xs:
        segs = []
        for (A, B, C) in tri:
            xa, xb, xc = A[:, 0], B[:, 0], C[:, 0]
            lo = np.minimum(np.minimum(xa, xb), xc)
            hi = np.maximum(np.maximum(xa, xb), xc)
            m = (lo < x) & (hi > x)
            if not m.any():
                segs.append(None)
                continue
            P = np.stack([A[m], B[m], C[m]], axis=1)       # (n, 3, 3)
            pts = []
            for i, j in ((0, 1), (1, 2), (2, 0)):
                p, q = P[:, i], P[:, j]
                d = q[:, 0] - p[:, 0]
                with np.errstate(divide="ignore", invalid="ignore"):
                    t = (x - p[:, 0]) / d
                ok = (t >= 0) & (t <= 1) & (d != 0)
                pt = p + (q - p) * np.where(ok, t, 0.0)[:, None]
                pts.append(np.where(ok[:, None], pt[:, 1:], np.nan))
            # each straddling triangle gives a segment between its two cuts
            S = np.stack(pts, axis=1)                        # (n, 3, 2)
            seg = []
            for row in S:
                good = row[~np.isnan(row[:, 0])]
                if len(good) >= 2:
                    seg.append((good[0], good[1]))
            segs.append(seg)
        zs_all = [p[1] for seg in segs if seg for s in seg for p in s]
        if not zs_all:
            out.append(0.0)
            continue
        z0, z1 = min(zs_all), max(zs_all)
        total = 0.0
        for z in np.arange(z0 + GRID / 2, z1, GRID):
            ivs = []
            for seg in segs:
                if not seg:
                    continue
                ys = []
                for (p, q) in seg:
                    if (p[1] > z) != (q[1] > z):
                        t = (z - p[1]) / (q[1] - p[1])
                        ys.append(p[0] + (q[0] - p[0]) * t)
                ys.sort()
                ivs += [(ys[i], ys[i + 1]) for i in range(0, len(ys) - 1, 2)]
            if not ivs:
                continue
            ivs.sort()
            cur0, cur1 = ivs[0]
            length = 0.0
            for a, b in ivs[1:]:
                if a > cur1:
                    length += cur1 - cur0
                    cur0, cur1 = a, b
                else:
                    cur1 = max(cur1, b)
            length += cur1 - cur0
            total += length * GRID
        # the mouths' lips are swept over x_lip_lo..x_lip_hi: the air they
        # swallow is taken off in step with them
        from parts import intakes
        lo, hi = intakes.lip_range()
        f = min(1.0, max(0.0, (x - lo) / (hi - lo)))
        total -= capture_area() * f
        out.append(total)
    return np.asarray(xs), np.asarray(out)


def assess(xs, A):
    L = xs[-1] + 5.0
    A_max = A.max()
    i_max = int(A.argmax())
    V = float(np.trapezoid(A, xs)) if hasattr(np, "trapezoid") else float(np.trapz(A, xs))
    t = xs / L
    ash = (16 * V / (3 * math.pi * L)) * np.clip(4 * t * (1 - t), 0, None) ** 1.5
    rms = float(np.sqrt(np.mean((A - ash) ** 2))) / A_max
    # humps: after the peak the area should only fall, before it only rise;
    # measure the worst reversal against the running extreme
    worst = 0.0
    run = 0.0
    for a in A[: i_max + 1]:
        run = max(run, a)
        worst = max(worst, (run - a) / A_max)
    run = A_max
    for a in A[i_max:]:
        run = min(run, a)
        worst = max(worst, (a - run) / A_max)
    return {"L": L, "A_max_m2": A_max * 1e-6, "x_max": xs[i_max],
            "x_max_frac": xs[i_max] / L, "rms_vs_sh": rms, "reversal": worst,
            "sh": ash, "volume_m3": V * 1e-9}


def svg(xs, A, r, path):
    W, H, pad = 900, 320, 40
    L = r["L"]
    amax = max(A.max(), r["sh"].max()) * 1.05
    sx = lambda x: pad + (W - 2 * pad) * x / L
    sy = lambda a: H - pad - (H - 2 * pad) * a / amax
    line = lambda ys: " ".join(f"{sx(x):.1f},{sy(a):.1f}" for x, a in zip(xs, ys))
    s = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
         f'font-family="sans-serif" font-size="12">',
         f'<rect width="{W}" height="{H}" fill="#1B2025"/>',
         f'<polyline points="{line(r["sh"])}" fill="none" stroke="#8B928D" '
         f'stroke-dasharray="5 4" stroke-width="1.5"/>',
         f'<polyline points="{line(A)}" fill="none" stroke="#C6A15B" stroke-width="2"/>',
         f'<text x="{pad}" y="22" fill="#DAD8D0">Nyx cross-sectional area '
         f'(solid) against a Sears-Haack body of the same length and volume '
         f'(dashed)</text>',
         f'<text x="{pad}" y="{H - 12}" fill="#8B928D">0 m</text>',
         f'<text x="{W - pad - 30}" y="{H - 12}" fill="#8B928D">{L / 1000:.1f} m</text>',
         '</svg>']
    open(path, "w").write("\n".join(s))


def main():
    xs, A = areas()
    r = assess(xs, A)
    print(f"length {r['L'] / 1000:.2f} m, peak area {r['A_max_m2']:.2f} m^2 at "
          f"x {r['x_max'] / 1000:.2f} m ({100 * r['x_max_frac']:.0f} % of length), "
          f"volume {r['volume_m3']:.1f} m^3")
    print(f"RMS departure from Sears-Haack {100 * r['rms_vs_sh']:.1f} % of peak; "
          f"worst reversal {100 * r['reversal']:.1f} % of peak")
    if "--plot" in sys.argv:
        svg(xs, A, r, sys.argv[sys.argv.index("--plot") + 1])
    return xs, A, r


if __name__ == "__main__":
    main()
