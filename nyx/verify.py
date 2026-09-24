"""The definition of done for the aircraft: what the audits cannot say.

    python3 nyx/verify.py

The audits in tools/ check that every part is closed, attached, clear of
its neighbours and joined into its circuits. This checks the aircraft
against its brief and against physics: that it stands on its wheels, that
it balances on them, that the engines it carries are the ones in the
Aether repo, and that it is as agile as the brief says -- short and wide,
lightly loaded, over-powered and unstable on purpose.
"""

import csv
import hashlib
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "aero"))
import spec      # noqa: E402
import agility   # noqa: E402
import vlm       # noqa: E402

results = []


def check(label, ok, detail=""):
    results.append(ok)
    print(f"  {'ok' if ok else 'x '}  {label:48s} {detail}")


def main():
    rows = list(csv.DictReader(open(os.path.join(ROOT, "build", "parts.csv"))))
    by = {r["name"]: r for r in rows}
    f = lambda r, k: float(r[k])

    print("COMPLETENESS")
    palette = set(json.load(open(os.path.join(ROOT, "viewer", "parts.json")))
                  ["parts"][r["name"]]["mat"] for r in rows)
    check("every part has a material", all(r["material"] for r in rows),
          f"{len(palette)} materials")
    check("no empty meshes", all(int(r["faces"]) > 0 for r in rows),
          f"{len(rows)} parts")
    for key in ("fuselage_skin", "wing_l", "wing_r", "canard_l", "canard_r",
                "fin_l", "fin_r", "canopy_glass", "seat", "intake_duct_l",
                "intake_duct_r", "missiles", "keel"):
        check(f"{key} is built", key in by)
    check("both engines are installed",
          any(n.startswith("engine_l_") for n in by)
          and any(n.startswith("engine_r_") for n in by))

    print("\nON THE GROUND")
    tyres = [r for r in rows if r["name"].startswith(("tyre", "tyres"))]
    lows = [f(r, "z_min_mm") for r in tyres]
    check("every tyre touches the ground",
          all(abs(z - spec.GROUND_Z) < 1.0 for z in lows),
          f"lowest points {min(lows):.0f} .. {max(lows):.0f}, ground "
          f"{spec.GROUND_Z:.0f}")
    below = [r["name"] for r in rows if f(r, "z_min_mm") < spec.GROUND_Z - 0.5]
    check("nothing is below the ground", not below, ", ".join(below[:4]))
    low = [r["name"] for r in rows
           if not r["name"].startswith(("tyre", "wheel", "gear"))
           and f(r, "z_min_mm") < spec.GROUND_Z + 250.0]
    check("250 mm of ground clearance under the airframe", not low,
          ", ".join(low[:4]))
    x_cg = spec.cg_x()
    x_main = spec.GEAR["main_x"]
    x_nose = spec.GEAR["nose_x"]
    check("the CG is between the nose and main gear",
          x_nose < x_cg < x_main,
          f"nose {x_nose:.0f}, CG {x_cg:.0f}, main {x_main:.0f}")
    share = (x_cg - x_nose) / (x_main - x_nose)
    check("the nose wheel carries 6-15 % of the weight",
          0.85 <= share <= 0.94, f"{100 * (1 - share):.0f} %")

    print("\nTHE BRIEF: AGILE, SHORT AND WIDE")
    fig = json.load(open(os.path.join(ROOT, "viewer", "parts.json")))["figures"]
    j = agility.Jet()
    sm, *_ = vlm.static_margin()
    check("span is at least 85 % of length",
          fig["span_mm"] >= 0.85 * fig["length_mm"],
          f"{fig['span_mm'] / 1000:.1f} m span, {fig['length_mm'] / 1000:.1f} m long")
    check("wing loading under 280 kg/m2", j.m / j.S < 280.0,
          f"{j.m / j.S:.0f}")
    check("thrust-to-weight over 1.4 on reheat", j.T0 / j.W > 1.4,
          f"{j.T0 / j.W:.2f}")
    check("relaxed stability: margin -10 to -2 % MAC",
          -0.10 <= sm <= -0.02, f"{100 * sm:+.1f} %")
    itr, _ = j.instantaneous(0.0)
    strate, _, _ = j.best_sustained(0.0)
    check("instantaneous turn over 30 deg/s at sea level", itr > 30.0,
          f"{itr:.1f}")
    check("sustained turn over 22 deg/s at sea level", strate > 22.0,
          f"{strate:.1f}")

    print("\nTHE ENGINES")
    ven = json.load(open(os.path.join(ROOT, "aether", "VENDOR.json")))
    bad = []
    for name, meta in ven["files"].items():
        p = os.path.join(ROOT, "aether", name)
        if not os.path.exists(p) or \
                hashlib.sha256(open(p, "rb").read()).hexdigest()[:12] != meta["sha"]:
            bad.append(name)
    check("vendored engine files are unmodified", not bad,
          f"{len(ven['files'])} files from {ven['source']} @ {ven['commit']}")
    src = os.path.join(os.path.dirname(ROOT), "aether-ax1")
    if os.path.isdir(src):
        stale = [n for n, m in ven["files"].items()
                 if not os.path.exists(os.path.join(src, m["from"])) or
                 hashlib.sha256(open(os.path.join(src, m["from"]), "rb").read())
                 .hexdigest()[:12] != m["sha"]]
        check("vendored engine matches its source", not stale,
              ", ".join(stale[:3]))
    tail = max(f(r, "x_max_mm") for r in rows
               if r["name"].startswith("fuselage_skin"))
    brg = spec.ENGINE_FAN_FACE_X + spec.ENGINE_BEARING_1
    # everything behind the swivel's front bearing turns when the jet is
    # vectored, so the airframe has to end ahead of it -- but not so far
    # ahead that the fixed ring and its motor hang out in the air
    check("the airframe ends just ahead of the swivels' front bearings",
          0.0 <= brg - tail <= 60.0, f"bearing {brg:.0f}, tail {tail:.0f}")

    n_bad = results.count(False)
    print("\n" + "=" * 70)
    print(f"PASS  all {len(results)} checks" if not n_bad
          else f"FAIL  {n_bad} of {len(results)} checks")
    return 1 if n_bad else 0


if __name__ == "__main__":
    sys.exit(main())
