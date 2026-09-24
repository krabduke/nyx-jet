"""Fail the build on parts too crude to be the thing they are named.

    python3 tools/audit_geometry.py

Part count is the wrong metric and easy to game; what matters is whether
each part has the shape it actually has. So every part must clear a vertex
floor unless it is listed here as genuinely simple, with the reason, and the
whole model must clear a total. Both are ratchets: raise them as the model
improves, never lower them to make a build pass.
"""

import csv
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# A part may be simple only if it really is simple.
EXEMPT = {
}

FLOOR = 50            # vertices, for anything not exempt
TOTAL = 1_400_000     # vertices, over the whole model


def main():
    rows = list(csv.DictReader(open(os.path.join(ROOT, "build", "parts.csv"))))
    crude = sorted((int(r["verts"]), r["name"]) for r in rows
                   if int(r["verts"]) < FLOOR
                   and not any(r["name"].startswith(k) for k in EXEMPT))
    total = sum(int(r["verts"]) for r in rows)
    print(f"{len(rows)} parts, {total:,} verts (floor {FLOOR}, total {TOTAL:,})")
    for v, n in crude:
        print(f"  x  {n:30s} {v:5d} verts")
    ok = not crude and total >= TOTAL
    print("\n" + ("PASS  geometry is up to standard" if ok
                  else "FAIL  geometry is too crude"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
