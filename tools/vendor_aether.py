"""Copy the Aether AX-1 engine sources into aether/.

The aircraft does not re-model its engines: it imports the generators from
the sibling aether-ax1 repo and installs two of them. That only stays true if
the copy in aether/ is the engine, so the copy is scripted and records which
commit it came from, and `check()` fails the build if either the copy or the
upstream sources have moved on without the other.

The engine's own list of intended joints (EXPECTED in its intersection
audit) comes across too: the aircraft's audit applies it to each installed
engine, so a blade root in its disc is still a joint and not a defect.

    python3 tools/vendor_aether.py [path-to-aether-ax1]
"""

import hashlib
import json
import os
import shutil
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_SRC = os.path.join(os.path.dirname(ROOT), "aether-ax1")

FILES = ["engine/spec.py", "engine/cycle.py", "engine/mesh.py",
         "engine/blades.py", "tools/audit_intersect.py"]
PART_MODULES = ["__init__.py", "common.py", "fan.py", "compressor.py",
                "combustor.py", "turbine.py", "augmentor.py", "nozzle.py",
                "frames.py", "spools.py", "accessories.py"]
# where each lands inside aether/
RENAME = {"tools/audit_intersect.py": "engine_joints.py"}


def digest(path):
    with open(path, "rb") as fh:
        return hashlib.sha256(fh.read()).hexdigest()[:12]


def _pairs():
    for f in FILES:
        yield f, RENAME.get(f, os.path.basename(f))
    for f in PART_MODULES:
        yield "engine/parts/" + f, "parts/" + f


def vendor(src=DEFAULT_SRC):
    dst = os.path.join(ROOT, "aether")
    os.makedirs(os.path.join(dst, "parts"), exist_ok=True)
    man = {"source": "aether-ax1", "files": {}}
    try:
        man["commit"] = subprocess.check_output(
            ["git", "-C", src, "rev-parse", "--short", "HEAD"], text=True).strip()
    except Exception:
        man["commit"] = "unknown"
    for rel_src, rel_dst in _pairs():
        s = os.path.join(src, rel_src)
        shutil.copy2(s, os.path.join(dst, rel_dst))
        man["files"][rel_dst] = {"from": rel_src, "sha": digest(s)}
    for d, _, _ in os.walk(dst):
        if d.endswith("__pycache__"):
            shutil.rmtree(d, ignore_errors=True)
    with open(os.path.join(dst, "VENDOR.json"), "w") as fh:
        json.dump(man, fh, indent=1)
    return man


def check(src=DEFAULT_SRC):
    """(ok, why): the copy matches its manifest, and the upstream sources
    still match the copy when they are reachable."""
    p = os.path.join(ROOT, "aether", "VENDOR.json")
    if not os.path.exists(p):
        return False, "aether/VENDOR.json missing -- run tools/vendor_aether.py"
    man = json.load(open(p))
    bad = []
    for rel, info in man["files"].items():
        f = os.path.join(ROOT, "aether", rel)
        if not os.path.exists(f):
            bad.append(rel + " (missing)")
        elif digest(f) != info["sha"]:
            bad.append(rel + " (edited)")
    if bad:
        return False, ", ".join(bad[:4])
    if not os.path.isdir(src):
        return True, (f"{len(man['files'])} files from {man['commit']} "
                      "(upstream not reachable, drift unchecked)")
    drift = [rel for rel, info in man["files"].items()
             if not os.path.exists(os.path.join(src, info["from"]))
             or digest(os.path.join(src, info["from"])) != info["sha"]]
    if drift:
        return False, "stale, re-run tools/vendor_aether.py: " + ", ".join(drift[:4])
    return True, f"{len(man['files'])} files from aether-ax1 @ {man['commit']}, matching"


if __name__ == "__main__":
    m = vendor(sys.argv[1] if len(sys.argv) > 1 else DEFAULT_SRC)
    print(f"vendored {len(m['files'])} files from aether-ax1 @ {m['commit']}")
