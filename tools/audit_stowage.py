"""Where does the gear go when it is up?

The build stands on its gear. This builds the aircraft again with the gear
retracted and every gear door shut (parts/gear.py, NYX_GEAR=up) and runs the
interference audit on that: every leg, wheel and door has to fit in its bay
without passing through the duct beside it, the wing it folds into, the
cockpit floor over it or the radar bulkhead in front of it.

    python3 tools/audit_stowage.py
"""

import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

if __name__ == "__main__":
    env = dict(os.environ, NYX_GEAR="up")
    r = subprocess.run([sys.executable, os.path.join(ROOT, "tools",
                                                     "audit_intersect.py")],
                       env=env, cwd=ROOT, capture_output=True, text=True)
    out = r.stdout + r.stderr
    for line in out.splitlines():
        if not line.startswith(("PASS", "FAIL")):
            print(line)
    ok = r.returncode == 0
    print("\n" + ("PASS  the gear stows: up and shut, nothing is where "
                   "something else is" if ok else
                   "FAIL  the gear does not stow cleanly"))
    sys.exit(0 if ok else 1)
