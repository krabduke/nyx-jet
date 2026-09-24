"""Every piece of the aircraft must be fastened to something.

    python3 tools/audit_support.py            (runs itself under Blender)
    python3 tools/audit_support.py --shrink   (after a fix: drop what is fixed)

A part is often many closed pieces -- a blade row is one piece per blade, a
bolt ring one per bolt. audit_joints.py asks whether the PARTS form one
assembly, and a part passes that as long as one of its pieces touches
something. This asks it of every piece: each must cross, touch within TOL,
or sit inside some other piece, of any part, its own included. A blade
standing off its disc, a vane stopping short of its casing, a bolt hung in
the air beside its flange all fail here and pass there.

DETACHED is how many free pieces each part is known to have. The list only
gets shorter; --shrink never adds anything. It started empty.
"""
import os, sys
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

PKG = "nyx/parts"
UNIT = 1.0            # mm of real part per model unit
TOL = 0.3             # mm: further off than this is not touching

# --- DETACHED: rewritten by --shrink, never by hand to add ---
DETACHED = {
}
# --- end DETACHED ---

if __name__ == "__main__":
    import _interfere
    sys.exit(_interfere.support_main(__file__, ROOT, PKG, DETACHED, TOL, UNIT))
