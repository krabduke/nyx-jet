"""Structural audit for Nyx. See tools/_structure.py for the checks.

    python3 tools/audit_structure.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _structure as S

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

CFG = {
    "gap_mm": 0.5,
    "exempt_attached": {},
    "mirror_tol_mm": 1.0,
    # The two engines are the same engine twice, not mirror images -- as on
    # any twin -- so a part with a side of its own inside one engine (its two
    # generators) pairs with the part in the other engine, not with its
    # neighbour in the same one.
    "exempt_mirror": {
        "engine_l_generator_": "identical engines, not mirrored ones",
        "engine_r_generator_": "identical engines, not mirrored ones",
    },
    "distinct_tol_mm": 0.5,
    "exempt_distinct": {},
    "exempt_shape": {
        # in a combustor the dome is the bulkhead across the head of the
        # annulus -- 10 mm deep and 700 across -- not a cap
        "engine_l_combustor_dome": "a combustor dome is a bulkhead, not a cap",
        "engine_r_combustor_dome": "a combustor dome is a bulkhead, not a cap",
    },
    "singletons": {
        "body skin": (("fuselage_skin",), 1),
        "keel": (("keel",), 1),
        "canopy": (("canopy_glass",), 1),
        "ejection seat": (("seat",), 1),
        "tail stinger": (("tail_stinger",), 1),
    },
}

if __name__ == "__main__":
    n = S.report(os.path.join(ROOT, "build", "parts.csv"), CFG, "Nyx")
    sys.exit(0 if n == 0 else 1)
