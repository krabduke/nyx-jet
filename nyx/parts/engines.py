"""Two Aether AX-1 engines, installed.

The engines are not re-modelled: the generators vendored in aether/ build
them exactly as the engine repo does, in the engine's own frame (fan face at
x = 0, on its axis), and they are moved into place. Every engine part is
renamed engine_l_<part> or engine_r_<part> -- port and starboard -- so the
aircraft's audits can hold each engine to the engine's own list of intended
joints (aether/engine_joints.py) and to the aircraft's.

Both engines are identical, not handed: the gearbox is under each, and the
spools turn the same way in both. (Handed engines would cancel the two
engines' gyroscopic couples, at the price of two part sets. A fighter
program would not pay it.)
"""

import importlib
import os
import sys

HERE = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import spec   # noqa: E402  -- the aircraft's

AETHER = os.path.join(HERE, "aether")
_SHADOWED = ("spec", "mesh", "blades", "cycle", "parts")

SIDES = (("l", -1.0), ("r", 1.0))     # port at -Y, starboard at +Y

# The engines are built a little coarser than the engine repo builds them:
# 1,186 aerofoils twice over at full resolution would be most of the
# aircraft's vertices and nothing a viewer of the whole aircraft can see.
RES = {"airfoil_chord_pts": 26, "airfoil_span_pts": 9}

_CACHE = {}
PROTO = {}


def _is_engine_module(k):
    return k in _SHADOWED or k.startswith("parts.")


def _with_engine(fn):
    """Run fn(engine_spec, import_module) with aether/ ahead on the path and
    the aircraft's own spec/mesh/parts out of the way, then put them back."""
    saved = {k: v for k, v in sys.modules.items() if _is_engine_module(k)}
    for k in saved:
        del sys.modules[k]
    sys.path.insert(0, AETHER)
    try:
        espec = importlib.import_module("spec")
        espec.RES.update(RES)
        return fn(espec, importlib.import_module)
    finally:
        sys.path.remove(AETHER)
        for k in [k for k in sys.modules if _is_engine_module(k)]:
            del sys.modules[k]
        sys.modules.update(saved)


def _build_engine():
    if "built" in _CACHE:
        return _CACHE["built"]

    def go(espec, imp):
        mods = ["fan", "compressor", "combustor", "turbine", "augmentor",
                "nozzle", "frames", "spools", "accessories"]
        built, proto = {}, {}
        mats = {}
        for m in mods:
            mod = imp(f"parts.{m}")
            built.update(mod.build())
            proto.update(getattr(mod, "PROTO", {}))
        # the engine's material for each of its parts, by its own rule
        for name in built:
            if name.startswith("cut:"):
                continue
            best, best_len = espec.DEFAULT_MATERIAL, -1
            for key, mat in espec.MATERIAL_MAP.items():
                if name.startswith(key) and len(key) > best_len:
                    best, best_len = mat, len(key)
            mats[name] = best
        noz = imp("parts.nozzle")
        info = {
            "palette": dict(espec.PALETTE),
            "materials": mats,
            "spools": {n: k for k, names in espec.SPOOLS.items() for n in names},
            "nozzle": {"sw_out": noz.SW_OUT, "h_trans": noz.H_TRANS,
                       "flap_t": espec.NOZZLE["flap_t"],
                       "x_trans1": espec.NOZZLE["x_trans1"],
                       "x_throat": espec.NOZZLE["x_throat"],
                       "x_exit": espec.NOZZLE["x_exit"],
                       "h_throat": noz.H_THROAT, "h_exit": noz.H_EXIT,
                       "lines": noz.flap_lines(),
                       "b_shroud": noz.shroud_outer()[2]},
            "thrust_ab": espec.THRUST_AB_N, "thrust_dry": espec.THRUST_DRY_N,
            "mass": espec.DRY_WEIGHT_KG, "name": espec.ENGINE_NAME,
            "trunnion_x": espec.MOUNTS["x_fwd"],
            "trunnion_r": espec.MOUNTS["trunnion_r"],
            "trunnion_top": espec.annulus(espec.THIRD_PATH, espec.MOUNTS["x_fwd"])[1]
                            + 6.0 + espec.MOUNTS["trunnion_len"],
            "aft_lug_x": espec.MOUNTS["x_aft"],
            "od_aft": espec.annulus(espec.THIRD_PATH, espec.MOUNTS["x_aft"])[1]
                      + espec.WALL["outer_case"],
            "inlet_bore": espec.annulus(espec.FAN_PATH, espec.SPINNER["x_base"])[1],
            "inlet_x0": espec.INLET["x0"],
            "inlet_wall": espec.WALL["fan_case"],
        }
        return built, proto, info

    _CACHE["built"] = _with_engine(go)
    return _CACHE["built"]


def info():
    return _build_engine()[2]


def offset(sy):
    return (spec.ENGINE_FAN_FACE_X, sy * spec.ENGINE_Y, spec.ENGINE_Z)


def _move(geom, off):
    v, f = geom
    dx, dy, dz = off
    return [(x + dx, y + dy, z + dz) for (x, y, z) in v], f


def nozzle_box_half():
    """Half-width and half-height, about the engine's axis, of what passes
    the tail closure: the nozzle shroud, and the hydraulic lines that run
    round its sides to the actuators."""
    n = info()["nozzle"]
    return n["sw_out"] + 62.0, n["b_shroud"] + 12.0


# Cutters the installed engines leave out. The augmentor liner's 1,200
# damping holes are inside the engine inside the aircraft, where nothing can
# see them, and cutting them twice is most of the aircraft's build time. The
# audits see the same liner the build makes, so leaving them out is honest;
# the engine repo builds them.
SKIP_CUTS = ("cut:augmentor_liner",)


def build():
    built, proto, _ = _build_engine()
    out = {}
    PROTO.clear()
    for side, sy in SIDES:
        off = offset(sy)
        pre = f"engine_{side}_"
        for name, geom in built.items():
            if name in SKIP_CUTS:
                continue
            if name.startswith("cut:"):
                out["cut:" + pre + name[4:]] = _move(geom, off)
            else:
                out[pre + name] = _move(geom, off)
        for name, (one, holes, count, rest) in proto.items():
            PROTO[pre + name] = (one, holes, count, rest, off)
    return out


def part_material(name):
    """The engine's own material for an installed engine part, or None."""
    for side, _ in SIDES:
        pre = f"engine_{side}_"
        if name.startswith(pre):
            return info()["materials"].get(name[len(pre):])
    return None


def part_spool(name):
    for side, _ in SIDES:
        pre = f"engine_{side}_"
        if name.startswith(pre):
            return info()["spools"].get(name[len(pre):], "")
    return ""
