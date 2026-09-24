"""PBR materials for the aircraft and its installed engines. Requires bpy.

The airframe is a matt grey-blue radar-absorbent finish, a shade darker on
the moving surfaces and door edges, which is how panels with different
coatings read on a real low-observable aircraft. The canopy is a gold-tinted
transmissive glass (the tint is the metallised layer that keeps radar out
of the cockpit). The engines keep the materials the engine repo gives them.
"""

import bpy


def build_all(palette):
    out = {}
    for name, (rgb, metallic, rough) in palette.items():
        mat = bpy.data.materials.get(name) or bpy.data.materials.new(name)
        mat.use_nodes = True
        nt = mat.node_tree
        bsdf = nt.nodes.get("Principled BSDF")
        bsdf.inputs["Base Color"].default_value = (*rgb, 1.0)
        bsdf.inputs["Metallic"].default_value = metallic
        bsdf.inputs["Roughness"].default_value = rough
        if name == "canopy":
            for key in ("Transmission Weight", "Transmission"):
                if key in bsdf.inputs:
                    bsdf.inputs[key].default_value = 0.92
            bsdf.inputs["IOR"].default_value = 1.5
            bsdf.inputs["Metallic"].default_value = 0.0
            bsdf.inputs["Base Color"].default_value = (0.95, 0.78, 0.45, 1.0)
        if name in ("skin", "skin_dark"):
            _grain(nt, bsdf, rough)
        out[name] = mat
    return out


def _grain(nt, bsdf, rough):
    """A faint mottle in the roughness, so a large matt panel reads as a
    coated surface and not as flat grey plastic."""
    coord = nt.nodes.new("ShaderNodeTexCoord")
    tex = nt.nodes.new("ShaderNodeTexNoise")
    tex.inputs["Scale"].default_value = 3.0
    tex.inputs["Detail"].default_value = 6.0
    rng = nt.nodes.new("ShaderNodeMapRange")
    rng.inputs["To Min"].default_value = rough - 0.08
    rng.inputs["To Max"].default_value = rough + 0.08
    nt.links.new(coord.outputs["Object"], tex.inputs["Vector"])
    nt.links.new(tex.outputs["Fac"], rng.inputs["Value"])
    nt.links.new(rng.outputs["Result"], bsdf.inputs["Roughness"])
