"""Render the aircraft. Run under `blender --background`.

    blender -b build/nyx.blend -P nyx/render.py -- <mode> [samples]

Modes: hero, plan, side, rear, xray, all (those five), closeups, or any
single close-up by name.
"""

import math
import os
import sys

import bpy
from mathutils import Vector

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
OUT = os.path.join(ROOT, "renders")
sys.path.insert(0, HERE)
import spec  # noqa: E402

MM = 0.001
XM = 7.3                      # aircraft centre, m
ZG = spec.GROUND_Z * MM


def world(strength=0.5):
    w = bpy.data.worlds.get("World") or bpy.data.worlds.new("World")
    bpy.context.scene.world = w
    w.use_nodes = True
    nt = w.node_tree
    nt.nodes.clear()
    out = nt.nodes.new("ShaderNodeOutputWorld")
    bg = nt.nodes.new("ShaderNodeBackground")
    grad = nt.nodes.new("ShaderNodeTexGradient")
    ramp = nt.nodes.new("ShaderNodeValToRGB")
    tc = nt.nodes.new("ShaderNodeTexCoord")
    mp = nt.nodes.new("ShaderNodeMapping")
    mp.inputs["Rotation"].default_value = (math.radians(90), 0, 0)
    ramp.color_ramp.elements[0].color = (0.020, 0.024, 0.030, 1)
    ramp.color_ramp.elements[1].color = (0.16, 0.18, 0.21, 1)
    nt.links.new(tc.outputs["Generated"], mp.inputs["Vector"])
    nt.links.new(mp.outputs["Vector"], grad.inputs["Vector"])
    nt.links.new(grad.outputs["Color"], ramp.inputs["Fac"])
    nt.links.new(ramp.outputs["Color"], bg.inputs["Color"])
    nt.links.new(bg.outputs["Background"], out.inputs["Surface"])
    bg.inputs["Strength"].default_value = strength


def ground():
    """A large matt floor under the wheels, darkening into the distance."""
    if "__ground" in bpy.data.objects:
        bpy.data.objects["__ground"].hide_render = False
        return
    bpy.ops.mesh.primitive_plane_add(size=120, location=(XM, 0, ZG))
    g = bpy.context.active_object
    g.name = "__ground"
    mat = bpy.data.materials.new("__ground")
    mat.use_nodes = True
    b = mat.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = (0.055, 0.058, 0.062, 1)
    b.inputs["Roughness"].default_value = 0.55
    g.data.materials.append(mat)


def light(name, loc, target, energy, size):
    d = bpy.data.lights.new(name, type="AREA")
    d.energy, d.size, d.shape, d.size_y = energy, size, "RECTANGLE", size * 0.6
    o = bpy.data.objects.new(name, d)
    o.location = loc
    bpy.context.scene.collection.objects.link(o)
    aim(o, target)


def lights(scale=1.0):
    for o in [o for o in bpy.data.objects if o.type == "LIGHT"]:
        bpy.data.objects.remove(o, do_unlink=True)
    c = (XM, 0, 0)
    light("key", (XM - 8, -14, 14), c, 5200 * scale, 16)
    light("fill", (XM + 10, 14, 4), c, 1300 * scale, 20)
    light("rim", (XM + 20, 4, 9), c, 3600 * scale, 10)
    light("top", (XM, 0, 20), c, 1400 * scale, 22)


def aim(o, target):
    d = Vector(target) - o.location
    o.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()


def camera(loc, target, lens=50.0, ortho=None):
    cam = bpy.data.cameras.new("cam")
    cam.lens = lens
    cam.clip_end = 400
    if ortho:
        cam.type = "ORTHO"
        cam.ortho_scale = ortho
    o = bpy.data.objects.new("cam", cam)
    o.location = loc
    bpy.context.scene.collection.objects.link(o)
    bpy.context.scene.camera = o
    aim(o, target)


def setup(samples, res=(1920, 1080)):
    s = bpy.context.scene
    s.render.engine = "CYCLES"
    s.cycles.samples = samples
    s.cycles.use_denoising = True
    s.cycles.max_bounces = 8
    s.render.resolution_x, s.render.resolution_y = res
    s.view_settings.view_transform = "AgX"
    s.view_settings.look = "AgX - Medium High Contrast"
    s.view_settings.exposure = -0.3
    try:
        prefs = bpy.context.preferences.addons["cycles"].preferences
        prefs.compute_device_type = "METAL"
        prefs.get_devices()
        for d in prefs.devices:
            d.use = True
        s.cycles.device = "GPU"
    except Exception as exc:
        print("  (GPU unavailable:", exc, ")")


def reset():
    for o in list(bpy.data.objects):
        if o.type == "CAMERA" or (o.name.startswith("__") and o.name != "__ground"):
            bpy.data.objects.remove(o, do_unlink=True)
    for o in bpy.data.objects:
        if o.type != "MESH":
            continue
        # the floor is kept between modes but shown only by those that ask
        # for it: the flight shot is in the air
        o.hide_render = o.name == "__ground"
        for m in list(o.modifiers):
            if m.name == "section":
                o.modifiers.remove(m)
        if "home_rot" in o:
            o.rotation_euler = tuple(o["home_rot"])
            o.location = tuple(o["home_loc"])
        else:
            o["home_rot"] = tuple(o.rotation_euler)
            o["home_loc"] = tuple(o.location)


def shoot(name):
    os.makedirs(OUT, exist_ok=True)
    p = os.path.join(OUT, name + ".png")
    bpy.context.scene.render.filepath = p
    bpy.ops.render.render(write_still=True)
    print("  ->", p)


def swing(obj_name, pivot, axis, deg):
    """Rotate an object about a hinge line through `pivot` (m) along axis."""
    o = bpy.data.objects.get(obj_name)
    if o is None:
        return
    from mathutils import Matrix
    R = Matrix.Rotation(math.radians(deg), 4, axis)
    T = Matrix.Translation(Vector(pivot))
    o.matrix_world = T @ R @ T.inverted() @ o.matrix_world


def open_bay():
    """Swing the weapons-bay doors open about their outboard hinge lines."""
    B = spec.BAY
    for side, sy in (("r", 1.0), ("l", -1.0)):
        y = sy * B["half_w"] * MM
        x = 0.5 * (B["x0"] + B["x1"]) * MM
        import sys as _s
        _s.path.insert(0, HERE)
        import shapes
        z = shapes.z_dn(0.5 * (B["x0"] + B["x1"]), sy * B["half_w"]) * MM
        swing(f"bay_door_{side}", (x, y, z), "X", sy * 100.0)
        # and each door's actuators after it: the body turned about its
        # anchor to point at the horn where the door has taken it, the rod
        # run out along it
        from parts import bay
        from mathutils import Matrix
        H = Vector((x, y, z))
        R = Matrix.Rotation(math.radians(sy * 100.0), 4, "X")
        for k, bx in enumerate(bay.BAY_ACT_X):
            A = Vector(bay.bay_anchor(bx, sy)) * MM
            L0 = Vector(bay.bay_horn(bx, sy)) * MM
            L1 = H + (R @ (L0 - H).to_4d()).to_3d()
            q = (L0 - A).rotation_difference(L1 - A).to_matrix().to_4x4()
            ext = (L1 - A).length - (L0 - A).length
            TA = Matrix.Translation(A)
            for name, shift in ((f"bay_door_act_{k + 1}_{side}", 0.0),
                                (f"bay_door_act_rod_{k + 1}_{side}", ext)):
                o = bpy.data.objects.get(name)
                if o is None:
                    continue
                d = (L1 - A).normalized() * shift
                o.matrix_world = Matrix.Translation(d) @ TA @ q @ TA.inverted() @ o.matrix_world


# --------------------------------------------------------------------------

def mode_hero(samples):
    reset(); setup(samples); world(); ground(); lights()
    camera((XM - 17.0, -15.5, 5.2), (XM - 0.6, 0, -0.5), 55)
    shoot("01_hero")


def mode_plan(samples):
    reset(); setup(samples); world(0.6); lights()
    if "__ground" in bpy.data.objects:
        bpy.data.objects["__ground"].hide_render = True
    camera((XM, 0, 40), (XM, 0, 0), ortho=17.5)
    shoot("02_plan")


def mode_side(samples):
    reset(); setup(samples, (1920, 800)); world(); ground(); lights()
    camera((XM, -40, -0.4), (XM, 0, -0.4), ortho=16.5)
    shoot("03_side")


def mode_rear(samples):
    reset(); setup(samples); world(); ground(); lights()
    camera((XM + 17.5, 12.0, 3.6), (XM + 1.8, 0, -0.4), 55)
    shoot("04_rear")


XRAY = ("fuselage_skin", "canopy_glass",
        "wing", "le_flap", "flaperon", "canard", "fin", "rudder", "bay_door")


def mode_xray(samples):
    """The skin and surfaces hidden on the near (port) side, so the engines,
    ducts, bay and gear show in place."""
    reset(); setup(samples); world(); ground(); lights()
    bpy.ops.mesh.primitive_cube_add(size=1)
    cut = bpy.context.active_object
    cut.name = "__section"
    cut.scale = (40, 20, 20)
    cut.location = (XM, -10.0, 0)
    cut.hide_render = True
    for o in bpy.data.objects:
        if o.type == "MESH" and o.name.startswith(XRAY):
            m = o.modifiers.new("section", "BOOLEAN")
            m.operation = "DIFFERENCE"
            m.solver = "FLOAT"
            m.object = cut
    camera((XM - 9.0, -17.0, 7.5), (XM + 0.4, 0, -0.3), 50)
    shoot("05_xray")


CLOSE = {
    "c1_cockpit": ((3.2, -4.2, 2.6), (4.4, 0.0, 0.6), 50),
    "c2_intake": ((2.8, -4.6, -0.9), (5.4, -1.3, -0.4), 42),
    "c3_nozzles": ((17.6, 4.2, 1.8), (14.1, 0.0, 0.0), 45),
    "c4_bay": ((6.3, -0.2, -2.3), (7.3, 0.05, -0.45), 24),
    "c5_main_gear": ((6.4, -7.4, -0.7), (9.5, -2.9, -1.05), 38),
    "c6_wing": ((12.0, -10.0, 4.0), (10.5, -4.0, 0.0), 45),
    "c7_avionics": ((1.15, -1.55, 1.35), (2.25, 0.0, 0.38), 32),
    "c8_ecs": ((11.25, -1.05, 1.45), (10.3, 0.0, 0.33), 30),
}
# what the avionics close-up takes off to see into the forward bay
SKIN_OFF = ("fuselage_skin", "panel_seams", "irst_", "canopy_glass",
            "canopy_rim", "canopy_frame_", "air_data_probe_", "formation_light_")


def mode_close(name, samples):
    reset(); setup(samples, (1600, 900)); world(); ground(); lights(0.8)
    if name == "c4_bay":
        open_bay()
        # the camera is under the aircraft, below where the ground plane is:
        # it was photographing the floor's underside, and the shot was black
        if "__ground" in bpy.data.objects:
            bpy.data.objects["__ground"].hide_render = True
    if name in ("c7_avionics", "c8_ecs"):
        off = SKIN_OFF + (("fuel_tank_centre", "keel", "frame_engine_fwd")
                          if name == "c8_ecs" else ())
        for o in bpy.data.objects:
            if o.type == "MESH" and o.name.startswith(off):
                o.hide_render = True
    loc, tgt, lens = CLOSE[name]
    if tgt[2] < -0.3:
        # under the aircraft the key and top lights are behind the airframe:
        # a low fill from the camera's side, as a hangar's floor lights give
        light("under", (loc[0] - 2.0, loc[1] - 3.0, -1.2), tgt, 900, 6)
    camera(loc, tgt, lens)
    shoot(name)


def pose_gear_up():
    """Raise the gear and shut its doors, from the same kinematics the
    viewer uses (parts/gear.py): each leg turned about its pivot, each door
    about its hinge."""
    from mathutils import Matrix, Vector
    from parts import gear
    k = gear.kinematics()

    def turn(names, point, axis, ang):
        c = Vector(point) * MM
        R = (Matrix.Translation(c) @ Matrix.Rotation(ang, 4, Vector(axis))
             @ Matrix.Translation(-c))
        for n in names:
            o = bpy.data.objects.get(n)
            if o is not None:
                o.matrix_world = R @ o.matrix_world
    for L in k["legs"]:
        turn(L["parts"], L["pivot"], L["axis"], L["stow"])
    for d in k["doors"]:
        turn([d["part"]], d["hinge"], d["axis"], d["close"])


def mode_flight(samples):
    """In the air, gear up, banking into a turn: how it spends its life."""
    reset(); setup(samples); world(0.55); lights()
    pose_gear_up()
    import math as _m
    from mathutils import Matrix
    from mathutils import Vector
    c = Vector((XM, 0.0, 0.0))
    bank = (Matrix.Translation(c) @ Matrix.Rotation(_m.radians(-28.0), 4, "X")
            @ Matrix.Rotation(_m.radians(6.0), 4, "Y") @ Matrix.Translation(-c))
    for o in bpy.data.objects:
        if o.type == "MESH" and not o.name.startswith("__"):
            o.matrix_world = bank @ o.matrix_world
    camera((XM - 15.5, -13.0, 7.5), (XM - 0.2, 0, 0.3), 55)
    shoot("00_flight")


MODES = {"hero": mode_hero, "plan": mode_plan, "side": mode_side,
         "rear": mode_rear, "xray": mode_xray, "flight": mode_flight}

if __name__ == "__main__":
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else ["hero"]
    mode = argv[0]
    samples = int(argv[1]) if len(argv) > 1 else 64
    if mode == "all":
        for m in MODES:
            MODES[m](samples)
    elif mode == "closeups":
        for n in CLOSE:
            mode_close(n, samples)
    elif mode in CLOSE:
        mode_close(mode, samples)
    else:
        MODES[mode](samples)
