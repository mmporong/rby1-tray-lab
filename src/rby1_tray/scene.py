import copy
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from .assets import asset_directory


def load_config(path=None):
    path = Path(path) if path else Path(__file__).resolve().parents[2] / "configs/default.json"
    config = json.loads(path.read_text())
    if config["base_mode"] != "fixture":
        raise ValueError("Only explicitly fixtured manipulation is implemented")
    if config.get("controller") != "ik_position_baseline":
        raise ValueError("Only the IK position baseline controller is implemented")
    if not 0 < config["physics_dt_s"] <= config["control_dt_s"]:
        raise ValueError("Invalid time steps")
    ratio = config["control_dt_s"] / config["physics_dt_s"]
    if not np.isclose(ratio, round(ratio)):
        raise ValueError("control_dt_s must be an integer multiple of physics_dt_s")
    if config.get("motion_duration_scale", 1.0) <= 0:
        raise ValueError("motion_duration_scale must be positive")
    if config.get("payload_mass_scale", 1.0) <= 0:
        raise ValueError("payload_mass_scale must be positive")
    return config


def numbers(values):
    return " ".join(str(float(v)) for v in values)


def build_scene(config):
    directory = asset_directory()
    root = ET.parse(directory / "rby1a_1.2.xml").getroot()
    root.find("compiler").set("meshdir", str(directory / "assets"))
    ET.SubElement(root, "option", timestep=str(config["physics_dt_s"]), integrator="implicitfast",
                  solver="Newton", iterations="40", cone="elliptic", impratio="100")
    ET.SubElement(root, "size", memory="64M")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth="1280", offheight="960")
    ET.SubElement(visual, "headlight", ambient="0.45 0.45 0.45", diffuse="0.7 0.7 0.7")
    ET.SubElement(root.find("asset"), "texture", type="skybox", builtin="gradient", rgb1=".94 .96 .99", rgb2=".94 .96 .99", width="256", height="256")
    world = root.find("worldbody")
    ET.SubElement(world, "light", pos="1 -1 4", dir="-0.2 0 -1", directional="true")
    ET.SubElement(world, "geom", name="floor", type="plane", size="4 4 .1", rgba=".91 .93 .95 1",
                  friction="0.8 0.01 0.001")
    # Remove the free base joint: an explicit fixture, never an object attachment.
    base = world.find("body[@name='base']")
    base.remove(base.find("joint[@name='world_j']"))
    # Adjacent assembly meshes overlap even in the upstream zero pose.
    # Exclude only these structural neighbors; retain arm/environment collisions.
    for neighbor in ("wheel_l", "wheel_r", "link_torso_0"):
        ET.SubElement(root.find("contact"), "exclude", body1="base", body2=neighbor)
    for index, geom in enumerate(world.findall(".//geom")):
        if not geom.get("name"):
            geom.set("name", f"robot_geom_{index}")
    for side, letter in [("left", "l"), ("right", "r")]:
        wrist = root.find(f".//body[@name='link_{side}_arm_6']")
        ET.SubElement(wrist, "site", name=f"{side}_tcp", pos="0 0 -0.228", size=".008", rgba="1 .4 0 1")
        for finger in (1, 2):
            body = root.find(f".//body[@name='ee_finger_{letter}{finger}']")
            for index, geom in enumerate(body.findall("geom")):
                if geom.get("class") != "visual":
                    geom.set("contype", "0")
                    geom.set("conaffinity", "0")
            ET.SubElement(body, "geom", name=f"{side}_pad_{finger}_proxy", type="box",
                          pos="0 0 -.027", size=".003 .014 .025", mass="0",
                          friction="2 .05 .01", condim="4", solref=".005 1", rgba=".25 .27 .3 1")
    for equality in root.find("equality"):
        equality.set("solref", ".003 1")
    tray = ET.SubElement(world, "body", name="tray", pos=numbers(config["tray_initial_pos_m"]))
    ET.SubElement(tray, "freejoint", name="tray_free")
    hx, hy, hz = config["tray_half_extents_m"]
    ET.SubElement(tray, "geom", name="tray_floor", type="box", size=numbers([hx, hy, hz]),
                  mass=str(config["tray_mass_kg"]), rgba=".16 .45 .66 1", friction=".8 .01 .001")
    for sign in (-1, 1):
        ET.SubElement(tray, "geom", name=f"tray_xrim_{sign}", type="box", size=numbers([.006, hy, .025]),
                      pos=numbers([sign * hx, 0, hz + .025]), mass=".012", rgba=".12 .35 .52 1")
        ET.SubElement(tray, "geom", name=f"tray_yrim_{sign}", type="box", size=numbers([hx, .006, .025]),
                      pos=numbers([0, sign * hy, hz + .025]), mass=".012", rgba=".12 .35 .52 1")
    for side, sign in [("left", 1), ("right", -1)]:
        y, z = sign * config["handle_y_m"], config["handle_z_m"]
        ET.SubElement(tray, "geom", name=f"handle_{side}", type="box", size=".014 .048 .012",
                      pos=numbers([0, y, z]), mass=".02", rgba=".9 .65 .15 1", friction="1 .02 .001", condim="4", solref=".005 1")
        for index, post_y in enumerate([y - .04, y + .04]):
            ET.SubElement(tray, "geom", name=f"handle_post_{side}_{index}", type="box", size=numbers([.008, .008, z / 2]),
                          pos=numbers([0, post_y, z / 2]), mass=".008", rgba=".8 .55 .1 1")
    rng = np.random.default_rng(config["seed"])
    for index, (shape, offset, size, mass, rgba) in enumerate([
        ("sphere", [-.07, -.08, .047], [.023], .035, ".95 .3 .2 1"),
        ("box", [.07, -.02, .048], [.025, .025, .024], .055, ".95 .75 .15 1"),
        ("cylinder", [0, .1, .07], [.02, .047], .045, ".25 .7 .4 1"),
    ]):
        pos = np.array(config["tray_initial_pos_m"]) + offset
        pos[:2] += rng.uniform(-.004, .004, 2)
        body = ET.SubElement(world, "body", name=f"payload_{index}", pos=numbers(pos))
        ET.SubElement(body, "freejoint", name=f"payload_{index}_free")
        ET.SubElement(body, "geom", name=f"payload_{index}_geom", type=shape, size=numbers(size),
                      mass=str(mass * config.get("payload_mass_scale", 1.0)), rgba=rgba, friction=".5 .01 .001", condim="4")
    # The target is placed laterally so the floor pickup and lift path are unobstructed.
    target = config["target"]
    cx, cy = target["center_xy_m"]
    tx, ty = target["inner_half_xy_m"]
    sz = target["support_z_m"]
    ET.SubElement(world, "geom", name="container_bottom", type="box", pos=numbers([cx, cy, sz - .015]),
                  size=numbers([tx, ty, .015]), rgba=".6 .65 .72 1")
    for sign in (-1, 1):
        ET.SubElement(world, "geom", name=f"container_xwall_{sign}", type="box",
                      pos=numbers([cx + sign * (tx + .012), cy, sz + .035]), size=numbers([.012, ty + .024, .035]), rgba=".65 .72 .8 .7")
        ET.SubElement(world, "geom", name=f"container_ywall_{sign}", type="box",
                      pos=numbers([cx, cy + sign * (ty + .012), sz + .035]), size=numbers([tx, .012, .035]), rgba=".65 .72 .8 .7")
    xml = ET.tostring(root, encoding="unicode")
    return mujoco.MjModel.from_xml_string(xml), xml


def audit_model(model):
    objects = ["tray", "payload_0", "payload_1", "payload_2"]
    object_ids = {model.body(name).id for name in objects}
    if model.nmocap:
        raise ValueError("Mocap bodies are prohibited in this task")
    for name in objects:
        body = model.body(name).id
        joint = model.body_jntadr[body]
        if model.body_mocapid[body] >= 0 or joint < 0 or model.jnt_type[joint] != mujoco.mjtJoint.mjJNT_FREE:
            raise ValueError(f"{name} must be a dynamic free body")
    for index in range(model.neq):
        if model.eq_type[index] == mujoco.mjtEq.mjEQ_WELD:
            raise ValueError("Weld constraints are prohibited in task objects")
        if model.eq_type[index] == mujoco.mjtEq.mjEQ_CONNECT and (
            model.eq_obj1id[index] in object_ids or model.eq_obj2id[index] in object_ids
        ):
            raise ValueError("Task objects cannot use equality connect constraints")
    return {"free_objects": True, "object_attachment": False, "runtime_state_overwrite": False,
            "base_mode": "fixture", "stability_claim": False,
            "robot": "RBY1-A 1.2", "nq": model.nq, "nv": model.nv, "nu": model.nu}
