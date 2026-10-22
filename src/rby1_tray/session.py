"""Actuated MuJoCo session; live free-body state is written only by mj_step."""
import copy
import os

import mujoco
import numpy as np

from .evaluation import Evaluator
from .planning import build_plan, smoothstep
from .scene import build_scene, audit_model, load_config


class Session:
    def __init__(self, config=None, lift_only=False):
        self.config = copy.deepcopy(config or load_config())
        self.lift_only = lift_only
        self.model, self.xml = build_scene(self.config)
        self.audit = audit_model(self.model)
        self.robot, self.plan, self.ik_errors = build_plan(self.model, self.config, lift_only)
        self.renderer = None
        self.finished = False
        self.reset()

    def reset(self):
        self.data = mujoco.MjData(self.model)
        self.data.qpos[self.robot.qadr] = self.plan[0].joints_rad
        for letter in ("l", "r"):
            for finger in (1, 2):
                self.data.joint(f"gripper_finger_{letter}{finger}").qpos[0] = -.045
        self.command_rad = self.plan[0].joints_rad.copy()
        self.command_gripper_m = -.045
        mujoco.mj_forward(self.model, self.data)
        for _ in range(250):
            self._set_controls(self.command_rad, self.command_gripper_m)
            mujoco.mj_step(self.model, self.data)
        self.data.time = 0.0
        self._expected_qpos = self.data.qpos.copy()
        self._expected_qvel = self.data.qvel.copy()
        self._expected_eq_active = self.data.eq_active.copy()
        self._expected_eq_data = self.model.eq_data.copy()
        self._expected_mocap_pos = self.data.mocap_pos.copy()
        self._expected_mocap_quat = self.data.mocap_quat.copy()
        self.audit["runtime_state_overwrite"] = False
        self.phase = "approach"
        self.end_times_s = np.cumsum([p.duration_s for p in self.plan])
        cfg = copy.deepcopy(self.config)
        cfg["initial_tray_bottom_z_m"] = self.data.body("tray").xpos[2] - cfg["tray_half_extents_m"][2]
        cfg["expected_payload_names"] = [f"payload_{i}" for i in range(3)]
        cfg["payload_max_height_m"] = .25
        tray_id = self.model.body("tray").id
        geom_ids = np.flatnonzero(self.model.geom_bodyid == tray_id)
        # All task tray geoms are unrotated boxes in the tray body frame.
        geom_low = self.model.geom_pos[geom_ids] - self.model.geom_size[geom_ids]
        geom_high = self.model.geom_pos[geom_ids] + self.model.geom_size[geom_ids]
        low, high = geom_low.min(axis=0), geom_high.max(axis=0)
        cfg["tray_containment_half_extents_m"] = ((high - low) / 2).tolist()
        cfg["tray_containment_center_m"] = ((high + low) / 2).tolist()
        self.evaluator = Evaluator(cfg)
        self.log = []
        self.finished = False
        self._last_record_s = -.1
        self.max_tracking_error_rad = 0.0
        self.max_joint_torque_nm = 0.0
        self.max_finger_force_n = 0.0
        self._next_control_s = 0.0
        self._held_velocity_rad_s = np.zeros(len(self.robot.names))
        self._record(force_log=True)

    def _set_controls(self, joints_rad, gripper_m, velocity_rad_s=None):
        # Gravity feedforward through the same force-limited position actuators.
        for i, aid in enumerate(self.robot.act):
            dof = self.robot.dadr[i]
            kp = self.model.actuator_gainprm[aid, 0]
            kv = -self.model.actuator_biasprm[aid, 2]
            desired_velocity = 0.0 if velocity_rad_s is None else velocity_rad_s[i]
            command = joints_rad[i] + (self.data.qfrc_bias[dof] + kv * desired_velocity) / kp
            self.data.ctrl[aid] = np.clip(command, *self.model.actuator_ctrlrange[aid])
        for aid in self.robot.grip_act:
            self.data.ctrl[aid] = gripper_m

    def _reference(self, time_s):
        index = min(int(np.searchsorted(self.end_times_s, time_s, side="right")), len(self.plan) - 1)
        waypoint = self.plan[index]
        start_s = self.end_times_s[index - 1] if index else 0.0
        previous = self.plan[index - 1] if index else waypoint
        fraction = smoothstep((time_s - start_s) / waypoint.duration_s)
        return waypoint.phase, previous.joints_rad + fraction * (waypoint.joints_rad - previous.joints_rad), previous.gripper_m + fraction * (waypoint.gripper_m - previous.gripper_m)

    def step(self, n_steps=1):
        for _ in range(n_steps):
            if self.finished: break
            if not np.array_equal(self.data.qpos, self._expected_qpos):
                self.audit["runtime_state_overwrite"] = True
            if not np.array_equal(self.data.qvel, self._expected_qvel) or np.any(self.data.xfrc_applied) or np.any(self.data.qfrc_applied):
                self.audit["runtime_state_overwrite"] = True
            if any(not np.array_equal(current, expected) for current, expected in (
                (self.data.eq_active, self._expected_eq_active),
                (self.model.eq_data, self._expected_eq_data),
                (self.data.mocap_pos, self._expected_mocap_pos),
                (self.data.mocap_quat, self._expected_mocap_quat),
            )):
                self.audit["runtime_state_overwrite"] = True
            if self.data.time + 1e-9 >= self._next_control_s:
                previous_reference = self.command_rad.copy()
                self.phase, self.command_rad, self.command_gripper_m = self._reference(self.data.time)
                self._held_velocity_rad_s = (self.command_rad - previous_reference) / self.config["control_dt_s"]
                self._set_controls(self.command_rad, self.command_gripper_m, self._held_velocity_rad_s)
                self._next_control_s += self.config["control_dt_s"]
            mujoco.mj_step(self.model, self.data)
            self._expected_qpos = self.data.qpos.copy()
            self._expected_qvel = self.data.qvel.copy()
            error = float(np.max(np.abs(self.data.qpos[self.robot.qadr] - self.command_rad)))
            self.max_tracking_error_rad = max(self.max_tracking_error_rad, error)
            self.max_joint_torque_nm = max(self.max_joint_torque_nm, float(np.max(np.abs(self.data.actuator_force[self.robot.act]))))
            self.max_finger_force_n = max(self.max_finger_force_n, float(np.max(np.abs(self.data.actuator_force[self.robot.grip_act]))))
            self._record()
            if self.data.time >= self.end_times_s[-1] and not self.finished:
                self.finished = True
                if not self.lift_only:
                    self.evaluator.finalize(float(self.data.time))
                    self.log.append(self.snapshot())
        return self.snapshot()

    def _contacts(self):
        pads = {"left_fingers": [False, False], "right_fingers": [False, False],
                "container_bottom": False, "forbidden": [], "robot_tray_contact": False}
        forces = {"left": 0.0, "right": 0.0}
        pad_forces = {side: [0.0, 0.0] for side in ("left", "right")}
        normals = {side: [np.zeros(3), np.zeros(3)] for side in ("left", "right")}
        supported = {f"payload_{i}": False for i in range(3)}
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            ids = [int(contact.geom1), int(contact.geom2)]
            names = [self.model.geom(g).name for g in ids]
            bodies = [self.model.body(int(self.model.geom_bodyid[g])).name for g in ids]
            force = np.zeros(6)
            mujoco.mj_contactForce(self.model, self.data, i, force)
            if force[0] <= .01: continue
            if "container_bottom" in names and "tray_floor" in names:
                pads["container_bottom"] = True
            for name in supported:
                if f"{name}_geom" in names and "tray_floor" in names:
                    payload_index = names.index(f"{name}_geom")
                    tray_up = self.data.body("tray").xmat.reshape(3, 3)[:, 2]
                    normal_on_payload = contact.frame.reshape(3, 3)[0] * (1 if payload_index == 1 else -1)
                    tolerance = self.config["thresholds"]["containment_tolerance_m"]
                    above_floor = np.dot(self.data.body(name).xpos - self.data.body("tray").xpos, tray_up) >= self.config["tray_half_extents_m"][2] - tolerance
                    if above_floor and np.dot(normal_on_payload, tray_up) > .5:
                        supported[name] = True
            for side in ("left", "right"):
                for finger in (1, 2):
                    if any(n.startswith(f"{side}_pad_{finger}_") for n in names) and f"handle_{side}" in names:
                        pads[f"{side}_fingers"][finger - 1] = True
                        forces[side] += float(force[0])
                        pad_forces[side][finger - 1] += float(force[0])
                        pad_index = next(j for j, name in enumerate(names) if name.startswith(f"{side}_pad_{finger}_"))
                        normals[side][finger - 1] += contact.frame.reshape(3, 3)[0] * force[0] * (1 if pad_index == 0 else -1)
            robot_flags = [n.startswith(("robot_geom_", "left_pad_", "right_pad_")) for n in names]
            if any(robot_flags) and "tray" in bodies:
                pads["robot_tray_contact"] = True
            if any(robot_flags) and contact.dist < -.002:
                allowed = any(n.startswith(("left_pad_", "right_pad_")) for n in names) and any(n.startswith("handle_") for n in names)
                base_floor = "floor" in names and any(b in {"base", "wheel_l", "wheel_r"} for b in bodies)
                if not allowed and not base_floor:
                    pads["forbidden"].append("/".join(names))
        pads["forbidden"] = sorted(set(pads["forbidden"]))
        for side in ("left", "right"):
            first, second = normals[side]
            denominator = np.linalg.norm(first) * np.linalg.norm(second)
            pads[f"{side}_opposed"] = bool(denominator > 1e-8 and np.dot(first, second) / denominator < -.5)
        return pads, forces, pad_forces, supported

    def snapshot(self):
        d, m = self.data, self.model
        tid = m.body("tray").id
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, tid, velocity, 0)
        pos, quat = d.xpos[tid].copy(), d.xquat[tid].copy()
        contacts, forces, pad_forces, supported = self._contacts()
        payloads = []
        for i in range(3):
            name = f"payload_{i}"
            gid = m.geom(f"{name}_geom").id
            shape, size = m.geom_type[gid], m.geom_size[gid]
            if shape == mujoco.mjtGeom.mjGEOM_SPHERE:
                radius = float(size[0])
            elif shape == mujoco.mjtGeom.mjGEOM_BOX:
                radius = float(np.linalg.norm(size))
            elif shape == mujoco.mjtGeom.mjGEOM_CYLINDER:
                radius = float(np.linalg.norm(size[:2]))
            else:
                raise ValueError(f"Unsupported payload bounding shape: {shape}")
            body_velocity = np.zeros(6)
            mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, m.body(name).id, body_velocity, 0)
            payloads.append({"name": name, "pos_m": d.body(name).xpos.tolist(), "radius_m": radius,
                             "supported_by_tray": supported[name], "linear_speed_m_s": float(np.linalg.norm(body_velocity[3:]))})
        clearance = {}
        rotation = d.xmat[tid].reshape(3, 3)
        for side, sign in [("left", 1), ("right", -1)]:
            handle = pos + rotation @ np.array([0, sign * self.config["handle_y_m"], self.config["handle_z_m"]])
            clearance[side] = float(np.linalg.norm(d.site(f"{side}_tcp").xpos - handle))
        joints = d.qpos[self.robot.qadr]
        violation = bool(np.any(joints < self.robot.low - .01) or np.any(joints > self.robot.high + .01))
        state = {"time_s": float(d.time), "phase": self.phase, "finished": self.finished,
            "tray": {"pos_m": pos.tolist(), "quat_wxyz": quat.tolist(),
                     "bottom_z_m": float(pos[2] - np.abs(rotation[2]) @ self.config["tray_half_extents_m"]),
                     "linear_velocity_m_s": velocity[3:].tolist(), "angular_velocity_rad_s": velocity[:3].tolist()},
            "payloads": payloads, "contacts": contacts, "grasp_force_n": forces,
            "pad_forces_n": pad_forces,
            "hand_clearance_m": clearance, "joint_limit_violation": violation, "audit": self.audit.copy(),
            "metrics": {"max_tracking_error_rad": self.max_tracking_error_rad,
                        "max_joint_torque_nm": self.max_joint_torque_nm,
                        "max_finger_actuator_force_n": self.max_finger_force_n},
            "result": self.evaluator.result() if hasattr(self, "evaluator") else {},
            "backend": "CPU MuJoCo", "controller": "IK position baseline", "base_mode": "fixture",
            "ik_errors": self.ik_errors}
        state["joint_state"] = {"names": self.robot.names, "position_rad": joints.tolist(),
                                "reference_rad": self.command_rad.tolist(),
                                "actuator_force": d.actuator_force[self.robot.act].tolist()}
        state["tcp_pos_m"] = {side: d.site(f"{side}_tcp").xpos.tolist() for side in ("left", "right")}
        state["gripper_position_m"] = {letter: [float(d.joint(f"gripper_finger_{letter}{finger}").qpos[0]) for finger in (1, 2)] for letter in ("l", "r")}
        return state

    def _record(self, force_log=False):
        mujoco.mj_forward(self.model, self.data)
        state = self.snapshot()
        state["result"] = self.evaluator.update(state)
        if force_log or state["result"]["terminal"] or self.data.time - self._last_record_s >= self.config["record_dt_s"] - 1e-9:
            self.log.append(state)
            self._last_record_s = self.data.time
        if state["result"]["terminal"]:
            self.finished = True

    def render(self, width=960, height=720, azimuth=135, elevation=-20):
        if self.renderer is None or self.renderer.width != width or self.renderer.height != height:
            if self.renderer: self.renderer.close()
            self.renderer = mujoco.Renderer(self.model, height, width)
        camera = mujoco.MjvCamera()
        camera.lookat[:] = [.4, .25, .75]
        camera.distance = 2.7
        camera.azimuth = azimuth
        camera.elevation = elevation
        self.renderer.update_scene(self.data, camera)
        return self.renderer.render().copy()

    def close(self):
        if self.renderer: self.renderer.close()
