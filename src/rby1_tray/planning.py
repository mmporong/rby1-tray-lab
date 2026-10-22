"""Kinematic references; physics execution is a separate layer."""
from dataclasses import dataclass

import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


@dataclass
class Waypoint:
    phase: str
    duration_s: float
    joints_rad: np.ndarray
    gripper_m: float


class Robot:
    POSITION_TOLERANCE_M = .005
    ORIENTATION_TOLERANCE_RAD = .02
    TORQUE_LIMIT_RATIO = .90

    def __init__(self, model, use_torso=True, fixed_torso_rad=None):
        self.model = model
        self.use_torso = bool(use_torso)
        self.names = [f"torso_{i}" for i in range(6)] + [
            f"{side}_arm_{i}" for side in ("right", "left") for i in range(7)]
        self.jids = np.array([model.joint(n).id for n in self.names])
        self.qadr = model.jnt_qposadr[self.jids]
        self.dadr = model.jnt_dofadr[self.jids]
        self.act = np.array([next(i for i in range(model.nu) if model.actuator_trnid[i, 0] == j) for j in self.jids])
        self.low = np.maximum(model.jnt_range[self.jids, 0], model.actuator_ctrlrange[self.act, 0]) + .005
        self.high = np.minimum(model.jnt_range[self.jids, 1], model.actuator_ctrlrange[self.act, 1]) - .005
        default_torso = np.array([0, 1.514, -2.20, 1.50, 0, 0], dtype=float)
        self.fixed_torso = np.asarray(
            default_torso if fixed_torso_rad is None else fixed_torso_rad,
            dtype=float,
        )
        if self.fixed_torso.shape != (6,) or not np.all(np.isfinite(self.fixed_torso)):
            raise ValueError("fixed_torso_rad must contain six finite joint values")
        if np.any(self.fixed_torso < self.low[:6]) or np.any(self.fixed_torso > self.high[:6]):
            raise ValueError("fixed_torso_rad exceeds the controllable torso joint limits")
        self.active = np.arange(20) if self.use_torso else np.arange(6, 20)
        self.tcp = [model.site(f"{s}_tcp").id for s in ("right", "left")]
        self.grip_act = [model.actuator(f"{s}_finger_act").id for s in ("right", "left")]
        self.grip_qadr = np.array([
            model.jnt_qposadr[model.joint(f"gripper_finger_{letter}{finger}").id]
            for letter in ("r", "l") for finger in (1, 2)
        ])
        self.force_limit = np.max(np.abs(model.actuator_forcerange[self.act]), axis=1)
        base = model.body("base").id
        self.robot_bodies = {
            body for body in range(model.nbody) if self._is_descendant(body, base)
        }
        self.work = mujoco.MjData(model)

    def _is_descendant(self, body, ancestor):
        while body:
            if body == ancestor:
                return True
            body = self.model.body_parentid[body]
        return body == ancestor

    def crouched_pickup_posture(self):
        """Return a low, gravity-torque-feasible starting posture."""
        posture = np.zeros(len(self.names))
        offset = 0
        posture[:6] = self.fixed_torso
        offset = 6
        posture[offset:offset + 7] = [0, -.8, 0, -1.5, 0, .5, 0]
        posture[offset + 7:offset + 14] = [0, .8, 0, -1.5, 0, .5, 0]
        return np.clip(posture, self.low, self.high)

    def _prepare(self, q, gripper_m):
        self.work.qpos[self.qadr] = q
        self.work.qpos[self.grip_qadr] = gripper_m
        mujoco.mj_forward(self.model, self.work)

    def _forbidden_contacts(self):
        forbidden = []
        structural = {
            frozenset(("base", "wheel_l")),
            frozenset(("base", "wheel_r")),
            frozenset(("base", "link_torso_0")),
        }
        for index in range(self.work.ncon):
            contact = self.work.contact[index]
            if contact.dist >= -.001:
                continue
            geoms = [int(contact.geom1), int(contact.geom2)]
            bodies = [int(self.model.geom_bodyid[geom]) for geom in geoms]
            if not any(body in self.robot_bodies for body in bodies):
                continue
            body_names = [self.model.body(body).name for body in bodies]
            geom_names = [self.model.geom(geom).name for geom in geoms]
            if frozenset(body_names) in structural:
                continue
            if "floor" in geom_names and any(
                    name in {"base", "wheel_l", "wheel_r"} for name in body_names):
                continue
            pad_handle = (
                any(name.startswith(("left_pad_", "right_pad_")) for name in geom_names)
                and any(name.startswith("handle_") for name in geom_names)
            )
            if pad_handle:
                continue
            forbidden.append({
                "distance_m": float(contact.dist),
                "geoms": "/".join(geom_names),
            })
        return forbidden

    def _candidate_metrics(self, q, targets_m, gripper_m):
        self._prepare(q, gripper_m)
        position_error = max(
            np.linalg.norm(self.work.site_xpos[site] - target)
            for site, target in zip(self.tcp, targets_m)
        )
        orientation_error = max(
            np.linalg.norm(Rotation.from_matrix(
                self.work.site_xmat[site].reshape(3, 3)).as_rotvec())
            for site in self.tcp
        )
        torque_ratio = float(np.max(
            np.abs(self.work.qfrc_bias[self.dadr]) / self.force_limit))
        return {
            "position_error_m": float(position_error),
            "orientation_error_rad": float(orientation_error),
            "gravity_torque_ratio": torque_ratio,
            "forbidden_contacts": self._forbidden_contacts(),
        }

    def solve(self, targets_m, seed=None, posture=None, gripper_m=-.045, max_nfev=300):
        if seed is None:
            seed = posture if posture is not None else np.zeros(len(self.names))
        seed = np.clip(np.asarray(seed, dtype=float), self.low, self.high)
        if seed.shape != (20,):
            raise ValueError("IK seed must contain all 20 torso and arm joints")
        if not self.use_torso:
            seed[:6] = self.fixed_torso
        posture = None if posture is None else np.clip(
            np.asarray(posture, dtype=float), self.low, self.high)
        if posture is not None and posture.shape != (20,):
            raise ValueError("IK posture must contain all 20 torso and arm joints")
        if posture is not None and not self.use_torso:
            posture[:6] = self.fixed_torso
        seeds = [seed]
        if posture is not None:
            continuous = posture.copy()
            continuous[6:] = seed[6:]
            seeds.extend((continuous, posture))
        elif self.use_torso:
            for torso in ([0, 1.3, -2, 1.2, 0, 0], [0, .9, -1.5, .6, 0, 0], [0, .4, -.8, .4, 0, 0]):
                candidate = seed.copy()
                candidate[:6] = torso
                seeds.append(np.clip(candidate, self.low, self.high))

        candidates = []
        for initial in seeds:
            reference = initial.copy()
            weights = np.full(len(self.names), .003)
            if posture is not None and self.use_torso:
                reference[:6] = posture[:6]
                weights[:6] = .10
            elif self.use_torso:
                weights[:6] = .01

            def residual(active_q):
                q = initial.copy()
                q[self.active] = active_q
                self.work.qpos[self.qadr] = q
                mujoco.mj_kinematics(self.model, self.work)
                result = []
                for site, target in zip(self.tcp, targets_m):
                    result.extend(8 * (self.work.site_xpos[site] - target))
                    result.extend(.7 * Rotation.from_matrix(
                        self.work.site_xmat[site].reshape(3, 3)).as_rotvec())
                result.extend(weights * (q - reference))
                return result

            solution = least_squares(
                residual, initial[self.active],
                bounds=(self.low[self.active], self.high[self.active]),
                max_nfev=max_nfev,
                ftol=1e-9, xtol=1e-9, gtol=1e-9,
            )
            q = initial.copy()
            q[self.active] = solution.x
            metrics = self._candidate_metrics(q, targets_m, gripper_m)
            feasible = (
                metrics["position_error_m"] <= self.POSITION_TOLERANCE_M
                and metrics["orientation_error_rad"] <= self.ORIENTATION_TOLERANCE_RAD
                and metrics["gravity_torque_ratio"] <= self.TORQUE_LIMIT_RATIO
                and not metrics["forbidden_contacts"]
            )
            score = (
                not feasible,
                len(metrics["forbidden_contacts"]),
                max((-.001 - c["distance_m"] for c in metrics["forbidden_contacts"]), default=0),
                max(0, metrics["gravity_torque_ratio"] - self.TORQUE_LIMIT_RATIO),
                metrics["position_error_m"],
                metrics["orientation_error_rad"],
                float(np.linalg.norm((q - seed)[self.active])),
            )
            candidates.append((score, q, metrics))
            if feasible:
                break

        score, q, metrics = min(candidates, key=lambda candidate: candidate[0])
        if score[0]:
            collision = metrics["forbidden_contacts"][0]["geoms"] if metrics["forbidden_contacts"] else "none"
            raise ValueError(
                "No feasible IK candidate: "
                f"position={metrics['position_error_m']:.4f} m, "
                f"orientation={metrics['orientation_error_rad']:.4f} rad, "
                f"gravity_ratio={metrics['gravity_torque_ratio']:.3f}, "
                f"collision={collision}"
            )
        metrics = {key: value for key, value in metrics.items() if key != "forbidden_contacts"}
        metrics["forbidden_contact_count"] = 0
        return q, metrics


def build_plan(model, config, lift_only=False):
    robot = Robot(
        model,
        use_torso=config.get("use_torso", True),
        fixed_torso_rad=config.get("fixed_torso_rad"),
    )
    x, y, z = config["tray_initial_pos_m"]
    hz, hy = config["handle_z_m"], config["handle_y_m"]
    target = config["target"]
    tx, ty = target["center_xy_m"]
    tz = target["support_z_m"] + config["tray_half_extents_m"][2]
    clearance_m = config.get("transport_clearance_m", .10)
    withdrawn = np.array([tx, ty, tz]) + config.get("withdraw_offset_m", [0, 0, .10])
    waypoints, ik_results = [], []
    q = None
    pickup_posture = robot.crouched_pickup_posture()
    sequence = [
        ("approach", [x, y, z + .10], -.045, 2.0),
        ("descend", [x, y, z], -.045, 2.0),
        ("grasp", [x, y, z], 0.0, 2.5),
        ("grasp", [x, y, z], 0.0, 1.0),
        ("lift", [x, y, z + .07], 0.0, 3.0),
    ]
    if not lift_only:
        sequence += [
            ("raise", [x, y, max(tz + clearance_m, .5)], 0.0, 12.0),
            ("transport", [tx, ty, tz + clearance_m], 0.0, 12.0),
            ("seat", [tx, ty, tz], 0.0, 5.0),
            ("settle", [tx, ty, tz], 0.0, 1.5),
            ("release", [tx, ty, tz], -.045, 2.0),
            ("withdraw", withdrawn, -.045, 4.0),
            ("hold", withdrawn, -.045, 1.0),
        ]
    previous_center = np.array(sequence[0][1], dtype=float)
    previous_grip = sequence[0][2]
    for phase, center, grip, duration_s in sequence:
        if phase in {"descend", "lift", "raise", "transport", "seat", "withdraw"}:
            duration_s *= config.get("motion_duration_scale", 1.0)
        center = np.array(center, dtype=float)
        samples = max(1, int(np.ceil(np.linalg.norm(center - previous_center) / .035)))
        for k in range(1, samples + 1):
            fraction = k / samples
            point = previous_center + fraction * (center - previous_center)
            grip_reference = previous_grip + fraction * (grip - previous_grip)
            targets = [point + [0, sign * hy, hz] for sign in (-1, 1)]
            posture = pickup_posture if phase in {"approach", "descend", "grasp", "lift"} else None
            try:
                q, errors = robot.solve(targets, q, posture=posture, gripper_m=grip_reference)
            except ValueError as exc:
                raise ValueError(f"{phase} at {point.tolist()}: {exc}") from exc
            ik_results.append({"phase": phase, **errors})
            waypoints.append(Waypoint(phase, duration_s / samples, q.copy(), grip_reference))
        previous_center, previous_grip = center, grip
    return robot, waypoints, ik_results


def smoothstep(t):
    t = np.clip(t, 0, 1)
    return t ** 3 * (10 - 15 * t + 6 * t ** 2)
