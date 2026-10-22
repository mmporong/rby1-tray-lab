"""Stage-independent evaluation for the loaded-tray task.

The evaluator intentionally ignores controller phase names when deciding success.
It derives progress only from contacts, geometry, velocities, and audit signals.
"""

from __future__ import annotations

from copy import deepcopy
from itertools import product
from math import cos, isfinite
from typing import Any

import numpy as np


class Evaluator:
    """Evaluate one episode from a stream of simulator snapshots."""

    _AUDIT_KEYS = ("free_objects", "object_attachment", "runtime_state_overwrite")

    def __init__(self, config: dict[str, Any]):
        self._config = deepcopy(config)
        self._target = config["target"]
        self._half = self._vector(config["tray_half_extents_m"], 3, "tray_half_extents_m")
        self._containment_half = self._vector(
            config.get("tray_containment_half_extents_m", self._half),
            3,
            "tray_containment_half_extents_m",
        )
        self._containment_center = self._vector(
            config.get("tray_containment_center_m", [0.0, 0.0, 0.0]),
            3,
            "tray_containment_center_m",
        )
        self._initial_bottom = float(config["initial_tray_bottom_z_m"])
        self._thresholds = self._config["thresholds"]
        self._expected_payload_names = tuple(config["expected_payload_names"])
        self._payload_max_height = config.get("payload_max_height_m")
        self._return_home = deepcopy(config.get("return_home"))
        self._thresholds.setdefault("min_pad_force_n", 0.1)
        self._thresholds.setdefault("payload_speed_m_s", 0.05)
        self._thresholds.setdefault("max_seat_tilt_rad", 0.05)
        self._validate_config()

        self._last_time: float | None = None
        self._full_contact_since: float | None = None
        self._settled_since: float | None = None
        self._withdraw_since: float | None = None
        self._return_home_since: float | None = None
        self._failure_reason: str | None = None
        self._failure_time: float | None = None
        self._milestones = {
            "grasped": False,
            "lifted": False,
            "seated": False,
            "released": False,
            "withdrawn": False,
        }
        if self._return_home is not None:
            self._milestones["returned_home"] = False
        self._milestone_times: dict[str, float] = {}
        self._last_checks: dict[str, bool] = {}
        self._last_metrics: dict[str, float] = {}
        self._updates = 0

    def update(self, snapshot: dict[str, Any]) -> dict[str, Any]:
        """Consume a snapshot and return the current episode result.

        A terminal failure is latched. Later snapshots cannot erase it or turn the
        episode into a success.
        """
        if self._failure_reason is not None or self._episode_complete():
            return self.result()

        try:
            now = float(snapshot["time_s"])
            if not isfinite(now):
                return self._fail("nonfinite_time", None)
            if self._last_time is not None and now <= self._last_time:
                return self._fail("nonmonotonic_time", now)

            audit = snapshot["audit"]
            if not bool(audit["free_objects"]):
                return self._fail("objects_not_free", now)
            if bool(audit["object_attachment"]):
                return self._fail("object_attachment", now)
            if bool(audit["runtime_state_overwrite"]):
                return self._fail("runtime_state_overwrite", now)
            if bool(snapshot["joint_limit_violation"]):
                return self._fail("joint_limit_violation", now)

            tray = snapshot["tray"]
            tray_pos = self._vector(tray["pos_m"], 3, "tray.pos_m")
            rotation = self._rotation(tray["quat_wxyz"])
            linear_speed = float(np.linalg.norm(self._vector(
                tray["linear_velocity_m_s"], 3, "tray.linear_velocity_m_s"
            )))
            angular_speed = float(np.linalg.norm(self._vector(
                tray["angular_velocity_rad_s"], 3, "tray.angular_velocity_rad_s"
            )))
            if not isfinite(linear_speed) or not isfinite(angular_speed):
                return self._fail("nonfinite_state", now)

            contacts = snapshot["contacts"]
            pads = tuple(bool(value) for side in ("left_fingers", "right_fingers")
                         for value in contacts[side])
            if len(pads) != 4:
                raise ValueError("exactly two finger contacts per hand are required")
            full_contact = all(pads)
            robot_tray_contact = bool(contacts["robot_tray_contact"])
            fully_released = not any(pads) and not robot_tray_contact
            opposed = bool(contacts["left_opposed"]) and bool(contacts["right_opposed"])
            pad_forces = tuple(
                float(value)
                for side in ("left", "right")
                for value in snapshot["pad_forces_n"][side]
            )
            if len(pad_forces) != 4 or not all(isfinite(value) for value in pad_forces):
                raise ValueError("exactly four finite pad forces are required")
            grasp_forces = tuple(float(snapshot["grasp_force_n"][side]) for side in ("left", "right"))
            if not all(isfinite(value) for value in grasp_forces):
                raise ValueError("grasp forces must be finite")
            force_sufficient = all(
                value >= float(self._thresholds["min_pad_force_n"]) for value in pad_forces
            )
            secure_grasp = full_contact and opposed and force_sufficient
            forbidden_contact = bool(contacts["forbidden"])

            floor_corners = self._box_corners(tray_pos, rotation, self._half)
            containment_corners = self._box_corners(
                tray_pos,
                rotation,
                self._containment_half,
                self._containment_center,
            )
            tray_bottom = float(np.min(floor_corners[:, 2]))
            lifted = tray_bottom >= (
                self._initial_bottom + float(self._thresholds["lift_clearance_m"])
            )
            payloads = snapshot["payloads"]
            if not self._payload_identity_matches(payloads):
                return self._fail("payload_identity_mismatch", now)
            payloads_contained = self._payloads_contained(payloads, tray_pos, rotation)
            payload_speeds = tuple(float(payload["linear_speed_m_s"]) for payload in payloads)
            if not all(isfinite(value) and value >= 0.0 for value in payload_speeds):
                raise ValueError("payload linear speeds must be finite and nonnegative")
            payloads_supported = all(bool(payload["supported_by_tray"]) for payload in payloads)
            payloads_settled = payloads_supported and all(
                value <= float(self._thresholds["payload_speed_m_s"])
                for value in payload_speeds
            )
            in_target_xy = self._in_target_xy(containment_corners)
            on_support = (
                bool(contacts["container_bottom"])
                and abs(tray_bottom - float(self._target["support_z_m"]))
                <= float(self._thresholds["containment_tolerance_m"])
            )
            slow = (
                linear_speed <= float(self._thresholds["linear_speed_m_s"])
                and angular_speed <= float(self._thresholds["angular_speed_rad_s"])
            )
            upright = bool(
                rotation[2, 2] >= cos(float(self._thresholds["max_seat_tilt_rad"]))
            )
            self._last_checks = {
                "full_pad_contact": full_contact,
                "opposed_contacts": opposed,
                "pad_force_sufficient": force_sufficient,
                "secure_grasp": secure_grasp,
                "fully_released": fully_released,
                "robot_tray_contact": robot_tray_contact,
                "payloads_contained": payloads_contained,
                "payloads_supported": payloads_supported,
                "payloads_settled": payloads_settled,
                "tray_lifted": lifted,
                "tray_in_target_xy": in_target_xy,
                "tray_on_support": on_support,
                "tray_slow": slow,
                "tray_upright": upright,
                "forbidden_contact": forbidden_contact,
            }
            self._last_metrics = {
                "min_pad_force_n": min(pad_forces),
                "left_grasp_force_n": grasp_forces[0],
                "right_grasp_force_n": grasp_forces[1],
                "max_payload_speed_m_s": max(payload_speeds),
            }

            home_candidate = False
            if self._return_home is not None:
                home_checks, home_metrics = self._evaluate_return_home(snapshot["robot_state"])
                self._last_checks.update(home_checks)
                self._last_metrics.update(home_metrics)
                home_candidate = home_checks["return_home_candidate"]

            if self._milestones["withdrawn"]:
                clearances = snapshot["hand_clearance_m"]
                threshold = float(self._thresholds["withdraw_clearance_m"])
                task_preserved = (
                    fully_released
                    and in_target_xy
                    and on_support
                    and slow
                    and upright
                    and payloads_contained
                    and payloads_supported
                    and payloads_settled
                    and float(clearances["left"]) >= threshold
                    and float(clearances["right"]) >= threshold
                )
                self._last_checks["task_preserved_after_withdrawal"] = task_preserved
                if not task_preserved:
                    return self._fail("task_invalidated_after_withdrawal", now)

            if forbidden_contact:
                return self._fail("forbidden_contact", now)
            if not payloads_contained:
                return self._fail("payload_lost", now)

            if secure_grasp:
                if self._full_contact_since is None:
                    self._full_contact_since = now
                if now - self._full_contact_since >= float(self._thresholds["grasp_hold_s"]):
                    self._mark("grasped", now)
            elif not self._milestones["grasped"]:
                self._full_contact_since = None

            if lifted:
                if not self._milestones["grasped"]:
                    return self._fail("lift_without_secure_grasp", now)
                self._mark("lifted", now)

            seating_candidate = (
                self._milestones["grasped"]
                and self._milestones["lifted"]
                and in_target_xy
                and on_support
                and slow
                and upright
                and payloads_contained
                and payloads_settled
            )
            if seating_candidate:
                if self._settled_since is None:
                    self._settled_since = now
                if now - self._settled_since >= float(self._thresholds["settle_hold_s"]):
                    self._mark("seated", now)
            else:
                self._settled_since = None

            if self._milestones["grasped"] and not secure_grasp and not self._milestones["seated"]:
                return self._fail("release_before_seat", now)

            if self._milestones["seated"] and fully_released:
                self._mark("released", now)

            if self._milestones["released"]:
                clearances = snapshot["hand_clearance_m"]
                threshold = float(self._thresholds["withdraw_clearance_m"])
                withdrawal_candidate = (
                    fully_released
                    and in_target_xy
                    and on_support
                    and slow
                    and upright
                    and payloads_contained
                    and payloads_settled
                    and float(clearances["left"]) >= threshold
                    and float(clearances["right"]) >= threshold
                )
                if withdrawal_candidate:
                    if self._withdraw_since is None:
                        self._withdraw_since = now
                    if now - self._withdraw_since >= float(self._thresholds["settle_hold_s"]):
                        self._mark("withdrawn", now)
                else:
                    self._withdraw_since = None

            if self._return_home is not None and self._milestones["withdrawn"]:
                if home_candidate:
                    if self._return_home_since is None:
                        self._return_home_since = now
                    if now - self._return_home_since >= float(
                        self._return_home["settle_hold_s"]
                    ):
                        self._mark("returned_home", now)
                else:
                    self._return_home_since = None

            self._last_time = now
            self._updates += 1
            return self.result()
        except (KeyError, TypeError, ValueError, IndexError) as exc:
            return self._fail("invalid_snapshot", self._safe_time(snapshot), str(exc))

    def finalize(self, time_s: float, reason: str = "episode_timeout") -> dict[str, Any]:
        """Close an unfinished episode with an explicit terminal failure."""
        if self._failure_reason is not None or self._episode_complete():
            return self.result()
        try:
            now = float(time_s)
        except (TypeError, ValueError):
            return self._fail("nonfinite_time", None)
        if not isfinite(now):
            return self._fail("nonfinite_time", None)
        if self._last_time is not None and now < self._last_time:
            return self._fail("nonmonotonic_time", now)
        if not isinstance(reason, str) or not reason:
            return self._fail("invalid_finalize_reason", now)
        return self._fail(reason, now)

    def result(self) -> dict[str, Any]:
        """Return a detached summary suitable for JSON serialization."""
        success = self._episode_complete() and self._failure_reason is None
        return {
            "success": success,
            "terminal": success or self._failure_reason is not None,
            "failure_reason": self._failure_reason,
            "failure_time_s": self._failure_time,
            "failure_event": (
                None
                if self._failure_reason is None
                else {"reason": self._failure_reason, "time_s": self._failure_time}
            ),
            "milestones": dict(self._milestones),
            "milestone_times_s": dict(self._milestone_times),
            "checks": dict(self._last_checks),
            "metrics": dict(self._last_metrics),
            "updates": self._updates,
            "last_time_s": self._last_time,
        }

    def _validate_config(self) -> None:
        self._vector(self._target["center_xy_m"], 2, "target.center_xy_m")
        inner = self._vector(self._target["inner_half_xy_m"], 2, "target.inner_half_xy_m")
        if (
            np.any(self._half <= 0.0)
            or np.any(self._containment_half <= 0.0)
            or np.any(inner <= 0.0)
        ):
            raise ValueError("tray and target half extents must be positive")
        if not isfinite(self._initial_bottom):
            raise ValueError("initial_tray_bottom_z_m must be finite")
        if not self._expected_payload_names or len(set(self._expected_payload_names)) != len(
            self._expected_payload_names
        ):
            raise ValueError("expected_payload_names must contain unique names")
        if not all(isinstance(name, str) and name for name in self._expected_payload_names):
            raise ValueError("expected_payload_names must contain nonempty strings")
        if self._payload_max_height is not None:
            self._payload_max_height = float(self._payload_max_height)
            if not isfinite(self._payload_max_height) or self._payload_max_height <= 0.0:
                raise ValueError("payload_max_height_m must be finite and positive")
        for name in (
            "grasp_hold_s", "settle_hold_s", "withdraw_clearance_m",
            "linear_speed_m_s", "angular_speed_rad_s", "lift_clearance_m",
            "payload_margin_m", "containment_tolerance_m",
            "min_pad_force_n", "payload_speed_m_s",
            "max_seat_tilt_rad",
        ):
            value = float(self._thresholds[name])
            if not isfinite(value) or value < 0.0:
                raise ValueError(f"threshold {name} must be finite and nonnegative")
        if float(self._thresholds["max_seat_tilt_rad"]) > np.pi:
            raise ValueError("threshold max_seat_tilt_rad must not exceed pi")
        if self._return_home is not None:
            self._validate_return_home_config()

    def _validate_return_home_config(self) -> None:
        if not isinstance(self._return_home, dict):
            raise ValueError("return_home must be an object")
        joint_names = self._return_home.get("joint_names")
        if (
            not isinstance(joint_names, list)
            or len(joint_names) != 20
            or len(set(joint_names)) != 20
            or not all(isinstance(name, str) and name for name in joint_names)
        ):
            raise ValueError("return_home.joint_names must contain 20 unique nonempty strings")
        self._return_home["joint_position_rad"] = self._vector(
            self._return_home.get("joint_position_rad"),
            20,
            "return_home.joint_position_rad",
        )
        self._return_home["head_position_rad"] = self._vector(
            self._return_home.get("head_position_rad"),
            2,
            "return_home.head_position_rad",
        )
        self._return_home["base_pose_xy_yaw"] = self._vector(
            self._return_home.get("base_pose_xy_yaw"),
            3,
            "return_home.base_pose_xy_yaw",
        )
        try:
            gripper = float(self._return_home.get("gripper_position_m"))
        except (TypeError, ValueError) as exc:
            raise ValueError("return_home.gripper_position_m must be finite") from exc
        if not isfinite(gripper):
            raise ValueError("return_home.gripper_position_m must be finite")
        self._return_home["gripper_position_m"] = gripper
        defaults = {
            "joint_tolerance_rad": 0.01,
            "joint_speed_tolerance_rad_s": 0.02,
            "head_tolerance_rad": 0.01,
            "head_speed_tolerance_rad_s": 0.02,
            "gripper_tolerance_m": 0.002,
            "base_position_tolerance_m": 0.02,
            "base_yaw_tolerance_rad": 0.02,
            "base_speed_tolerance_m_s": 0.012,
            "base_yaw_speed_tolerance_rad_s": 0.025,
            "settle_hold_s": 0.5,
        }
        for name, default in defaults.items():
            try:
                value = float(self._return_home.get(name, default))
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    f"return_home.{name} must be finite and nonnegative"
                ) from exc
            if not isfinite(value) or value < 0.0:
                raise ValueError(f"return_home.{name} must be finite and nonnegative")
            self._return_home[name] = value

    def _evaluate_return_home(
        self, robot_state: Any
    ) -> tuple[dict[str, bool], dict[str, float]]:
        if not isinstance(robot_state, dict):
            raise ValueError("robot_state must be an object")
        names = robot_state.get("joint_names")
        if names != self._return_home["joint_names"]:
            raise ValueError("robot_state.joint_names must match return_home.joint_names")
        joints = self._vector(robot_state.get("position_rad"), 20, "robot_state.position_rad")
        joint_velocity = self._vector(
            robot_state.get("velocity_rad_s"), 20, "robot_state.velocity_rad_s"
        )
        head = self._vector(
            robot_state.get("head_position_rad"), 2, "robot_state.head_position_rad"
        )
        head_velocity = self._vector(
            robot_state.get("head_velocity_rad_s"), 2, "robot_state.head_velocity_rad_s"
        )
        gripper = self._vector(
            robot_state.get("gripper_position_m"), 4, "robot_state.gripper_position_m"
        )
        base = self._vector(
            robot_state.get("base_pose_xy_yaw"), 3, "robot_state.base_pose_xy_yaw"
        )
        base_linear_speed = float(robot_state.get("base_linear_speed_m_s"))
        base_yaw_speed = abs(float(robot_state.get("base_yaw_rate_rad_s")))
        if (
            not isfinite(base_linear_speed)
            or base_linear_speed < 0.0
            or not isfinite(base_yaw_speed)
        ):
            raise ValueError("robot_state base speeds must be finite and nonnegative")

        joint_error = float(np.max(np.abs(joints - self._return_home["joint_position_rad"])))
        joint_speed = float(np.max(np.abs(joint_velocity)))
        head_error = float(np.max(np.abs(head - self._return_home["head_position_rad"])))
        head_speed = float(np.max(np.abs(head_velocity)))
        gripper_error = float(
            np.max(np.abs(gripper - self._return_home["gripper_position_m"]))
        )
        base_position_error = float(
            np.linalg.norm(base[:2] - self._return_home["base_pose_xy_yaw"][:2])
        )
        yaw_delta = base[2] - self._return_home["base_pose_xy_yaw"][2]
        base_yaw_error = abs(float(np.arctan2(np.sin(yaw_delta), np.cos(yaw_delta))))
        checks = {
            "return_home_joint_position": (
                joint_error <= self._return_home["joint_tolerance_rad"]
            ),
            "return_home_joint_stopped": (
                joint_speed <= self._return_home["joint_speed_tolerance_rad_s"]
            ),
            "return_home_head_position": (
                head_error <= self._return_home["head_tolerance_rad"]
            ),
            "return_home_head_stopped": (
                head_speed <= self._return_home["head_speed_tolerance_rad_s"]
            ),
            "return_home_gripper_position": (
                gripper_error <= self._return_home["gripper_tolerance_m"]
            ),
            "return_home_base_position": (
                base_position_error <= self._return_home["base_position_tolerance_m"]
            ),
            "return_home_base_yaw": (
                base_yaw_error <= self._return_home["base_yaw_tolerance_rad"]
            ),
            "return_home_base_stopped": (
                base_linear_speed <= self._return_home["base_speed_tolerance_m_s"]
                and base_yaw_speed <= self._return_home["base_yaw_speed_tolerance_rad_s"]
            ),
        }
        checks["return_home_candidate"] = all(checks.values())
        metrics = {
            "return_home_max_joint_error_rad": joint_error,
            "return_home_max_joint_speed_rad_s": joint_speed,
            "return_home_max_head_error_rad": head_error,
            "return_home_max_head_speed_rad_s": head_speed,
            "return_home_max_gripper_error_m": gripper_error,
            "return_home_base_position_error_m": base_position_error,
            "return_home_base_yaw_error_rad": base_yaw_error,
            "return_home_base_linear_speed_m_s": base_linear_speed,
            "return_home_base_yaw_speed_rad_s": base_yaw_speed,
        }
        return checks, metrics

    def _episode_complete(self) -> bool:
        terminal_milestone = (
            "returned_home" if self._return_home is not None else "withdrawn"
        )
        return self._milestones[terminal_milestone]

    @staticmethod
    def _vector(value: Any, size: int, name: str) -> np.ndarray:
        vector = np.asarray(value, dtype=float)
        if vector.shape != (size,) or not np.all(np.isfinite(vector)):
            raise ValueError(f"{name} must contain {size} finite values")
        return vector

    @classmethod
    def _rotation(cls, quat_wxyz: Any) -> np.ndarray:
        quat = cls._vector(quat_wxyz, 4, "tray.quat_wxyz")
        norm = float(np.linalg.norm(quat))
        if norm < 1e-12:
            raise ValueError("tray quaternion has zero norm")
        w, x, y, z = quat / norm
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])

    @staticmethod
    def _box_corners(
        position: np.ndarray,
        rotation: np.ndarray,
        half_extents: np.ndarray,
        center_local: np.ndarray | None = None,
    ) -> np.ndarray:
        center = np.zeros(3) if center_local is None else center_local
        local = np.array(list(product(*[(-value, value) for value in half_extents]))) + center
        return local @ rotation.T + position

    def _payloads_contained(
        self, payloads: list[dict[str, Any]], tray_pos: np.ndarray, rotation: np.ndarray
    ) -> bool:
        margin = float(self._thresholds["payload_margin_m"])
        tolerance = float(self._thresholds["containment_tolerance_m"])
        for payload in payloads:
            world_pos = self._vector(payload["pos_m"], 3, f"payload {payload.get('name', '')}.pos_m")
            radius = float(payload["radius_m"])
            if not isfinite(radius) or radius <= 0.0:
                raise ValueError("payload radius_m must be finite and positive")
            local = rotation.T @ (world_pos - tray_pos)
            if np.any(np.abs(local[:2]) + radius + margin > self._half[:2] + tolerance):
                return False
            if local[2] < self._half[2] - tolerance:
                return False
            if (
                self._payload_max_height is not None
                and local[2] + radius
                > self._half[2] + self._payload_max_height + tolerance
            ):
                return False
        return True

    def _payload_identity_matches(self, payloads: Any) -> bool:
        if not isinstance(payloads, list):
            return False
        names = [payload.get("name") for payload in payloads if isinstance(payload, dict)]
        return (
            len(names) == len(payloads)
            and len(names) == len(set(names))
            and set(names) == set(self._expected_payload_names)
        )

    def _in_target_xy(self, corners: np.ndarray) -> bool:
        center = self._vector(self._target["center_xy_m"], 2, "target.center_xy_m")
        half = self._vector(self._target["inner_half_xy_m"], 2, "target.inner_half_xy_m")
        tolerance = float(self._thresholds["containment_tolerance_m"])
        return bool(np.all(np.abs(corners[:, :2] - center) <= half + tolerance))

    def _mark(self, milestone: str, now: float) -> None:
        if not self._milestones[milestone]:
            self._milestones[milestone] = True
            self._milestone_times[milestone] = now

    def _fail(self, reason: str, now: float | None, detail: str | None = None) -> dict[str, Any]:
        if self._failure_reason is None:
            self._failure_reason = reason if detail is None else f"{reason}: {detail}"
            self._failure_time = now
            self._updates += 1
            if now is not None and isfinite(now) and (
                self._last_time is None or now > self._last_time
            ):
                self._last_time = now
        return self.result()

    @staticmethod
    def _safe_time(snapshot: dict[str, Any]) -> float | None:
        try:
            value = float(snapshot.get("time_s"))
            return value if isfinite(value) else None
        except (TypeError, ValueError):
            return None
