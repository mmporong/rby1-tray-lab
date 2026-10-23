"""Physical wheel-drive control and diagnostics for the free-base RB-Y1 M model.

The upstream mesh models each mecanum wheel as one rigid collision mesh, without
individually simulated rollers.  This controller therefore validates only
longitudinal driving and in-place yaw.  It never writes qpos/qvel, applies an
external force, or adds a virtual base actuator.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Iterable

import mujoco
import numpy as np


WHEEL_ACTUATOR_NAMES = (
    "front_right_wheel_act",
    "front_left_wheel_act",
    "rear_right_wheel_act",
    "rear_left_wheel_act",
)
WHEEL_JOINT_NAMES = ("wheel_fr", "wheel_fl", "wheel_rr", "wheel_rl")


@dataclass(frozen=True)
class DriveConfig:
    """Simulation-only drive parameters, expressed in SI units."""

    wheel_radius_m: float = 0.076
    half_track_m: float = 0.245
    max_linear_speed_m_s: float = 0.12
    max_yaw_rate_rad_s: float = 0.35
    max_wheel_speed_rad_s: float = 3.14
    max_wheel_accel_rad_s2: float = 2.0
    position_kp_s_inv: float = 0.8
    yaw_kp_s_inv: float = 1.2
    position_tolerance_m: float = 0.012
    yaw_tolerance_rad: float = 0.02
    stop_speed_m_s: float = 0.012
    stop_yaw_rate_rad_s: float = 0.025
    lateral_target_tolerance_m: float = 0.05
    tilt_limit_rad: float = 0.10
    wheel_velocity_gain_nms_per_rad: float = 100.0
    encoder_wrap_period_rad: float = 2 * math.pi
    initial_pose_xy_yaw: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class DriveSensors:
    """Replaceable proprioceptive input boundary for simulation or hardware."""

    wheel_angle_rad: tuple[float, float, float, float]
    imu_yaw_rad: float | None = None


def _wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _roll_pitch_yaw(quat_wxyz: np.ndarray) -> tuple[float, float, float]:
    w, x, y, z = (float(value) for value in quat_wxyz)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(float(np.clip(2 * (w * y - z * x), -1.0, 1.0)))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


class DriveController:
    """Closed-loop longitudinal/yaw controller using the four wheel actuators."""

    def __init__(self, model: mujoco.MjModel, config: DriveConfig | None = None):
        self.config = config or DriveConfig()
        self._validate_config()
        self.base_body_id = model.body("base").id
        self.base_joint_id = model.joint("world_j").id
        if model.jnt_type[self.base_joint_id] != mujoco.mjtJoint.mjJNT_FREE:
            raise ValueError("RB-Y1 M mobility requires the upstream free base joint")

        self.actuator_ids = np.array(
            [model.actuator(name).id for name in WHEEL_ACTUATOR_NAMES], dtype=int
        )
        self.joint_ids = np.array(
            [model.joint(name).id for name in WHEEL_JOINT_NAMES], dtype=int
        )
        self.dof_ids = model.jnt_dofadr[self.joint_ids].copy()
        expected_joint_ids = model.actuator_trnid[self.actuator_ids, 0]
        if not np.array_equal(expected_joint_ids, self.joint_ids):
            raise ValueError("Wheel actuator-to-joint mapping differs from RB-Y1 M 1.2")
        if np.any(model.actuator_ctrlrange[self.actuator_ids, 0] > -3.14) or np.any(
            model.actuator_ctrlrange[self.actuator_ids, 1] < 3.14
        ):
            raise ValueError("Wheel actuator control range must include +/-3.14 rad/s")

        # The upstream <velocity> elements inherit kv=4000 from the default
        # position class.  With rigid roller meshes this generates a large
        # launch/braking impulse.  The exposed simulation gain below produces
        # convergent wheel-speed tracking; it is not a hardware calibration.
        model.actuator_gainprm[self.actuator_ids, 0] = (
            self.config.wheel_velocity_gain_nms_per_rad
        )
        model.actuator_biasprm[self.actuator_ids, 2] = (
            -self.config.wheel_velocity_gain_nms_per_rad
        )

        self._command_rad_s = np.zeros(4)
        self._wheel_odometry = np.asarray(self.config.initial_pose_xy_yaw, dtype=float)
        self._last_wheel_angle_rad: np.ndarray | None = None
        self._last_imu_yaw_rad: float | None = None
        self._estimated_linear_speed_m_s = 0.0
        self._estimated_yaw_rate_rad_s = 0.0
        self._last_target: np.ndarray | None = None
        self._last_metrics: dict = {}
        self.log: list[dict] = []

    def _validate_config(self) -> None:
        positive = (
            self.config.wheel_radius_m,
            self.config.half_track_m,
            self.config.max_linear_speed_m_s,
            self.config.max_yaw_rate_rad_s,
            self.config.max_wheel_speed_rad_s,
            self.config.max_wheel_accel_rad_s2,
            self.config.position_kp_s_inv,
            self.config.yaw_kp_s_inv,
            self.config.position_tolerance_m,
            self.config.yaw_tolerance_rad,
            self.config.stop_speed_m_s,
            self.config.stop_yaw_rate_rad_s,
            self.config.lateral_target_tolerance_m,
            self.config.tilt_limit_rad,
            self.config.wheel_velocity_gain_nms_per_rad,
            self.config.encoder_wrap_period_rad,
        )
        if not all(np.isfinite(positive)) or not all(value > 0 for value in positive):
            raise ValueError("DriveConfig values must be finite and positive")
        if len(self.config.initial_pose_xy_yaw) != 3 or not all(
            np.isfinite(self.config.initial_pose_xy_yaw)
        ):
            raise ValueError("initial_pose_xy_yaw must contain three finite values")

    def read_sim_sensors(
        self, model: mujoco.MjModel, data: mujoco.MjData
    ) -> DriveSensors:
        """Read only signals that have real encoder equivalents.

        Ground-truth body pose is deliberately excluded.  An integration can
        supply an IMU yaw separately through :class:`DriveSensors`.
        """
        wheel_angles = tuple(
            float(data.qpos[int(model.jnt_qposadr[joint_id])])
            for joint_id in self.joint_ids
        )
        return DriveSensors(wheel_angle_rad=wheel_angles)

    def reset(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        sensors: DriveSensors | None = None,
    ) -> None:
        """Reset estimator memory from configured pose and encoder readings."""
        sensors = sensors or self.read_sim_sensors(model, data)
        wheel_angles, imu_yaw = self._validate_sensors(sensors)
        self._wheel_odometry[:] = self.config.initial_pose_xy_yaw
        if imu_yaw is not None:
            self._wheel_odometry[2] = _wrap_angle(imu_yaw)
        self._last_wheel_angle_rad = wheel_angles
        self._last_imu_yaw_rad = imu_yaw
        self._estimated_linear_speed_m_s = 0.0
        self._estimated_yaw_rate_rad_s = 0.0
        self._command_rad_s.fill(0)
        self._last_target = None
        self._last_metrics = self.metrics(model, data)
        self.log.clear()

    def update(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        target_xy_yaw: Iterable[float],
        dt: float,
        sensors: DriveSensors | None = None,
        *, speed_scale: float = 1.0,
    ) -> dict[str, float]:
        """Advance control memory and write only the four wheel controls.

        ``target_xy_yaw`` is a world-frame ``(x_m, y_m, yaw_rad)`` target.
        A target requiring mecanum lateral translation is rejected because the
        upstream rigid wheel meshes cannot validate roller-level side motion.
        """
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError("dt must be finite and positive")
        if not np.isfinite(speed_scale) or not 0 <= speed_scale <= 1:
            raise ValueError("speed_scale must be finite and in [0, 1]")
        target = np.asarray(tuple(target_xy_yaw), dtype=float)
        if target.shape != (3,) or not np.all(np.isfinite(target)):
            raise ValueError("target_xy_yaw must contain three finite values")

        sensors = sensors or self.read_sim_sensors(model, data)
        self._update_odometry(sensors, dt)
        position = self._wheel_odometry[:2]
        yaw = float(self._wheel_odometry[2])
        delta = target[:2] - position
        longitudinal_error = math.cos(yaw) * delta[0] + math.sin(yaw) * delta[1]
        lateral_error = -math.sin(yaw) * delta[0] + math.cos(yaw) * delta[1]
        if abs(lateral_error) > self.config.lateral_target_tolerance_m:
            raise ValueError(
                "Target requires unvalidated lateral mecanum motion: "
                f"{lateral_error:.4f} m"
            )
        yaw_error = _wrap_angle(float(target[2]) - yaw)

        desired_linear = float(
            np.clip(
                self.config.position_kp_s_inv * longitudinal_error,
                -self.config.max_linear_speed_m_s,
                self.config.max_linear_speed_m_s,
            )
        )
        desired_yaw_rate = float(
            np.clip(
                self.config.yaw_kp_s_inv * yaw_error,
                -self.config.max_yaw_rate_rad_s,
                self.config.max_yaw_rate_rad_s,
            )
        )
        if (
            abs(longitudinal_error) <= self.config.position_tolerance_m
            and abs(self._estimated_linear_speed_m_s) <= self.config.stop_speed_m_s
        ):
            desired_linear = 0.0
        if (
            abs(yaw_error) <= self.config.yaw_tolerance_rad
            and abs(self._estimated_yaw_rate_rad_s) <= self.config.stop_yaw_rate_rad_s
        ):
            desired_yaw_rate = 0.0

        desired_linear *= speed_scale
        desired_yaw_rate *= speed_scale
        right = (
            desired_linear + self.config.half_track_m * desired_yaw_rate
        ) / self.config.wheel_radius_m
        left = (
            desired_linear - self.config.half_track_m * desired_yaw_rate
        ) / self.config.wheel_radius_m
        desired_wheels = np.array([right, left, right, left])
        desired_wheels = np.clip(
            desired_wheels,
            -self.config.max_wheel_speed_rad_s,
            self.config.max_wheel_speed_rad_s,
        )
        max_delta = self.config.max_wheel_accel_rad_s2 * dt
        self._command_rad_s += np.clip(
            desired_wheels - self._command_rad_s, -max_delta, max_delta
        )
        data.ctrl[self.actuator_ids] = self._command_rad_s

        self._last_target = target.copy()
        self._last_metrics = self.metrics(model, data)
        self._last_metrics.update(
            {
                "target_xy_yaw": target.tolist(),
                "longitudinal_error_m": longitudinal_error,
                "lateral_error_m": lateral_error,
                "yaw_error_rad": yaw_error,
                "desired_linear_speed_m_s": desired_linear,
                "desired_yaw_rate_rad_s": desired_yaw_rate,
            }
        )
        self.log.append(self._last_metrics.copy())
        return self.commands(model)

    def stop(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        dt: float,
        sensors: DriveSensors | None = None,
    ) -> dict[str, float]:
        """Ramp the wheel speed command to zero without changing physical state."""
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError("dt must be finite and positive")
        max_delta = self.config.max_wheel_accel_rad_s2 * dt
        self._command_rad_s += np.clip(-self._command_rad_s, -max_delta, max_delta)
        data.ctrl[self.actuator_ids] = self._command_rad_s
        self._update_odometry(sensors or self.read_sim_sensors(model, data), dt)
        self._last_metrics = self.metrics(model, data)
        self.log.append(self._last_metrics.copy())
        return self.commands(model)

    def _validate_sensors(
        self, sensors: DriveSensors
    ) -> tuple[np.ndarray, float | None]:
        wheel_angles = np.asarray(sensors.wheel_angle_rad, dtype=float)
        if wheel_angles.shape != (4,) or not np.all(np.isfinite(wheel_angles)):
            raise ValueError("wheel_angle_rad must contain four finite encoder values")
        imu_yaw = sensors.imu_yaw_rad
        if imu_yaw is not None and not np.isfinite(imu_yaw):
            raise ValueError("imu_yaw_rad must be finite when provided")
        return wheel_angles, imu_yaw

    def _update_odometry(self, sensors: DriveSensors, dt: float | None = None) -> None:
        wheel_angles, imu_yaw = self._validate_sensors(sensors)
        if self._last_wheel_angle_rad is None:
            self._last_wheel_angle_rad = wheel_angles
            self._last_imu_yaw_rad = imu_yaw
            if imu_yaw is not None:
                self._wheel_odometry[2] = _wrap_angle(imu_yaw)
            return
        wheel_delta = wheel_angles - self._last_wheel_angle_rad
        period = self.config.encoder_wrap_period_rad
        wheel_delta = (wheel_delta + period / 2) % period - period / 2
        self._last_wheel_angle_rad = wheel_angles
        self._last_imu_yaw_rad = imu_yaw
        odom_distance = self.config.wheel_radius_m * float(np.mean(wheel_delta))
        odom_yaw_delta = self.config.wheel_radius_m * float(
            (wheel_delta[0] + wheel_delta[2] - wheel_delta[1] - wheel_delta[3])
            / (4 * self.config.half_track_m)
        )
        if dt is not None:
            self._estimated_linear_speed_m_s = odom_distance / dt
            self._estimated_yaw_rate_rad_s = odom_yaw_delta / dt
        previous_yaw = float(self._wheel_odometry[2])
        next_yaw = _wrap_angle(
            float(imu_yaw) if imu_yaw is not None else previous_yaw + odom_yaw_delta
        )
        midpoint_yaw = _wrap_angle(
            previous_yaw + _wrap_angle(next_yaw - previous_yaw) / 2
        )
        self._wheel_odometry[0] += odom_distance * math.cos(midpoint_yaw)
        self._wheel_odometry[1] += odom_distance * math.sin(midpoint_yaw)
        self._wheel_odometry[2] = next_yaw

    def commands(self, model: mujoco.MjModel) -> dict[str, float]:
        return {
            model.actuator(int(actuator_id)).name: float(command)
            for actuator_id, command in zip(self.actuator_ids, self._command_rad_s)
        }

    def metrics(self, model: mujoco.MjModel, data: mujoco.MjData) -> dict:
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(
            model, data, mujoco.mjtObj.mjOBJ_BODY, self.base_body_id, velocity, 0
        )
        roll, pitch, yaw = _roll_pitch_yaw(data.body(self.base_body_id).xquat)
        wheel_velocity = data.qvel[self.dof_ids].copy()
        wheel_force = data.actuator_force[self.actuator_ids].copy()
        ground_truth_xy = data.body(self.base_body_id).xpos[:2]
        odometry_xy_error = float(
            np.linalg.norm(self._wheel_odometry[:2] - ground_truth_xy)
        )
        planar_speed = float(np.linalg.norm(velocity[3:5]))
        yaw_rate = float(velocity[2])
        return {
            "time_s": float(data.time),
            "ground_truth_pose_xy_yaw": [
                float(data.body(self.base_body_id).xpos[0]),
                float(data.body(self.base_body_id).xpos[1]),
                yaw,
            ],
            "wheel_odometry_pose_xy_yaw": self._wheel_odometry.tolist(),
            "wheel_encoder_angle_rad": {
                name: float(value)
                for name, value in zip(
                    WHEEL_JOINT_NAMES,
                    self._last_wheel_angle_rad
                    if self._last_wheel_angle_rad is not None
                    else np.zeros(4),
                )
            },
            "imu_yaw_rad": self._last_imu_yaw_rad,
            "estimated_linear_speed_m_s": self._estimated_linear_speed_m_s,
            "estimated_yaw_rate_rad_s": self._estimated_yaw_rate_rad_s,
            "evaluation_only_odometry_error_xy_m": odometry_xy_error,
            "evaluation_only_odometry_yaw_error_rad": abs(
                _wrap_angle(float(self._wheel_odometry[2]) - yaw)
            ),
            "base_linear_velocity_m_s": velocity[3:].tolist(),
            "base_angular_velocity_rad_s": velocity[:3].tolist(),
            "wheel_velocity_rad_s": {
                name: float(value) for name, value in zip(WHEEL_JOINT_NAMES, wheel_velocity)
            },
            "wheel_actuator_torque_nm": {
                name: float(value)
                for name, value in zip(WHEEL_ACTUATOR_NAMES, wheel_force)
            },
            "max_abs_wheel_actuator_torque_nm": float(np.max(np.abs(wheel_force))),
            "wheel_command_rad_s": {
                name: float(value)
                for name, value in zip(WHEEL_ACTUATOR_NAMES, self._command_rad_s)
            },
            "base_roll_rad": roll,
            "base_pitch_rad": pitch,
            "base_tilt_rad": float(math.hypot(roll, pitch)),
            "within_tilt_limit": bool(
                math.hypot(roll, pitch) <= self.config.tilt_limit_rad
            ),
            "estimated_is_stopped": bool(
                abs(self._estimated_linear_speed_m_s) <= self.config.stop_speed_m_s
                and abs(self._estimated_yaw_rate_rad_s)
                <= self.config.stop_yaw_rate_rad_s
            ),
            "evaluation_only_is_physically_stopped": bool(
                planar_speed <= self.config.stop_speed_m_s
                and abs(yaw_rate) <= self.config.stop_yaw_rate_rad_s
            ),
            "simulation_parameters": asdict(self.config),
            "model_scope": (
                "rigid-mesh wheels; validated longitudinal/stop; "
                "no lateral-motion claim"
            ),
        }

