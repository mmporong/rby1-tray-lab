"""Closed-loop mobile manipulation mission for the sensor-driven RB-Y1 task.

Only initialization writes generalized position.  After reset, the robot and
all task objects move through actuator controls and ``mj_step``.  The control
path receives the tray pose from RGB-D perception and the base pose from wheel
encoder odometry; simulator object pose is retained only by ``Session`` for
independent evaluation.
"""
from __future__ import annotations

import copy
from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Callable

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from .camera import CameraCalibration, NamedRGBDCamera, RGBDFrame, RGBDNoise
from .evaluation import Evaluator
from .cooperation import CooperationConfig, CooperationSupervisor
from .mobility import DriveConfig, DriveController, WHEEL_JOINT_NAMES
from .perception import PerceptionError, TrayCAD, TrayEstimate, estimate_tray_pose
from .planning import Robot, smoothstep
from .scene import audit_model, build_scene, load_config
from .session import Session


# Synthetic IK warm start selected by a deterministic multi-start sweep for
# RBY1-M 1.2, 1.367 m TCP height, 0.65 m forward reach, and native grippers.
# It is not a hardware calibration; Robot.solve rechecks pose, torque, and
# collision gates against the runtime-built model before accepting it.
HIGH_CARRY_WARM_START_RAD = np.array([
    0.00020493762294933665, 0.16777511570893686, 3.078687552511791e-05,
    -0.025660358667805895, -8.361012339345612e-05, -3.7202436924684835e-05,
    -2.1976009643680308, -0.764997179934647, 2.089399999984652,
    -0.30995716476520696, -2.3631448911570145, 1.9148621769999998,
    1.5491231692540874, -2.197200392873641, 0.7656759840869669,
    -2.0893999999892303, -0.30969730939197376, 2.3630025957887244,
    1.9148621769999998, 1.5915926228840773,
])


@dataclass(frozen=True)
class ArmMotion:
    """One time-parameterized upper-body reference in a known base frame."""

    phase: str
    duration_s: float
    start_joints_rad: np.ndarray
    target_joints_rad: np.ndarray
    start_gripper_m: float
    target_gripper_m: float
    tray_center_world_m: tuple[float, float, float]
    base_pose_xy_yaw: tuple[float, float, float]
    joint_path_rad: tuple[np.ndarray, ...]
    ik_metrics: dict

    def __post_init__(self) -> None:
        if not self.phase or not np.isfinite(self.duration_s) or self.duration_s <= 0:
            raise ValueError("ArmMotion requires a phase and positive finite duration")
        for name, values, shape in (
            ("start_joints_rad", self.start_joints_rad, (20,)),
            ("target_joints_rad", self.target_joints_rad, (20,)),
            ("tray_center_world_m", self.tray_center_world_m, (3,)),
            ("base_pose_xy_yaw", self.base_pose_xy_yaw, (3,)),
        ):
            value = np.asarray(values, dtype=float)
            if value.shape != shape or not np.all(np.isfinite(value)):
                raise ValueError(f"{name} must have shape {shape} and finite values")
        if len(self.joint_path_rad) < 2:
            raise ValueError("joint_path_rad must contain start and target references")
        for values in self.joint_path_rad:
            value = np.asarray(values, dtype=float)
            if value.shape != (20,) or not np.all(np.isfinite(value)):
                raise ValueError("joint_path_rad entries must contain 20 finite values")


def tray_tcp_targets(
    center_world_m: tuple[float, float, float] | np.ndarray,
    tray_yaw_rad: float,
    side_offset_y_m: float,
    tcp_height_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Convert a perceived tray pose into right/left native-rim TCP targets."""

    center = np.asarray(center_world_m, dtype=float)
    values = (tray_yaw_rad, side_offset_y_m, tcp_height_m)
    if center.shape != (3,) or not np.all(np.isfinite(center)):
        raise ValueError("center_world_m must contain three finite values")
    if not all(np.isfinite(values)) or side_offset_y_m <= 0 or tcp_height_m <= 0:
        raise ValueError("grasp dimensions and yaw must be finite and positive")
    tray_rotation = Rotation.from_euler("z", tray_yaw_rad).as_matrix()
    positions = np.array(
        [
            center + tray_rotation @ np.array([0.0, sign * side_offset_y_m, tcp_height_m])
            for sign in (-1.0, 1.0)
        ]
    )
    return positions, tray_rotation


def interpolate_joint_path(
    joint_path_rad: tuple[np.ndarray, ...] | list[np.ndarray], scaled: float
) -> np.ndarray:
    """Evaluate a C1 Hermite path with zero endpoint velocity."""

    path = np.asarray(joint_path_rad, dtype=float)
    if path.ndim != 2 or path.shape[0] < 2 or path.shape[1] != 20:
        raise ValueError("joint path must contain at least two 20-joint waypoints")
    value = float(np.clip(scaled, 0.0, path.shape[0] - 1))
    segment = min(int(value), path.shape[0] - 2)
    alpha = value - segment
    before = path[segment]
    after = path[segment + 1]
    tangents = _joint_path_tangents(path)
    tangent_before = tangents[segment]
    tangent_after = tangents[segment + 1]
    a2 = alpha * alpha
    a3 = a2 * alpha
    return (
        (2 * a3 - 3 * a2 + 1) * before
        + (a3 - 2 * a2 + alpha) * tangent_before
        + (-2 * a3 + 3 * a2) * after
        + (a3 - a2) * tangent_after
    )


def _joint_path_tangents(path: np.ndarray) -> np.ndarray:
    """Return shape-preserving C1 tangents for uniformly spaced waypoints."""

    delta = np.diff(path, axis=0)
    tangents = np.zeros_like(path)
    previous = delta[:-1]
    following = delta[1:]
    same_direction = previous * following > 0.0
    denominator = previous + following
    np.divide(
        2.0 * previous * following,
        denominator,
        out=tangents[1:-1],
        where=same_direction & (denominator != 0.0),
    )
    return tangents


def joint_path_limit_violation(
    joint_path_rad: tuple[np.ndarray, ...] | list[np.ndarray],
    low_rad: np.ndarray,
    high_rad: np.ndarray,
) -> float:
    """Return the exact maximum joint-limit violation along the cubic path."""

    path = np.asarray(joint_path_rad, dtype=float)
    low = np.asarray(low_rad, dtype=float)
    high = np.asarray(high_rad, dtype=float)
    if path.ndim != 2 or path.shape[0] < 2 or path.shape[1] != 20:
        raise ValueError("joint path must contain at least two 20-joint waypoints")
    if low.shape != (20,) or high.shape != (20,) or np.any(low > high):
        raise ValueError("joint limits must contain ordered 20-joint bounds")
    violation = 0.0
    tangents = _joint_path_tangents(path)
    for segment in range(path.shape[0] - 1):
        before = path[segment]
        after = path[segment + 1]
        tangent_before = tangents[segment]
        tangent_after = tangents[segment + 1]
        cubic = 2 * before - 2 * after + tangent_before + tangent_after
        quadratic = -3 * before + 3 * after - 2 * tangent_before - tangent_after
        linear = tangent_before
        candidates = [before, after]
        for joint in range(20):
            roots = np.roots([3 * cubic[joint], 2 * quadratic[joint], linear[joint]])
            for root in roots:
                if abs(root.imag) <= 1e-10 and 0.0 < root.real < 1.0:
                    alpha = float(root.real)
                    candidates.append(
                        ((cubic * alpha + quadratic) * alpha + linear) * alpha
                        + before
                    )
        values = np.asarray(candidates)
        violation = max(
            violation,
            float(np.max(low - values)),
            float(np.max(values - high)),
        )
    return max(0.0, violation)


class MobileSession(Session):
    """Sensor-driven floor-pick, mobile carry, and high-placement rollout."""

    # A 0.22 rad waypoint bound admits the observed slow retract transition
    # (0.217 rad over roughly one second) while rejecting the 0.24/0.32/2.38
    # rad high-reach branch changes that generated the physical contact impulse.
    HELD_IK_MAX_JOINT_STEP_RAD = 0.22
    PREGRASP_DEPTH_RESIDUAL_MAX_M = 0.02
    PRESEAT_EXPECTED_LOAD_FRACTION_MIN = 0.25

    ARM_PHASES = {
        "prepare_navigation",
        "prepare_pickup",
        "pickup_approach",
        "pickup_descend",
        "grasp",
        "grasp_hold",
        "lift",
        "retract_low",
        "raise_high",
        "seat",
        "settle",
        "release",
        "withdraw",
        "return_home",
    }
    POSTURE_PHASES = {"prepare_navigation", "prepare_pickup", "return_home"}
    OBSERVATION_PHASES = {"observe_close", "observe_pickup", "observe_pregrasp"}
    COOPERATIVE_PHASES = {"retract_low", "drive_loaded", "raise_high", "drive_high"}
    DRIVE_PHASE_TARGET = {
        "drive_approach": "pickup_pose_xy_yaw",
        "drive_loaded": "low_carry_pose_xy_yaw",
        "drive_high": "placement_pose_xy_yaw",
        "drive_return": "initial_pose_xy_yaw",
    }

    def __init__(
        self,
        config=None,
        *,
        observation_provider: Callable[[mujoco.MjData], RGBDFrame] | None = None,
        pose_estimator: Callable[..., TrayEstimate] = estimate_tray_pose,
        wrist_verifier: Callable[["MobileSession"], dict] | None = None,
    ):
        default_config = Path(__file__).resolve().parents[2] / "configs/mobile-native.json"
        self.config = copy.deepcopy(config or load_config(default_config))
        if self.config.get("base_mode") != "mobile":
            raise ValueError("MobileSession requires base_mode=mobile")
        cycle = self.config.get("mission_cycle", {})
        if cycle.get("home_source") != "fixed_upstream_model_qpos0":
            raise ValueError("MobileSession requires mission_cycle home_source=fixed_upstream_model_qpos0")
        for name in ("initial_hold_s", "prepare_duration_s", "return_duration_s", "return_drive_timeout_s"):
            value = float(cycle.get(name, float("nan")))
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f"mission_cycle.{name} must be finite and positive")
        self.lift_only = False
        self.model, self.xml = build_scene(self.config)
        self.audit = audit_model(self.model)
        self.robot = Robot(self.model)
        self._head_joint_names = ("head_0", "head_1")
        head_joint_ids = np.array(
            [self.model.joint(name).id for name in self._head_joint_names], dtype=int
        )
        self._head_joint_ids = head_joint_ids
        self._head_qadr = self.model.jnt_qposadr[head_joint_ids]
        self._head_dadr = self.model.jnt_dofadr[head_joint_ids]
        self.renderer = None
        self._camera = None
        self._wrist_cameras: dict[str, NamedRGBDCamera] = {}
        self._monitor_cameras: dict[str, NamedRGBDCamera] = {}
        self._observation_provider = observation_provider
        self._pose_estimator = pose_estimator
        self._wrist_verifier = wrist_verifier or self._contact_wrist_verifier
        self.sensor_frames: list[RGBDFrame] = []
        self.perception_history: list[dict] = []
        self.command_history: list[dict] = []
        self.ik_errors: list[dict] = []
        self.plan: list[ArmMotion] = []
        self.finished = False
        self.reset()

    def _drive_config(self) -> DriveConfig:
        values = self.config.get("drive", {})
        initial = tuple(float(v) for v in self.config["base"]["initial_pose_xy_yaw"])
        return DriveConfig(
            max_linear_speed_m_s=float(values.get("max_linear_speed_m_s", 0.06)),
            max_wheel_accel_rad_s2=float(values.get("max_wheel_accel_rad_s2", 0.5)),
            wheel_velocity_gain_nms_per_rad=float(values.get("wheel_velocity_gain", 100.0)),
            initial_pose_xy_yaw=initial,
        )

    def _tcp_height_m(self) -> float:
        return float(self.config["grasp"]["tcp_height_m"])

    def reset(self):
        if self._camera is not None:
            self._camera.close()
            self._camera = None
        for camera in self._wrist_cameras.values():
            camera.close()
        self._wrist_cameras = {}
        for camera in self._monitor_cameras.values():
            camera.close()
        self._monitor_cameras = {}
        self.data = mujoco.MjData(self.model)
        base_qpos = self.data.joint("world_j").qpos
        initial_base = np.asarray(self.config["base"]["initial_pose_xy_yaw"], dtype=float)
        base_qpos[:3] = (initial_base[0], initial_base[1], 0.0)
        base_qpos[3:7] = (math.cos(initial_base[2] / 2), 0.0, 0.0, math.sin(initial_base[2] / 2))
        # The fixed upstream model has no keyframes.  Its qpos0 is the source
        # for both departure and return, including the native gripper sliders.
        self.home_joints_rad = self.model.qpos0[self.robot.qadr].copy()
        self.home_head_rad = self.model.qpos0[self._head_qadr].copy()
        home_fingers = self.model.qpos0[self.robot.grip_qadr]
        if not np.allclose(home_fingers, home_fingers[0], rtol=0, atol=1e-12):
            raise ValueError("model qpos0 requires independently controlled home fingers")
        self.home_gripper_m = float(home_fingers[0])
        initial_joints = self.home_joints_rad.copy()
        self.data.qpos[self.robot.qadr] = initial_joints
        self.data.qpos[self._head_qadr] = self.home_head_rad
        for letter in ("l", "r"):
            for finger in (1, 2):
                self.data.joint(f"gripper_finger_{letter}{finger}").qpos[0] = self.home_gripper_m
        self.command_rad = initial_joints.copy()
        self.command_gripper_m = self.home_gripper_m
        self.drive = DriveController(self.model, self._drive_config())
        mujoco.mj_forward(self.model, self.data)
        self.drive.reset(self.model, self.data)
        for _ in range(250):
            self._set_controls(self.command_rad, self.command_gripper_m)
            self.drive.stop(self.model, self.data, self.model.opt.timestep)
            mujoco.mj_step(self.model, self.data)
        self.data.time = 0.0
        self.drive.reset(self.model, self.data)
        self._wrist_ft_tare = {
            side: self._read_wrist_ft(side) for side in ("left", "right")
        }
        self._expected_qpos = self.data.qpos.copy()
        self._expected_qvel = self.data.qvel.copy()
        self._expected_eq_active = self.data.eq_active.copy()
        self._expected_eq_data = self.model.eq_data.copy()
        self._expected_mocap_pos = self.data.mocap_pos.copy()
        self._expected_mocap_quat = self.data.mocap_quat.copy()
        self.audit["runtime_state_overwrite"] = False

        cfg = copy.deepcopy(self.config)
        cfg["initial_tray_bottom_z_m"] = (
            self.data.body("tray").xpos[2] - cfg["tray_half_extents_m"][2]
        )
        cfg["expected_payload_names"] = [f"payload_{i}" for i in range(3)]
        cfg["payload_max_height_m"] = 1.45
        tray_id = self.model.body("tray").id
        geom_ids = np.flatnonzero(self.model.geom_bodyid == tray_id)
        geom_low = self.model.geom_pos[geom_ids] - self.model.geom_size[geom_ids]
        geom_high = self.model.geom_pos[geom_ids] + self.model.geom_size[geom_ids]
        low, high = geom_low.min(axis=0), geom_high.max(axis=0)
        cfg["tray_containment_half_extents_m"] = ((high - low) / 2).tolist()
        cfg["tray_containment_center_m"] = ((high + low) / 2).tolist()
        self.home_reference = {
            **copy.deepcopy(self.config["mission_cycle"]["home_tolerances"]),
            "joint_names": list(self.robot.names),
            "joint_position_rad": self.home_joints_rad.tolist(),
            "head_position_rad": self.home_head_rad.tolist(),
            "gripper_position_m": self.home_gripper_m,
            "base_pose_xy_yaw": initial_base.tolist(),
        }
        cfg["return_home"] = copy.deepcopy(self.home_reference)
        self.evaluator = Evaluator(cfg)

        # Navigate to the surveyed pickup station from wheel odometry first.
        # The floor tray is partially outside the initial head-camera frame;
        # grasp planning waits for the complete close-range RGB-D observation.
        self.phase = "home_start"
        self.phase_started_s = 0.0
        self._active_motion: ArmMotion | None = None
        self._detected_tray: TrayEstimate | None = None
        self._planned_tray_center: np.ndarray | None = None
        self._drive_start_pose: np.ndarray | None = None
        self._drive_start_tray_center: np.ndarray | None = None
        self._mission_failure: dict | None = None
        self._drive_stopped_since_s: float | None = None
        self._motion_settled_since_s: float | None = None
        self._preseat_hand_load_n: float | None = None
        self._pregrasp_wrist_verification: dict | None = None
        self._pending_head_frame = None
        self._next_control_s = 0.0
        self._held_velocity_rad_s = np.zeros(20)
        self._motion_clock_s = 0.0
        self._reference_finished_s = None
        self._next_cooperation_s = 0.0
        self._next_monitor_s = 0.0
        self._feedback_fk = mujoco.MjData(self.model)
        self._reference_fk = mujoco.MjData(self.model)
        cooperation_values = self.config.get("cooperation", {})
        self.cooperation = CooperationSupervisor(CooperationConfig(**{
            name: cooperation_values[name] for name in CooperationConfig.__dataclass_fields__
            if name in cooperation_values
        }))
        self.cooperation_history = []
        self._last_record_s = -0.1
        self.max_tracking_error_rad = 0.0
        self.max_joint_torque_nm = 0.0
        self.max_finger_force_n = 0.0
        self.log = []
        self.sensor_frames = []
        self.monitor_frames = []
        self.perception_history = []
        self.command_history = []
        self.ik_errors = []
        self.finished = False
        self._record(force_log=True)
        return self.snapshot()

    def _navigation_posture(self):
        """Reference the SDK mobile example; validate against this scene first."""
        degrees = [0, 45, -90, 45, 0, 0,
                   0, -5, 0, -120, 0, 45, 0,
                   0, 5, 0, -120, 0, 45, 0]
        return np.clip(np.deg2rad(degrees), self.robot.low, self.robot.high)

    def _capture_monitors(self):
        cfg = self.config.get("monitoring", {})
        if not cfg.get("enabled", False) or float(self.data.time) + 1e-9 < self._next_monitor_s:
            return
        interval = float(cfg.get("period_s", 0.5))
        if not np.isfinite(interval) or interval <= 0:
            raise ValueError("monitoring.period_s must be finite and positive")
        self._next_monitor_s = float(self.data.time) + interval
        for sensor in self.config["sensors"].values():
            name = sensor["camera_name"]
            source_width, source_height = sensor["resolution_px"]
            width = int(cfg.get("width_px", 320))
            height = round(width * source_height / source_width)
            calibration = self._controller_camera_calibration(name, width, height)
            if name not in self._monitor_cameras:
                self._monitor_cameras[name] = NamedRGBDCamera(
                    self.model, name, width=width, height=height,
                    calibration=calibration, noise=self._camera_noise(sensor),
                    depth_min_m=float(sensor["range_m"][0]), depth_max_m=float(sensor["range_m"][1]),
                    sensor_profile=sensor["model"], distortion_model=sensor["distortion_model"],
                    rectified_aligned=bool(sensor["rgb_depth_aligned"]),
                )
            camera = self._monitor_cameras[name]
            camera.calibration_override = calibration
            frame = self._controller_frame(self._apply_sensor_range(camera.capture(self.data), sensor))
            frame.metadata["observation_role"] = "monitor_only"
            frame.metadata["capture_batch_id"] = f"monitor-{float(self.data.time):.6f}"
            frame.metadata["sample_period_s"] = interval
            self.monitor_frames.append(frame)

    def _capture_and_estimate(self) -> TrayEstimate | None:
        sensor_cfg = self.config["sensors"]["head"]
        if self._pending_head_frame is not None:
            frame = self._pending_head_frame
        elif self._observation_provider is None:
            if self._camera is None:
                width, height = sensor_cfg["resolution_px"]
                self._camera = NamedRGBDCamera(
                    self.model,
                    sensor_cfg["camera_name"],
                    width=int(width),
                    height=int(height),
                    calibration=self._controller_camera_calibration(
                        sensor_cfg["camera_name"], int(width), int(height)
                    ),
                    noise=self._camera_noise(sensor_cfg),
                    depth_min_m=float(sensor_cfg["range_m"][0]),
                    depth_max_m=float(sensor_cfg["range_m"][1]),
                    sensor_profile=str(sensor_cfg["model"]),
                    distortion_model=str(sensor_cfg["distortion_model"]),
                    rectified_aligned=bool(sensor_cfg["rgb_depth_aligned"]),
                )
            else:
                width, height = sensor_cfg["resolution_px"]
                self._camera.calibration_override = self._controller_camera_calibration(
                    sensor_cfg["camera_name"], int(width), int(height)
                )
            frame = self._camera.capture(self.data)
        else:
            frame = self._observation_provider(self.data)
        if not isinstance(frame, RGBDFrame):
            raise PerceptionError("observation_provider must return RGBDFrame")
        self._validate_rgbd_frame(frame, sensor_cfg)
        delay_s = float(sensor_cfg.get("delivery_delay_s", 0.0))
        if not np.isfinite(delay_s) or delay_s < 0:
            raise PerceptionError("RGB-D delivery delay must be finite and nonnegative")
        if float(self.data.time) + 1e-9 < float(frame.timestamp_sim_s) + delay_s:
            self._pending_head_frame = frame
            return None
        self._pending_head_frame = None
        self._validate_rgbd_frame(frame, sensor_cfg)
        frame = self._controller_frame(self._apply_sensor_range(frame, sensor_cfg))
        self.sensor_frames.append(frame)
        cad = TrayCAD(
            half_extents_m=tuple(self.config["tray_half_extents_m"]),
            grasp_height_m=self._tcp_height_m(),
        )
        if (self.config.get("perception", {}).get("estimator") == "depth_cad"
                and self._pose_estimator is estimate_tray_pose):
            from .perception import estimate_tray_pose_from_depth
            estimate = estimate_tray_pose_from_depth(
                frame.depth_m, frame.K, frame.T_world_optical,
                cad=cad, dimension_relative_tolerance=0.15,
            )
        else:
            estimate = self._pose_estimator(
                frame.rgb, frame.depth_m, frame.K, frame.T_world_optical,
                cad=cad, dimension_relative_tolerance=0.15,
            )
        innovation = None
        if self._detected_tray is not None:
            from .perception import validate_tray_estimate_update
            innovation = validate_tray_estimate_update(
                self._detected_tray, estimate,
                max_position_innovation_m=float(self.config.get("perception", {}).get("max_update_position_m", 0.025)),
                max_yaw_innovation_rad=float(self.config.get("perception", {}).get("max_update_yaw_rad", 0.08)),
            )
        if self.phase == "observe_close" and float(
            estimate.quality.get("score_0_to_1", 0.0)
        ) < 0.30:
            raise PerceptionError("close-range tray estimate quality is below 0.30")
        record = {
            "timestamp_sim_s": frame.timestamp_sim_s,
            "delivery_time_sim_s": float(self.data.time),
            "observation_age_s": float(self.data.time) - frame.timestamp_sim_s,
            "camera_name": frame.metadata.get("camera_name", sensor_cfg["camera_name"]),
            "resolution_px": [int(frame.rgb.shape[1]), int(frame.rgb.shape[0])],
            "calibration_source": frame.metadata.get("calibration_source", "unspecified"),
            "calibration_input": "encoder_odometry_plus_joint_encoder_fk",
            "live_model_calibration_role": "evaluation_only",
            "valid_depth_fraction": float(np.mean(np.isfinite(frame.depth_m) & (frame.depth_m > 0))),
            "estimate": estimate.to_dict(),
            "phase": self.phase,
            "innovation_gate": innovation,
        }
        self.perception_history.append(record)
        return estimate

    @staticmethod
    def _controller_frame(frame: RGBDFrame) -> RGBDFrame:
        """Remove renderer-only ground truth calibration from controller data."""

        metadata = copy.deepcopy(frame.metadata)
        metadata.pop("model_calibration", None)
        metadata["controller_calibration"] = (
            "encoder odometry plus measured robot-joint forward kinematics"
        )
        metadata["ground_truth_extrinsics_in_controller_packet"] = False
        metadata.setdefault("observation_role", "controller_observation")
        return RGBDFrame(
            rgb=frame.rgb,
            depth_m=frame.depth_m,
            rgb_raw=frame.rgb_raw,
            depth_raw_m=frame.depth_raw_m,
            K=frame.K,
            T_world_optical=frame.T_world_optical,
            timestamp_sim_s=frame.timestamp_sim_s,
            metadata=metadata,
        )

    def _camera_noise(self, sensor_config: dict) -> RGBDNoise:
        values = {**self.config.get("sensor_noise", {}), **sensor_config.get("noise", {})}
        camera_order = {"head_rgbd": 0, "left_wrist_rgbd": 1, "right_wrist_rgbd": 2}
        seed = int(values.get("seed", self.config.get("seed", 0)))
        seed += camera_order.get(str(sensor_config.get("camera_name", "")), 0)
        return RGBDNoise(
            rgb_std=float(values.get("rgb_std", 0.0)),
            depth_std_m=float(values.get("depth_std_m", 0.0)),
            dropout_probability=float(values.get("dropout_probability", 0.0)),
            seed=seed,
            source=str(values.get("source", "synthetic_assumption")),
        )

    def _validate_rgbd_frame(self, frame: RGBDFrame, sensor_config: dict) -> None:
        """Reject stale, malformed, or ambiguously framed controller images."""

        timestamp = float(frame.timestamp_sim_s)
        if not np.isfinite(timestamp):
            raise PerceptionError("RGB-D timestamp must be finite")
        age_s = float(self.data.time) - timestamp
        maximum_age_s = float(
            sensor_config.get(
                "max_rgbd_age_s", self.config.get("max_rgbd_age_s", 0.1)
            )
        )
        future_tolerance_s = float(self.config["physics_dt_s"])
        if age_s < -future_tolerance_s - 1e-12 or age_s > maximum_age_s:
            raise PerceptionError(
                f"stale RGB-D frame: age={age_s:.6f}s, allowed="
                f"[-{future_tolerance_s:.6f}, {maximum_age_s:.6f}]s"
            )
        rgb = np.asarray(frame.rgb)
        depth = np.asarray(frame.depth_m)
        if rgb.ndim != 3 or rgb.shape[2] != 3 or depth.shape != rgb.shape[:2]:
            raise PerceptionError("RGB-D shapes must be HxWx3 RGB and matching HxW depth")
        width, height = (int(value) for value in sensor_config["resolution_px"])
        if rgb.shape[:2] != (height, width):
            raise PerceptionError(
                f"RGB-D resolution {rgb.shape[1]}x{rgb.shape[0]} does not match "
                f"declared {width}x{height}"
            )
        expected_frame = "+X right, +Y down, +Z forward"
        if frame.metadata.get("optical_frame") != expected_frame:
            raise PerceptionError(
                f"RGB-D optical_frame must be declared as {expected_frame!r}"
            )

    @staticmethod
    def _apply_sensor_range(frame: RGBDFrame, sensor_config: dict) -> RGBDFrame:
        near_m, far_m = (float(value) for value in sensor_config["range_m"])
        depth = frame.depth_m.copy()
        invalid = (~np.isfinite(depth)) | (depth < near_m) | (depth > far_m)
        depth[invalid] = np.nan
        metadata = copy.deepcopy(frame.metadata)
        metadata["configured_operating_range_m"] = [near_m, far_m]
        metadata["sensor_profile"] = str(sensor_config["model"])
        metadata["range_filter_applied"] = True
        metadata["raw_channels_role"] = "evaluation_only_ideal_render"
        return RGBDFrame(
            rgb=frame.rgb,
            depth_m=depth,
            rgb_raw=frame.rgb_raw,
            depth_raw_m=frame.depth_raw_m,
            K=frame.K,
            T_world_optical=frame.T_world_optical,
            timestamp_sim_s=frame.timestamp_sim_s,
            metadata=metadata,
        )

    def _controller_camera_calibration(
        self, camera_name: str, width: int, height: int
    ) -> CameraCalibration:
        """Build camera extrinsics from hardware-equivalent proprioception.

        MuJoCo's live ``cam_xpos`` includes the ground-truth free-base pose.  It
        is intentionally excluded.  This scratch FK receives wheel-encoder
        odometry plus measured one-DoF robot joints; task-object qpos is never
        copied into it.
        """

        scratch = mujoco.MjData(self.model)
        odom = self.drive.metrics(self.model, self.data)[
            "wheel_odometry_pose_xy_yaw"
        ]
        x, y, yaw = (float(value) for value in odom)
        base = scratch.joint("world_j").qpos
        base[:3] = (x, y, 0.0)
        base[3:7] = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
        for joint_id in range(self.model.njnt):
            if self.model.jnt_type[joint_id] not in (
                mujoco.mjtJoint.mjJNT_HINGE,
                mujoco.mjtJoint.mjJNT_SLIDE,
            ):
                continue
            qpos_address = int(self.model.jnt_qposadr[joint_id])
            scratch.qpos[qpos_address] = self.data.qpos[qpos_address]
        mujoco.mj_forward(self.model, scratch)
        camera_id = self.model.camera(camera_name).id
        fovy = math.radians(float(self.model.cam_fovy[camera_id]))
        focal = 0.5 * height / math.tan(0.5 * fovy)
        intrinsic = np.array(
            [
                [focal, 0.0, (width - 1) / 2],
                [0.0, focal, (height - 1) / 2],
                [0.0, 0.0, 1.0],
            ]
        )
        rotation_world_gl = scratch.cam_xmat[camera_id].reshape(3, 3)
        transform = np.eye(4)
        transform[:3, :3] = rotation_world_gl @ np.diag([1.0, -1.0, -1.0])
        transform[:3, 3] = scratch.cam_xpos[camera_id]
        profile = next(p for p in self.config["sensors"].values() if p["camera_name"] == camera_name)
        translation = np.asarray(profile.get("calibration_translation_bias_m", [0, 0, 0]), dtype=float)
        angles = np.asarray(profile.get("calibration_rotation_bias_rad", [0, 0, 0]), dtype=float)
        if translation.shape != (3,) or angles.shape != (3,) or not np.all(np.isfinite([translation, angles])):
            raise ValueError("synthetic camera calibration bias must contain finite 3-vectors")
        bias = np.eye(4)
        bias[:3, :3] = Rotation.from_euler("xyz", angles).as_matrix()
        bias[:3, 3] = translation
        transform = transform @ bias
        return CameraCalibration(intrinsic, transform, "synthetic_model")

    def _set_work_base(self, base_pose_xy_yaw) -> None:
        x, y, yaw = (float(value) for value in base_pose_xy_yaw)
        qpos = self.robot.work.joint("world_j").qpos
        qpos[:3] = (x, y, 0.0)
        qpos[3:7] = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))

    def _set_work_task_prediction(self, tray_center_world_m, tray_yaw_rad: float) -> None:
        """Place carried CAD bodies in scratch IK from the planned pose only."""

        center = np.asarray(tray_center_world_m, dtype=float)
        rotation = Rotation.from_euler("z", tray_yaw_rad).as_matrix()
        tray_initial = self.model.qpos0[self.model.jnt_qposadr[self.model.joint("tray_free").id]:][:3]
        tray_qpos = self.robot.work.joint("tray_free").qpos
        tray_qpos[:3] = center
        tray_qpos[3:7] = (
            math.cos(tray_yaw_rad / 2), 0.0, 0.0, math.sin(tray_yaw_rad / 2)
        )
        for index in range(3):
            joint_name = f"payload_{index}_free"
            address = int(self.model.jnt_qposadr[self.model.joint(joint_name).id])
            initial = self.model.qpos0[address: address + 3]
            initial_rotation = Rotation.from_euler("z", -float(self.config.get("tray_initial_yaw_rad", 0.0))).as_matrix()
            predicted = center + rotation @ initial_rotation @ (initial - tray_initial)
            qpos = self.robot.work.joint(joint_name).qpos
            qpos[:3] = predicted
            qpos[3:7] = tray_qpos[3:7]

    def _predicted_task_center(self, phase: str, tcp_center_world_m) -> np.ndarray:
        """Keep scratch task CAD consistent with the phase's grasp state."""

        if phase in {"pickup_approach", "pickup_descend", "grasp", "grasp_hold"}:
            return np.asarray(self._detected_tray.center_world_m, dtype=float)
        if phase in {"release", "withdraw"}:
            if self._planned_tray_center is None:
                raise RuntimeError("seated tray prediction is unavailable")
            return self._planned_tray_center.copy()
        return np.asarray(tcp_center_world_m, dtype=float)

    def _sensor_vector(self, name: str) -> np.ndarray:
        sensor_id = self.model.sensor(name).id
        address = int(self.model.sensor_adr[sensor_id])
        dimension = int(self.model.sensor_dim[sensor_id])
        return self.data.sensordata[address: address + dimension].copy()

    def _read_wrist_ft(self, side: str) -> dict[str, list[float]]:
        return {
            "force_n": self._sensor_vector(f"{side}_wrist_force_n").tolist(),
            "torque_nm": self._sensor_vector(f"{side}_wrist_torque_nm").tolist(),
        }

    def _wrist_proprioception(self) -> dict:
        readings = {}
        for index, (side, letter) in enumerate((("right", "r"), ("left", "l"))):
            current = self._read_wrist_ft(side)
            tare = self._wrist_ft_tare[side]
            force_delta = np.asarray(current["force_n"]) - np.asarray(tare["force_n"])
            torque_delta = np.asarray(current["torque_nm"]) - np.asarray(tare["torque_nm"])
            fingers = [
                float(self.data.joint(f"gripper_finger_{letter}{finger}").qpos[0])
                for finger in (1, 2)
            ]
            effort = float(abs(self.data.actuator_force[self.robot.grip_act[index]]))
            readings[side] = {
                "wrench": current,
                "tare": copy.deepcopy(tare),
                "force_delta_n": force_delta.tolist(),
                "torque_delta_nm": torque_delta.tolist(),
                "force_delta_norm_n": float(np.linalg.norm(force_delta)),
                "torque_delta_norm_nm": float(np.linalg.norm(torque_delta)),
                "finger_position_m": fingers,
                "gripper_actuator_force_n": effort,
            }
        return readings

    def _world_vertical_hand_load(self) -> dict:
        """Project tared wrist forces onto world gravity using encoder FK."""

        odom = self.drive.metrics(self.model, self.data)["wheel_odometry_pose_xy_yaw"]
        self._set_work_base(odom)
        self.robot.work.qpos[self.robot.qadr] = self.data.qpos[self.robot.qadr]
        mujoco.mj_forward(self.model, self.robot.work)
        proprioception = self._wrist_proprioception()
        forces = {}
        for side in ("right", "left"):
            local = np.asarray(proprioception[side]["force_delta_n"], dtype=float)
            rotation_world_site = self.robot.work.site(
                f"{side}_ft_frame"
            ).xmat.reshape(3, 3)
            world = rotation_world_site @ local
            forces[side] = {
                "force_delta_local_n": local.tolist(),
                "force_delta_world_n": world.tolist(),
                "signed_world_vertical_component_n": float(world[2]),
                "upward_support_component_n": float(max(0.0, world[2])),
                "downward_component_n": float(max(0.0, -world[2])),
            }
        return {
            "hands": forces,
            "signed_world_vertical_total_n": float(sum(
                value["signed_world_vertical_component_n"]
                for value in forces.values()
            )),
            "total_upward_support_component_n": float(sum(
                value["upward_support_component_n"] for value in forces.values()
            )),
            "total_downward_component_n": float(sum(
                value["downward_component_n"] for value in forces.values()
            )),
            "gravity_axis_world": [0.0, 0.0, -1.0],
            "source": "wheel_odometry_plus_joint_encoder_fk_plus_tared_wrist_ft",
            "force_sign_provenance": (
                "MuJoCo flange sensor convention; hardware driver must declare and "
                "validate its flange-force sign before replay"
            ),
        }

    @classmethod
    def _load_transfer_valid(
        cls, baseline_n: float | None, settled_n: float, expected_weight_n: float
    ) -> tuple[bool, float]:
        minimum_baseline_n = cls.PRESEAT_EXPECTED_LOAD_FRACTION_MIN * expected_weight_n
        finite = (
            baseline_n is not None
            and np.isfinite(baseline_n)
            and np.isfinite(settled_n)
            and np.isfinite(expected_weight_n)
        )
        passed = bool(
            finite
            and expected_weight_n > 0.0
            and baseline_n >= minimum_baseline_n
            and settled_n <= 0.8 * baseline_n
        )
        return passed, float(minimum_baseline_n)

    def _solve_motion(
        self,
        phase: str,
        tray_center_world_m,
        base_pose_xy_yaw,
        duration_s: float,
        target_gripper_m: float,
    ) -> ArmMotion:
        if self._detected_tray is None:
            raise RuntimeError("tray perception is required before IK")
        grasp = self.config["grasp"]
        positions, _ = tray_tcp_targets(
            tray_center_world_m,
            self._detected_tray.yaw_rad,
            float(grasp["side_offset_y_m"]),
            self._tcp_height_m(),
        )
        tool_yaw = self._detected_tray.yaw_rad + float(grasp["yaw_rad"])
        rotations = np.repeat(
            Rotation.from_euler("z", tool_yaw).as_matrix()[None, :, :], 2, axis=0
        )
        target_center = np.asarray(tray_center_world_m, dtype=float)
        start_center = (
            target_center.copy()
            if self._planned_tray_center is None
            else self._planned_tray_center.copy()
        )
        path_centers = [start_center]
        if phase == "raise_high":
            # The target deck begins before the low carried tray's front edge.
            # Retract in free space, raise above it, then extend at clearance.
            deck_near_x = (
                float(self.config["target"]["center_xy_m"][0])
                - float(self.config["target"]["inner_half_xy_m"][0])
            )
            safe_x = min(
                float(start_center[0]),
                deck_near_x - float(self.config["tray_half_extents_m"][0]) - 0.03,
            )
            path_centers.extend((
                np.array([safe_x, start_center[1], start_center[2]]),
                np.array([safe_x, target_center[1], target_center[2]]),
            ))
        path_centers.append(target_center)
        sampled_centers = []
        cartesian_step_m = 0.01 if phase == "raise_high" else 0.025
        for leg_start, leg_target in zip(path_centers[:-1], path_centers[1:]):
            distance = float(np.linalg.norm(leg_target - leg_start))
            if distance <= 1e-9:
                continue
            samples = max(
                1, int(np.ceil(distance / cartesian_step_m))
            )
            sampled_centers.extend(
                leg_start + (index / samples) * (leg_target - leg_start)
                for index in range(1, samples + 1)
            )
        if phase == "pickup_approach":
            # Keep the torso crouched while the hands clear the base and loose
            # payloads.  The first point moves only the arms to a forward,
            # raised pre-approach; the high-pose numerical seed selects the
            # final arm branch but is never sent as a joint reference.
            sampled_centers = [target_center + [0.05, 0.0, 0.11], target_center]
        elif phase in {"pickup_descend", "grasp", "grasp_hold"}:
            sampled_centers = [target_center]
        elif not sampled_centers:
            # Hold-only phases such as settle/release may intentionally keep
            # the same Cartesian tray pose.  Retain one IK waypoint so the
            # tracking/dwell gates still run without reducing an empty path.
            sampled_centers = [target_center]
        segment_count = len(sampled_centers)
        path = [self.command_rad.copy()]
        segment_metrics = []
        phase_start = self.command_rad.copy()
        seed = phase_start.copy()
        continuation_phase = phase in {
            "lift", "retract_low", "raise_high", "seat", "settle", "release", "withdraw"
        }
        for index, point in enumerate(sampled_centers, start=1):
            fraction = index / segment_count
            segment_gripper = self.command_gripper_m + fraction * (
                target_gripper_m - self.command_gripper_m
            )
            point_positions, _ = tray_tcp_targets(
                point,
                self._detected_tray.yaw_rad,
                float(grasp["side_offset_y_m"]),
                self._tcp_height_m(),
            )
            self._set_work_base(base_pose_xy_yaw)
            predicted_object_center = self._predicted_task_center(phase, point)
            self._set_work_task_prediction(
                predicted_object_center, self._detected_tray.yaw_rad
            )
            if phase == "raise_high":
                # Keep the local arm branch while gradually extending the
                # torso toward the separately collision-checked high seed.
                # A late fallback to that seed causes a discontinuous wrist
                # branch switch while the tray is loaded.
                posture = seed.copy()
                posture[:6] = phase_start[:6] + smoothstep(fraction) * (
                    HIGH_CARRY_WARM_START_RAD[:6] - phase_start[:6]
                )
            else:
                posture = (
                    None
                    if continuation_phase
                    else self.robot.crouched_pickup_posture()
                )
            solve_seed = (
                HIGH_CARRY_WARM_START_RAD.copy()
                if phase == "pickup_approach" and index == segment_count
                else seed
            )
            solved, result = self.robot.solve(
                point_positions,
                seed=solve_seed,
                posture=posture,
                gripper_m=segment_gripper,
                target_rotations=rotations,
            )
            seed = solved
            path.append(solved.copy())
            segment_metrics.append(copy.deepcopy(result))
        target = path[-1]
        if phase == "pickup_approach":
            self._set_work_base(base_pose_xy_yaw)
            self._set_work_task_prediction(
                self._detected_tray.center_world_m,
                self._detected_tray.yaw_rad,
            )
            for segment_index in range(len(path) - 1):
                for alpha in np.linspace(0.0, 1.0, 51):
                    self.robot._prepare(
                        interpolate_joint_path(path, segment_index + alpha),
                        target_gripper_m,
                    )
                    forbidden = self.robot._forbidden_contacts()
                    if forbidden:
                        raise ValueError(
                            "pickup free-space reconfiguration collision at "
                            f"segment {segment_index}, alpha={alpha:.3f}: "
                            f"{forbidden[0]['geoms']}"
                        )
        joint_steps = np.array([
            float(np.max(np.abs(after - before)))
            for before, after in zip(path[:-1], path[1:])
        ])
        joint_limit_violation = (
            joint_path_limit_violation(path, self.robot.low, self.robot.high)
        )
        if joint_limit_violation > 1e-10:
            raise ValueError(
                f"{phase} interpolated joint-limit violation: "
                f"{joint_limit_violation:.9f} rad"
            )
        held_phase = phase not in {"pickup_approach", "pickup_descend"}
        if held_phase and float(np.max(joint_steps)) > self.HELD_IK_MAX_JOINT_STEP_RAD:
            index = int(np.argmax(joint_steps))
            raise ValueError(
                f"{phase} IK branch discontinuity at segment {index}: "
                f"{joint_steps[index]:.6f} rad exceeds "
                f"{self.HELD_IK_MAX_JOINT_STEP_RAD:.6f} rad"
            )
        interpolation_metrics = []
        if held_phase and len(path) == len(sampled_centers) + 1:
            centers = [start_center, *sampled_centers]
            for segment_index in range(len(path) - 1):
                for alpha in (0.25, 0.5, 0.75):
                    center = (
                        (1.0 - alpha) * centers[segment_index]
                        + alpha * centers[segment_index + 1]
                    )
                    interpolated_q = interpolate_joint_path(
                        path, segment_index + alpha
                    )
                    if (
                        np.any(interpolated_q < self.robot.low)
                        or np.any(interpolated_q > self.robot.high)
                    ):
                        raise ValueError(
                            f"{phase} interpolated joint-limit gate failed at segment "
                            f"{segment_index}, alpha={alpha:.2f}"
                        )
                    interpolated_gripper = self.command_gripper_m + (
                        (segment_index + alpha) / segment_count
                    ) * (target_gripper_m - self.command_gripper_m)
                    interpolated_positions, _ = tray_tcp_targets(
                        center,
                        self._detected_tray.yaw_rad,
                        float(grasp["side_offset_y_m"]),
                        self._tcp_height_m(),
                    )
                    self._set_work_base(base_pose_xy_yaw)
                    self._set_work_task_prediction(
                        self._predicted_task_center(phase, center),
                        self._detected_tray.yaw_rad,
                    )
                    check = self.robot._candidate_metrics(
                        interpolated_q,
                        interpolated_positions,
                        interpolated_gripper,
                        rotations,
                    )
                    interpolation_metrics.append(check)
                    if (
                        check["position_error_m"] > self.robot.POSITION_TOLERANCE_M
                        or check["orientation_error_rad"]
                        > self.robot.ORIENTATION_TOLERANCE_RAD
                        or check["gravity_torque_ratio"] > self.robot.TORQUE_LIMIT_RATIO
                        or check["forbidden_contacts"]
                    ):
                        raise ValueError(
                            f"{phase} interpolated Cartesian gate failed at segment "
                            f"{segment_index}, alpha={alpha:.2f}: "
                            f"position={check['position_error_m']:.6f}m, "
                            f"orientation={check['orientation_error_rad']:.6f}rad, "
                            f"gravity={check['gravity_torque_ratio']:.6f}, "
                            f"collision={bool(check['forbidden_contacts'])}"
                        )
        metrics = copy.deepcopy(segment_metrics[-1])
        metrics.update(
            {
                "phase": phase,
                "target_tcp_positions_m": positions.tolist(),
                "target_tcp_rotations": rotations.tolist(),
                "base_pose_xy_yaw": list(base_pose_xy_yaw),
                "pose_source": "rgbd_estimate_plus_encoder_odometry",
                "cartesian_segment_count": segment_count,
                "cartesian_max_step_m": cartesian_step_m,
                "joint_path_max_step_rad": float(np.max(joint_steps)),
                "joint_path_continuity_limit_rad": self.HELD_IK_MAX_JOINT_STEP_RAD,
                "interpolated_joint_limit_violation_rad": joint_limit_violation,
                "seed_policy": (
                    "continuous_seed_with_gradual_validated_high_torso_posture"
                    if phase == "raise_high"
                    else "crouched_arm_clearance_preapproach_then_high_branch_ik_seed_without_executed_waypoint"
                    if phase == "pickup_approach"
                    else "previous_command_continuation"
                ),
                "segment_position_error_max_m": max(
                    item["position_error_m"] for item in segment_metrics
                ),
                "segment_orientation_error_max_rad": max(
                    item["orientation_error_rad"] for item in segment_metrics
                ),
                "interpolated_position_error_max_m": max(
                    (item["position_error_m"] for item in interpolation_metrics),
                    default=0.0,
                ),
                "interpolated_orientation_error_max_rad": max(
                    (item["orientation_error_rad"] for item in interpolation_metrics),
                    default=0.0,
                ),
                "pickup_preapproach_offset_m": (
                    [0.05, 0.0, 0.11] if phase == "pickup_approach" else None
                ),
            }
        )
        self.ik_errors.append(metrics)
        motion = ArmMotion(
            phase=phase,
            duration_s=float(duration_s),
            start_joints_rad=self.command_rad.copy(),
            target_joints_rad=target,
            start_gripper_m=float(self.command_gripper_m),
            target_gripper_m=float(target_gripper_m),
            tray_center_world_m=tuple(float(v) for v in tray_center_world_m),
            base_pose_xy_yaw=tuple(float(v) for v in base_pose_xy_yaw),
            joint_path_rad=tuple(path),
            ik_metrics=metrics,
        )
        self.plan.append(motion)
        return motion

    def _joint_home_motion(self, phase: str) -> ArmMotion:
        """Validate an unloaded posture change using the executed cubic path."""

        preparing = phase == "prepare_pickup"
        navigating = phase == "prepare_navigation"
        target = (self.robot.crouched_pickup_posture() if preparing
                  else self._navigation_posture() if navigating else self.home_joints_rad)
        target_gripper = (
            float(self.config["grasp"]["open_gripper_m"])
            if preparing else self.home_gripper_m
        )
        base = tuple(self.drive.metrics(self.model, self.data)["wheel_odometry_pose_xy_yaw"])
        if phase == "prepare_navigation":
            # Surveyed initial CAD is used only to reject a preparation path;
            # the subsequent grasp still waits for a complete RGB-D estimate.
            center = np.asarray(self.config["tray_initial_pos_m"], dtype=float)
            yaw = 0.0
        elif preparing:
            if self._detected_tray is None:
                raise RuntimeError("pickup posture requires RGB-D perception")
            center = np.asarray(self._detected_tray.center_world_m, dtype=float)
            yaw = float(self._detected_tray.yaw_rad)
        else:
            center = np.array([
                *self.config["target"]["center_xy_m"],
                self.config["target"]["support_z_m"] + self.config["tray_half_extents_m"][2],
            ])
            yaw = float(self._detected_tray.yaw_rad)
        path = [self.command_rad.copy(), target.copy()]
        limit_error = joint_path_limit_violation(path, self.robot.low, self.robot.high)
        if limit_error > 1e-10:
            raise ValueError(f"{phase} joint-limit violation: {limit_error:.9f} rad")
        self._set_work_base(base)
        self._set_work_task_prediction(center, yaw)
        max_gravity_ratio = 0.0
        for alpha in np.linspace(0.0, 1.0, 101):
            self.robot._prepare(
                interpolate_joint_path(path, alpha),
                self.command_gripper_m + alpha * (target_gripper - self.command_gripper_m),
            )
            forbidden = self.robot._forbidden_contacts()
            ratio = float(np.max(np.abs(self.robot.work.qfrc_bias[self.robot.dadr]) / self.robot.force_limit))
            max_gravity_ratio = max(max_gravity_ratio, ratio)
            if forbidden or ratio > self.robot.TORQUE_LIMIT_RATIO:
                raise ValueError(f"{phase} free-joint path rejected at {alpha:.3f}: collision={forbidden}, gravity={ratio:.6f}")
        duration_key = "prepare_duration_s" if preparing or navigating else "return_duration_s"
        metrics = {
            "phase": phase,
            "pose_source": ("SDK_mobile_example_scene_validated" if navigating
                            else "task_crouched_pickup_posture" if preparing else "model_qpos0"),
            "collision_samples": 101,
            "gravity_torque_ratio": max_gravity_ratio,
            "interpolated_joint_limit_violation_rad": limit_error,
            "base_pose_xy_yaw": list(base),
        }
        self.ik_errors.append(metrics)
        motion = ArmMotion(
            phase, float(self.config["mission_cycle"][duration_key]),
            self.command_rad.copy(), target.copy(), self.command_gripper_m,
            target_gripper, tuple(center), base, tuple(path), metrics,
        )
        self.plan.append(motion)
        return motion

    def _phase_motion_spec(self, phase: str):
        estimate = np.asarray(self._detected_tray.center_world_m, dtype=float)
        base_now = tuple(self.drive.metrics(self.model, self.data)["wheel_odometry_pose_xy_yaw"])
        carry = self.config["carry"]
        target = self.config["target"]
        open_grip = float(self.config["grasp"]["open_gripper_m"])
        # Keep a small position-loop preload after contact.  This remains
        # inside the unchanged upstream native gripper limits and avoids
        # losing normal force as the mobile base accelerates.
        initial_closed_grip = min(
            self.robot.grip_high,
            float(self.config["grasp"]["closed_gripper_m"])
            + float(self.config["grasp"]["initial_preload_m"]),
        )
        loaded_closed_grip = min(
            self.robot.grip_high,
            float(self.config["grasp"]["closed_gripper_m"])
            + float(self.config["grasp"]["loaded_preload_m"]),
        )
        low = np.array(
            [base_now[0] + carry["relative_center_x_m"], base_now[1], carry["low_tray_center_z_m"]]
        )
        planned_xy = (
            np.asarray(self._planned_tray_center[:2], dtype=float)
            if self._planned_tray_center is not None
            else np.asarray([base_now[0] + carry["relative_center_x_m"], base_now[1]])
        )
        high = np.array(
            [planned_xy[0], planned_xy[1],
             target["support_z_m"] + self.config["tray_half_extents_m"][2] + self.config["transport_clearance_m"]]
        )
        seat = np.array(
            [*target["center_xy_m"], target["support_z_m"] + self.config["tray_half_extents_m"][2]]
        )
        specs = {
            "pickup_approach": (estimate + [0, 0, .10], base_now, 10.0, open_grip),
            "pickup_descend": (estimate, base_now, 4.0, open_grip),
            "grasp": (estimate, base_now, 2.5, initial_closed_grip),
            "grasp_hold": (estimate, base_now, 1.0, initial_closed_grip),
            "lift": (estimate + [0, 0, .07], base_now, 5.0, loaded_closed_grip),
            "retract_low": (low, base_now, 12.0, loaded_closed_grip),
            "raise_high": (high, base_now, 30.0, loaded_closed_grip),
            "seat": (seat, base_now, 6.0, loaded_closed_grip),
            "settle": (seat, base_now, 1.5, loaded_closed_grip),
            "release": (seat, base_now, 2.0, open_grip),
            "withdraw": (seat + [0, 0, .10], base_now, 4.0, open_grip),
        }
        return specs[phase]

    def _enter_phase(self, phase: str) -> None:
        self.phase = phase
        self.phase_started_s = float(self.data.time)
        self._drive_stopped_since_s = None
        self._motion_settled_since_s = None
        self._active_motion = None
        self._drive_start_pose = None
        self._drive_start_tray_center = None
        self._motion_clock_s = 0.0
        self._reference_finished_s = None
        self._pending_head_frame = None
        if phase == "grasp":
            # Tare in the final open-gripper pose.  Hardware uses the same
            # boundary immediately before closure.
            self._wrist_ft_tare = {
                side: self._read_wrist_ft(side) for side in ("left", "right")
            }
            pregrasp = self._contact_wrist_verifier(self)
            observations = pregrasp["wrist_observations"]
            pregrasp_vision_valid = all(
                item["expected_grasp_in_frame"]
                and item["depth_in_sensor_range"]
                and item["roi_valid_depth_fraction"] >= 0.5
                and item["median_depth_m"] is not None
                and item["expected_depth_m"] is not None
                and 0.0 <= item["expected_depth_m"] - item["median_depth_m"]
                <= float(self.config["grasp"].get(
                    "pregrasp_depth_residual_max_m",
                    self.PREGRASP_DEPTH_RESIDUAL_MAX_M,
                ))
                and (item["depth_cad_rim"]["passed"]
                     if self.config.get("perception", {}).get("estimator") == "depth_cad"
                     else item["blue_rim_pixel_fraction"] >= 0.05)
                for item in observations.values()
            )
            self._pregrasp_wrist_verification = {
                "passed": bool(pregrasp_vision_valid),
                "timestamp_sim_s": float(self.data.time),
                "gripper_state": "open_before_closure",
                "wrist_observations": observations,
                "source": ("wrist_depth_CAD_rim_plus_expected_depth"
                           if self.config.get("perception", {}).get("estimator") == "depth_cad"
                           else "wrist_rgbd_expected_depth_plus_blue_rim"),
                "depth_residual_gate_provenance": (
                    "synthetic CAD and camera-noise assumption; replace with "
                    "measured D405 registration calibration"
                ),
            }
            self.perception_history.append({
                "timestamp_sim_s": float(self.data.time),
                "pregrasp_wrist_verification": copy.deepcopy(
                    self._pregrasp_wrist_verification
                ),
            })
            if not pregrasp_vision_valid:
                self._fail(
                    "pregrasp_wrist_verification_failed",
                    repr(self._pregrasp_wrist_verification),
                )
                return
        if phase in self.POSTURE_PHASES:
            self._active_motion = self._joint_home_motion(phase)
        elif phase in self.ARM_PHASES:
            center, base, duration, gripper = self._phase_motion_spec(phase)
            self._active_motion = self._solve_motion(phase, center, base, duration, gripper)
        elif phase in {"drive_loaded", "drive_high"} and self._planned_tray_center is not None:
            self._drive_start_pose = np.asarray(
                self.drive.metrics(self.model, self.data)["wheel_odometry_pose_xy_yaw"],
                dtype=float,
            )
            self._drive_start_tray_center = self._planned_tray_center.copy()

    def _advance_phase(self) -> None:
        sequence = (
            "home_start", "prepare_navigation",
            "drive_approach", "observe_close", "prepare_pickup", "observe_pickup",
            "pickup_approach", "observe_pregrasp", "pickup_descend",
            "grasp", "grasp_hold", "lift", "retract_low", "drive_loaded",
            "raise_high", "drive_high", "seat", "settle", "release",
            "withdraw", "drive_return", "return_home", "home_settle",
        )
        index = sequence.index(self.phase)
        self._enter_phase(sequence[min(index + 1, len(sequence) - 1)])

    def _contact_wrist_verifier(self, _session: "MobileSession") -> dict:
        proprioception = self._wrist_proprioception()
        open_reference = float(self.config["grasp"]["open_gripper_m"])
        force_threshold = max(
            0.5, float(self.config["thresholds"]["min_pad_force_n"])
        )
        proprio_valid = all(
            reading["force_delta_norm_n"] >= force_threshold
            and reading["gripper_actuator_force_n"]
            >= float(self.config["thresholds"]["min_pad_force_n"])
            # A contacted rim can stop one native finger before the requested
            # position.  Require measured closure travel plus load, rather
            # than pretending that exact setpoint tracking proves a grasp.
            and float(np.mean(reading["finger_position_m"]))
            >= open_reference + 0.012
            for reading in proprioception.values()
        )
        wrist_observations = {}
        grasp_by_side = {
            grasp.side: np.asarray(grasp.position_world_m, dtype=float)
            for grasp in self._detected_tray.grasps
        }
        for side in ("left", "right"):
            cfg = self.config["sensors"][f"{side}_wrist"]
            width, height = (int(value) for value in cfg["resolution_px"])
            calibration = self._controller_camera_calibration(
                cfg["camera_name"], width, height
            )
            camera = self._wrist_cameras.get(side)
            if camera is None:
                camera = NamedRGBDCamera(
                    self.model,
                    cfg["camera_name"],
                    width=width,
                    height=height,
                    calibration=calibration,
                    noise=self._camera_noise(cfg),
                    depth_min_m=float(cfg["range_m"][0]),
                    depth_max_m=float(cfg["range_m"][1]),
                    sensor_profile=str(cfg["model"]),
                    distortion_model=str(cfg["distortion_model"]),
                    rectified_aligned=bool(cfg["rgb_depth_aligned"]),
                )
                self._wrist_cameras[side] = camera
            else:
                camera.calibration_override = calibration
            frame = camera.capture(self.data)
            self._validate_rgbd_frame(frame, cfg)
            frame = self._controller_frame(self._apply_sensor_range(frame, cfg))
            self.sensor_frames.append(frame)
            world_point = np.append(grasp_by_side[side], 1.0)
            optical = np.linalg.inv(frame.T_world_optical) @ world_point
            if optical[2] > 0:
                pixel = frame.K @ optical[:3]
                uv = pixel[:2] / pixel[2]
            else:
                uv = np.array([np.nan, np.nan])
            in_frame = bool(
                np.all(np.isfinite(uv))
                and 0 <= uv[0] < width
                and 0 <= uv[1] < height
            )
            roi_valid_fraction = 0.0
            measured_depth_m = None
            expected_depth_m = float(optical[2]) if optical[2] > 0 else None
            depth_residual_m = None
            blue_rim_fraction = 0.0
            if in_frame:
                col, row = int(round(uv[0])), int(round(uv[1]))
                col = int(np.clip(col, 0, width - 1))
                row = int(np.clip(row, 0, height - 1))
                radius = 5
                roi = frame.depth_m[
                    max(0, row - radius): min(height, row + radius + 1),
                    max(0, col - radius): min(width, col + radius + 1),
                ]
                valid = np.isfinite(roi) & (roi > 0)
                roi_valid_fraction = float(np.mean(valid)) if valid.size else 0.0
                if np.any(valid):
                    measured_depth_m = float(np.median(roi[valid]))
                    depth_residual_m = abs(measured_depth_m - expected_depth_m)
                # The finger mesh can occlude the exact projected contact;
                # inspect a local ring large enough to include the adjacent rim.
                color_radius = 24
                rgb_roi = frame.rgb[
                    max(0, row - color_radius): min(height, row + color_radius + 1),
                    max(0, col - color_radius): min(width, col + color_radius + 1),
                ]
                if rgb_roi.size:
                    blue = (
                        (rgb_roi[..., 2].astype(int) - rgb_roi[..., 0].astype(int) >= 20)
                        & (rgb_roi[..., 2].astype(int) - rgb_roi[..., 1].astype(int) >= 10)
                    )
                    blue_rim_fraction = float(np.mean(blue))
            range_min, range_max = (float(value) for value in cfg["range_m"])
            in_range = bool(
                measured_depth_m is not None
                and range_min <= measured_depth_m <= range_max
            )
            wrist_observations[side] = {
                "camera_name": cfg["camera_name"],
                "timestamp_sim_s": frame.timestamp_sim_s,
                "expected_grasp_pixel_uv": uv.tolist(),
                "expected_grasp_in_frame": in_frame,
                "roi_valid_depth_fraction": roi_valid_fraction,
                "median_depth_m": measured_depth_m,
                "expected_depth_m": expected_depth_m,
                "depth_residual_m": depth_residual_m,
                "blue_rim_pixel_fraction": blue_rim_fraction,
                "sensor_range_m": [range_min, range_max],
                "depth_in_sensor_range": in_range,
                "calibration_input": "encoder_odometry_plus_joint_encoder_fk",
            }
            from .perception import confirm_grasp_rim_from_depth
            wrist_observations[side]["depth_cad_rim"] = confirm_grasp_rim_from_depth(
                frame.depth_m, frame.K, frame.T_world_optical, grasp_by_side[side],
                self._detected_tray.center_world_m, self._detected_tray.yaw_rad,
                cad=TrayCAD(half_extents_m=tuple(self.config["tray_half_extents_m"]),
                            grasp_height_m=self._tcp_height_m()),
            )
        closed_depth_valid = all(
            observation["expected_grasp_in_frame"]
            and observation["depth_in_sensor_range"]
            and observation["roi_valid_depth_fraction"] >= 0.5
            # The RGB-D ray sees the near face of the native rim before the
            # planned TCP centre.  Accept only a CAD-bounded, nearer return.
            and observation["median_depth_m"] is not None
            and observation["expected_depth_m"] is not None
            and 0.0 <= (
                observation["expected_depth_m"] - observation["median_depth_m"]
            ) <= 0.05
            for observation in wrist_observations.values()
        )
        vision_valid = bool(
            self._pregrasp_wrist_verification
            and self._pregrasp_wrist_verification.get("passed")
            and closed_depth_valid
        )
        return {
            "passed": bool(proprio_valid and vision_valid),
            "wrist_ft_and_gripper_proprioception_passed": bool(proprio_valid),
            "wrist_rgbd_passed": bool(vision_valid),
            "pregrasp_wrist_verification": copy.deepcopy(
                self._pregrasp_wrist_verification
            ),
            "proprioception": proprioception,
            "wrist_observations": wrist_observations,
            "source": "wrist_rgbd_plus_six_axis_ft_plus_gripper_proprioception",
            "contact_geometry_ids_used_by_controller": False,
            "hardware_replacement": "measured wrist RGB-D/FT drivers and gripper current",
            "sim_gripper_effort_provenance": (
                "MuJoCo actuator force; hardware adapter must supply calibrated current/effort"
            ),
        }

    def _fail(self, reason: str, detail: str) -> None:
        self._mission_failure = {
            "reason": reason,
            "detail": detail,
            "time_s": float(self.data.time),
        }
        self.evaluator.finalize(float(self.data.time), reason)
        self.finished = True

    def _seat_sensor_verification(self) -> dict:
        """Confirm support height and hand-load transfer before opening fingers."""

        odom = self.drive.metrics(self.model, self.data)["wheel_odometry_pose_xy_yaw"]
        self._set_work_base(odom)
        self.robot.work.qpos[self.robot.qadr] = self.data.qpos[self.robot.qadr]
        mujoco.mj_forward(self.model, self.robot.work)
        tcp_z = float(np.mean(self.robot.work.site_xpos[self.robot.tcp, 2]))
        inferred_center_z = tcp_z - self._tcp_height_m()
        expected_center_z = (
            float(self.config["target"]["support_z_m"])
            + float(self.config["tray_half_extents_m"][2])
        )
        vertical_load = self._world_vertical_hand_load()
        load_n = vertical_load["total_upward_support_component_n"]
        baseline = self._preseat_hand_load_n
        task_mass_kg = sum(
            float(self.model.body(name).mass[0])
            for name in ("tray", "payload_0", "payload_1", "payload_2")
        )
        expected_weight_n = task_mass_kg * float(np.linalg.norm(self.model.opt.gravity))
        load_transferred, minimum_baseline_n = self._load_transfer_valid(
            baseline, load_n, expected_weight_n
        )
        result = {
            "passed": bool(abs(inferred_center_z - expected_center_z) <= 0.015 and load_transferred),
            "inferred_tray_center_z_m": inferred_center_z,
            "expected_tray_center_z_m": expected_center_z,
            "height_error_m": abs(inferred_center_z - expected_center_z),
            "preseat_hand_load_n": baseline,
            "settled_hand_load_n": load_n,
            "expected_task_weight_n": expected_weight_n,
            "minimum_preseat_hand_load_n": minimum_baseline_n,
            "minimum_preseat_load_fraction": self.PRESEAT_EXPECTED_LOAD_FRACTION_MIN,
            "load_values_finite": bool(
                baseline is not None
                and np.isfinite(baseline)
                and np.isfinite(load_n)
                and np.isfinite(expected_weight_n)
            ),
            "world_vertical_hand_load": vertical_load,
            "load_transfer_ratio_max": 0.8,
            "source": "wheel_odometry_plus_joint_encoder_fk_plus_wrist_ft",
            "load_gate_provenance": (
                "synthetic 25% expected task-weight observability threshold; replace "
                "with hardware FT noise and payload identification validation"
            ),
            "simulator_contact_or_object_pose_used": False,
        }
        return result

    def _audit_external_state(self) -> None:
        if not np.array_equal(self.data.qpos, self._expected_qpos):
            self.audit["runtime_state_overwrite"] = True
        if not np.array_equal(self.data.qvel, self._expected_qvel):
            self.audit["runtime_state_overwrite"] = True
        if np.any(self.data.xfrc_applied) or np.any(self.data.qfrc_applied):
            self.audit["runtime_state_overwrite"] = True
        if any(not np.array_equal(current, expected) for current, expected in (
            (self.data.eq_active, self._expected_eq_active),
            (self.model.eq_data, self._expected_eq_data),
            (self.data.mocap_pos, self._expected_mocap_pos),
            (self.data.mocap_quat, self._expected_mocap_quat),
        )):
            self.audit["runtime_state_overwrite"] = True

    def _arm_reference(self) -> tuple[np.ndarray, float, np.ndarray]:
        motion = self._active_motion
        if motion is None:
            return self.command_rad.copy(), self.command_gripper_m, np.zeros(20)
        elapsed = self._motion_clock_s
        raw = np.clip(elapsed / motion.duration_s, 0.0, 1.0)
        segment_count = len(motion.joint_path_rad) - 1
        # Time-scale every path once.  Applying smoothstep to each Cartesian
        # segment would repeatedly stop and restart the carried tray.
        progress = smoothstep(raw)
        scaled = min(progress * segment_count, float(segment_count))
        joints = interpolate_joint_path(motion.joint_path_rad, scaled)
        gripper = motion.start_gripper_m + smoothstep(raw) * (
            motion.target_gripper_m - motion.start_gripper_m
        )
        velocity = (joints - self.command_rad) / float(self.config["control_dt_s"])
        return joints, float(gripper), velocity

    def _drive_control(self) -> None:
        if self.phase in self.DRIVE_PHASE_TARGET:
            target = self._drive_target()
            self.drive.update(
                self.model, self.data, target, float(self.config["control_dt_s"]),
                speed_scale=self.cooperation.speed_scale,
            )
        else:
            self.drive.stop(
                self.model, self.data, float(self.config["control_dt_s"])
            )

    def _feedback_kinematics(self, scratch, joints):
        """Use encoders and wheel odometry; no task pose or live base TF."""
        odom = self.drive.metrics(self.model, self.data)["wheel_odometry_pose_xy_yaw"]
        x, y, yaw = map(float, odom)
        base = scratch.joint("world_j").qpos
        base[:3] = (x, y, 0)
        base[3:7] = (math.cos(yaw / 2), 0, 0, math.sin(yaw / 2))
        scratch.qpos[self.robot.qadr] = joints
        mujoco.mj_kinematics(self.model, scratch)
        right, left = self.robot.tcp
        rotation = scratch.site_xmat[right].reshape(3, 3)
        relative_position = rotation.T @ (scratch.site_xpos[left] - scratch.site_xpos[right])
        relative_rotation = rotation.T @ scratch.site_xmat[left].reshape(3, 3)
        return relative_position, relative_rotation

    def _update_cooperation(self):
        now = float(self.data.time)
        if now + 1e-9 < self._next_cooperation_s:
            return
        self._next_cooperation_s = now + self.cooperation.config.sample_period_s
        active = self.phase in self.COOPERATIVE_PHASES
        if not active:
            self.cooperation.update(now, 0.0, 0.0, [0.0, 0.0], active=False)
            return
        measured_p, measured_r = self._feedback_kinematics(self._feedback_fk, self.data.qpos[self.robot.qadr])
        reference_p, reference_r = self._feedback_kinematics(self._reference_fk, self.command_rad)
        position_error = float(np.linalg.norm(measured_p - reference_p))
        orientation_error = float(np.linalg.norm(Rotation.from_matrix(reference_r.T @ measured_r).as_rotvec()))
        loads = []
        for side in ("left", "right"):
            rotation = self._feedback_fk.site(f"{side}_ft_frame").xmat.reshape(3, 3)
            raw_world = rotation @ np.asarray(self._read_wrist_ft(side)["force_n"])
            mount = self.model.body(f"{side}_ft_mount").id
            tool_mass = sum(float(self.model.body_mass[index]) for index in range(self.model.nbody)
                            if self.robot._is_descendant(index, mount))
            # The flange force sensor is positive opposite gravity at rest.
            # Remove model tool weight in world axes rather than subtracting
            # a local-frame tare taken at a different wrist orientation.
            loads.append(float(raw_world[2] - tool_mass * abs(self.model.opt.gravity[2])))
        decision = self.cooperation.update(now, position_error, orientation_error, loads, active=True)
        self.cooperation_history.append(copy.deepcopy(decision))
        if decision["abort"]:
            self._fail("cooperative_transport_fault", json.dumps(decision))

    def _drive_target(self) -> list[float]:
        key = self.DRIVE_PHASE_TARGET[self.phase]
        target = list(self.config["base"][key])
        if self.phase == "drive_high":
            relative_x = float(self.config["carry"]["relative_center_x_m"])
            target[0] = float(self.config["target"]["center_xy_m"][0]) - relative_x
            target[1] = float(self.config["target"]["center_xy_m"][1])
        return target

    def _controller_observation(self) -> dict:
        """Return hardware-equivalent proprioception at the command tick."""

        drive = self.drive.metrics(self.model, self.data)
        wheel_angles = self.drive.read_sim_sensors(self.model, self.data)
        return {
            "timestamp_sim_s": float(self.data.time),
            "upper_body_joint_position_rad": self.data.qpos[self.robot.qadr].tolist(),
            "upper_body_joint_velocity_rad_s": self.data.qvel[self.robot.dadr].tolist(),
            "head_joint_position_rad": self.data.qpos[self._head_qadr].tolist(),
            "head_joint_velocity_rad_s": self.data.qvel[self._head_dadr].tolist(),
            "wheel_encoder_angle_rad": list(wheel_angles.wheel_angle_rad),
            "wheel_encoder_velocity_rad_s": self.data.qvel[self.drive.dof_ids].tolist(),
            "wheel_odometry_pose_xy_yaw": drive["wheel_odometry_pose_xy_yaw"],
            "estimated_linear_speed_m_s": drive["estimated_linear_speed_m_s"],
            "estimated_yaw_rate_rad_s": drive["estimated_yaw_rate_rad_s"],
            "imu_yaw_rad": drive["imu_yaw_rad"],
            "gripper_position_m": {
                letter: [
                    float(self.data.joint(f"gripper_finger_{letter}{finger}").qpos[0])
                    for finger in (1, 2)
                ]
                for letter in ("l", "r")
            },
            "wrist_wrench_local": {
                side: self._read_wrist_ft(side) for side in ("left", "right")
            },
            "frame_conventions": {
                "odometry": "world-aligned wheel-encoder odometry x/y/yaw",
                "wrist_wrench": "named wrist FT site local frame",
            },
            "ground_truth_body_pose_or_contact_included": False,
        }

    def _drive_arrived(self) -> bool:
        target = np.asarray(self._drive_target(), dtype=float)
        metrics = self.drive.metrics(self.model, self.data)
        odom = np.asarray(metrics["wheel_odometry_pose_xy_yaw"], dtype=float)
        xy_error = float(np.linalg.norm(target[:2] - odom[:2]))
        yaw_error = abs((target[2] - odom[2] + math.pi) % (2 * math.pi) - math.pi)
        stopped = bool(metrics["estimated_is_stopped"])
        within = (
            # The wheel controller brakes at its configured deadband.  Allow
            # one millimetre of encoder/step quantisation at that boundary,
            # while still requiring the estimated base to remain stopped.
            xy_error <= self.drive.config.position_tolerance_m + 0.001
            and yaw_error <= self.drive.config.yaw_tolerance_rad
            and stopped
        )
        if not within:
            self._drive_stopped_since_s = None
            return False
        if self._drive_stopped_since_s is None:
            self._drive_stopped_since_s = float(self.data.time)
        return float(self.data.time) - self._drive_stopped_since_s >= 0.5

    def _check_transition(self) -> None:
        if self.finished:
            return
        try:
            if self.phase == "home_start":
                if float(self.data.time) - self.phase_started_s >= self.config["mission_cycle"]["initial_hold_s"]:
                    self._advance_phase()
                return
            if self.phase in self.DRIVE_PHASE_TARGET and self._drive_arrived():
                if (
                    self._drive_start_pose is not None
                    and self._drive_start_tray_center is not None
                ):
                    current_pose = np.asarray(
                        self.drive.metrics(self.model, self.data)[
                            "wheel_odometry_pose_xy_yaw"
                        ],
                        dtype=float,
                    )
                    start_xy = self._drive_start_pose[:2]
                    start_yaw = float(self._drive_start_pose[2])
                    current_yaw = float(current_pose[2])
                    relative_xy = self._drive_start_tray_center[:2] - start_xy
                    relative_xy = (
                        Rotation.from_euler("z", -start_yaw).as_matrix()[:2, :2]
                        @ relative_xy
                    )
                    moved_xy = (
                        current_pose[:2]
                        + Rotation.from_euler("z", current_yaw).as_matrix()[:2, :2]
                        @ relative_xy
                    )
                    self._planned_tray_center = self._drive_start_tray_center.copy()
                    self._planned_tray_center[:2] = moved_xy
                self._advance_phase()
                return
            if self.phase in self.ARM_PHASES and self._active_motion is not None:
                if self._motion_clock_s + 1e-9 >= self._active_motion.duration_s:
                    if self._reference_finished_s is None:
                        self._reference_finished_s = float(self.data.time)
                    tracking_error = float(np.max(np.abs(
                        self.data.qpos[self.robot.qadr]
                        - self._active_motion.target_joints_rad
                    )))
                    joint_speed = float(np.max(np.abs(self.data.qvel[self.robot.dadr])))
                    if tracking_error > 0.01 or joint_speed > 0.02:
                        self._motion_settled_since_s = None
                        if (
                            float(self.data.time) - self._reference_finished_s > 4.0
                        ):
                            self._fail(
                                "arm_tracking_timeout",
                                f"{self.phase}: error={tracking_error:.6f}rad, "
                                f"speed={joint_speed:.6f}rad/s",
                            )
                        return
                    if self._motion_settled_since_s is None:
                        self._motion_settled_since_s = float(self.data.time)
                        return
                    if float(self.data.time) - self._motion_settled_since_s < 0.15:
                        return
                    if self.phase == "grasp_hold":
                        verification = self._wrist_verifier(self)
                        self.perception_history.append(
                            {"timestamp_sim_s": float(self.data.time), "wrist_verification": verification}
                        )
                        if not bool(verification.get("passed")):
                            self._fail("wrist_verification_failed", repr(verification))
                            return
                    if self.phase == "raise_high":
                        self._preseat_hand_load_n = self._world_vertical_hand_load()[
                            "total_upward_support_component_n"
                        ]
                    if self.phase == "settle":
                        verification = self._seat_sensor_verification()
                        self.perception_history.append({
                            "timestamp_sim_s": float(self.data.time),
                            "seat_verification": verification,
                        })
                        if not verification["passed"]:
                            self._fail("seat_verification_failed", repr(verification))
                            return
                    if self.phase not in self.POSTURE_PHASES:
                        self._planned_tray_center = np.asarray(
                            self._active_motion.tray_center_world_m, dtype=float
                        )
                    self._advance_phase()
        except (ValueError, RuntimeError) as exc:
            self._fail("ik_planning_failed", str(exc))

    def _check_phase_timeout(self) -> None:
        if self.phase in self.OBSERVATION_PHASES:
            if float(self.data.time) - self.phase_started_s > 2.0:
                self._fail("rgbd_delivery_timeout", "stationary RGB-D delivery exceeded 2 s")
            return
        if self.phase == "home_settle" and not self.finished:
            if float(self.data.time) - self.phase_started_s > 4.0 + self.home_reference["settle_hold_s"]:
                self._fail("home_settle_timeout", "model qpos0 return and stationary base were not verified")
            return
        if self.phase not in self.DRIVE_PHASE_TARGET or self.finished:
            return
        timeout_s = {
            "drive_approach": 20.0,
            "drive_loaded": 15.0,
            "drive_high": 20.0,
            "drive_return": float(self.config["mission_cycle"]["return_drive_timeout_s"]),
        }[self.phase]
        if float(self.data.time) - self.phase_started_s > timeout_s:
            self._fail("phase_timeout", f"{self.phase} exceeded {timeout_s:.1f} s")

    def step(self, n_steps=1):
        for _ in range(n_steps):
            if self.finished:
                break
            if float(self.data.time) > float(self.config.get("max_episode_time_s", 300.0)):
                self._fail("mission_timeout", "configured episode time limit exceeded")
                break
            self._audit_external_state()
            if self.phase in self.OBSERVATION_PHASES:
                try:
                    observation = self._capture_and_estimate()
                except (PerceptionError, ValueError, RuntimeError) as exc:
                    self._fail("perception_failed", str(exc))
                    self._record(force_log=True)
                    break
                if observation is not None:
                    self._detected_tray = observation
                    self._planned_tray_center = np.asarray(observation.center_world_m, dtype=float)
                    try:
                        self._advance_phase()
                    except (ValueError, RuntimeError) as exc:
                        self._fail("ik_planning_failed", str(exc))
                        self._record(force_log=True)
                        break
            if self.data.time + 1e-9 >= self._next_control_s:
                self._update_cooperation()
                if self.finished:
                    break
                if self._active_motion is not None:
                    self._motion_clock_s += float(self.config["control_dt_s"]) * self.cooperation.speed_scale
                previous = self.command_rad.copy()
                self.command_rad, self.command_gripper_m, velocity = self._arm_reference()
                self._held_velocity_rad_s = velocity
                self._set_controls(self.command_rad, self.command_gripper_m, velocity)
                self._drive_control()
                self._next_control_s += float(self.config["control_dt_s"])
                self.command_history.append(
                    {
                        "time_s": float(self.data.time),
                        "phase": self.phase,
                        "upper_body_position_reference_rad": self.command_rad.tolist(),
                        "upper_body_velocity_reference_rad_s": velocity.tolist(),
                        "head_position_reference_rad": self.home_head_rad.tolist(),
                        "gripper_position_reference_m": self.command_gripper_m,
                        "wheel_velocity_reference_rad_s": self.drive.commands(self.model),
                        "observation": self._controller_observation(),
                        "controller_inputs": {
                            "tray_pose": "rgbd_estimate" if self._detected_tray is not None else "not_yet_observed",
                            "base_pose": "wheel_encoder_odometry",
                            "simulator_object_pose_used": False,
                        },
                        "cooperative_transport": copy.deepcopy(self.cooperation.last),
                    }
                )
            mujoco.mj_step(self.model, self.data)
            self._expected_qpos = self.data.qpos.copy()
            self._expected_qvel = self.data.qvel.copy()
            error = float(np.max(np.abs(self.data.qpos[self.robot.qadr] - self.command_rad)))
            self.max_tracking_error_rad = max(self.max_tracking_error_rad, error)
            self.max_joint_torque_nm = max(
                self.max_joint_torque_nm,
                float(np.max(np.abs(self.data.actuator_force[self.robot.act]))),
            )
            self.max_finger_force_n = max(
                self.max_finger_force_n,
                float(np.max(np.abs(self.data.actuator_force[self.robot.grip_act]))),
            )
            drive_metrics = self.drive.metrics(self.model, self.data)
            if not drive_metrics["within_tilt_limit"]:
                self._fail("base_tilt_limit", repr(drive_metrics["base_tilt_rad"]))
            self._capture_monitors()
            self._check_phase_timeout()
            self._check_transition()
            self._record()
        return self.snapshot()

    def snapshot(self):
        state = super().snapshot()
        drive = self.drive.metrics(self.model, self.data)
        state["robot_state"] = {
            "joint_names": list(self.robot.names),
            "position_rad": self.data.qpos[self.robot.qadr].tolist(),
            "velocity_rad_s": self.data.qvel[self.robot.dadr].tolist(),
            "head_position_rad": self.data.qpos[self._head_qadr].tolist(),
            "head_velocity_rad_s": self.data.qvel[self._head_dadr].tolist(),
            "gripper_position_m": self.data.qpos[self.robot.grip_qadr].tolist(),
            "base_pose_xy_yaw": drive["ground_truth_pose_xy_yaw"],
            "base_linear_speed_m_s": float(np.linalg.norm(drive["base_linear_velocity_m_s"])),
            "base_yaw_rate_rad_s": float(drive["base_angular_velocity_rad_s"][2]),
            "role": "independent_evaluation_only; ground_truth_base_pose_excluded_from_controller",
        }
        state["controller"] = "RGB-D + encoder odometry + IK position + wheel velocity"
        state["mission"] = {
            "phase": self.phase,
            "phase_elapsed_s": float(self.data.time) - self.phase_started_s,
            "failure": copy.deepcopy(self._mission_failure),
            "perception": copy.deepcopy(self.perception_history[-1]) if self.perception_history else None,
            "command": copy.deepcopy(self.command_history[-1]) if self.command_history else None,
            "drive": drive,
            "home_reference": copy.deepcopy(self.home_reference),
            "cooperation": copy.deepcopy(self.cooperation.last),
            "cooperation_summary": copy.deepcopy(self.cooperation.summary),
            "monitoring": {
                "frame_count": len(self.monitor_frames),
                "sample_period_s": self.config.get("monitoring", {}).get("period_s"),
                "source": "recorded_named_RGBD_cameras_not_external_view",
            },
            "wrist_proprioception": self._wrist_proprioception(),
            "data_contract": {
                "head_observation": "registered RGB uint8 + metric axial depth + K + T_world_optical",
                "base_observation": "four wheel encoder angles; optional IMU yaw",
                "wrist_observation": "replaceable RGB-D/force verification hook",
                "upper_body_command": "20 joint position/velocity references in SI units",
                "base_command": "four wheel velocity references in rad/s",
                "ground_truth_policy": "evaluation only; excluded from controller inputs",
            },
        }
        if self._detected_tray is not None:
            state["mission"]["detected_tray"] = self._detected_tray.to_dict()
        return state

    def _record(self, force_log=False):
        super()._record(force_log)
        if self.evaluator.result()["success"]:
            self.phase = "complete"

    def export_sim2real_dataset(self, output_directory: str | Path) -> Path:
        """Export controller inputs/outputs without simulator object ground truth."""

        output = Path(output_directory)
        observations = output / "observations"
        observations.mkdir(parents=True, exist_ok=True)
        frame_index = []
        for index, frame in enumerate(self.sensor_frames):
            camera_name = str(frame.metadata.get("camera_name", "rgbd")).replace("/", "_")
            relative = Path("observations") / f"{index:04d}_{camera_name}.npz"
            frame.save_npz(output / relative)
            frame_index.append(
                {
                    "index": index,
                    "path": relative.as_posix(),
                    "camera_name": camera_name,
                    "timestamp_sim_s": frame.timestamp_sim_s,
                }
            )
        (output / "frames.json").write_text(
            json.dumps(frame_index, indent=2, sort_keys=True) + "\n"
        )
        with (output / "perception.jsonl").open("w") as stream:
            for item in self.perception_history:
                stream.write(json.dumps(item, sort_keys=True) + "\n")
        with (output / "commands.jsonl").open("w") as stream:
            for item in self.command_history:
                stream.write(json.dumps(item, sort_keys=True) + "\n")
        schema = {
            "schema_version": 1,
            "purpose": "sim-to-real controller interface replay",
            "home_reference": {"source": "fixed upstream model qpos0", **copy.deepcopy(self.home_reference)},
            "observations": {
                "rgb": "uint8 HxWx3 RGB",
                "depth_m": "float metric optical-axis depth",
                "K": "3x3 pinhole intrinsics",
                "T_world_optical": "4x4 extrinsics from odometry and joint FK",
                "wheel_encoder_angle_rad": "four wheel angles",
                "wrist_force_n": "three-axis local sensor-frame force",
                "wrist_torque_nm": "three-axis local sensor-frame torque",
                "gripper_position_m": "two finger joint positions per hand",
            },
            "commands": {
                "upper_body_position_reference_rad": 20,
                "upper_body_velocity_reference_rad_s": 20,
                "head_position_reference_rad": 2,
                "gripper_position_reference_m": 1,
                "wheel_velocity_reference_rad_s": 4,
                "observation_alignment": "same pre-mj_step controller tick as command",
            },
            "array_order": {
                "upper_body_joint_position_and_velocity": list(self.robot.names),
                "head_joint_position_and_velocity": list(self._head_joint_names),
                "wheel_encoder_and_velocity": list(WHEEL_JOINT_NAMES),
                "wheel_command": [
                    "front_right_wheel_act", "front_left_wheel_act",
                    "rear_right_wheel_act", "rear_left_wheel_act",
                ],
                "gripper_command": (
                    "one shared position reference applied to right and left native grippers"
                ),
            },
            "control_timing": {
                "physics_dt_s": float(self.config["physics_dt_s"]),
                "control_dt_s": float(self.config["control_dt_s"]),
                "record_dt_s": float(self.config["record_dt_s"]),
                "source": "simulation configuration; replace with measured SDK timing",
            },
            "limits": {
                "upper_body_joint_lower_rad": self.robot.low.tolist(),
                "upper_body_joint_upper_rad": self.robot.high.tolist(),
                "head_joint_lower_rad": self.model.jnt_range[
                    self._head_joint_ids, 0
                ].tolist(),
                "head_joint_upper_rad": self.model.jnt_range[
                    self._head_joint_ids, 1
                ].tolist(),
                "gripper_position_m": [self.robot.grip_low, self.robot.grip_high],
            },
            "wrist_force_torque": {
                "left_frame": "left_ft_frame",
                "right_frame": "right_ft_frame",
                "measurement_boundary": "flange fixed-child boundary including downstream tool mass",
                "source": "MuJoCo force/torque sensors; hardware adapter uses RB-Y1 FT freshness and tare",
            },
            "controller_ground_truth_policy": (
                "task-object qpos and live free-base cam_xpos are excluded"
            ),
            "hardware_adapters": {
                "head_rgbd": "registered RGB/depth + measured K and robot-to-camera extrinsic",
                "wrist_rgbd": "registered RGB/depth + measured K and flange extrinsic",
                "base": "four wheel encoders and optional IMU yaw",
                "wrench": "left/right wrist six-axis force-torque",
                "actuation": "20 upper-body position loops, two grippers, four wheel velocity loops",
            },
        }
        (output / "schema.json").write_text(
            json.dumps(schema, indent=2, sort_keys=True) + "\n"
        )
        monitor_index = []
        for index, frame in enumerate(self.monitor_frames):
            relative = Path("monitor-observations") / f"{index:05d}_{frame.metadata['camera_name']}.npz"
            path = output / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            # Monitor exports retain metric depth, not just display PNGs.
            # Ideal renderer channels are omitted from this auxiliary stream.
            np.savez_compressed(path, rgb=frame.rgb, depth_m=frame.depth_m,
                                K=frame.K, T_world_optical=frame.T_world_optical,
                                timestamp_sim_s=np.array(frame.timestamp_sim_s),
                                metadata_json=np.array(json.dumps(frame.metadata, sort_keys=True)))
            monitor_index.append({"path": relative.as_posix(), "camera_name": frame.metadata["camera_name"],
                                  "timestamp_sim_s": frame.timestamp_sim_s, "role": "monitor_only"})
        (output / "monitor-frames.json").write_text(json.dumps(monitor_index, indent=2) + "\n")
        with (output / "cooperation.jsonl").open("w") as stream:
            for item in self.cooperation_history:
                stream.write(json.dumps(item, allow_nan=False) + "\n")
        return output

    def close(self) -> None:
        if self._camera is not None:
            self._camera.close()
            self._camera = None
        for camera in self._wrist_cameras.values():
            camera.close()
        self._wrist_cameras = {}
        for camera in self._monitor_cameras.values():
            camera.close()
        self._monitor_cameras = {}
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None

