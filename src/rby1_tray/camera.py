"""Calibrated RGB-D capture adapter for a named MuJoCo camera."""

from __future__ import annotations

from dataclasses import dataclass, field
import json
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from .perception import validate_calibration


@dataclass(frozen=True)
class CameraCalibration:
    K: np.ndarray
    T_world_optical: np.ndarray
    source: str = "synthetic_model"

    def __post_init__(self) -> None:
        validate_calibration(self.K, self.T_world_optical)
        if self.source not in {"synthetic_model", "measured"}:
            raise ValueError("calibration source must be synthetic_model or measured")


@dataclass(frozen=True)
class RGBDNoise:
    rgb_std: float = 0.0
    depth_std_m: float = 0.0
    dropout_probability: float = 0.0
    seed: int = 0
    source: str = "synthetic_assumption"

    def __post_init__(self) -> None:
        if self.rgb_std < 0 or self.depth_std_m < 0:
            raise ValueError("noise standard deviations cannot be negative")
        if not 0 <= self.dropout_probability < 1:
            raise ValueError("dropout_probability must be in [0, 1)")


@dataclass(frozen=True)
class RGBDFrame:
    rgb: np.ndarray
    depth_m: np.ndarray
    rgb_raw: np.ndarray
    depth_raw_m: np.ndarray
    K: np.ndarray
    T_world_optical: np.ndarray
    timestamp_sim_s: float
    metadata: dict[str, Any] = field(default_factory=dict)

    def save_npz(self, path: str | Path) -> Path:
        output = Path(path)
        output.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            output,
            rgb=self.rgb,
            depth_m=self.depth_m,
            rgb_raw=self.rgb_raw,
            depth_raw_m=self.depth_raw_m,
            K=self.K,
            T_world_optical=self.T_world_optical,
            timestamp_sim_s=np.array(self.timestamp_sim_s, dtype=np.float64),
            metadata_json=np.array(json.dumps(self.metadata, sort_keys=True)),
        )
        return output


def apply_depth_range(
    depth_m: np.ndarray, minimum_m: float = 0.0, maximum_m: float = np.inf
) -> np.ndarray:
    """Return depth with invalid or out-of-profile samples replaced by NaN."""

    if not np.isfinite(minimum_m) or minimum_m < 0:
        raise ValueError("minimum depth range must be finite and non-negative")
    if not (np.isfinite(maximum_m) or np.isposinf(maximum_m)) or maximum_m <= minimum_m:
        raise ValueError("maximum depth range must be greater than minimum depth range")
    depth = np.asarray(depth_m, dtype=float).copy()
    valid = (
        np.isfinite(depth)
        & (depth > 0)
        & (depth >= minimum_m)
        & (depth <= maximum_m)
    )
    depth[~valid] = np.nan
    return depth


class NamedRGBDCamera:
    """Render synchronized RGB and metric optical-axis depth from one camera.

    MuJoCo camera local axes are +X right, +Y up, and it looks along -Z.
    The exported optical frame is +X right, +Y down, +Z forward.
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        name: str = "task_camera",
        *,
        width: int = 640,
        height: int = 480,
        calibration: CameraCalibration | None = None,
        noise: RGBDNoise = RGBDNoise(),
        depth_min_m: float = 0.0,
        depth_max_m: float = np.inf,
        sensor_profile: str = "generic_mujoco_rgbd",
        data_source: str = "synthetic_renderer",
        distortion_model: str = "none",
        rectified_aligned: bool = True,
        clock_domain: str = "simulation_time",
    ) -> None:
        if width <= 0 or height <= 0:
            raise ValueError("camera image dimensions must be positive")
        # Validate once at construction; the same limits are recorded per frame.
        apply_depth_range(np.array([1.0]), depth_min_m, depth_max_m)
        camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name)
        if camera_id < 0:
            raise ValueError(f"named MuJoCo camera does not exist: {name}")
        self.model = model
        self.name = name
        self.camera_id = camera_id
        self.width = int(width)
        self.height = int(height)
        self.calibration_override = calibration
        self.noise = noise
        self.depth_min_m = float(depth_min_m)
        self.depth_max_m = float(depth_max_m)
        self.sensor_profile = str(sensor_profile)
        self.data_source = str(data_source)
        self.distortion_model = str(distortion_model)
        self.rectified_aligned = bool(rectified_aligned)
        self.clock_domain = str(clock_domain)
        self._rng = np.random.default_rng(noise.seed)
        self._renderer = mujoco.Renderer(model, height=height, width=width)
        self._render_data = mujoco.MjData(model)

    def _model_calibration(self, data: mujoco.MjData) -> CameraCalibration:
        fovy = math_radians(float(self.model.cam_fovy[self.camera_id]))
        focal = 0.5 * self.height / np.tan(0.5 * fovy)
        K = np.array([
            [focal, 0.0, (self.width - 1) / 2],
            [0.0, focal, (self.height - 1) / 2],
            [0.0, 0.0, 1.0],
        ])
        rotation_world_gl = data.cam_xmat[self.camera_id].reshape(3, 3).copy()
        rotation_world_optical = rotation_world_gl @ np.diag([1.0, -1.0, -1.0])
        transform = np.eye(4)
        transform[:3, :3] = rotation_world_optical
        transform[:3, 3] = data.cam_xpos[self.camera_id]
        return CameraCalibration(K, transform, "synthetic_model")

    def capture(self, data: mujoco.MjData) -> RGBDFrame:
        # Rendering must not alter live solver caches or warm-start state.
        # Monitoring frequency can then change without changing the rollout.
        mujoco.mj_copyData(self._render_data, self.model, data)
        data = self._render_data
        mujoco.mj_forward(self.model, data)
        self._renderer.disable_depth_rendering()
        self._renderer.update_scene(data, camera=self.name)
        rgb_raw = self._renderer.render().copy()
        self._renderer.enable_depth_rendering()
        depth_raw = self._renderer.render().copy()
        self._renderer.disable_depth_rendering()

        model_calibration = self._model_calibration(data)
        calibration = self.calibration_override or model_calibration
        rgb = rgb_raw.copy()
        depth = depth_raw.copy()
        if self.noise.rgb_std:
            noisy = rgb.astype(float) + self._rng.normal(0, self.noise.rgb_std, rgb.shape)
            rgb = np.clip(np.rint(noisy), 0, 255).astype(np.uint8)
        valid = np.isfinite(depth) & (depth > 0)
        if self.noise.depth_std_m:
            depth[valid] += self._rng.normal(0, self.noise.depth_std_m, int(valid.sum()))
            depth[depth <= 0] = np.nan
        if self.noise.dropout_probability:
            dropout = self._rng.random(depth.shape) < self.noise.dropout_probability
            depth[dropout] = np.nan
        depth = apply_depth_range(depth, self.depth_min_m, self.depth_max_m)

        metadata = {
            "schema_version": 1,
            "camera_name": self.name,
            "sensor_profile": self.sensor_profile,
            "data_source": self.data_source,
            "resolution_px": [self.width, self.height],
            "depth_definition": "metric optical-axis Z in metres",
            "depth_range_m": {
                "minimum": self.depth_min_m,
                "maximum": None if np.isposinf(self.depth_max_m) else self.depth_max_m,
            },
            "optical_frame": "+X right, +Y down, +Z forward",
            "mujoco_camera_frame": "+X right, +Y up, view direction -Z",
            "distortion_model": self.distortion_model,
            "rectified_aligned": self.rectified_aligned,
            "clock_domain": self.clock_domain,
            "calibration_source": calibration.source,
            "model_calibration": {
                "usage": "evaluation_only; prohibited as mission pose input",
                "K": model_calibration.K.tolist(),
                "T_world_optical": model_calibration.T_world_optical.tolist(),
            },
            "noise": {
                "rgb_std": self.noise.rgb_std,
                "depth_std_m": self.noise.depth_std_m,
                "dropout_probability": self.noise.dropout_probability,
                "seed": self.noise.seed,
                "source": self.noise.source,
            },
            "provenance": {
                "renderer": "mujoco.Renderer",
                "depth_conversion": "MuJoCo reversed-Z buffer converted to metric depth",
                "calibration_override_applied": self.calibration_override is not None,
                "mission_extrinsic_source": (
                    calibration.source if self.calibration_override is not None
                    else "synthetic_model"
                ),
            },
        }
        return RGBDFrame(
            rgb=rgb,
            depth_m=depth,
            rgb_raw=rgb_raw,
            depth_raw_m=depth_raw,
            K=calibration.K.copy(),
            T_world_optical=calibration.T_world_optical.copy(),
            timestamp_sim_s=float(data.time),
            metadata=metadata,
        )

    def close(self) -> None:
        self._renderer.close()

    def __enter__(self) -> "NamedRGBDCamera":
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()


def math_radians(degrees: float) -> float:
    """Small local helper keeps camera calibration free of scipy dependency."""

    return degrees * np.pi / 180.0
