"""Verified, pre-rendered playback bundles for the local portfolio viewer."""

from __future__ import annotations

from dataclasses import dataclass
from io import BytesIO
import gzip
import hashlib
import json
import math
from pathlib import Path, PurePosixPath
import shutil
import tempfile
from typing import Any

import numpy as np
from PIL import Image


SCHEMA_VERSION = 1
BASE_SPEED = 8
FPS = 60
WIDTH = 960
HEIGHT = 720
VIEWS = ("overview", "pickup", "placement")
SENSOR_CAMERAS = frozenset({"head_rgbd", "left_wrist_rgbd", "right_wrist_rgbd"})
SENSOR_KINDS = ("rgb", "depth")


def _depth_display_range(metadata: dict[str, Any]) -> tuple[float, float]:
    """Return the fixed metric display range recorded for one sensor."""

    value = metadata.get("depth_range_m")
    if not isinstance(value, dict):
        raise ValueError("sensor frame is missing depth_range_m metadata")
    minimum = value.get("minimum")
    maximum = value.get("maximum")
    if (
        isinstance(minimum, bool)
        or isinstance(maximum, bool)
        or not isinstance(minimum, (int, float))
        or not isinstance(maximum, (int, float))
        or not math.isfinite(minimum)
        or not math.isfinite(maximum)
        or minimum < 0
        or maximum <= minimum
    ):
        raise ValueError("sensor depth_range_m must contain a finite increasing range")
    return float(minimum), float(maximum)


def _validated_sensor_arrays(
    frame: Any,
) -> tuple[np.ndarray, np.ndarray, float, float]:
    """Validate one RGB-D frame before any display conversion can hide defects."""

    metadata = getattr(frame, "metadata", None)
    if not isinstance(metadata, dict):
        raise ValueError("sensor frame metadata must be an object")
    camera_name = metadata.get("camera_name")
    if camera_name not in SENSOR_CAMERAS:
        raise ValueError(f"unsupported sensor camera_name: {camera_name!r}")
    timestamp = getattr(frame, "timestamp_sim_s", None)
    if (
        isinstance(timestamp, bool)
        or not isinstance(timestamp, (int, float))
        or not math.isfinite(timestamp)
        or timestamp < 0
    ):
        raise ValueError("sensor timestamp_sim_s must be finite and non-negative")
    rgb = np.asarray(frame.rgb)
    depth = np.asarray(frame.depth_m)
    if rgb.dtype != np.uint8:
        raise ValueError("RGB sensor preview source must use uint8 pixels")
    if rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError("RGB sensor preview must have shape HxWx3")
    if depth.ndim != 2 or not np.issubdtype(depth.dtype, np.floating):
        raise ValueError("depth sensor preview must be a floating-point HxW array")
    if rgb.shape[:2] != depth.shape:
        raise ValueError("RGB and depth sensor preview dimensions must match")
    low, high = _depth_display_range(metadata)
    return rgb, depth.astype(float, copy=False), low, high


def canonical_config_bytes(config: dict[str, Any]) -> bytes:
    """Serialize the effective configuration for stable cache validation."""

    return json.dumps(
        config,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")


def canonical_config_sha256(config: dict[str, Any]) -> str:
    return hashlib.sha256(canonical_config_bytes(config)).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


class StagedOutput:
    """Own one temporary build directory and publish it only after success."""

    def __init__(self, destination: str | Path):
        self.destination = Path(destination)
        self.staging: Path | None = None

    def create(self) -> Path:
        if self.destination.exists():
            raise FileExistsError(f"playback output already exists: {self.destination}")
        self.destination.parent.mkdir(parents=True, exist_ok=True)
        self.staging = Path(
            tempfile.mkdtemp(
                prefix=f".{self.destination.name}.building-",
                dir=self.destination.parent,
            )
        )
        return self.staging

    def commit(self) -> None:
        if self.staging is None or not self.staging.is_dir():
            raise RuntimeError("playback staging directory is unavailable")
        if self.destination.exists():
            raise FileExistsError(f"playback output appeared during build: {self.destination}")
        self.staging.rename(self.destination)
        self.staging = None

    def commit_if_sources_match(
        self, expected: dict[str, str], current: dict[str, str]
    ) -> None:
        if current != expected:
            raise RuntimeError(
                "source changed during playback generation; discard the incomplete bundle and rerun"
            )
        self.commit()

    def cleanup(self) -> None:
        if self.staging is None:
            return
        staging = self.staging
        self.staging = None
        if staging.is_dir():
            shutil.rmtree(staging)


def nearest_step_count(current_time_s: float, target_time_s: float, step_s: float) -> int:
    """Return steps that land nearest a target without moving simulation backward."""

    values = (current_time_s, target_time_s, step_s)
    if not all(math.isfinite(value) for value in values) or step_s <= 0:
        raise ValueError("playback sampling times must be finite and step_s positive")
    if target_time_s <= current_time_s:
        return 0
    return max(1, int(math.floor((target_time_s - current_time_s) / step_s + 0.5)))


def encode_sensor_preview(frame: Any, kind: str) -> tuple[bytes, dict[str, Any]]:
    """Encode a display preview and describe its source and value mapping."""

    pixels, depth, low, high = _validated_sensor_arrays(frame)
    if kind == "rgb":
        image = Image.fromarray(pixels, mode="RGB")
        metadata = {
            "encoding": "uint8 RGB copied from the recorded controller frame",
            "metric_data": False,
        }
    elif kind == "depth":
        valid = np.isfinite(depth) & (depth > 0) & (depth >= low) & (depth <= high)
        preview = np.empty((*depth.shape, 3), dtype=np.uint8)
        preview[:] = [230, 55, 135]
        if np.any(valid):
            grayscale = np.rint(
                1.0 + 254.0 * (high - depth[valid]) / (high - low)
            ).astype(np.uint8)
            preview[valid] = grayscale[:, None]
        image = Image.fromarray(preview, mode="RGB")
        metadata = {
            "encoding": "fixed-range inverse grayscale; invalid depth is magenta RGB(230,55,135)",
            "metric_data": False,
            "display_depth_min_m": low,
            "display_depth_max_m": high,
            "valid_pixel_count": int(np.count_nonzero(valid)),
            "metric_source": "recorded RGBDFrame.depth_m; use controller data for metric depth",
        }
    else:
        raise ValueError(f"unsupported sensor preview kind: {kind}")
    payload = BytesIO()
    image.save(payload, format="PNG", optimize=True)
    return payload.getvalue(), metadata


def _safe_artifact_name(name: str) -> str | None:
    if not isinstance(name, str) or not name or "\\" in name:
        return None
    path = PurePosixPath(name)
    if path.is_absolute() or any(part in {"", ".", ".."} for part in path.parts):
        return None
    normalized = path.as_posix()
    fixed = {
        "overview.mp4",
        "pickup.mp4",
        "placement.mp4",
        "states.json.gz",
        "sensors.json",
        "config.json",
        "result.json",
    }
    if normalized in fixed:
        return normalized
    if len(path.parts) == 2 and path.parts[0] == "sensors" and path.suffix == ".png":
        return normalized
    return None


def _read_json(path: Path) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid playback JSON: {path.name}") from exc


@dataclass(frozen=True)
class _SensorEntry:
    camera_name: str
    timestamp_sim_s: float
    rgb_path: str
    depth_path: str
    metadata: dict[str, Any]

    @property
    def capture_batch_id(self) -> str | None:
        value = self.metadata.get("capture_batch_id")
        return value if isinstance(value, str) and value else None

    @property
    def observation_role(self) -> str:
        return str(self.metadata.get("observation_role", "controller"))


class PlaybackAssets:
    """Validate and expose one immutable playback bundle.

    Construction verifies the effective configuration, every declared artifact
    hash, state-frame continuity, and sensor preview paths.  A viewer can catch
    ``FileNotFoundError`` or ``ValueError`` and fall back to live stepping.
    """

    def __init__(self, directory: str | Path, config: dict[str, Any]):
        self.directory = Path(directory)
        manifest_path = self.directory / "manifest.json"
        if not manifest_path.is_file():
            raise FileNotFoundError(manifest_path)
        manifest = _read_json(manifest_path)
        if not isinstance(manifest, dict) or manifest.get("schema_version") != SCHEMA_VERSION:
            raise ValueError("unsupported playback manifest schema")
        self._validate_manifest_scalars(manifest)
        active_step = config.get("physics_dt_s")
        if (
            isinstance(active_step, bool)
            or not isinstance(active_step, (int, float))
            or not math.isfinite(active_step)
            or active_step <= 0
        ):
            raise ValueError("active playback physics_dt_s must be a finite positive number")
        if float(manifest["physics_dt_s"]) != float(active_step):
            raise ValueError("playback physics_dt_s does not match the active configuration")
        if manifest.get("config_sha256") != canonical_config_sha256(config):
            raise ValueError("playback configuration hash does not match the active configuration")
        self.manifest = manifest

        hashes = manifest.get("artifact_sha256")
        if not isinstance(hashes, dict) or not hashes:
            raise ValueError("playback manifest has no artifact hashes")
        self._assets: dict[str, Path] = {}
        for raw_name, expected in hashes.items():
            name = _safe_artifact_name(raw_name)
            if name is None or not isinstance(expected, str) or len(expected) != 64:
                raise ValueError(f"invalid playback artifact declaration: {raw_name!r}")
            path = self.directory / name
            if not path.is_file():
                raise FileNotFoundError(path)
            try:
                path.resolve(strict=True).relative_to(self.directory.resolve(strict=True))
            except (OSError, ValueError) as exc:
                raise ValueError(f"playback artifact escapes its bundle: {name}") from exc
            if path.is_symlink():
                raise ValueError(f"playback artifacts cannot be symbolic links: {name}")
            if sha256_file(path) != expected:
                raise ValueError(f"playback artifact hash mismatch: {name}")
            self._assets[name] = path

        required = {
            *(f"{view}.mp4" for view in VIEWS),
            "states.json.gz",
            "sensors.json",
            "config.json",
            "result.json",
        }
        missing = required.difference(self._assets)
        if missing:
            raise ValueError(f"playback manifest is missing required artifacts: {sorted(missing)}")
        self._validate_recorded_config(config)
        self._state_count = self._validate_states()
        if manifest.get("frame_count") != self._state_count:
            raise ValueError("playback manifest frame_count does not match states")
        self._sensor_entries = self._validate_sensors()

    @staticmethod
    def _validate_manifest_scalars(manifest: dict[str, Any]) -> None:
        expected = {
            "base_speed": BASE_SPEED,
            "fps": FPS,
            "width": WIDTH,
            "height": HEIGHT,
        }
        for name, value in expected.items():
            if manifest.get(name) != value:
                raise ValueError(f"invalid playback {name}")
        duration = manifest.get("duration_sim_s")
        if (
            isinstance(duration, bool)
            or not isinstance(duration, (int, float))
            or not math.isfinite(duration)
            or duration < 0
        ):
            raise ValueError("invalid playback duration_sim_s")
        physics_step = manifest.get("physics_dt_s")
        if (
            isinstance(physics_step, bool)
            or not isinstance(physics_step, (int, float))
            or not math.isfinite(physics_step)
            or physics_step <= 0
        ):
            raise ValueError("playback physics_dt_s must be a finite positive number")
        result = manifest.get("result")
        if not isinstance(result, dict) or not result.get("terminal") or not result.get("success"):
            raise ValueError("playback bundle does not contain a terminal successful result")

    def _validate_recorded_config(self, config: dict[str, Any]) -> None:
        recorded = _read_json(self._assets["config.json"])
        if canonical_config_sha256(recorded) != canonical_config_sha256(config):
            raise ValueError("recorded playback config does not match the active configuration")

    def _validate_states(self) -> int:
        try:
            with gzip.open(self._assets["states.json.gz"], "rt", encoding="utf-8") as stream:
                states = json.load(stream)
        except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ValueError("invalid playback states") from exc
        if not isinstance(states, list) or not states:
            raise ValueError("playback states must be a non-empty list")
        previous_time = -math.inf
        for index, row in enumerate(states):
            if not isinstance(row, dict) or row.get("frame_index") != index:
                raise ValueError("playback state frame indices must be contiguous from zero")
            actual = row.get("actual_time_s")
            target = row.get("target_time_s")
            if not all(isinstance(value, (int, float)) and math.isfinite(value) for value in (actual, target)):
                raise ValueError("playback state times must be finite")
            if actual < previous_time:
                raise ValueError("playback state times must be monotonic")
            if index == 0 and (actual != 0 or target != 0):
                raise ValueError("playback state zero must record simulation time zero")
            expected_target = index * BASE_SPEED / FPS
            if not math.isclose(float(target), expected_target, abs_tol=1e-12):
                raise ValueError("playback target times must match the 8x/60 FPS frame grid")
            snapshot = row.get("state")
            snapshot_time = snapshot.get("time_s") if isinstance(snapshot, dict) else None
            if (
                not isinstance(snapshot_time, (int, float))
                or not math.isfinite(snapshot_time)
                or not math.isclose(float(snapshot_time), float(actual), abs_tol=1e-12)
            ):
                raise ValueError("playback snapshot time does not match its frame metadata")
            if index and not row.get("terminal_frame", False):
                tolerance = float(self.manifest.get("physics_dt_s", 0.0)) / 2 + 1e-9
                if abs(actual - target) > tolerance:
                    raise ValueError("playback state is not the nearest physics step")
            previous_time = float(actual)
        terminal_state = states[-1].get("state")
        if not isinstance(terminal_state, dict):
            raise ValueError("terminal playback state is missing")
        terminal_result = terminal_state.get("result")
        if not states[-1].get("terminal_frame") or not isinstance(terminal_result, dict):
            raise ValueError("last playback frame is not marked terminal")
        if not terminal_result.get("terminal") or not terminal_result.get("success"):
            raise ValueError("last playback state is not successful")
        if not math.isclose(
            float(self.manifest["duration_sim_s"]),
            float(states[-1]["actual_time_s"]),
            abs_tol=1e-12,
        ):
            raise ValueError("playback duration does not match the terminal state")
        return len(states)

    def _validate_sensors(self) -> tuple[_SensorEntry, ...]:
        document = _read_json(self._assets["sensors.json"])
        if not isinstance(document, dict) or document.get("schema_version") != 1:
            raise ValueError("invalid playback sensor index")
        rows = document.get("frames")
        if not isinstance(rows, list) or len(rows) < len(SENSOR_CAMERAS):
            raise ValueError("playback sensor index must contain all RGB-D cameras")
        entries = []
        for row in rows:
            if not isinstance(row, dict):
                raise ValueError("invalid playback sensor row")
            timestamp = row.get("timestamp_sim_s")
            if not isinstance(timestamp, (int, float)) or not math.isfinite(timestamp):
                raise ValueError("sensor timestamps must be finite")
            camera = row.get("camera_name")
            rgb_path, depth_path = row.get("rgb_path"), row.get("depth_path")
            provenance = row.get("recorded_frame_provenance")
            if camera not in SENSOR_CAMERAS:
                raise ValueError(f"unsupported playback sensor camera_name: {camera!r}")
            if not isinstance(provenance, dict):
                raise ValueError("sensor frame provenance must be an object")
            _depth_display_range(provenance)
            for path in (rgb_path, depth_path):
                if _safe_artifact_name(path) is None or path not in self._assets:
                    raise ValueError("sensor preview path is not a verified artifact")
            entries.append(
                _SensorEntry(camera, float(timestamp), rgb_path, depth_path, provenance)
            )
        coverage = {entry.camera_name for entry in entries}
        if coverage != SENSOR_CAMERAS:
            raise ValueError(
                "playback sensor index must cover head and both wrist RGB-D cameras"
            )
        return tuple(sorted(entries, key=lambda entry: entry.timestamp_sim_s))

    def info(self) -> dict[str, Any]:
        return {
            "enabled": True,
            "schema_version": self.manifest["schema_version"],
            "base_speed": self.manifest["base_speed"],
            "speed_options": [1, 1.5, 2],
            "simulation_speed_options": [8, 12, 16],
            "fps": self.manifest["fps"],
            "frame_count": self._state_count,
            "duration_sim_s": self.manifest["duration_sim_s"],
            "width": self.manifest["width"],
            "height": self.manifest["height"],
            "source_commit": self.manifest.get("source_commit"),
            "source_dirty": bool(self.manifest.get("source_dirty", False)),
            "result": self.manifest["result"],
            "videos": {view: f"/playback/{view}.mp4" for view in VIEWS},
            "states_url": "/playback/states.json.gz",
            "sensor_frame_count": len(self._sensor_entries),
        }

    def asset(self, name: str) -> tuple[Path, str, str | None] | None:
        normalized = _safe_artifact_name(name)
        if normalized is None or normalized not in self._assets:
            return None
        path = self._assets[normalized]
        if path.suffix == ".mp4":
            return path, "video/mp4", None
        if path.suffix == ".png":
            return path, "image/png", None
        if normalized == "states.json.gz":
            return path, "application/json", "gzip"
        return path, "application/json; charset=utf-8", None

    def sensor_png(
        self, camera: str, kind: str, time_s: float
    ) -> tuple[bytes, float, dict[str, Any]] | None:
        if not isinstance(time_s, (int, float)) or not math.isfinite(time_s):
            raise ValueError("sensor playback time must be finite")
        if kind not in SENSOR_KINDS:
            return None
        eligible = [
            entry
            for entry in self._sensor_entries
            if entry.timestamp_sim_s <= float(time_s) + 1e-12
        ]
        monitor_batches: dict[str, list[_SensorEntry]] = {}
        for entry in eligible:
            if entry.observation_role == "monitor_only" and entry.capture_batch_id:
                monitor_batches.setdefault(entry.capture_batch_id, []).append(entry)
        complete_batches = [
            batch
            for batch in monitor_batches.values()
            if {entry.camera_name for entry in batch} == SENSOR_CAMERAS
            and len(batch) == len(SENSOR_CAMERAS)
            and max(entry.timestamp_sim_s for entry in batch)
            - min(entry.timestamp_sim_s for entry in batch)
            <= 1e-9
        ]
        if complete_batches:
            selected_batch = max(
                complete_batches,
                key=lambda batch: max(entry.timestamp_sim_s for entry in batch),
            )
            candidates = [entry for entry in selected_batch if entry.camera_name == camera]
        else:
            candidates = [entry for entry in eligible if entry.camera_name == camera]
        if not candidates:
            return None
        selected = candidates[-1]
        name = selected.rgb_path if kind == "rgb" else selected.depth_path
        return self._assets[name].read_bytes(), selected.timestamp_sim_s, selected.metadata
