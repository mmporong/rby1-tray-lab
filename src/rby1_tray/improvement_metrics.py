"""Compact, trace-derived metrics for paired transport studies."""

from __future__ import annotations

import gzip
import json
import math
from pathlib import Path
from typing import Any, Iterable, Mapping


TRANSPORT_PHASES = frozenset(
    {"lift", "retract_low", "drive_loaded", "raise_high", "drive_high", "seat"}
)


def _trace_path(run_directory: Path) -> Path | None:
    plain = run_directory / "trace.jsonl"
    if plain.is_file():
        return plain
    compressed = run_directory / "trace.jsonl.gz"
    return compressed if compressed.is_file() else None


def _rows(path: Path) -> Iterable[Mapping[str, Any]]:
    opener = gzip.open if path.suffix == ".gz" else open
    with opener(path, "rt", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            try:
                value = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"invalid trace JSON at line {line_number}: {exc}") from exc
            if not isinstance(value, Mapping):
                raise ValueError(f"trace line {line_number} is not an object")
            yield value


def _rpy_and_tilt(quaternion: Any) -> tuple[float, float, float] | None:
    if not isinstance(quaternion, list) or len(quaternion) != 4:
        return None
    try:
        w, x, y, z = (float(value) for value in quaternion)
    except (TypeError, ValueError):
        return None
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if not math.isfinite(norm) or norm <= 0:
        return None
    w, x, y, z = (value / norm for value in (w, x, y, z))
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch_sine = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(pitch_sine)
    tray_z_world_z = max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))
    tilt = math.acos(tray_z_world_z)
    return roll, pitch, tilt


def _rotate_world_to_tray(vector: list[float], quaternion: list[float]) -> list[float]:
    w, x, y, z = (float(value) for value in quaternion)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if not math.isfinite(norm) or norm <= 0:
        raise ValueError("tray quaternion is nonfinite or has zero norm")
    w, x, y, z = (value / norm for value in (w, x, y, z))
    # Transpose of the unit-quaternion rotation matrix (world -> tray).
    return [
        (1 - 2 * (y * y + z * z)) * vector[0]
        + 2 * (x * y + w * z) * vector[1]
        + 2 * (x * z - w * y) * vector[2],
        2 * (x * y - w * z) * vector[0]
        + (1 - 2 * (x * x + z * z)) * vector[1]
        + 2 * (y * z + w * x) * vector[2],
        2 * (x * z + w * y) * vector[0]
        + 2 * (y * z - w * x) * vector[1]
        + (1 - 2 * (x * x + y * y)) * vector[2],
    ]


def _payload_positions_in_tray(row: Mapping[str, Any]) -> dict[str, list[float]] | None:
    tray = row.get("tray")
    payloads = row.get("payloads")
    if not isinstance(tray, Mapping) or not isinstance(payloads, list):
        return None
    center = tray.get("pos_m")
    quaternion = tray.get("quat_wxyz")
    if not isinstance(center, list) or len(center) != 3:
        return None
    if not isinstance(quaternion, list) or len(quaternion) != 4:
        return None
    try:
        center_values = [float(value) for value in center]
        quaternion_values = [float(value) for value in quaternion]
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in center_values + quaternion_values):
        return None
    result: dict[str, list[float]] = {}
    for payload in payloads:
        if not isinstance(payload, Mapping):
            return None
        name = payload.get("name")
        position = payload.get("pos_m")
        if not isinstance(name, str) or not isinstance(position, list) or len(position) != 3:
            return None
        try:
            position_values = [float(value) for value in position]
        except (TypeError, ValueError):
            return None
        if not all(math.isfinite(value) for value in position_values):
            return None
        relative_world = [
            position_values[index] - center_values[index] for index in range(3)
        ]
        try:
            rotated = _rotate_world_to_tray(relative_world, quaternion_values)
        except ValueError:
            return None
        if not all(math.isfinite(value) for value in rotated):
            return None
        result[name] = rotated
    return result


def extract_trace_metrics(run_directory: Path) -> dict[str, Any]:
    """Return compact transport metrics while preserving missing-data semantics."""
    path = _trace_path(Path(run_directory))
    if path is None:
        return {
            "trace_metrics": None,
            "trace_metrics_missing_reason": "trace.jsonl or trace.jsonl.gz not found",
        }

    maxima: dict[str, float | None] = {"roll": None, "pitch": None, "tilt": None}
    baseline: dict[str, list[float]] | None = None
    baseline_phase: str | None = None
    baseline_time_s: float | None = None
    max_payload_displacement: float | None = None
    transport_samples = 0
    valid_tray_pose_samples = 0
    payload_displacement_samples = 0
    first_transport_time: float | None = None
    last_transport_time: float | None = None
    try:
        for row in _rows(path):
            phase = str(row.get("phase", ""))
            pose = row.get("tray")
            rpy = _rpy_and_tilt(pose.get("quat_wxyz") if isinstance(pose, Mapping) else None)
            try:
                row_time_s = float(row["time_s"])
            except (KeyError, TypeError, ValueError):
                row_time_s = None
            if row_time_s is not None and not math.isfinite(row_time_s):
                row_time_s = None
            if phase == "grasp_hold" and baseline is None:
                positions = _payload_positions_in_tray(row)
                if positions is not None:
                    baseline = positions
                    baseline_phase = "grasp_hold"
                    baseline_time_s = row_time_s
            if phase not in TRANSPORT_PHASES:
                continue
            transport_samples += 1
            if row_time_s is not None:
                first_transport_time = (
                    row_time_s if first_transport_time is None else first_transport_time
                )
                last_transport_time = row_time_s
            if rpy is not None:
                valid_tray_pose_samples += 1
                for key, value in zip(
                    ("roll", "pitch", "tilt"),
                    (abs(rpy[0]), abs(rpy[1]), rpy[2]),
                ):
                    previous = maxima[key]
                    maxima[key] = value if previous is None else max(previous, value)
            positions = _payload_positions_in_tray(row)
            if baseline is None and positions is not None:
                baseline = positions
                baseline_phase = phase
                baseline_time_s = row_time_s
            if positions is None or baseline is None or set(positions) != set(baseline):
                continue
            payload_displacement_samples += 1
            for name, position in positions.items():
                displacement = math.sqrt(
                    sum((position[index] - baseline[name][index]) ** 2 for index in range(3))
                )
                max_payload_displacement = max(
                    displacement,
                    max_payload_displacement if max_payload_displacement is not None else 0.0,
                )
    except (OSError, ValueError) as exc:
        return {
            "trace_metrics": None,
            "trace_metrics_missing_reason": f"trace read failed: {exc}",
        }

    if transport_samples == 0:
        return {
            "trace_metrics": None,
            "trace_metrics_missing_reason": "no transport-phase samples in trace",
        }
    missing_reasons = []
    if valid_tray_pose_samples == 0:
        missing_reasons.append("no valid finite tray quaternion in transport-phase samples")
    if max_payload_displacement is None:
        missing_reasons.append("no valid finite payload-to-tray displacement samples")
    if baseline_phase == "grasp_hold":
        displacement_reference = "first valid grasp_hold sample in tray frame"
    elif baseline_phase is not None:
        displacement_reference = "first valid transport sample in tray frame"
    else:
        displacement_reference = None
    return {
        "trace_metrics": {
            "scope": "lift_through_seat",
            "payload_displacement_reference": displacement_reference,
            "payload_baseline_phase": baseline_phase,
            "payload_baseline_time_s": baseline_time_s,
            "transport_samples": transport_samples,
            "valid_tray_pose_samples": valid_tray_pose_samples,
            "payload_displacement_samples": payload_displacement_samples,
            "transport_start_time_s": first_transport_time,
            "transport_end_time_s": last_transport_time,
            "max_abs_tray_roll_rad": maxima["roll"],
            "max_abs_tray_pitch_rad": maxima["pitch"],
            "max_tray_tilt_rad": maxima["tilt"],
            "max_payload_to_tray_displacement_m": max_payload_displacement,
        },
        "trace_metrics_missing_reason": "; ".join(missing_reasons) or None,
    }
