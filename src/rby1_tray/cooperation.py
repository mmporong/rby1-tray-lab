"""Encoder/FT-based supervision for cooperative tray transport.

This supervisor changes reference speed, not contact forces.  All thresholds
are synthetic design settings, separately exercised by disturbance trials.
"""
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class CooperationConfig:
    enabled: bool = True
    sample_period_s: float = 0.02
    filter_time_constant_s: float = 0.15
    position_warning_m: float = 0.004
    position_stop_m: float = 0.020
    orientation_warning_rad: float = 0.025
    orientation_stop_rad: float = 0.12
    imbalance_warning: float = 0.25
    imbalance_stop: float = 0.75
    minimum_speed_scale: float = 0.35
    stop_hold_s: float = 0.30

    def __post_init__(self):
        for name, value in vars(self).items():
            if name == "enabled":
                if not isinstance(value, bool):
                    raise ValueError("cooperation.enabled must be boolean")
            elif not math.isfinite(value) or value <= 0:
                raise ValueError(f"cooperation.{name} must be finite and positive")
        for stem in ("position", "orientation", "imbalance"):
            suffix = "_m" if stem == "position" else "_rad" if stem == "orientation" else ""
            if getattr(self, stem + "_warning" + suffix) >= getattr(self, stem + "_stop" + suffix):
                raise ValueError(f"cooperation {stem} warning must be below stop")
        if self.minimum_speed_scale > 1:
            raise ValueError("minimum_speed_scale cannot exceed 1")


class CooperationSupervisor:
    def __init__(self, config: CooperationConfig):
        self.config = config
        self.filtered_imbalance = 0.0
        self.baseline_imbalance = None
        self.speed_scale = 1.0
        self.stop_since_s = None
        self.last = {"active": False, "speed_scale": 1.0}
        self.summary = {
            "samples": 0, "slowdown_samples": 0, "stop_events": 0,
            "max_relative_position_error_m": 0.0,
            "max_relative_orientation_error_rad": 0.0,
            "max_load_imbalance_change": 0.0,
            "minimum_speed_scale": 1.0,
        }

    def update(self, timestamp_s, position_error_m, orientation_error_rad, loads_n, *, active):
        values = np.asarray([timestamp_s, position_error_m, orientation_error_rad, *loads_n], dtype=float)
        if values.shape != (5,) or not np.all(np.isfinite(values)) or np.any(values[:3] < 0):
            raise ValueError("cooperation observations must be finite with nonnegative time/errors")
        if not active:
            self.speed_scale = 1.0
            self.stop_since_s = None
            self.last = {"active": False, "speed_scale": 1.0, "enabled": self.config.enabled}
            return self.last
        total = abs(loads_n[0]) + abs(loads_n[1])
        imbalance = (loads_n[0] - loads_n[1]) / max(total, 0.25)
        if self.baseline_imbalance is None:
            self.baseline_imbalance = float(imbalance)
            self.filtered_imbalance = float(imbalance)
        alpha = 1 - math.exp(-self.config.sample_period_s / self.config.filter_time_constant_s)
        self.filtered_imbalance += alpha * (imbalance - self.filtered_imbalance)
        imbalance_change = abs(self.filtered_imbalance - self.baseline_imbalance)
        ratios = [
            (position_error_m - self.config.position_warning_m) / (self.config.position_stop_m - self.config.position_warning_m),
            (orientation_error_rad - self.config.orientation_warning_rad) / (self.config.orientation_stop_rad - self.config.orientation_warning_rad),
            (imbalance_change - self.config.imbalance_warning) / (self.config.imbalance_stop - self.config.imbalance_warning),
        ]
        severity = float(np.clip(max(ratios), 0, 1))
        wanted = 1.0 - severity * (1.0 - self.config.minimum_speed_scale)
        # Avoid a reference-velocity jump at the 50 Hz supervision boundary.
        if self.config.enabled:
            self.speed_scale += float(np.clip(wanted - self.speed_scale, -0.05, 0.025))
        else:
            self.speed_scale = 1.0
        critical = bool(max(ratios) >= 1)
        if critical:
            if self.stop_since_s is None:
                self.stop_since_s = timestamp_s
        else:
            self.stop_since_s = None
        abort = bool(self.config.enabled and self.stop_since_s is not None and timestamp_s - self.stop_since_s >= self.config.stop_hold_s)
        if abort and not self.last.get("abort", False):
            self.summary["stop_events"] += 1
        self.summary["samples"] += 1
        self.summary["slowdown_samples"] += int(self.speed_scale < 0.999)
        for key, value in (
            ("max_relative_position_error_m", position_error_m),
            ("max_relative_orientation_error_rad", orientation_error_rad),
            ("max_load_imbalance_change", imbalance_change),
        ):
            self.summary[key] = max(self.summary[key], float(value))
        self.summary["minimum_speed_scale"] = min(self.summary["minimum_speed_scale"], self.speed_scale)
        self.last = {
            "active": True, "enabled": self.config.enabled,
            "timestamp_sim_s": float(timestamp_s), "speed_scale": self.speed_scale,
            "relative_position_error_m": float(position_error_m),
            "relative_orientation_error_rad": float(orientation_error_rad),
            "gravity_compensated_vertical_load_n": list(map(float, loads_n)),
            "load_imbalance": float(imbalance),
            "baseline_load_imbalance": self.baseline_imbalance,
            "filtered_load_imbalance_change": float(imbalance_change),
            "critical": critical, "abort": abort,
            "observation_source": "joint_encoder_FK_and_wrist_FT_only",
            "policy": "reference_time_scaling_and_sustained_fault_abort",
        }
        return self.last
