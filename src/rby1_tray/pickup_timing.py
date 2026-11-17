"""Opt-in pickup IK seed blending and C1 timing for a two-segment arm path.

This module does not change controller gains, acceptance gates, or physics.
The numeric timing reference must be measured against the same scene/config.
"""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class PickupSchedule:
    first_s: float
    second_s: float
    original_total_s: float
    baseline_second_jump_rad: float
    second_jump_rad: float
    second_factor: float

    @property
    def duration_s(self) -> float:
        return self.first_s + self.second_s


def blend_seed(previous: np.ndarray, high: np.ndarray, alpha: float) -> np.ndarray:
    """Choose the final approach IK *initial guess*, not an executed waypoint."""
    if not math.isfinite(alpha) or not 0.0 <= alpha <= 1.0:
        raise ValueError("pickup_optimization.seed_blend_alpha must be in [0, 1]")
    before = np.asarray(previous, dtype=float)
    reference = np.asarray(high, dtype=float)
    if before.shape != reference.shape or not np.all(np.isfinite(before)) or not np.all(np.isfinite(reference)):
        raise ValueError("pickup IK seeds must have matching finite shapes")
    return (1.0 - alpha) * before + alpha * reference


def make_schedule(path, original_total_s: float, baseline_second_jump_rad: float,
                  second_factor: float, minimum_second_s: float = 1.5) -> PickupSchedule:
    """Keep first half of original profile; shorten the second based on travel.

    Reference segment displacement must be recorded for the exact baseline.
    This is an experiment-specific heuristic, not a guaranteed optimum.
    """
    joints = np.asarray(path, dtype=float)
    vals = (original_total_s, baseline_second_jump_rad, second_factor, minimum_second_s)
    if joints.ndim != 2 or joints.shape[0] != 3 or not np.all(np.isfinite(joints)):
        raise ValueError("pickup time optimization requires exactly 3 finite IK waypoints")
    if not all(math.isfinite(v) for v in vals) or min(vals) <= 0:
        raise ValueError("pickup time schedule values must be positive and finite")
    first = original_total_s / 2.0
    if minimum_second_s > first:
        raise ValueError("minimum_second_s cannot exceed original half duration")
    jump = float(np.max(np.abs(joints[2] - joints[1])))
    second = max(minimum_second_s, min(first, first * jump / baseline_second_jump_rad * second_factor))
    return PickupSchedule(first, second, original_total_s, baseline_second_jump_rad, jump, second_factor)


def segmented_progress(elapsed_s: float, schedule: PickupSchedule) -> float:
    """Return continuous, C1 path position [0, 2] for the chosen schedule."""
    if elapsed_s <= 0.0:
        return 0.0
    first, second = schedule.first_s, schedule.second_s
    if elapsed_s <= first:
        u = elapsed_s / (2.0 * first)
        return 2.0 * (3.0*u*u - 2.0*u*u*u)
    if elapsed_s >= first + second:
        return 2.0
    u = (elapsed_s - first) / second
    u2, u3 = u*u, u*u*u
    initial_slope = 1.5 / first
    result = ((2*u3 - 3*u2 + 1)
              + (u3 - 2*u2 + u) * second * initial_slope
              + (-2*u3 + 3*u2) * 2.0)
    return max(1.0, min(2.0, float(result)))
