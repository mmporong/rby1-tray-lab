"""Geometry-only clearance checks for a perceived, carried tray.

The controller may use the known MuJoCo model geometry and the RGB-D yaw
estimate.  These helpers deliberately do not read ``MjData`` or task-object
state, keeping evaluation ground truth outside the planning path.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import mujoco
import numpy as np


@dataclass(frozen=True)
class TrayBox:
    """One tray collision box expressed in the tray body frame."""

    name: str
    center_m: tuple[float, float, float]
    half_size_m: tuple[float, float, float]


@dataclass(frozen=True)
class StaticBox:
    """One axis-aligned static collision box expressed in world coordinates."""

    name: str
    center_m: tuple[float, float, float]
    half_size_m: tuple[float, float, float]


def _named_box(
    model: mujoco.MjModel,
    name: str,
    *,
    require_body_id: int | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    geom_id = int(model.geom(name).id)
    if int(model.geom_type[geom_id]) != int(mujoco.mjtGeom.mjGEOM_BOX):
        raise ValueError(f"{name} must be a box collision geom")
    if require_body_id is not None and int(model.geom_bodyid[geom_id]) != require_body_id:
        raise ValueError(f"{name} is attached to an unexpected body")
    if not np.allclose(model.geom_quat[geom_id], [1.0, 0.0, 0.0, 0.0], atol=1e-12):
        raise ValueError(f"{name} must be axis-aligned in its parent body")
    return model.geom_pos[geom_id].copy(), model.geom_size[geom_id].copy()


def tray_collision_boxes(model: mujoco.MjModel) -> tuple[TrayBox, ...]:
    """Read the floor and four rim boxes from the compiled model."""

    names = (
        "tray_floor",
        "tray_xrim_-1",
        "tray_xrim_1",
        "tray_yrim_-1",
        "tray_yrim_1",
    )
    tray_body_id = int(model.body("tray").id)
    boxes = []
    for name in names:
        center, size = _named_box(model, name, require_body_id=tray_body_id)
        boxes.append(TrayBox(name, tuple(center), tuple(size)))
    return tuple(boxes)


def container_wall_boxes(model: mujoco.MjModel) -> tuple[StaticBox, ...]:
    """Read the four mapped container walls from the compiled model."""

    names = (
        "container_xwall_-1",
        "container_xwall_1",
        "container_ywall_-1",
        "container_ywall_1",
    )
    boxes = []
    for name in names:
        center, size = _named_box(model, name, require_body_id=0)
        boxes.append(StaticBox(name, tuple(center), tuple(size)))
    return tuple(boxes)


def rotated_tray_x_support_m(model: mujoco.MjModel, yaw_rad: float) -> float:
    """Return the furthest +X extent of all tray boxes from tray origin."""

    if not math.isfinite(yaw_rad):
        raise ValueError("yaw_rad must be finite")
    cosine, sine = math.cos(yaw_rad), math.sin(yaw_rad)
    support = -math.inf
    for box in tray_collision_boxes(model):
        cx, cy, _ = box.center_m
        sx, sy, _ = box.half_size_m
        rotated_center_x = cosine * cx - sine * cy
        projected_half_x = abs(cosine) * sx + abs(sine) * sy
        support = max(support, rotated_center_x + projected_half_x)
    return float(support)


def container_near_wall_outer_x_m(model: mujoco.MjModel) -> float:
    """Return the approach-side outer face of the negative-X container wall."""

    center, size = _named_box(model, "container_xwall_-1", require_body_id=0)
    return float(center[0] - size[0])


def geometry_clearance_safe_x_m(
    model: mujoco.MjModel,
    yaw_rad: float,
    start_center_x_m: float,
    clearance_m: float,
) -> float:
    """Place the tray's rotated +X support before the near wall by clearance."""

    if not math.isfinite(start_center_x_m):
        raise ValueError("start_center_x_m must be finite")
    if not math.isfinite(clearance_m) or clearance_m <= 0.0:
        raise ValueError("clearance_m must be finite and positive")
    limit = (
        container_near_wall_outer_x_m(model)
        - rotated_tray_x_support_m(model, yaw_rad)
        - clearance_m
    )
    return min(float(start_center_x_m), float(limit))


def carried_tray_wall_clearance_m(
    model: mujoco.MjModel,
    center_world_m: np.ndarray | tuple[float, float, float],
    yaw_rad: float,
) -> float | None:
    """Return conservative XY clearance while tray and wall overlap in Z.

    ``None`` means that every tray collision box is vertically separated from
    every wall.  A negative result denotes projected penetration.
    """

    center = np.asarray(center_world_m, dtype=float)
    if center.shape != (3,) or not np.all(np.isfinite(center)):
        raise ValueError("center_world_m must contain three finite values")
    if not math.isfinite(yaw_rad):
        raise ValueError("yaw_rad must be finite")
    cosine, sine = math.cos(yaw_rad), math.sin(yaw_rad)
    clearances = []
    for tray in tray_collision_boxes(model):
        local = np.asarray(tray.center_m)
        tray_center = center + np.array([
            cosine * local[0] - sine * local[1],
            sine * local[0] + cosine * local[1],
            local[2],
        ])
        sx, sy, sz = tray.half_size_m
        tray_half_x = abs(cosine) * sx + abs(sine) * sy
        tray_half_y = abs(sine) * sx + abs(cosine) * sy
        for wall in container_wall_boxes(model):
            wall_center = np.asarray(wall.center_m)
            wx, wy, wz = wall.half_size_m
            vertical_separation = abs(tray_center[2] - wall_center[2]) - (sz + wz)
            if vertical_separation >= -1e-12:
                continue
            x_separation = abs(tray_center[0] - wall_center[0]) - (tray_half_x + wx)
            y_separation = abs(tray_center[1] - wall_center[1]) - (tray_half_y + wy)
            clearances.append(max(float(x_separation), float(y_separation)))
    return min(clearances) if clearances else None
