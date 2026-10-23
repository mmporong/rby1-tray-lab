"""Load-path terms expressed at one cut origin and in one cut frame."""

from __future__ import annotations

import numpy as np


def subtree_body_ids(model, body_id):
    """Return ``body_id`` and all descendants in model body order."""
    body_id = int(body_id)
    if body_id < 0 or body_id >= int(model.nbody):
        raise ValueError("body_id is outside the model")
    result = []
    for candidate in range(body_id, int(model.nbody)):
        current = candidate
        while current > body_id:
            current = int(model.body_parentid[current])
        if current == body_id:
            result.append(candidate)
    return np.asarray(result, dtype=int)


def _frame(origin_world_m, rotation_world):
    origin = np.asarray(origin_world_m, dtype=float)
    rotation = np.asarray(rotation_world, dtype=float)
    if (
        origin.shape != (3,)
        or rotation.shape != (3, 3)
        or not np.all(np.isfinite(origin))
        or not np.all(np.isfinite(rotation))
        or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-10)
        or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-10)
    ):
        raise ValueError("cut origin and rotation must define a finite right-handed frame")
    return origin, rotation


def gravity_support_wrench(model, data, body_ids, origin_world_m, rotation_world):
    """Support wrench balancing subtree gravity, expressed in the cut frame."""
    origin, rotation = _frame(origin_world_m, rotation_world)
    ids = np.asarray(body_ids, dtype=int)
    if ids.ndim != 1 or np.any(ids < 0) or np.any(ids >= int(model.nbody)):
        raise ValueError("body_ids must be valid model body indices")
    gravity = np.asarray(model.opt.gravity, dtype=float)
    forces = np.asarray(model.body_mass, dtype=float)[ids, None] * gravity
    support_force_world = -forces.sum(axis=0)
    support_moment_world = -np.cross(
        np.asarray(data.xipos, dtype=float)[ids] - origin, forces
    ).sum(axis=0)
    return rotation.T @ support_force_world, rotation.T @ support_moment_world


def external_support_wrench(model, data, body_ids, origin_world_m, rotation_world):
    """Reaction balancing MuJoCo external/contact wrenches on a subtree.

    ``cfrc_ext`` uses world-aligned c-frames whose origin is the root tree's
    subtree COM. The accumulated wrench is transported from that common origin
    to the requested cut before its balancing reaction is returned.
    """
    origin, rotation = _frame(origin_world_m, rotation_world)
    ids = np.asarray(body_ids, dtype=int)
    if ids.ndim != 1 or len(ids) == 0 or np.any(ids < 0) or np.any(ids >= int(model.nbody)):
        raise ValueError("body_ids must contain valid model body indices")
    roots = np.asarray(model.body_rootid, dtype=int)[ids]
    if np.any(roots != roots[0]):
        raise ValueError("all subtree bodies must share one kinematic tree root")
    c_origin = np.asarray(data.subtree_com, dtype=float)[int(roots[0])]
    external = np.asarray(data.cfrc_ext, dtype=float)[ids].sum(axis=0)
    external_moment_world = external[:3] + np.cross(c_origin - origin, external[3:])
    return rotation.T @ -external[3:], rotation.T @ -external_moment_world


def load_path_components(model, data, body_id, origin_world_m, rotation_world, *, action_sign=1.0):
    """Return gravity and external support terms for one cut action."""
    if action_sign not in (-1, 1, -1.0, 1.0):
        raise ValueError("action_sign must be +1 or -1")
    ids = subtree_body_ids(model, body_id)
    gravity_force, gravity_moment = gravity_support_wrench(
        model, data, ids, origin_world_m, rotation_world
    )
    external_force, external_moment = external_support_wrench(
        model, data, ids, origin_world_m, rotation_world
    )
    sign = float(action_sign)
    return (
        np.concatenate((gravity_force, gravity_moment)) * sign,
        np.concatenate((external_force, external_moment)) * sign,
    )


def residual_wrench(total_wrench, gravity_support, external_support):
    """Close the accounting identity without assigning a causal percentage."""
    total = np.asarray(total_wrench, dtype=float)
    gravity = np.asarray(gravity_support, dtype=float)
    external = np.asarray(external_support, dtype=float)
    if any(value.shape != (6,) or not np.all(np.isfinite(value)) for value in (total, gravity, external)):
        raise ValueError("all wrench terms must be finite 6-vectors")
    return total - gravity - external


__all__ = [
    "external_support_wrench",
    "gravity_support_wrench",
    "load_path_components",
    "residual_wrench",
    "subtree_body_ids",
]
