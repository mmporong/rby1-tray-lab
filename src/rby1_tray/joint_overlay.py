"""Validated joint-load overlay metadata for the synchronized robot scene."""

from __future__ import annotations

import importlib
from typing import Any

import numpy as np


JOINT_LOAD_SCOPE = (
    "20개 단위 기어 회전 관절의 모델 액추에이터 부하율입니다. "
    "하드웨어 정격·안전율·구조응력이 아닙니다."
)


def _joint_label(name: str) -> str:
    if name.startswith("torso_"):
        return f"몸통 관절 {int(name.rsplit('_', 1)[1]) + 1}"
    if name.startswith("right_arm_"):
        return f"오른팔 관절 {int(name.rsplit('_', 1)[1]) + 1}"
    if name.startswith("left_arm_"):
        return f"왼팔 관절 {int(name.rsplit('_', 1)[1]) + 1}"
    return name


def build_joint_load_metadata(session: Any) -> dict[str, Any]:
    """Build JSON-only overlay metadata from one initialized robot session.

    Actuator force is interpretable as joint torque only for a scalar +1 gear
    transmission on a hinge. Any incomplete or different model fails closed.
    """
    mujoco = importlib.import_module("mujoco")

    model = session.model
    robot = session.robot
    names = list(robot.names)
    joint_ids = np.asarray(robot.jids, dtype=int)
    actuator_ids = np.asarray(robot.act, dtype=int)
    limits = np.asarray(robot.force_limit, dtype=float)
    if len(names) != 20 or len(set(names)) != 20:
        raise ValueError("joint-load overlay requires 20 unique robot joint names")
    if joint_ids.shape != (20,) or actuator_ids.shape != (20,) or limits.shape != (20,):
        raise ValueError("joint-load overlay arrays must follow the 20-joint order")
    if np.any(joint_ids < 0) or np.any(joint_ids >= model.njnt):
        raise ValueError("joint-load overlay contains an invalid joint id")
    if np.any(actuator_ids < 0) or np.any(actuator_ids >= model.nu):
        raise ValueError("joint-load overlay contains an invalid actuator id")
    if not np.all(np.isfinite(limits)) or np.any(limits <= 0):
        raise ValueError("joint-load overlay torque limits must be finite and positive")

    joints = []
    for name, joint_id, actuator_id, limit in zip(
        names, joint_ids, actuator_ids, limits
    ):
        if not isinstance(name, str) or not name:
            raise ValueError("joint-load overlay names must be non-empty strings")
        if model.joint(int(joint_id)).name != name:
            raise ValueError("robot joint name order does not match the MuJoCo model")
        if int(model.jnt_type[joint_id]) != int(mujoco.mjtJoint.mjJNT_HINGE):
            raise ValueError(f"joint-load overlay requires hinge joints: {name}")
        if int(model.actuator_trntype[actuator_id]) != int(mujoco.mjtTrn.mjTRN_JOINT):
            raise ValueError(f"joint-load overlay requires joint transmission: {name}")
        if int(model.actuator_trnid[actuator_id, 0]) != int(joint_id):
            raise ValueError(f"actuator transmission does not target joint {name}")
        gear = np.asarray(model.actuator_gear[actuator_id], dtype=float)
        if (
            not np.all(np.isfinite(gear))
            or not np.isclose(gear[0], 1.0, rtol=0.0, atol=1e-12)
            or not np.allclose(gear[1:], 0.0, rtol=0.0, atol=1e-12)
        ):
            raise ValueError(f"joint-load overlay requires scalar +1 gear: {name}")
        force_range = np.asarray(model.actuator_forcerange[actuator_id], dtype=float)
        model_limit = float(np.max(np.abs(force_range)))
        if (
            not bool(model.actuator_forcelimited[actuator_id])
            or not np.all(np.isfinite(force_range))
            or model_limit <= 0
            or not np.allclose(
                force_range, [-model_limit, model_limit], rtol=0.0, atol=1e-12
            )
            or not np.isclose(limit, model_limit, rtol=0.0, atol=1e-12)
        ):
            raise ValueError(f"joint-load limit does not match actuator force range: {name}")
        body_id = int(model.jnt_bodyid[joint_id])
        part_id = model.body(body_id).name
        anchor = np.asarray(model.jnt_pos[joint_id], dtype=float)
        if not isinstance(part_id, str) or not part_id or anchor.shape != (3,) or not np.all(np.isfinite(anchor)):
            raise ValueError(f"joint-load anchor/body metadata is invalid: {name}")
        joints.append(
            {
                "name": name,
                "label": _joint_label(name),
                "part_id": part_id,
                "anchor_body_m": anchor.tolist(),
                "limit_nm": float(limit),
            }
        )
    return {
        "available": True,
        "scope": JOINT_LOAD_SCOPE,
        "joint_names_order": names,
        "ratio_definition": "abs(joint_state.actuator_force[name]) / limit_nm",
        "joints": joints,
    }


def joint_load_metadata(session: Any) -> dict[str, Any]:
    """Return validated metadata or an explicit unavailable status."""
    try:
        return build_joint_load_metadata(session)
    except (AttributeError, IndexError, TypeError, ValueError) as exc:
        return {
            "available": False,
            "scope": JOINT_LOAD_SCOPE,
            "reason": f"관절 부하 메타데이터를 검증하지 못했습니다: {exc}",
        }


__all__ = ["JOINT_LOAD_SCOPE", "build_joint_load_metadata", "joint_load_metadata"]
