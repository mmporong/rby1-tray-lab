#!/usr/bin/env python3
"""Replay recorded physics and append synchronized cut load-path terms."""

from __future__ import annotations

import argparse
import gzip
import hashlib
import importlib
import json
from pathlib import Path
import shutil
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]

import numpy as np

from rby1_tray.cli import rendering_environment
from rby1_tray.load_path import load_path_components, residual_wrench, subtree_body_ids
from rby1_tray.playback import nearest_step_count
from rby1_tray.section_stress import body_interaction_wrench
from tools.build_load_view import physical_vector


SCENE_ASSETS = ("geometry.bin", "frames.bin", "fem-surface.bin", "fem-cutaway.bin")


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _subtree_mass(model, body_id):
    return float(np.asarray(model.body_mass)[subtree_body_ids(model, body_id)].sum())


def _body_label(model, body_id, labels):
    name = model.body(int(body_id)).name
    return labels.get(name, name)


def build(scene, output, bundle):
    rendering_environment()
    mujoco = importlib.import_module("mujoco")
    from rby1_tray.mission import MobileSession
    from rby1_tray.scene import load_config

    scene = Path(scene)
    output = Path(output)
    bundle = Path(bundle)
    manifest_path = scene / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    for name in SCENE_ASSETS:
        if sha256(scene / name) != manifest["assets"][name]:
            raise ValueError(f"source structural asset hash mismatch: {name}")
    if manifest.get("load_diagnostics"):
        raise ValueError("source structural scene already contains load diagnostics")
    with gzip.open(bundle / "states.json.gz", "rt", encoding="utf-8") as stream:
        reference = json.load(stream)
    if sha256(bundle / "states.json.gz") != manifest["reference_states_sha256"]:
        raise ValueError("scene and replay reference-state hashes differ")
    frame_count = int(manifest["frame_count"])
    parts = manifest["parts"]
    part_count = len(parts)
    if len(reference) != frame_count:
        raise ValueError("scene and replay frame counts differ")
    source_frames = np.fromfile(scene / "frames.bin", dtype="<f4")
    expected = frame_count * part_count * int(manifest["part_stride_floats"])
    if source_frames.size != expected:
        raise ValueError("source frames.bin shape does not match its manifest")
    source_frames = source_frames.reshape(frame_count, part_count, 19)

    if output.exists():
        raise FileExistsError(f"output already exists: {output}")
    output.mkdir(parents=True)
    started = time.monotonic()
    session = MobileSession(load_config(bundle / "config.json"))
    scratch = mujoco.MjData(session.model)
    diagnostics = np.zeros((frame_count, part_count, 18), dtype=np.float64)
    maximum_defined_field_difference = 0.0
    maximum_frame_wrench_difference = 0.0
    try:
        model = session.model
        labels = {part["id"]: part["label"] for part in parts}
        base_id = int(model.body("base").id)
        base_sources = manifest["validation"]["base_dominant_mount_per_frame"]
        if len(base_sources) != frame_count:
            raise ValueError("base source-body list length does not match frames")
        left_site_id = int(model.site("left_ft_frame").id)
        left_ft_body = int(model.site_bodyid[left_site_id])

        descriptors = []
        for part in parts:
            identifier = part["id"]
            if part["kind"] == "section":
                body_id = int(model.body(identifier).id)
                section = part["section"]
                origin_body = np.asarray(section["origin_body_m"], dtype=float)
                basis_body = np.asarray(section["basis_body"], dtype=float).reshape(3, 3)
                descriptors.append(("base" if body_id == base_id else "section", body_id, origin_body, basis_body))
            elif identifier == manifest["fem_part_id"]:
                body_rotation = session.data.xmat[left_ft_body].reshape(3, 3)
                origin_body = body_rotation.T @ (
                    session.data.site_xpos[left_site_id] - session.data.xpos[left_ft_body]
                )
                basis_body = body_rotation.T @ session.data.site_xmat[left_site_id].reshape(3, 3)
                descriptors.append(("fem", left_ft_body, origin_body, basis_body))
            else:
                raise ValueError(f"unsupported structural part: {identifier}")

        part_metadata = []
        for part, (kind, body_id, origin_body, basis_body) in zip(parts, descriptors):
            parent_id = int(model.body_parentid[body_id])
            if kind == "base":
                parent_label = labels.get("base", "base")
                source_label = "프레임별 지배 부착부"
            elif kind == "fem":
                parent_label = _body_label(model, parent_id, labels)
                source_label = _body_label(model, body_id, labels)
            else:
                parent_label = _body_label(model, parent_id, labels)
                source_label = part["label"]
            item = {
                "id": part["id"],
                "parent_label": parent_label,
                "source_body_label": source_label,
                "subtree_mass_kg": _subtree_mass(model, body_id) if kind != "base" else None,
                "origin_body_m": origin_body.tolist(),
                "basis_body": basis_body.ravel().tolist(),
                "action": "parent_on_child" if kind == "section" else "child_on_parent",
            }
            if kind == "base":
                unique_sources = sorted(set(base_sources))
                item["source_body_per_frame"] = base_sources
                item["subtree_mass_kg_by_source"] = {
                    name: _subtree_mass(model, int(model.body(name).id)) for name in unique_sources
                }
                item["source_labels"] = {
                    name: labels.get(name, name) for name in unique_sources
                }
            part_metadata.append(item)

        for frame_index, row in enumerate(reference):
            steps = nearest_step_count(
                float(session.data.time), row["actual_time_s"], session.config["physics_dt_s"]
            )
            session.step(steps)
            actual_state = session.snapshot()
            defined_difference = float(
                np.max(np.abs(physical_vector(actual_state) - physical_vector(row["state"])))
            )
            maximum_defined_field_difference = max(
                maximum_defined_field_difference, defined_difference
            )
            if defined_difference > 1e-9 or actual_state["phase"] != row["state"]["phase"]:
                raise RuntimeError(
                    f"physics replay mismatch at frame {frame_index}: {defined_difference}"
                )
            mujoco.mj_copyData(scratch, model, session.data)
            mujoco.mj_forward(model, scratch)
            mujoco.mj_rnePostConstraint(model, scratch)

            for part_index, (part, descriptor) in enumerate(zip(parts, descriptors)):
                kind, body_id, origin_body, basis_body = descriptor
                if kind == "fem":
                    source_body = body_id
                    origin_world = np.asarray(scratch.site_xpos[left_site_id], dtype=float)
                    rotation_world = np.asarray(scratch.site_xmat[left_site_id], dtype=float).reshape(3, 3)
                    sign = -1.0
                else:
                    frame_body = body_id
                    body_rotation = np.asarray(scratch.xmat[frame_body], dtype=float).reshape(3, 3)
                    origin_world = np.asarray(scratch.xpos[frame_body], dtype=float) + body_rotation @ origin_body
                    rotation_world = body_rotation @ basis_body
                    if kind == "base":
                        source_body = int(model.body(base_sources[frame_index]).id)
                        sign = -1.0
                    else:
                        source_body = body_id
                        sign = 1.0

                force, moment = body_interaction_wrench(
                    model, scratch, source_body, origin_world, rotation_world
                )
                actual_wrench = np.concatenate((force, moment)) * sign
                stored_wrench = source_frames[frame_index, part_index, 12:18].astype(float)
                wrench_difference = float(np.max(np.abs(actual_wrench - stored_wrench)))
                maximum_frame_wrench_difference = max(
                    maximum_frame_wrench_difference, wrench_difference
                )
                if not np.allclose(actual_wrench, stored_wrench, rtol=2e-6, atol=2e-6):
                    raise RuntimeError(
                        f"structural wrench mismatch at frame {frame_index}, part {part['id']}: "
                        f"{wrench_difference}"
                    )
                gravity, external = load_path_components(
                    model,
                    scratch,
                    source_body,
                    origin_world,
                    rotation_world,
                    action_sign=sign,
                )
                motion = residual_wrench(actual_wrench, gravity, external)
                diagnostics[frame_index, part_index] = np.concatenate(
                    (gravity, external, motion)
                )

            if frame_index % 150 == 0:
                print(
                    json.dumps(
                        {
                            "frame": frame_index,
                            "sim_time_s": float(scratch.time),
                            "defined_field_difference": defined_difference,
                            "frame_wrench_difference": maximum_frame_wrench_difference,
                        }
                    ),
                    flush=True,
                )

        encoded = diagnostics.astype("<f4")
        reconstructed = encoded[:, :, :6] + encoded[:, :, 6:12] + encoded[:, :, 12:18]
        reconstruction_difference = float(
            np.max(np.abs(reconstructed - source_frames[:, :, 12:18]))
        )
        for name in SCENE_ASSETS:
            shutil.copy2(scene / name, output / name)
        encoded.tofile(output / "diagnostics.bin")

        metadata = {
            "asset": "diagnostics.bin",
            "sha256": sha256(output / "diagnostics.bin"),
            "part_stride_floats": 18,
            "frame_stride_floats": part_count * 18,
            "terms": [
                {"id": "gravity_support", "offset": 0, "size": 6, "unit_order": ["Fx_N", "Fy_N", "Fz_N", "Mx_Nm", "My_Nm", "Mz_Nm"]},
                {"id": "external_support", "offset": 6, "size": 6, "unit_order": ["Fx_N", "Fy_N", "Fz_N", "Mx_Nm", "My_Nm", "Mz_Nm"]},
                {"id": "motion_residual", "offset": 12, "size": 6, "unit_order": ["Fx_N", "Fy_N", "Fz_N", "Mx_Nm", "My_Nm", "Mz_Nm"]},
            ],
            "parts": part_metadata,
            "interpretation": [
                "gravity_support는 선택 subtree의 중력을 절단면에서 지지하는 반력입니다.",
                "external_support는 MuJoCo cfrc_ext의 외력·접촉·구속 하중을 절단면에서 지지하는 반력입니다.",
                "motion_residual은 전체 내력에서 앞의 두 항을 뺀 회계상 잔차입니다. 관성·구동·내부 결합 효과가 섞이며 독립 원인별 기여율이 아닙니다.",
            ],
            "validation": {
                "maximum_matched_physical_defined_field_difference": maximum_defined_field_difference,
                "maximum_frame_wrench_absolute_difference": maximum_frame_wrench_difference,
                "maximum_float32_reconstruction_absolute_difference": reconstruction_difference,
                "derived_dynamics_on_scratch_only": True,
            },
            "source": {
                "scene_manifest_sha256": sha256(manifest_path),
                "reference_states_sha256": sha256(bundle / "states.json.gz"),
                "builder_sha256": sha256(__file__),
                "load_path_module_sha256": sha256(ROOT / "src/rby1_tray/load_path.py"),
            },
        }
        manifest["assets"]["diagnostics.bin"] = metadata["sha256"]
        manifest["load_diagnostics"] = metadata
        manifest["elapsed_load_diagnostics_s"] = time.monotonic() - started
        (output / "manifest.json").write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        print(
            json.dumps(
                {
                    "frames": frame_count,
                    "parts": part_count,
                    "defined_field_difference": maximum_defined_field_difference,
                    "frame_wrench_difference": maximum_frame_wrench_difference,
                    "float32_reconstruction_difference": reconstruction_difference,
                    "elapsed_s": manifest["elapsed_load_diagnostics_s"],
                }
            ),
            flush=True,
        )
    finally:
        session.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--bundle", type=Path, default=ROOT / "evidence/playback-native")
    arguments = parser.parse_args()
    build(arguments.scene, arguments.output, arguments.bundle)
