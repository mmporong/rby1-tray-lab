#!/usr/bin/env python3
"""Reconstruct recorded raise-high tray/container contacts from run evidence.

The audit intentionally evaluates only recorded trace samples.  It does not
claim continuous 500 Hz collision coverage between those samples.
"""

from __future__ import annotations

import argparse
import gzip
import hashlib
import json
from pathlib import Path
import sys
from typing import Any, Iterator
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


TRAY_GEOMS = ("tray_floor", "tray_xrim_-1", "tray_xrim_1", "tray_yrim_-1", "tray_yrim_1")
WALL_GEOMS = (
    "container_xwall_-1",
    "container_xwall_1",
    "container_ywall_-1",
    "container_ywall_1",
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _trace_path(run: Path) -> Path:
    plain = run / "trace.jsonl"
    compressed = run / "trace.jsonl.gz"
    if plain.is_file():
        return plain
    if compressed.is_file():
        return compressed
    raise FileNotFoundError(f"trace.jsonl(.gz) not found in {run}")


def _decompressed_trace_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    opener = gzip.open if path.suffix == ".gz" else open
    with opener(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _trace_rows(path: Path) -> Iterator[dict[str, Any]]:
    opener = gzip.open if path.suffix == ".gz" else open
    with opener(path, "rt", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if line.strip():
                try:
                    yield json.loads(line)
                except json.JSONDecodeError as exc:
                    raise ValueError(f"invalid trace JSON at line {line_number}") from exc


def _verify_run(run: Path) -> tuple[dict[str, Any], dict[str, Any], Path]:
    manifest_path = run / "manifest.json"
    config_path = run / "config.json"
    if not manifest_path.is_file() or not config_path.is_file():
        raise FileNotFoundError("run requires manifest.json and config.json")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    artifacts = manifest.get("artifact_sha256", {})
    expected_config = artifacts.get("config.json")
    if expected_config is None or _sha256(config_path) != expected_config:
        raise RuntimeError("config.json does not match the run manifest")
    trace_path = _trace_path(run)
    expected_trace = artifacts.get("trace.jsonl")
    actual_trace = _decompressed_trace_sha256(trace_path)
    if expected_trace is None or actual_trace != expected_trace:
        raise RuntimeError("decompressed trace does not match the run manifest")
    for filename in ("scene.xml", "ik.json"):
        expected = artifacts.get(filename)
        if expected is None or _sha256(run / filename) != expected:
            raise RuntimeError(f"{filename} does not match the run manifest")
    return manifest, json.loads(config_path.read_text(encoding="utf-8")), trace_path


def _numbers(values: np.ndarray) -> str:
    return " ".join(f"{float(value):.17g}" for value in values)


def _extract_collision_model(run: Path) -> tuple[mujoco.MjModel, mujoco.MjData]:
    scene_path = run / "scene.xml"
    if not scene_path.is_file():
        raise FileNotFoundError("scene.xml is required to recover compiled geom values")
    source = mujoco.MjModel.from_xml_path(str(scene_path))
    root = ET.Element("mujoco", model="recorded_transport_contact_audit")
    ET.SubElement(root, "option", timestep="0.002", gravity="0 0 0")
    world = ET.SubElement(root, "worldbody")
    tray_body_id = source.body("tray").id
    tray = ET.SubElement(world, "body", name="tray")
    ET.SubElement(tray, "freejoint", name="tray_free")
    for name in TRAY_GEOMS:
        geom_id = source.geom(name).id
        if int(source.geom_type[geom_id]) != int(mujoco.mjtGeom.mjGEOM_BOX):
            raise RuntimeError(f"{name} is not a compiled box geom")
        if int(source.geom_bodyid[geom_id]) != tray_body_id:
            raise RuntimeError(f"{name} is not attached to the tray body")
        ET.SubElement(
            tray,
            "geom",
            name=name,
            type="box",
            pos=_numbers(source.geom_pos[geom_id]),
            quat=_numbers(source.geom_quat[geom_id]),
            size=_numbers(source.geom_size[geom_id]),
            contype="1",
            conaffinity="1",
        )
    for name in WALL_GEOMS:
        geom_id = source.geom(name).id
        if int(source.geom_type[geom_id]) != int(mujoco.mjtGeom.mjGEOM_BOX):
            raise RuntimeError(f"{name} is not a compiled box geom")
        body_id = int(source.geom_bodyid[geom_id])
        if body_id != 0:
            raise RuntimeError(f"{name} is not a static world geom")
        ET.SubElement(
            world,
            "geom",
            name=name,
            type="box",
            pos=_numbers(source.geom_pos[geom_id]),
            quat=_numbers(source.geom_quat[geom_id]),
            size=_numbers(source.geom_size[geom_id]),
            contype="1",
            conaffinity="1",
        )
    xml = ET.tostring(root, encoding="unicode")
    model = mujoco.MjModel.from_xml_string(xml)
    return model, mujoco.MjData(model)


def _minimum_preflight_clearance(run: Path) -> float | None:
    ik_path = run / "ik.json"
    if not ik_path.is_file():
        return None
    rows = json.loads(ik_path.read_text(encoding="utf-8"))
    values = []
    for row in rows:
        if row.get("phase") != "raise_high":
            continue
        transport = row.get("transport_clearance")
        if isinstance(transport, dict):
            value = transport.get("minimum_preflight_clearance_m")
            if value is not None:
                values.append(float(value))
    return min(values) if values else None


def audit_run(run_directory: str | Path) -> dict[str, Any]:
    run = Path(run_directory).resolve()
    manifest, config, trace_path = _verify_run(run)
    model, data = _extract_collision_model(run)
    tray_joint = model.joint("tray_free")
    qpos_address = int(model.jnt_qposadr[tray_joint.id])
    tray_ids = {model.geom(name).id for name in TRAY_GEOMS}
    wall_ids = {model.geom(name).id for name in WALL_GEOMS}
    contacts: list[dict[str, Any]] = []
    frame_count = 0
    minimum_distance: float | None = None
    for row in _trace_rows(trace_path):
        if row.get("phase") != "raise_high":
            continue
        frame_count += 1
        position = np.asarray(row["tray"]["pos_m"], dtype=float)
        quaternion = np.asarray(row["tray"]["quat_wxyz"], dtype=float)
        if position.shape != (3,) or quaternion.shape != (4,) or not np.all(np.isfinite(np.r_[position, quaternion])):
            raise ValueError("raise_high trace row contains an invalid tray pose")
        quaternion_norm = float(np.linalg.norm(quaternion))
        if quaternion_norm <= 1e-12:
            raise ValueError("raise_high trace row contains a zero tray quaternion")
        data.qpos[qpos_address : qpos_address + 3] = position
        data.qpos[qpos_address + 3 : qpos_address + 7] = quaternion / quaternion_norm
        mujoco.mj_forward(model, data)
        pairs = []
        distances = []
        for contact in data.contact[: data.ncon]:
            first, second = int(contact.geom1), int(contact.geom2)
            if not ((first in tray_ids and second in wall_ids) or (second in tray_ids and first in wall_ids)):
                continue
            names = sorted((model.geom(first).name, model.geom(second).name))
            distance = float(contact.dist)
            pairs.append({"pair": names, "distance_m": distance})
            distances.append(distance)
        if distances:
            frame_minimum = min(distances)
            minimum_distance = frame_minimum if minimum_distance is None else min(minimum_distance, frame_minimum)
            contacts.append(
                {
                    "time_s": float(row["time_s"]),
                    "collision_pair_count": len(pairs),
                    "minimum_contact_distance_m": frame_minimum,
                    "pairs": sorted(pairs, key=lambda item: (item["pair"], item["distance_m"])),
                }
            )
    return {
        "schema_version": 1,
        "scope": "recorded_raise_high_frames_only",
        "sampling_limit": "recorded trace samples only; no continuous 500 Hz contact claim",
        "sample_period_s": float(config["record_dt_s"]),
        "frame_count": frame_count,
        "contact_frame_count": len(contacts),
        "first_contact_time_s": contacts[0]["time_s"] if contacts else None,
        "minimum_contact_distance_m": minimum_distance,
        "contacts": contacts,
        "minimum_preflight_clearance_m": _minimum_preflight_clearance(run),
        "source_trace_sha256": _decompressed_trace_sha256(trace_path),
        "run_manifest_sha256": _sha256(run / "manifest.json"),
        "source_commit": manifest.get("commit"),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Audit recorded raise-high tray/container contacts")
    parser.add_argument("run", help="validated raw run directory")
    parser.add_argument("--output", help="output JSON (default: RUN/transport-audit.json)")
    args = parser.parse_args(argv)
    run = Path(args.run).resolve()
    output = Path(args.output).resolve() if args.output else run / "transport-audit.json"
    report = audit_run(run)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
