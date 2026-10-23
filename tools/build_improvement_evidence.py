#!/usr/bin/env python3
"""Publish compact, validated study metadata without modifying raw runs."""
from __future__ import annotations

import argparse
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess

from rby1_tray.experiments import STUDIES, build_run_specs, canonical_sha256
from rby1_tray.improvement_evidence import (
    ImprovementAssets, STUDY_COUNTS, assess_studies, read_json,
    replay_result_matches, result_derived_fields, sha256_file,
    trace_derived_fields, validate_video_render_contract,
)
from rby1_tray.improvement_metrics import extract_trace_metrics


ROOT = Path(__file__).resolve().parents[1]
_AUDIT_SPEC = importlib.util.spec_from_file_location(
    "transport_contact_audit", Path(__file__).with_name("audit_transport_contacts.py")
)
if _AUDIT_SPEC is None or _AUDIT_SPEC.loader is None:
    raise RuntimeError("unable to load transport contact audit")
_AUDIT_MODULE = importlib.util.module_from_spec(_AUDIT_SPEC)
_AUDIT_SPEC.loader.exec_module(_AUDIT_MODULE)
audit_run = _AUDIT_MODULE.audit_run


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def _trace_artifact_sha256(run: Path) -> str:
    plain = run / "trace.jsonl"
    if plain.is_file():
        return sha256_file(plain)
    compressed = run / "trace.jsonl.gz"
    if not compressed.is_file():
        raise ValueError("raw run trace is missing")
    with gzip.open(compressed, "rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def _recomputed_trace_fields(run: Path, result_fields: dict) -> dict:
    extracted = extract_trace_metrics(run)
    metrics = extracted.get("trace_metrics")
    row = {
        "trace_metrics": metrics,
        "trace_metrics_missing_reason": extracted["trace_metrics_missing_reason"],
        "max_abs_tray_roll_rad": metrics.get("max_abs_tray_roll_rad") if metrics else None,
        "max_abs_tray_pitch_rad": metrics.get("max_abs_tray_pitch_rad") if metrics else None,
        "max_tray_tilt_rad": metrics.get("max_tray_tilt_rad") if metrics else None,
        "max_payload_to_tray_displacement_m": (
            metrics.get("max_payload_to_tray_displacement_m") if metrics else None
        ),
    }
    row["stability_metrics"] = {
        "max_abs_tray_roll_rad": row["max_abs_tray_roll_rad"],
        "max_abs_tray_pitch_rad": row["max_abs_tray_pitch_rad"],
        "max_tray_tilt_rad": row["max_tray_tilt_rad"],
        "max_payload_to_tray_displacement_m": row[
            "max_payload_to_tray_displacement_m"
        ],
        "max_bimanual_relative_position_error_m": result_fields[
            "max_bimanual_relative_position_error_m"
        ],
        "max_bimanual_relative_orientation_error_rad": result_fields[
            "max_bimanual_relative_orientation_error_rad"
        ],
        "cooperation_minimum_speed_scale": result_fields[
            "cooperation_minimum_speed_scale"
        ],
        "cooperation_slowdown_samples": result_fields[
            "cooperation_slowdown_samples"
        ],
        "transport_scope": metrics.get("scope") if metrics else None,
        "payload_displacement_reference": (
            metrics.get("payload_displacement_reference") if metrics else None
        ),
        "payload_baseline_phase": metrics.get("payload_baseline_phase") if metrics else None,
        "payload_baseline_time_s": metrics.get("payload_baseline_time_s") if metrics else None,
        "valid_tray_pose_samples": metrics.get("valid_tray_pose_samples") if metrics else 0,
        "payload_displacement_samples": metrics.get("payload_displacement_samples") if metrics else 0,
        "missing_reason": extracted["trace_metrics_missing_reason"],
    }
    return trace_derived_fields(row)


def _verify_encoded_video(path: Path, contract: dict) -> None:
    ffprobe = shutil.which("ffprobe")
    if ffprobe is None:
        raise RuntimeError("ffprobe is required to validate study videos")
    completed = subprocess.run(
        [
            ffprobe,
            "-v", "error",
            "-count_frames",
            "-select_streams", "v:0",
            "-show_entries",
            "stream=width,height,r_frame_rate,nb_read_frames,pix_fmt,codec_name:format=duration",
            "-of", "json",
            str(path),
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    if completed.returncode:
        raise ValueError(
            f"ffprobe rejected study video: {completed.stderr.strip() or 'no diagnostics'}"
        )
    try:
        payload = json.loads(completed.stdout)
        stream = payload["streams"][0]
        duration = float(payload["format"]["duration"])
    except (KeyError, IndexError, TypeError, ValueError, json.JSONDecodeError) as exc:
        raise ValueError("ffprobe returned incomplete study video metadata") from exc
    expected = {
        "width": contract["width"],
        "height": contract["height"],
        "r_frame_rate": f"{contract['fps']}/1",
        "nb_read_frames": str(contract["frame_count"]),
        "pix_fmt": "yuv420p",
        "codec_name": "h264",
    }
    mismatch = {
        key: {"expected": value, "actual": stream.get(key)}
        for key, value in expected.items()
        if stream.get(key) != value
    }
    if mismatch:
        raise ValueError(f"encoded study video contract mismatch: {mismatch}")
    if abs(duration - contract["duration_video_s"]) > 1 / contract["fps"]:
        raise ValueError("encoded study video duration disagrees with render manifest")


def build(
    raw_root: Path,
    output: Path,
    videos: list | None = None,
    base_config_path: Path | None = None,
):
    raw_root = raw_root.resolve()
    output.mkdir(parents=True, exist_ok=False)
    base_config_path = (base_config_path or ROOT / "configs/mobile-native.json").resolve()
    base_config = read_json(base_config_path)
    shutil.copyfile(base_config_path, output / "base-config.json")
    studies, audits, commits, source_snapshots, base_hashes, run_rows = (
        {}, {}, set(), set(), set(), {}
    )
    report = {
        "schema_version": 1,
        "base_config": "base-config.json",
        "studies": {},
        "videos": [],
    }
    for name in STUDY_COUNTS:
        source = raw_root / name
        destination = output / name
        destination.mkdir()
        for filename in ("summary.json", "preregistered.json", "dataset-manifest.json"):
            shutil.copyfile(source / filename, destination / filename)
        summary = read_json(source / "summary.json")
        prereg = read_json(source / "preregistered.json")
        studies[name] = summary
        commits.add(prereg["provenance"]["commit"])
        source_snapshots.add(canonical_sha256(prereg["source_sha256"]))
        base_hashes.add(prereg["base_config_sha256"])
        if prereg["base_config_sha256"] != canonical_sha256(base_config):
            raise ValueError("frozen base config does not match study preregistration")
        if prereg["source_sha256"].get("configs/mobile-native.json") != sha256_file(base_config_path):
            raise ValueError("base config bytes do not match study source snapshot")
        specs = {
            item["run_id"]: item
            for item in build_run_specs(
                base_config,
                STUDIES[name].cases,
                STUDIES[name].seeds,
                STUDIES[name].variants,
            )
        }
        rows = summary.get("runs")
        if (
            not isinstance(rows, list)
            or len(rows) != len(specs)
            or any(not isinstance(row, dict) for row in rows)
            or {row.get("run_id") for row in rows} != set(specs)
        ):
            raise ValueError("summary run inventory differs from the frozen study matrix")
        report["studies"][name] = {"summary": f"{name}/summary.json"}
        for row in rows:
            run_id = row["run_id"]
            raw_run = source / run_id
            compact_run = destination / "runs" / run_id
            compact_run.mkdir(parents=True)
            for filename in ("manifest.json", "config.json", "result.json"):
                shutil.copyfile(raw_run / filename, compact_run / filename)
            raw_manifest = read_json(raw_run / "manifest.json")
            result = read_json(raw_run / "result.json")
            result_fields = result_derived_fields(specs[run_id], result)
            if any(row.get(key) != value for key, value in result_fields.items()):
                raise ValueError("summary result-derived fields disagree with result.json")
            trace_sha256 = _trace_artifact_sha256(raw_run)
            if trace_sha256 != raw_manifest.get("artifact_sha256", {}).get("trace.jsonl"):
                raise ValueError("raw trace bytes do not match run manifest")
            trace_fields = _recomputed_trace_fields(raw_run, result_fields)
            if trace_derived_fields(row) != trace_fields:
                raise ValueError("summary trace-derived fields disagree with raw trace")
            write_json(compact_run / "derived-validation.json", {
                "schema_version": 1,
                "source_run_manifest_sha256": sha256_file(raw_run / "manifest.json"),
                "source_result_sha256": sha256_file(raw_run / "result.json"),
                "source_trace_sha256": trace_sha256,
                "trace_derived_fields": trace_fields,
            })
            run_rows[(name, run_id)] = row
            audit_path = raw_run / "transport-audit.json"
            if audit_path.is_file():
                supplied_audit = read_json(audit_path)
                authoritative_audit = audit_run(raw_run)
                if supplied_audit != authoritative_audit:
                    raise ValueError("transport audit disagrees with frozen raw run")
                write_json(compact_run / audit_path.name, authoritative_audit)
                audits[f"{name}/{run_id}"] = authoritative_audit
    if len(commits) != 1 or len(source_snapshots) != 1 or len(base_hashes) != 1:
        raise ValueError("studies must share one frozen source snapshot and base config")
    report["source_commit"] = commits.pop()
    report["assessment"] = assess_studies(studies, audits)
    for index, video in enumerate(videos or []):
        if (
            not isinstance(video, dict)
            or "source_manifest" not in video
            or not set(video).issubset({"source_manifest", "label"})
        ):
            raise ValueError("video input accepts only source_manifest and optional label")
        source_manifest_path = Path(video.get("source_manifest", "")).resolve()
        render_manifest = read_json(source_manifest_path)
        source_run = Path(render_manifest.get("source_run", "")).resolve()
        try:
            relative_run = source_run.relative_to(raw_root)
        except ValueError as exc:
            raise ValueError("render manifest source run is outside raw study root") from exc
        if len(relative_run.parts) != 2:
            raise ValueError("render manifest source run must identify one study run")
        study_name, run_id = relative_run.parts
        row = run_rows.get((study_name, run_id))
        if row is None:
            raise ValueError("render manifest source run is not in a registered study")
        source_video = source_manifest_path.with_name("video.mp4")
        render_contract = validate_video_render_contract(render_manifest)
        if (
            render_manifest.get("artifact_sha256", {}).get("video.mp4")
            != sha256_file(source_video)
        ):
            raise ValueError("rendered video hash does not match its manifest")
        _verify_encoded_video(source_video, render_contract)
        source_run_manifest_sha256 = sha256_file(source_run / "manifest.json")
        source_config_sha256 = sha256_file(source_run / "config.json")
        source_result_sha256 = sha256_file(source_run / "result.json")
        source_result = read_json(source_run / "result.json")
        validated_result = render_manifest.get("validated_result")
        source_run_manifest = read_json(source_run / "manifest.json")
        playback_speed = render_contract["base_speed"]
        if (
            render_manifest.get("schema_version") != 1
            or render_manifest.get("case") != row["case"]
            or render_manifest.get("seed") != row["seed"]
            or render_manifest.get("profile") != row["variant"]
            or render_manifest.get("config_sha256") != row["config_sha256"]
            or render_manifest.get("source_run_manifest_sha256")
            != source_run_manifest_sha256
            or source_run_manifest_sha256 != row["run_manifest_sha256"]
            or render_manifest.get("source_commit") != report["source_commit"]
            or render_manifest.get("replay_source_commit") != report["source_commit"]
            or render_manifest.get("source_sha256")
            != source_run_manifest.get("source_sha256")
            or render_manifest.get("replay_source_dirty") is not False
            or render_manifest.get("config_file_sha256") != source_config_sha256
            or render_manifest.get("result_file_sha256") != source_result_sha256
            or not isinstance(validated_result, dict)
            or not replay_result_matches(
                validated_result,
                source_result,
                render_manifest.get("result_match_tolerance"),
            )
            or isinstance(playback_speed, bool)
            or not isinstance(playback_speed, (int, float))
            or playback_speed <= 0
        ):
            raise ValueError("render manifest does not match its registered source run")
        destination = output / "videos"
        destination.mkdir(exist_ok=True)
        relative = f"videos/{index}.mp4"
        manifest_relative = f"videos/{index}.manifest.json"
        shutil.copyfile(source_video, output / relative)
        shutil.copyfile(source_manifest_path, output / manifest_relative)
        report["videos"].append({
            "path": relative,
            "manifest": manifest_relative,
            "label": video.get("label"),
            "study": study_name,
            "run_id": run_id,
            "case": row["case"],
            "seed": row["seed"],
            "profile": row["variant"],
            "config_sha256": row["config_sha256"],
            "source_commit": report["source_commit"],
            "source_run_manifest_sha256": source_run_manifest_sha256,
            "success": row["success"],
            "failure_reason": row.get("failure_reason"),
            "video_sha256": sha256_file(source_video),
            "playback_speed": playback_speed,
        })
    write_json(output / "report.json", report)
    write_json(output / "manifest.json", {
        "schema_version": 1,
        "scope": "compact run metadata and media; raw RGB-D and command/trace archives excluded",
        "artifact_sha256": {path.relative_to(output).as_posix(): sha256_file(path)
                            for path in sorted(output.rglob("*")) if path.is_file()},
    })
    return ImprovementAssets(output).info()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("raw_root", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument(
        "--videos",
        type=Path,
        help="JSON list of source_manifest paths and optional display labels",
    )
    parser.add_argument(
        "--base-config",
        type=Path,
        default=ROOT / "configs/mobile-native.json",
        help="exact base config used by the preregistered studies",
    )
    args = parser.parse_args()
    info = build(
        args.raw_root,
        args.output,
        read_json(args.videos) if args.videos else None,
        args.base_config,
    )
    print(json.dumps({"enabled": info["enabled"], "assessment": info["assessment"]}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
