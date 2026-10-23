"""Validated, read-only compact evidence for the paired improvement study."""
from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
import re
from typing import Any, Mapping, Sequence

from .experiments import (
    CASES,
    STUDIES,
    VARIANTS,
    _result_record,
    build_run_specs,
    canonical_sha256,
    summarize,
)


STUDY_COUNTS = {"improvement": 24, "safety": 4, "cooperation": 16}

VIDEO_PROVENANCE = (
    "fresh deterministic MobileSession rerun of the source config; not trace interpolation"
)
VIDEO_SAMPLING = (
    "nearest physics state to i/fps*base_speed; terminal frame appended by the deterministic rerun"
)
VIDEO_BASE_SPEED = 8
VIDEO_FPS = 60
VIDEO_WIDTH = 960
VIDEO_HEIGHT = 720


PAIR_METRICS = {
    "task_time_s": "simulation_time_s",
    "max_joint_torque_nm": "max_joint_torque_nm",
    "max_tray_tilt_rad": "max_tray_tilt_rad",
    "max_payload_to_tray_displacement_m": "max_payload_to_tray_displacement_m",
    "max_bimanual_relative_position_error_m": "max_bimanual_relative_position_error_m",
    "max_bimanual_relative_orientation_error_rad": "max_bimanual_relative_orientation_error_rad",
}


def _row_metric(row: Mapping[str, Any], key: str) -> float | None:
    value = row.get(key)
    if value is None and isinstance(row.get("stability_metrics"), Mapping):
        value = row["stability_metrics"].get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value)


def _summary(values: Sequence[float]) -> dict[str, float | int] | None:
    if not values:
        return None
    return {
        "n": len(values),
        "mean": sum(values) / len(values),
        "min": min(values),
        "max": max(values),
    }


def cooperation_effect(summary: Mapping[str, Any]) -> dict[str, Any]:
    """Compare exact cooperation on/off pairs; deltas are on minus off."""
    pairs: dict[tuple[str, int, str, str], dict[bool, Mapping[str, Any]]] = {}
    for row in summary.get("runs", []):
        if row.get("execution_status") != "completed":
            continue
        key = (
            str(row["case"]),
            int(row["seed"]),
            str(row["estimator"]),
            str(row["controller_profile"]),
        )
        enabled = bool(row["cooperation_enabled"])
        if enabled in pairs.setdefault(key, {}):
            raise ValueError(f"duplicate cooperation effect observation: {key}")
        pairs[key][enabled] = row

    records = []
    deltas: dict[str, list[float]] = {name: [] for name in PAIR_METRICS}
    complete_pairs = 0
    both_success = 0
    cooperation_only_success = 0
    no_cooperation_only_success = 0
    both_failed = 0
    for key, observations in sorted(pairs.items()):
        if set(observations) != {False, True}:
            continue
        complete_pairs += 1
        enabled, disabled = observations[True], observations[False]
        enabled_success = bool(enabled.get("success"))
        disabled_success = bool(disabled.get("success"))
        if enabled_success and disabled_success:
            both_success += 1
        elif enabled_success:
            cooperation_only_success += 1
        elif disabled_success:
            no_cooperation_only_success += 1
        else:
            both_failed += 1
        record = {
            "case": key[0],
            "seed": key[1],
            "estimator": key[2],
            "controller_profile": key[3],
            "cooperation_success": enabled_success,
            "no_cooperation_success": disabled_success,
            "both_success": enabled_success and disabled_success,
            "deltas": {},
        }
        for label, row_key in PAIR_METRICS.items():
            delta = None
            if enabled_success and disabled_success:
                enabled_value = _row_metric(enabled, row_key)
                disabled_value = _row_metric(disabled, row_key)
                if enabled_value is not None and disabled_value is not None:
                    delta = enabled_value - disabled_value
                    deltas[label].append(delta)
            record["deltas"][label] = delta
        records.append(record)
    return {
        "sign": "coop_minus_no_coop",
        "complete_pairs": complete_pairs,
        "both_success_pairs": both_success,
        "candidate_pairs": len(pairs),
        "missing_variant_pairs": len(pairs) - complete_pairs,
        "cooperation_only_success": cooperation_only_success,
        "no_cooperation_only_success": no_cooperation_only_success,
        "both_failed": both_failed,
        "metric_deltas": {name: _summary(values) for name, values in deltas.items()},
        "pairs": records,
    }


def _expected_preregistration(name: str) -> dict[str, Any]:
    study = STUDIES[name]
    return {
        "study": name,
        "study_purpose": study.purpose,
        "cases": [
            {
                "name": case,
                "factor": CASES[case].factor,
                "values": dict(CASES[case].values),
                "expected_outcome": CASES[case].expected_outcome,
            }
            for case in study.cases
        ],
        "seeds": list(study.seeds),
        "variants": [
            {
                "name": variant,
                "cooperation_enabled": VARIANTS[variant].cooperation_enabled,
                "estimator": VARIANTS[variant].estimator,
                "controller_profile": VARIANTS[variant].controller_profile,
            }
            for variant in study.variants
        ],
    }


def _validate_preregistered_matrix(name: str, prereg: Mapping[str, Any]) -> None:
    expected = _expected_preregistration(name)
    if prereg.get("schema_version") != 2:
        raise ValueError(f"unsupported preregistration schema: {name}")
    for key, value in expected.items():
        if prereg.get(key) != value:
            raise ValueError(f"study definition mismatch for {name}: {key}")
    study = STUDIES[name]
    expected_rows = []
    for case in study.cases:
        for seed in study.seeds:
            for variant_name in study.variants:
                variant = VARIANTS[variant_name]
                expected_rows.append(
                    {
                        "run_id": f"{case}__{variant_name}__seed-{seed}",
                        "case": case,
                        "factor": CASES[case].factor,
                        "expected_outcome": CASES[case].expected_outcome,
                        "seed": seed,
                        "variant": variant_name,
                        "cooperation_enabled": variant.cooperation_enabled,
                        "estimator": variant.estimator,
                        "controller_profile": variant.controller_profile,
                    }
                )
    matrix = prereg.get("run_matrix")
    if not isinstance(matrix, list) or len(matrix) != len(expected_rows):
        raise ValueError(f"unexpected run matrix size: {name}")
    matrix_keys = {
        "run_id", "case", "factor", "expected_outcome", "seed", "variant",
        "cooperation_enabled", "estimator", "controller_profile",
        "target_height_m", "config_sha256",
    }
    if any(not isinstance(row, Mapping) or set(row) != matrix_keys for row in matrix):
        raise ValueError(f"unexpected run matrix fields: {name}")
    keys = tuple(expected_rows[0])
    observed = [{key: row.get(key) for key in keys} for row in matrix]
    if observed != expected_rows:
        raise ValueError(f"run matrix differs from frozen study definition: {name}")
    if prereg.get("expected_runs") != len(expected_rows):
        raise ValueError(f"unexpected preregistered run count: {name}")


def assess_studies(studies: dict, audits: dict) -> dict:
    """Evaluate the preregistered gates; do not infer general success rates."""
    if set(studies) != set(STUDIES) or any(
        not studies[name].get("complete") for name in STUDIES
    ):
        return {"status": "incomplete", "label": "실험 미완료", "summary": "실행 오류 또는 누락이 있어 개선 여부를 판정하지 않았습니다."}
    rows = studies["improvement"]["runs"]
    candidates = [r for r in rows if r["controller_profile"] == "geometry_clearance"]
    mandatory = {"nominal", "tray-yaw-0.035", "tray-yaw-minus-0.035", "tray-yaw-0.025-heldout"}
    mandatory_candidates = [r for r in candidates if r["case"] in mandatory]
    core = len(mandatory_candidates) == len(mandatory) * 2 and all(
        r["success"] for r in mandatory_candidates
    )
    pairs = {(r["case"], r["seed"]): r for r in rows if r["controller_profile"] == "legacy"}
    expected_pair_keys = {
        (case, seed)
        for case in STUDIES["improvement"].cases
        for seed in STUDIES["improvement"].seeds
    }
    no_regression = (
        set(pairs) == expected_pair_keys
        and len(candidates) == len(expected_pair_keys)
        and all(
            not pairs[(r["case"], r["seed"])]["success"] or r["success"]
            for r in candidates
        )
    )
    transport = all(
        (audit := audits.get("improvement/" + r["run_id"])) is not None
        and audit.get("frame_count", 0) > 0
        and audit.get("contact_frame_count") == 0
        and audit.get("minimum_preflight_clearance_m") is not None
        and audit["minimum_preflight_clearance_m"] >= .03 - 1e-9
        for r in mandatory_candidates
    ) and len(mandatory_candidates) == len(mandatory) * 2
    safety_rows = studies["safety"]["runs"]
    safety = len(safety_rows) == STUDY_COUNTS["safety"] and all(
        r.get("expected_rejection_observed") is True for r in safety_rows
    )
    passed = core and no_regression and transport and safety
    return {
        "status": "passed" if passed else "partial",
        "label": "사전 지정 조건 통과" if passed else "개선 조건 일부 미통과",
        "summary": "동일 소스·동일 seed의 조건별 비교입니다. 실물 성능이나 일반 환경 성공률을 뜻하지 않습니다.",
        "gates": {"core_task_success": core, "no_paired_regression": no_regression,
                  "sampled_transport_clearance": transport, "safety_rejections": safety},
    }


def sha256_file(path: Path) -> str:
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def read_json(path: Path) -> Any:
    def reject_constant(value: str) -> None:
        raise ValueError(f"nonfinite JSON value: {value}")
    return json.loads(path.read_text(encoding="utf-8"), parse_constant=reject_constant)


def replay_result_matches(
    replay: Mapping[str, Any], result: Mapping[str, Any], tolerance: float
) -> bool:
    """Match the deterministic replay fields validated by the video renderer."""
    if isinstance(tolerance, bool) or not isinstance(tolerance, (int, float)):
        return False
    if tolerance < 0:
        return False

    def matches(expected: Any, actual: Any) -> bool:
        if isinstance(expected, bool) or expected is None or isinstance(expected, str):
            return expected == actual
        if isinstance(expected, (int, float)) and isinstance(actual, (int, float)):
            if isinstance(actual, bool):
                return False
            return abs(float(expected) - float(actual)) <= float(tolerance)
        return False

    for key in ("success", "failure_reason", "simulation_time_s"):
        if not matches(result.get(key), replay.get(key)):
            return False
    if "failure_time_s" in result and not matches(
        result.get("failure_time_s"), replay.get("failure_time_s")
    ):
        return False
    expected_metrics = result.get("metrics")
    replay_metrics = replay.get("metrics")
    if not isinstance(expected_metrics, Mapping) or not isinstance(replay_metrics, Mapping):
        return False
    return set(expected_metrics) == set(replay_metrics) and all(
        matches(expected, replay_metrics[key])
        for key, expected in expected_metrics.items()
    )


def validate_video_render_contract(manifest: Mapping[str, Any]) -> dict[str, Any]:
    """Validate the fixed 8x/60 FPS overview rendering contract."""
    rendering = manifest.get("rendering")
    result = manifest.get("validated_result")
    if not isinstance(rendering, Mapping) or not isinstance(result, Mapping):
        raise ValueError("render manifest is missing rendering or result metadata")
    exact = {
        "base_speed": VIDEO_BASE_SPEED,
        "fps": VIDEO_FPS,
        "width": VIDEO_WIDTH,
        "height": VIDEO_HEIGHT,
        "view": "overview",
        "sampling": VIDEO_SAMPLING,
    }
    if manifest.get("provenance_statement") != VIDEO_PROVENANCE or any(
        rendering.get(key) != value for key, value in exact.items()
    ):
        raise ValueError("render manifest violates the fixed overview video contract")
    frame_count = rendering.get("frame_count")
    duration_video = rendering.get("duration_video_s")
    duration_sim = rendering.get("duration_sim_s")
    simulation_time = result.get("simulation_time_s")
    if (
        isinstance(frame_count, bool)
        or not isinstance(frame_count, int)
        or frame_count < 2
    ):
        raise ValueError("render manifest has invalid frame or duration values")
    durations: list[float] = []
    for value in (duration_video, duration_sim, simulation_time):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError("render manifest has invalid frame or duration values")
        durations.append(float(value))
    duration_video, duration_sim, simulation_time = durations
    expected_video_duration = frame_count / VIDEO_FPS
    if abs(float(duration_video) - expected_video_duration) > 1e-9:
        raise ValueError("rendered frame count and video duration disagree")
    if abs(float(duration_sim) - float(simulation_time)) > 1e-9:
        raise ValueError("rendered simulation duration disagrees with validated result")
    sample_period = VIDEO_BASE_SPEED / VIDEO_FPS
    lower = (frame_count - 2) * sample_period
    upper = (frame_count - 1) * sample_period
    if not lower - 1e-9 <= float(duration_sim) <= upper + 1e-9:
        raise ValueError("rendered frame count and simulation duration disagree")
    return {**exact, "frame_count": frame_count, "duration_video_s": float(duration_video)}


def result_derived_fields(
    registered: Mapping[str, Any], result: Mapping[str, Any]
) -> dict[str, Any]:
    """Recreate every summary field derived from result.json."""
    expected = _result_record(registered, result)
    return {
        key: expected[key]
        for key in expected
        if key not in {
            "run_id", "case", "factor", "expected_outcome", "seed", "variant",
            "cooperation_enabled", "estimator", "controller_profile",
            "target_height_m", "config_sha256",
        }
    }


def trace_derived_fields(row: Mapping[str, Any]) -> dict[str, Any]:
    """Select fields whose values require the separately retained raw trace."""
    return {
        key: copy.deepcopy(row.get(key))
        for key in (
            "trace_metrics",
            "trace_metrics_missing_reason",
            "max_abs_tray_roll_rad",
            "max_abs_tray_pitch_rad",
            "max_tray_tilt_rad",
            "max_payload_to_tray_displacement_m",
            "stability_metrics",
        )
    }


def safe_path(directory: Path, relative: str) -> Path:
    path = Path(relative)
    if not relative or path.is_absolute() or ".." in path.parts or "\\" in relative:
        raise ValueError("invalid evidence path")
    resolved = (directory / path).resolve()
    if not resolved.is_relative_to(directory.resolve()):
        raise ValueError("evidence path escapes bundle")
    return resolved


class ImprovementAssets:
    """Require manifest, source/config and run-result agreement before display.

    Compact bundles retain run manifests, configurations and terminal results.
    Full RGB-D/command/trace bytes remain in the raw experiment archive; their
    original digests are retained, but are not claimed as revalidated here.
    """

    def __init__(self, directory: str | Path):
        self.directory = Path(directory).resolve()
        manifest = read_json(self.directory / "manifest.json")
        if manifest.get("schema_version") != 1:
            raise ValueError("unsupported improvement evidence schema")
        hashes = manifest.get("artifact_sha256")
        if not isinstance(hashes, dict) or "report.json" not in hashes:
            raise ValueError("missing improvement artifact hashes")
        for relative, digest in hashes.items():
            if not isinstance(digest, str) or not re.fullmatch(r"[0-9a-f]{64}", digest):
                raise ValueError("invalid artifact digest")
            if sha256_file(safe_path(self.directory, relative)) != digest:
                raise ValueError(f"improvement artifact hash mismatch: {relative}")
        report = read_json(self.directory / "report.json")
        if report.get("schema_version") != 1 or set(report.get("studies", {})) != set(STUDY_COUNTS):
            raise ValueError("incomplete improvement study inventory")
        base_config_path = report.get("base_config")
        if base_config_path != "base-config.json" or base_config_path not in hashes:
            raise ValueError("missing hashed frozen base config")
        base_config = read_json(safe_path(self.directory, base_config_path))
        commits = set()
        source_snapshots = set()
        base_config_hashes = set()
        self.studies = {}
        audits = {}
        all_runs = {}
        all_run_sources = {}
        for name, count in STUDY_COUNTS.items():
            spec = report["studies"][name]
            required = [f"{name}/{file}" for file in ("summary.json", "preregistered.json", "dataset-manifest.json")]
            if any(path not in hashes for path in required):
                raise ValueError(f"unhashed study metadata: {name}")
            summary, prereg, dataset = [read_json(safe_path(self.directory, path)) for path in required]
            if spec.get("summary") != required[0]:
                raise ValueError("study summary path mismatch")
            _validate_preregistered_matrix(name, prereg)
            base_config_hash = prereg.get("base_config_sha256")
            if base_config_hash != canonical_sha256(base_config):
                raise ValueError("study base config does not match frozen base config")
            base_config_hashes.add(base_config_hash)
            source_snapshot = prereg.get("source_sha256")
            if not isinstance(source_snapshot, dict):
                raise ValueError("study source snapshot missing")
            source_snapshots.add(canonical_sha256(source_snapshot))
            config_source_digest = source_snapshot.get("configs/mobile-native.json")
            if config_source_digest != hashes[base_config_path]:
                raise ValueError("base config bytes disagree with source snapshot")
            if summary.get("expected_runs") != count or prereg.get("expected_runs") != count:
                raise ValueError(f"unexpected study matrix size: {name}")
            matrix = {row["run_id"]: row for row in prereg["run_matrix"]}
            expected_specs = {
                spec["run_id"]: spec
                for spec in build_run_specs(
                    base_config,
                    STUDIES[name].cases,
                    STUDIES[name].seeds,
                    STUDIES[name].variants,
                )
            }
            rows = summary.get("runs", [])
            if len(matrix) != count or len(rows) != count or len({r["run_id"] for r in rows}) != count:
                raise ValueError("incomplete or duplicate run matrix")
            if set(matrix) != {row["run_id"] for row in rows}:
                raise ValueError("run matrix differs from preregistration")
            if summarize(rows, count) != summary:
                raise ValueError("study aggregate disagrees with run rows")
            commit = prereg["provenance"].get("commit")
            if not isinstance(commit, str) or not re.fullmatch(r"[0-9a-f]{40}", commit):
                raise ValueError("missing source commit")
            if prereg["provenance"].get("source_dirty_at_start") is not False:
                raise ValueError("study source tree was not clean")
            commits.add(commit)
            if dataset.get("schema_version") != 1:
                raise ValueError("unsupported dataset manifest schema")
            if dataset.get("source_sha256") != prereg.get("source_sha256"):
                raise ValueError("study source snapshot mismatch")
            if dataset.get("complete") is not True or summary.get("complete") is not True:
                raise ValueError("compact evidence requires a complete study dataset")
            for file in ("summary.json", "preregistered.json"):
                if dataset["artifact_sha256"].get(file) != hashes[f"{name}/{file}"]:
                    raise ValueError("dataset metadata digest mismatch")
            dataset_manifests = dataset.get("run_manifests")
            if not isinstance(dataset_manifests, dict) or set(dataset_manifests) != set(matrix):
                raise ValueError("dataset run manifest inventory mismatch")
            normalized_preregistered_sources = {
                key.removeprefix("src/rby1_tray/"): value
                for key, value in prereg["source_sha256"].items()
                if key.startswith("src/rby1_tray/")
            }
            if not normalized_preregistered_sources:
                raise ValueError("preregistered package source snapshot is empty")
            for row in rows:
                run_id = row["run_id"]
                registered = matrix[run_id]
                for key in (
                    "case", "factor", "expected_outcome", "seed", "variant",
                    "config_sha256", "controller_profile", "cooperation_enabled",
                    "estimator", "target_height_m",
                ):
                    if registered.get(key) != row.get(key):
                        raise ValueError(f"preregistered {key} mismatch: {run_id}")
                prefix = f"{name}/runs/{run_id}"
                paths = [
                    f"{prefix}/{file}"
                    for file in (
                        "manifest.json", "config.json", "result.json",
                        "derived-validation.json",
                    )
                ]
                if any(path not in hashes for path in paths):
                    raise ValueError("missing compact run metadata")
                run_manifest, config, result, derived = [
                    read_json(safe_path(self.directory, path)) for path in paths
                ]
                if hashes[paths[0]] != row.get("run_manifest_sha256"):
                    raise ValueError("run manifest digest mismatch")
                dataset_entry = dataset_manifests[run_id]
                if dataset_entry != {
                    "path": row.get("run_manifest"),
                    "sha256": row.get("run_manifest_sha256"),
                }:
                    raise ValueError("dataset run manifest link mismatch")
                if run_manifest.get("commit") != commit or canonical_sha256(config) != row["config_sha256"]:
                    raise ValueError("run source/config provenance mismatch")
                expected_spec = expected_specs[run_id]
                if config != expected_spec["config"]:
                    raise ValueError("run config differs from frozen case/variant/seed")
                if (
                    config.get("seed") != row["seed"]
                    or config.get("cooperation", {}).get("enabled")
                    != row["cooperation_enabled"]
                    or config.get("perception", {}).get("estimator") != row["estimator"]
                    or config.get("target", {}).get("support_z_m")
                    != row["target_height_m"]
                ):
                    raise ValueError("run config disagrees with registered factors")
                configured_profile = config.get("transport_planning", {}).get(
                    "profile", "legacy"
                )
                if configured_profile != row["controller_profile"]:
                    raise ValueError("run config controller profile mismatch")
                run_sources = run_manifest.get("source_sha256")
                if not isinstance(run_sources, dict):
                    raise ValueError("run source snapshot missing")
                if run_sources != normalized_preregistered_sources:
                    raise ValueError("run and preregistered source snapshots disagree")
                for file, path in zip(("config.json", "result.json"), paths[1:3]):
                    if run_manifest["artifact_sha256"].get(file) != hashes[path]:
                        raise ValueError("run artifact digest mismatch")
                expected_result_fields = result_derived_fields(registered, result)
                if any(row.get(key) != value for key, value in expected_result_fields.items()):
                    raise ValueError("summary result-derived fields disagree with result.json")
                expected_derived = {
                    "schema_version": 1,
                    "source_run_manifest_sha256": hashes[paths[0]],
                    "source_result_sha256": hashes[paths[2]],
                    "source_trace_sha256": run_manifest["artifact_sha256"].get("trace.jsonl"),
                    "trace_derived_fields": trace_derived_fields(row),
                }
                if derived != expected_derived:
                    raise ValueError("trace-derived summary validation disagrees with source run")
                audit_path = f"{prefix}/transport-audit.json"
                if audit_path in hashes:
                    audit = read_json(safe_path(self.directory, audit_path))
                    if audit.get("run_manifest_sha256") != hashes[paths[0]]:
                        raise ValueError("transport audit source mismatch")
                    if audit.get("source_trace_sha256") != run_manifest["artifact_sha256"].get("trace.jsonl"):
                        raise ValueError("transport audit trace mismatch")
                    audits[name + "/" + run_id] = audit
                all_runs[(name, run_id)] = row
                all_run_sources[(name, run_id)] = run_sources
            self.studies[name] = summary
        if (
            len(commits) != 1
            or report.get("source_commit") not in commits
            or len(source_snapshots) != 1
            or len(base_config_hashes) != 1
        ):
            raise ValueError("paired studies use different source snapshots or base configs")
        assessment = assess_studies(self.studies, audits)
        if report.get("assessment") != assessment:
            raise ValueError("assessment disagrees with measured study gates")
        self._videos = {}
        videos = []
        for video in report.get("videos", []):
            relative = video["path"]
            video_manifest_relative = video.get("manifest")
            if (
                relative not in hashes
                or not relative.endswith(".mp4")
                or video_manifest_relative not in hashes
                or not str(video_manifest_relative).endswith(".manifest.json")
            ):
                raise ValueError("unhashed or invalid study video")
            row = all_runs.get((video.get("study"), video.get("run_id")))
            if row is None:
                raise ValueError("video has no matching registered run")
            video_manifest = read_json(
                safe_path(self.directory, str(video_manifest_relative))
            )
            render_contract = validate_video_render_contract(video_manifest)
            run_sources = all_run_sources[(video["study"], row["run_id"])]
            validated_result = video_manifest.get("validated_result")
            result_path = f"{video['study']}/runs/{row['run_id']}/result.json"
            config_path = f"{video['study']}/runs/{row['run_id']}/config.json"
            original_result = read_json(safe_path(self.directory, result_path))
            expected_video_metadata = {
                "study": video.get("study"),
                "run_id": row["run_id"],
                "case": row["case"],
                "seed": row["seed"],
                "profile": row["variant"],
                "config_sha256": row["config_sha256"],
                "source_commit": report["source_commit"],
                "source_run_manifest_sha256": row["run_manifest_sha256"],
                "success": row["success"],
                "failure_reason": row.get("failure_reason"),
                "video_sha256": hashes[relative],
                "playback_speed": render_contract["base_speed"],
            }
            source_run_parts = Path(str(video_manifest.get("source_run", ""))).parts
            if any(video.get(key) != value for key, value in expected_video_metadata.items()):
                raise ValueError("published video metadata disagrees with registered run")
            if (
                video_manifest.get("schema_version") != 1
                or video_manifest.get("case") != row["case"]
                or video_manifest.get("seed") != row["seed"]
                or video_manifest.get("profile") != row["variant"]
                or video_manifest.get("config_sha256") != row["config_sha256"]
                or video_manifest.get("source_run_manifest_sha256")
                != row["run_manifest_sha256"]
                or video_manifest.get("source_commit") != report["source_commit"]
                or video_manifest.get("replay_source_commit") != report["source_commit"]
                or video_manifest.get("source_sha256") != run_sources
                or video_manifest.get("artifact_sha256", {}).get("video.mp4")
                != hashes[relative]
                or video_manifest.get("config_file_sha256") != hashes[config_path]
                or video_manifest.get("result_file_sha256") != hashes[result_path]
                or video_manifest.get("replay_source_dirty") is not False
                or tuple(source_run_parts[-2:]) != (video["study"], row["run_id"])
                or not isinstance(validated_result, Mapping)
                or not replay_result_matches(
                    validated_result,
                    original_result,
                    video_manifest.get("result_match_tolerance"),
                )
            ):
                raise ValueError("render manifest disagrees with registered run")
            self._videos[relative] = safe_path(self.directory, relative)
            videos.append({**video, "url": "/improvement/" + relative})
        self._info = {
            "enabled": True,
            "schema_version": 1,
            "assessment": assessment,
            "studies": self.studies,
            "cooperation_effect": cooperation_effect(self.studies["cooperation"]),
            "videos": videos,
            "provenance": {
                "source_commit": report["source_commit"],
                "validation_scope": "compact metadata and media; full raw archives are separately retained",
                "trace_metric_validation": (
                    "recomputed from raw trace while building the bundle and bound to the "
                    "decompressed trace SHA-256; loading rechecks compact consistency only"
                ),
                "transport_audit_validation": (
                    "recomputed from the frozen scene, IK record, and raw trace while building "
                    "the bundle; loading rechecks compact consistency only"
                ),
                "video_render_validation": (
                    "requires the recorded clean replay Git SHA, full runtime source snapshot, "
                    "fixed 8x/60 FPS contract, and ffprobe media checks; the manifest is evidence "
                    "of that execution, not an independent process attestation"
                ),
            },
        }

    def info(self) -> dict[str, Any]:
        return copy.deepcopy(self._info)

    def asset(self, name: str):
        path = self._videos.get(name)
        return (path, "video/mp4", None) if path else None


def load_improvement_assets(directory: str | Path):
    """Return a useful isolated status when optional evidence is unavailable."""
    try:
        return ImprovementAssets(directory)
    except (OSError, ValueError, KeyError, TypeError) as error:
        class Unavailable:
            def info(self):
                return {"enabled": False, "error": "개선 실험 자료가 없거나 무결성 검증을 통과하지 못했습니다."}

            def asset(self, name):
                return None
        print(f"[viewer] Improvement evidence unavailable: {error}")
        return Unavailable()
