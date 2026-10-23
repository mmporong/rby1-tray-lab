"""Preregistered mobile-task experiment matrix and aggregate statistics."""

from __future__ import annotations

import copy
import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Iterable, Mapping, Sequence


@dataclass(frozen=True)
class CaseDefinition:
    name: str
    factor: str
    values: Mapping[str, Any]
    expected_outcome: str = "task_success_evaluated"


@dataclass(frozen=True)
class VariantDefinition:
    name: str
    cooperation_enabled: bool
    estimator: str


CASES = {
    "nominal": CaseDefinition("nominal", "none", {}),
    "tray-x-plus-0.01": CaseDefinition(
        "tray-x-plus-0.01", "tray_initial_x_offset_m", {"offset_m": 0.01}
    ),
    "payload-mass-scale-1.2": CaseDefinition(
        "payload-mass-scale-1.2", "payload_mass_scale", {"scale": 1.2}
    ),
    "depth-noise-0.003-dropout-0.03": CaseDefinition(
        "depth-noise-0.003-dropout-0.03",
        "depth_sensor_degradation",
        {"depth_std_m": 0.003, "dropout_probability": 0.03},
    ),
    "target-height-1.10": CaseDefinition(
        "target-height-1.10", "target_support_z_m", {"height_m": 1.10}
    ),
    "target-height-1.20": CaseDefinition(
        "target-height-1.20", "target_support_z_m", {"height_m": 1.20}
    ),
    "target-height-1.25": CaseDefinition(
        "target-height-1.25", "target_support_z_m", {"height_m": 1.25}
    ),
    "gray-tray": CaseDefinition(
        "gray-tray", "tray_rgba", {"rgba": [0.55, 0.55, 0.55, 1.0]}
    ),
    "tray-yaw-0.035": CaseDefinition(
        "tray-yaw-0.035", "tray_initial_yaw_rad", {"yaw_rad": 0.035}
    ),
    "low-friction-0.7": CaseDefinition(
        "low-friction-0.7", "tray_friction_scale", {"scale": 0.7}
    ),
    "rgbd-delay-0.05": CaseDefinition(
        "rgbd-delay-0.05", "head_rgbd_delivery_delay_s", {"delay_s": 0.05}
    ),
    "rgbd-stale-0.15": CaseDefinition(
        "rgbd-stale-0.15",
        "head_rgbd_delivery_delay_s",
        {"delay_s": 0.15},
        expected_outcome="stale_rgbd_rejection",
    ),
    "calibration-bias-0.005": CaseDefinition(
        "calibration-bias-0.005",
        "head_calibration_translation_bias_m",
        {"translation_bias_m": [0.005, 0.0, 0.0]},
    ),
}

VARIANTS = {
    "coop-depth-cad": VariantDefinition("coop-depth-cad", True, "depth_cad"),
    "no-coop-depth-cad": VariantDefinition("no-coop-depth-cad", False, "depth_cad"),
    "coop-blue-rgbd": VariantDefinition("coop-blue-rgbd", True, "blue_rgbd"),
    "no-coop-blue-rgbd": VariantDefinition("no-coop-blue-rgbd", False, "blue_rgbd"),
}


def _write_json(path: Path, value: Any) -> None:
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    )


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def canonical_sha256(value: Any) -> str:
    encoded = json.dumps(
        value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def parse_names(raw: str, choices: Mapping[str, Any], label: str) -> list[str]:
    names = [item.strip() for item in raw.split(",") if item.strip()]
    if not names:
        raise ValueError(f"{label} must contain at least one value")
    if len(names) != len(set(names)):
        raise ValueError(f"{label} must not contain duplicates")
    unknown = sorted(set(names) - set(choices))
    if unknown:
        raise ValueError(f"unknown {label}: {', '.join(unknown)}")
    return names


def parse_seeds(raw: str) -> list[int]:
    items = [item.strip() for item in raw.split(",") if item.strip()]
    if not items:
        raise ValueError("seeds must contain at least one integer")
    try:
        seeds = [int(item) for item in items]
    except ValueError as exc:
        raise ValueError("seeds must be comma-separated integers") from exc
    if any(seed < 0 for seed in seeds):
        raise ValueError("seeds must be nonnegative")
    if len(seeds) != len(set(seeds)):
        raise ValueError("seeds must not contain duplicates")
    return seeds


def apply_case(base_config: Mapping[str, Any], case_name: str) -> dict[str, Any]:
    if case_name not in CASES:
        raise ValueError(f"unknown case: {case_name}")
    config = copy.deepcopy(base_config)
    case = CASES[case_name]
    if case.factor == "tray_initial_x_offset_m":
        config["tray_initial_pos_m"][0] = (
            float(config["tray_initial_pos_m"][0]) + float(case.values["offset_m"])
        )
    elif case.factor == "payload_mass_scale":
        config["payload_mass_scale"] = float(case.values["scale"])
    elif case.factor == "depth_sensor_degradation":
        noise = config.setdefault("sensor_noise", {})
        noise["depth_std_m"] = float(case.values["depth_std_m"])
        noise["dropout_probability"] = float(case.values["dropout_probability"])
    elif case.factor == "target_support_z_m":
        config["target"]["support_z_m"] = float(case.values["height_m"])
    elif case.factor == "tray_rgba":
        config["tray_rgba"] = list(case.values["rgba"])
    elif case.factor == "tray_initial_yaw_rad":
        config["tray_initial_yaw_rad"] = float(case.values["yaw_rad"])
    elif case.factor == "tray_friction_scale":
        config["tray_friction_scale"] = float(case.values["scale"])
    elif case.factor == "head_rgbd_delivery_delay_s":
        config["sensors"]["head"]["delivery_delay_s"] = float(
            case.values["delay_s"]
        )
    elif case.factor == "head_calibration_translation_bias_m":
        config["sensors"]["head"]["calibration_translation_bias_m"] = list(
            case.values["translation_bias_m"]
        )
    return config


def apply_variant(config: Mapping[str, Any], variant_name: str) -> dict[str, Any]:
    if variant_name not in VARIANTS:
        raise ValueError(f"unknown variant: {variant_name}")
    result = copy.deepcopy(config)
    variant = VARIANTS[variant_name]
    result.setdefault("cooperation", {})["enabled"] = variant.cooperation_enabled
    result.setdefault("perception", {})["estimator"] = variant.estimator
    result.setdefault("monitoring", {})["enabled"] = False
    return result


def build_run_specs(
    base_config: Mapping[str, Any],
    cases: Sequence[str],
    seeds: Sequence[int],
    variants: Sequence[str],
) -> list[dict[str, Any]]:
    if base_config.get("base_mode") != "mobile":
        raise ValueError("mobile benchmark requires base_mode='mobile'")
    if not cases or not seeds or not variants:
        raise ValueError("cases, seeds, and variants must be nonempty")
    specs = []
    for case_name in cases:
        case_config = apply_case(base_config, case_name)
        for seed in seeds:
            if not isinstance(seed, int) or isinstance(seed, bool) or seed < 0:
                raise ValueError("seeds must be nonnegative integers")
            for variant_name in variants:
                config = apply_variant(case_config, variant_name)
                config["seed"] = seed
                run_id = f"{case_name}__{variant_name}__seed-{seed}"
                specs.append(
                    {
                        "run_id": run_id,
                        "case": case_name,
                        "factor": CASES[case_name].factor,
                        "expected_outcome": CASES[case_name].expected_outcome,
                        "seed": seed,
                        "variant": variant_name,
                        "cooperation_enabled": VARIANTS[variant_name].cooperation_enabled,
                        "estimator": VARIANTS[variant_name].estimator,
                        "target_height_m": float(config["target"]["support_z_m"]),
                        "config_sha256": canonical_sha256(config),
                        "config": config,
                    }
                )
    if len({spec["run_id"] for spec in specs}) != len(specs):
        raise ValueError("run matrix contains duplicate run identifiers")
    return specs


def wilson_interval(successes: int, sample_size: int, z: float = 1.959963984540054) -> dict[str, float]:
    if not isinstance(successes, int) or not isinstance(sample_size, int):
        raise TypeError("successes and sample_size must be integers")
    if sample_size <= 0 or not 0 <= successes <= sample_size:
        raise ValueError("require 0 <= successes <= sample_size and sample_size > 0")
    if not math.isfinite(z) or z <= 0:
        raise ValueError("z must be finite and positive")
    proportion = successes / sample_size
    denominator = 1 + z * z / sample_size
    center = (proportion + z * z / (2 * sample_size)) / denominator
    radius = (
        z
        * math.sqrt(
            proportion * (1 - proportion) / sample_size
            + z * z / (4 * sample_size * sample_size)
        )
        / denominator
    )
    return {"low": max(0.0, center - radius), "high": min(1.0, center + radius)}


def _metric_summary(values: Iterable[float]) -> dict[str, float] | None:
    finite = [float(value) for value in values if math.isfinite(float(value))]
    if not finite:
        return None
    return {
        "mean": sum(finite) / len(finite),
        "min": min(finite),
        "max": max(finite),
    }


def aggregate(rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    recorded_count = len(rows)
    if recorded_count == 0:
        raise ValueError("cannot aggregate an empty run set")
    completed = [
        row
        for row in rows
        if row.get("execution_status") in {None, "completed"}
    ]
    sample_size = len(completed)
    successful = [row for row in completed if row.get("success") is True]
    successes = len(successful)
    return {
        "recorded_count": recorded_count,
        "sample_size": sample_size,
        "execution_errors": recorded_count - sample_size,
        "successes": successes,
        "failures": sample_size - successes,
        "success_rate": successes / sample_size if sample_size else None,
        "wilson_95_ci": wilson_interval(successes, sample_size) if sample_size else None,
        "successful_run_metrics": {
            "sample_size": successes,
            "task_time_s": _metric_summary(
                row["simulation_time_s"] for row in successful if row.get("simulation_time_s") is not None
            ),
            "max_joint_torque_nm": _metric_summary(
                row["max_joint_torque_nm"] for row in successful if row.get("max_joint_torque_nm") is not None
            ),
            "highest_tray_center_z_m": _metric_summary(
                row["highest_tray_center_z_m"] for row in successful if row.get("highest_tray_center_z_m") is not None
            ),
        },
    }


def paired_cooperation_counts(rows: Sequence[Mapping[str, Any]]) -> dict[str, int]:
    pairs: dict[tuple[str, int, str], dict[bool, bool]] = {}
    for row in rows:
        if row.get("execution_status") != "completed":
            continue
        key = (str(row["case"]), int(row["seed"]), str(row["estimator"]))
        enabled = bool(row["cooperation_enabled"])
        if enabled in pairs.setdefault(key, {}):
            raise ValueError(f"duplicate cooperation observation for {key}")
        pairs[key][enabled] = bool(row.get("success"))
    counts = {
        "candidate_pairs": len(pairs),
        "complete_pairs": 0,
        "missing_variant_pairs": 0,
        "both_success": 0,
        "cooperation_only_success": 0,
        "no_cooperation_only_success": 0,
        "both_failed": 0,
    }
    for outcomes in pairs.values():
        if set(outcomes) != {False, True}:
            counts["missing_variant_pairs"] += 1
            continue
        counts["complete_pairs"] += 1
        if outcomes[True] and outcomes[False]:
            counts["both_success"] += 1
        elif outcomes[True]:
            counts["cooperation_only_success"] += 1
        elif outcomes[False]:
            counts["no_cooperation_only_success"] += 1
        else:
            counts["both_failed"] += 1
    return counts


def summarize(rows: Sequence[Mapping[str, Any]], expected_runs: int) -> dict[str, Any]:
    grouped: dict[str, dict[str, list[Mapping[str, Any]]]] = {
        "by_case": {}, "by_variant": {}
    }
    for row in rows:
        grouped["by_case"].setdefault(str(row["case"]), []).append(row)
        grouped["by_variant"].setdefault(str(row["variant"]), []).append(row)
    completed = [row for row in rows if row.get("execution_status") == "completed"]
    grid_rows = [row for row in completed if row.get("factor") == "target_support_z_m"]
    successful_grid = sorted(
        {float(row["target_height_m"]) for row in grid_rows if row.get("success") is True}
    )
    expected_rejections = [
        row
        for row in completed
        if row.get("expected_outcome") == "stale_rgbd_rejection"
    ]
    return {
        "schema_version": 1,
        "complete": len(rows) == expected_runs and len(completed) == expected_runs,
        "expected_runs": expected_runs,
        "recorded_runs": len(rows),
        "completed_runs": len(completed),
        "runs": list(rows),
        "overall": aggregate(rows),
        "by_case": {name: aggregate(items) for name, items in grouped["by_case"].items()},
        "by_variant": {name: aggregate(items) for name, items in grouped["by_variant"].items()},
        "paired_cooperation": paired_cooperation_counts(rows),
        "expected_rejections": {
            "sample_size": len(expected_rejections),
            "observed": sum(
                row.get("expected_rejection_observed") is True
                for row in expected_rejections
            ),
            "policy": "safety_rejection_is_reported_separately_from_task_success",
        },
        "target_height_grid": {
            "tested_heights_m": sorted({float(row["target_height_m"]) for row in grid_rows}),
            "successful_heights_m": successful_grid,
            "maximum_successful_tested_height_m": max(successful_grid) if successful_grid else None,
            "scope": "preregistered_tested_grid_only_not_global_reach",
        },
        "historical_dataset_policy": "mobile_native_runs_only; historical_fixture_or_handle_data_not_combined",
    }


def repository_source_snapshot(repository: Path, config_path: Path) -> dict[str, str]:
    paths = sorted((repository / "src" / "rby1_tray").rglob("*.py"))
    paths.append(repository / "tools" / "benchmark_mobile.py")
    paths.append(config_path.resolve())
    snapshot = {}
    for path in paths:
        if not path.is_file():
            raise FileNotFoundError(path)
        try:
            name = path.relative_to(repository).as_posix()
        except ValueError:
            name = f"external-config:{path}"
        snapshot[name] = _sha256(path)
    return snapshot


def _result_record(spec: Mapping[str, Any], result: Mapping[str, Any]) -> dict[str, Any]:
    metrics = result.get("metrics") if isinstance(result.get("metrics"), Mapping) else {}
    mission = result.get("mission") if isinstance(result.get("mission"), Mapping) else {}
    cooperation = mission.get("cooperation_summary")
    if not isinstance(cooperation, Mapping):
        cooperation = mission.get("cooperation")
    if not isinstance(cooperation, Mapping):
        cooperation = result.get("cooperation_summary")
    if not isinstance(cooperation, Mapping):
        cooperation = None
    mission_failure = mission.get("failure")
    if not isinstance(mission_failure, Mapping):
        mission_failure = None
    expected_rejection_observed = None
    if spec["expected_outcome"] == "stale_rgbd_rejection":
        detail = str(mission_failure.get("detail", "")) if mission_failure else ""
        expected_rejection_observed = bool(
            mission_failure
            and mission_failure.get("reason") == "perception_failed"
            and "stale RGB-D frame" in detail
        )
    return {
        key: spec[key]
        for key in (
            "run_id", "case", "factor", "expected_outcome", "seed", "variant",
            "cooperation_enabled", "estimator", "target_height_m", "config_sha256",
        )
    } | {
        "execution_status": "completed",
        "success": bool(result.get("success", False)),
        "failure_reason": result.get("failure_reason"),
        "simulation_time_s": result.get("simulation_time_s"),
        "max_joint_torque_nm": metrics.get("max_joint_torque_nm"),
        "highest_tray_center_z_m": result.get("highest_tray_center_z_m"),
        "cooperation_summary": copy.deepcopy(cooperation),
        "mission_failure": copy.deepcopy(mission_failure),
        "expected_rejection_observed": expected_rejection_observed,
    }


def run_benchmark(
    base_config: Mapping[str, Any],
    output: Path,
    cases: Sequence[str],
    seeds: Sequence[int],
    variants: Sequence[str],
    run_episode: Callable[[Mapping[str, Any], Path], Mapping[str, Any]],
    source_snapshot: Callable[[], Mapping[str, str]],
    provenance: Mapping[str, Any],
) -> dict[str, Any]:
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    specs = build_run_specs(base_config, cases, seeds, variants)
    frozen_sources = dict(source_snapshot())
    preregistration = {
        "schema_version": 1,
        "base_config_sha256": canonical_sha256(base_config),
        "cases": [
            {
                "name": name,
                "factor": CASES[name].factor,
                "values": dict(CASES[name].values),
                "expected_outcome": CASES[name].expected_outcome,
            }
            for name in cases
        ],
        "seeds": list(seeds),
        "variants": [
            {
                "name": name,
                "cooperation_enabled": VARIANTS[name].cooperation_enabled,
                "estimator": VARIANTS[name].estimator,
            }
            for name in variants
        ],
        "expected_runs": len(specs),
        "source_sha256": frozen_sources,
        "provenance": dict(provenance),
        "run_matrix": [
            {key: spec[key] for key in spec if key != "config"} for spec in specs
        ],
    }
    _write_json(output / "preregistered.json", preregistration)
    rows: list[dict[str, Any]] = []
    run_manifests: dict[str, dict[str, str]] = {}
    for spec in specs:
        if dict(source_snapshot()) != frozen_sources:
            raise RuntimeError("source changed after preregistration; restart from a fixed source tree")
        destination = output / spec["run_id"]
        try:
            result = run_episode(copy.deepcopy(spec["config"]), destination)
            row = _result_record(spec, result)
        except Exception as exc:  # Preserve infrastructure failures without hiding them.
            row = {
                key: spec[key]
                for key in (
                    "run_id", "case", "factor", "expected_outcome", "seed", "variant",
                    "cooperation_enabled", "estimator", "target_height_m", "config_sha256",
                )
            } | {
                "execution_status": "runner_error",
                "success": False,
                "failure_reason": f"{type(exc).__name__}: {exc}",
                "simulation_time_s": None,
                "max_joint_torque_nm": None,
                "highest_tray_center_z_m": None,
                "cooperation_summary": None,
                "mission_failure": None,
                "expected_rejection_observed": None,
            }
        manifest = destination / "manifest.json"
        if manifest.is_file():
            relative = manifest.relative_to(output).as_posix()
            run_manifests[spec["run_id"]] = {"path": relative, "sha256": _sha256(manifest)}
            row["run_manifest"] = relative
            row["run_manifest_sha256"] = run_manifests[spec["run_id"]]["sha256"]
        else:
            row["run_manifest"] = None
            row["run_manifest_sha256"] = None
            if row["execution_status"] == "completed":
                row["execution_status"] = "missing_manifest"
                row["success"] = False
                row["failure_reason"] = "run_episode returned without manifest.json"
        rows.append(row)
        _write_json(output / "summary.json", summarize(rows, len(specs)))
    if dict(source_snapshot()) != frozen_sources:
        raise RuntimeError("source changed during benchmark; results remain incomplete")
    summary = summarize(rows, len(specs))
    _write_json(output / "summary.json", summary)
    top_artifacts = {
        name: _sha256(output / name) for name in ("preregistered.json", "summary.json")
    }
    _write_json(
        output / "dataset-manifest.json",
        {
            "schema_version": 1,
            "complete": summary["complete"],
            "source_sha256": frozen_sources,
            "run_manifests": run_manifests,
            "artifact_sha256": top_artifacts,
        },
    )
    return summary
