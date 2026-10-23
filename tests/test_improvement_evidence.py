import copy
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from unittest import mock

from rby1_tray.experiments import (
    CASES,
    STUDIES,
    VARIANTS,
    _result_record,
    build_run_specs,
    canonical_sha256,
    summarize,
)
from rby1_tray.improvement_evidence import (
    ImprovementAssets, load_improvement_assets, safe_path, sha256_file,
)
from rby1_tray.scene import load_config


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("compact_builder", ROOT / "tools/build_improvement_evidence.py")
assert spec is not None and spec.loader is not None
builder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(builder)


def write_json(path, value):
    path.write_text(json.dumps(value, allow_nan=False) + "\n")


def refresh_bundle_hashes(bundle):
    manifest_path = bundle / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    manifest["artifact_sha256"] = {
        relative: sha256_file(bundle / relative)
        for relative in manifest["artifact_sha256"]
    }
    write_json(manifest_path, manifest)


class ImprovementEvidenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.video_temp = tempfile.TemporaryDirectory()
        cls.valid_video = Path(cls.video_temp.name) / "video.mp4"
        subprocess.run(
            [
                "ffmpeg", "-v", "error", "-f", "lavfi", "-i",
                "color=c=blue:s=960x720:r=60", "-frames:v", "1134",
                "-c:v", "libx264", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
                "-y", str(cls.valid_video),
            ],
            check=True,
        )

    @classmethod
    def tearDownClass(cls):
        cls.video_temp.cleanup()

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        raw = self.root / "raw"
        raw.mkdir()
        self.raw = raw
        base = load_config(ROOT / "configs/mobile-native.json")
        for name, study in STUDIES.items():
            folder = raw / name
            folder.mkdir()
            specs = build_run_specs(base, study.cases, study.seeds, study.variants)
            rows = []
            prereg = {
                "schema_version": 2,
                "study": name,
                "study_purpose": study.purpose,
                "base_config_sha256": canonical_sha256(base),
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
                "expected_runs": len(specs),
                "source_sha256": {
                    "src/rby1_tray/example.py": "1" * 64,
                    "tools/example.py": "3" * 64,
                    "configs/mobile-native.json": sha256_file(
                        ROOT / "configs/mobile-native.json"
                    ),
                },
                "provenance": {"commit": "a" * 40, "source_dirty_at_start": False},
                "run_matrix": [{k: v for k, v in s.items() if k != "config"} for s in specs],
            }
            for item in specs:
                run = folder / item["run_id"]
                run.mkdir()
                cooperation_enabled = item["cooperation_enabled"]
                simulation_time = 151.0 if cooperation_enabled else 150.0
                result = {
                    "success": True,
                    "simulation_time_s": simulation_time,
                    "metrics": {
                        "max_joint_torque_nm": 11.0 if cooperation_enabled else 10.0
                    },
                    "mission": {
                        "cooperation_summary": {
                            "max_relative_position_error_m": .003 if cooperation_enabled else .002,
                            "max_relative_orientation_error_rad": .004 if cooperation_enabled else .003,
                            "minimum_speed_scale": .8 if cooperation_enabled else 1.0,
                            "slowdown_samples": 2 if cooperation_enabled else 0,
                        }
                    },
                }
                write_json(run / "config.json", item["config"])
                write_json(run / "result.json", result)
                trace_rows = [
                    {
                        "time_s": 0.0,
                        "phase": "grasp_hold",
                        "tray": {"pos_m": [0.0, 0.0, 0.0], "quat_wxyz": [1.0, 0.0, 0.0, 0.0]},
                        "payloads": [{"name": "payload", "pos_m": [0.0, 0.0, 0.1]}],
                    },
                    {
                        "time_s": 1.0,
                        "phase": "lift",
                        "tray": {"pos_m": [0.0, 0.0, 0.0], "quat_wxyz": [1.0, 0.0, 0.0, 0.0]},
                        "payloads": [{"name": "payload", "pos_m": [0.0, 0.0, 0.1]}],
                    },
                ]
                (run / "trace.jsonl").write_text(
                    "".join(json.dumps(value) + "\n" for value in trace_rows)
                )
                manifest = {
                    "commit": "a" * 40,
                    "source_sha256": {"example.py": "1" * 64},
                    "artifact_sha256": {
                        filename: sha256_file(run / filename)
                        for filename in ("config.json", "result.json", "trace.jsonl")
                    },
                }
                write_json(run / "manifest.json", manifest)
                row = _result_record(item, result)
                row.update(builder._recomputed_trace_fields(run, row))
                row.update(run_manifest=f'{item["run_id"]}/manifest.json', run_manifest_sha256=sha256_file(run / "manifest.json"))
                rows.append(row)
            write_json(folder / "summary.json", summarize(rows, len(specs)))
            write_json(folder / "preregistered.json", prereg)
            write_json(folder / "dataset-manifest.json", {
                "schema_version": 1,
                "complete": True,
                "source_sha256": prereg["source_sha256"],
                "run_manifests": {
                    row["run_id"]: {
                        "path": row["run_manifest"],
                        "sha256": row["run_manifest_sha256"],
                    }
                    for row in rows
                },
                "artifact_sha256": {filename: sha256_file(folder / filename) for filename in ("summary.json", "preregistered.json")},
            })
        self.output = self.root / "compact"
        builder.build(raw, self.output)

    def tearDown(self):
        self.temp.cleanup()

    def test_valid_compact_bundle_retains_small_sample_and_missing_audit_limits(self):
        assets = ImprovementAssets(self.output)
        info = assets.info()
        self.assertTrue(info["enabled"])
        self.assertEqual(info["assessment"]["status"], "partial")
        self.assertFalse(info["assessment"]["gates"]["sampled_transport_clearance"])
        self.assertEqual(info["studies"]["improvement"]["completed_runs"], 24)
        info["studies"].clear()
        self.assertEqual(len(assets.info()["studies"]), 3)
        effect = assets.info()["cooperation_effect"]
        self.assertEqual(effect["sign"], "coop_minus_no_coop")
        self.assertEqual(effect["complete_pairs"], 8)
        self.assertEqual(effect["both_success_pairs"], 8)
        self.assertEqual(effect["metric_deltas"]["task_time_s"], {
            "n": 8, "mean": 1.0, "min": 1.0, "max": 1.0,
        })
        self.assertEqual(effect["metric_deltas"]["max_joint_torque_nm"]["mean"], 1.0)
        self.assertAlmostEqual(
            effect["metric_deltas"]["max_tray_tilt_rad"]["mean"], 0.0
        )
        self.assertTrue(all(pair["both_success"] for pair in effect["pairs"]))

    def test_corrupt_bytes_fail_closed_without_disabling_viewer(self):
        path = self.output / "improvement/summary.json"
        path.write_text(path.read_text() + " ")
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            ImprovementAssets(self.output)
        unavailable = load_improvement_assets(self.output)
        self.assertFalse(unavailable.info()["enabled"])
        self.assertIsNone(unavailable.asset("videos/0.mp4"))

    def test_false_assessment_rejected_even_with_updated_hash(self):
        path = self.output / "report.json"
        report = json.loads(path.read_text())
        report["assessment"]["status"] = "passed"
        write_json(path, report)
        manifest_path = self.output / "manifest.json"
        manifest = json.loads(manifest_path.read_text())
        manifest["artifact_sha256"]["report.json"] = sha256_file(path)
        write_json(manifest_path, manifest)
        with self.assertRaisesRegex(ValueError, "assessment disagrees"):
            ImprovementAssets(self.output)

    def test_frozen_study_matrix_tamper_is_rejected_after_hashes_are_updated(self):
        prereg_path = self.output / "improvement/preregistered.json"
        prereg = json.loads(prereg_path.read_text())
        prereg["study"] = "cooperation"
        write_json(prereg_path, prereg)
        dataset_path = self.output / "improvement/dataset-manifest.json"
        dataset = json.loads(dataset_path.read_text())
        dataset["artifact_sha256"]["preregistered.json"] = sha256_file(prereg_path)
        write_json(dataset_path, dataset)
        refresh_bundle_hashes(self.output)
        with self.assertRaisesRegex(ValueError, "study definition mismatch"):
            ImprovementAssets(self.output)

    def _render_manifest(self):
        summary = json.loads((self.raw / "improvement/summary.json").read_text())
        row = next(
            item for item in summary["runs"]
            if item["variant"] == "candidate-depth-cad" and item["case"] == "nominal"
        )
        source_run = self.raw / "improvement" / row["run_id"]
        rendered = self.root / "rendered"
        rendered.mkdir()
        shutil.copyfile(self.valid_video, rendered / "video.mp4")
        manifest = {
            "schema_version": 1,
            "provenance_statement": "fresh deterministic MobileSession rerun of the source config; not trace interpolation",
            "case": row["case"],
            "profile": row["variant"],
            "seed": row["seed"],
            "source_run": source_run.as_posix(),
            "source_run_manifest_sha256": sha256_file(source_run / "manifest.json"),
            "source_commit": "a" * 40,
            "replay_source_commit": "a" * 40,
            "replay_source_dirty": False,
            "source_sha256": {"example.py": "1" * 64},
            "config_sha256": row["config_sha256"],
            "config_file_sha256": sha256_file(source_run / "config.json"),
            "result_file_sha256": sha256_file(source_run / "result.json"),
            "result_match_tolerance": 1e-9,
            "validated_result": {
                "success": row["success"],
                "failure_reason": row["failure_reason"],
                "simulation_time_s": row["simulation_time_s"],
                "metrics": json.loads((source_run / "result.json").read_text())["metrics"],
            },
            "artifact_sha256": {"video.mp4": sha256_file(rendered / "video.mp4")},
            "rendering": {
                "base_speed": 8,
                "fps": 60,
                "width": 960,
                "height": 720,
                "frame_count": 1134,
                "duration_sim_s": row["simulation_time_s"],
                "duration_video_s": 18.9,
                "view": "overview",
                "sampling": "nearest physics state to i/fps*base_speed; terminal frame appended by the deterministic rerun",
            },
        }
        write_json(rendered / "manifest.json", manifest)
        return rendered / "manifest.json", row

    def test_video_metadata_is_derived_from_verified_render_manifest(self):
        source_manifest, row = self._render_manifest()
        output = self.root / "compact-with-video"
        info = builder.build(
            self.raw,
            output,
            [{"source_manifest": str(source_manifest), "label": "후보 성공"}],
        )
        video = info["videos"][0]
        self.assertEqual(video["run_id"], row["run_id"])
        self.assertEqual(video["profile"], "candidate-depth-cad")
        self.assertEqual(video["label"], "후보 성공")
        self.assertEqual(video["playback_speed"], 8.0)
        self.assertTrue((output / video["manifest"]).is_file())
        asset = ImprovementAssets(output).asset(video["path"])
        assert asset is not None
        self.assertEqual(asset[1], "video/mp4")

    def test_video_outcome_tamper_is_rejected_after_all_hashes_are_updated(self):
        source_manifest, _ = self._render_manifest()
        output = self.root / "compact-with-video"
        builder.build(self.raw, output, [{"source_manifest": str(source_manifest)}])
        copied_manifest_path = output / "videos/0.manifest.json"
        copied_manifest = json.loads(copied_manifest_path.read_text())
        copied_manifest["validated_result"]["success"] = False
        write_json(copied_manifest_path, copied_manifest)
        report_path = output / "report.json"
        report = json.loads(report_path.read_text())
        report["videos"][0]["success"] = False
        write_json(report_path, report)
        refresh_bundle_hashes(output)
        with self.assertRaisesRegex(ValueError, "video metadata|render manifest"):
            ImprovementAssets(output)

    def test_extra_or_missing_run_source_is_rejected(self):
        for index, mutate in enumerate((
            lambda sources: sources.pop("example.py"),
            lambda sources: sources.__setitem__("unregistered.py", "2" * 64),
        )):
            bundle = self.root / f"source-tamper-{index}"
            builder.build(self.raw, bundle)
            manifest_path = next(bundle.glob("improvement/runs/*/manifest.json"))
            original = json.loads(manifest_path.read_text())
            mutate(original["source_sha256"])
            write_json(manifest_path, original)
            summary_path = bundle / "improvement/summary.json"
            summary = json.loads(summary_path.read_text())
            row = next(
                item for item in summary["runs"]
                if item["run_id"] == manifest_path.parent.name
            )
            row["run_manifest_sha256"] = sha256_file(manifest_path)
            write_json(summary_path, summarize(summary["runs"], 24))
            dataset_path = bundle / "improvement/dataset-manifest.json"
            dataset = json.loads(dataset_path.read_text())
            dataset["run_manifests"][row["run_id"]]["sha256"] = row["run_manifest_sha256"]
            dataset["artifact_sha256"]["summary.json"] = sha256_file(summary_path)
            write_json(dataset_path, dataset)
            refresh_bundle_hashes(bundle)
            with self.assertRaisesRegex(ValueError, "source snapshots disagree"):
                ImprovementAssets(bundle)

    def test_video_metrics_tamper_is_rejected_by_builder(self):
        source_manifest, _ = self._render_manifest()
        manifest = json.loads(source_manifest.read_text())
        manifest["validated_result"]["metrics"]["max_joint_torque_nm"] += 0.1
        write_json(source_manifest, manifest)
        with self.assertRaisesRegex(ValueError, "render manifest"):
            builder.build(
                self.raw,
                self.root / "compact-invalid-video",
                [{"source_manifest": str(source_manifest)}],
            )

    def test_case_config_tamper_is_rejected_after_internal_hash_chain_update(self):
        run_id = "tray-yaw-0.035__candidate-depth-cad__seed-0"
        prefix = self.output / "improvement/runs" / run_id
        config_path = prefix / "config.json"
        config = json.loads(config_path.read_text())
        config["tray_initial_yaw_rad"] = 0.0
        write_json(config_path, config)
        config_sha = canonical_sha256(config)

        prereg_path = self.output / "improvement/preregistered.json"
        prereg = json.loads(prereg_path.read_text())
        next(row for row in prereg["run_matrix"] if row["run_id"] == run_id)[
            "config_sha256"
        ] = config_sha
        write_json(prereg_path, prereg)

        summary_path = self.output / "improvement/summary.json"
        summary = json.loads(summary_path.read_text())
        summary_row = next(row for row in summary["runs"] if row["run_id"] == run_id)
        summary_row["config_sha256"] = config_sha

        run_manifest_path = prefix / "manifest.json"
        run_manifest = json.loads(run_manifest_path.read_text())
        run_manifest["artifact_sha256"]["config.json"] = sha256_file(config_path)
        write_json(run_manifest_path, run_manifest)
        summary_row["run_manifest_sha256"] = sha256_file(run_manifest_path)
        write_json(summary_path, summarize(summary["runs"], 24))

        derived_path = prefix / "derived-validation.json"
        derived = json.loads(derived_path.read_text())
        derived["source_run_manifest_sha256"] = summary_row["run_manifest_sha256"]
        write_json(derived_path, derived)
        dataset_path = self.output / "improvement/dataset-manifest.json"
        dataset = json.loads(dataset_path.read_text())
        dataset["run_manifests"][run_id]["sha256"] = summary_row["run_manifest_sha256"]
        dataset["artifact_sha256"]["summary.json"] = sha256_file(summary_path)
        dataset["artifact_sha256"]["preregistered.json"] = sha256_file(prereg_path)
        write_json(dataset_path, dataset)
        refresh_bundle_hashes(self.output)
        with self.assertRaisesRegex(ValueError, "frozen case/variant/seed"):
            ImprovementAssets(self.output)

    def test_summary_result_metric_tamper_is_rejected(self):
        summary_path = self.output / "cooperation/summary.json"
        summary = json.loads(summary_path.read_text())
        summary["runs"][0]["max_joint_torque_nm"] = 111.0
        write_json(summary_path, summarize(summary["runs"], 16))
        dataset_path = self.output / "cooperation/dataset-manifest.json"
        dataset = json.loads(dataset_path.read_text())
        dataset["artifact_sha256"]["summary.json"] = sha256_file(summary_path)
        write_json(dataset_path, dataset)
        refresh_bundle_hashes(self.output)
        with self.assertRaisesRegex(ValueError, "result-derived fields"):
            ImprovementAssets(self.output)

    def test_studies_must_share_the_full_source_snapshot(self):
        prereg_path = self.output / "safety/preregistered.json"
        prereg = json.loads(prereg_path.read_text())
        prereg["source_sha256"]["tools/example.py"] = "4" * 64
        write_json(prereg_path, prereg)
        dataset_path = self.output / "safety/dataset-manifest.json"
        dataset = json.loads(dataset_path.read_text())
        dataset["source_sha256"] = prereg["source_sha256"]
        dataset["artifact_sha256"]["preregistered.json"] = sha256_file(prereg_path)
        write_json(dataset_path, dataset)
        refresh_bundle_hashes(self.output)
        with self.assertRaisesRegex(ValueError, "different source"):
            ImprovementAssets(self.output)

    def test_video_source_snapshot_is_required(self):
        source_manifest, _ = self._render_manifest()
        manifest = json.loads(source_manifest.read_text())
        manifest.pop("source_sha256")
        write_json(source_manifest, manifest)
        with self.assertRaisesRegex(ValueError, "render manifest"):
            builder.build(
                self.raw,
                self.root / "compact-video-source-missing",
                [{"source_manifest": str(source_manifest)}],
            )

    def test_forged_transport_audit_is_rejected_by_raw_recomputation(self):
        summary = json.loads((self.raw / "improvement/summary.json").read_text())
        run = self.raw / "improvement" / summary["runs"][0]["run_id"]
        authoritative = {
            "schema_version": 1,
            "scope": "recorded_raise_high_frames_only",
            "frame_count": 10,
            "contact_frame_count": 2,
            "minimum_preflight_clearance_m": 0.03,
        }
        forged = {
            **authoritative,
            "frame_count": 1,
            "contact_frame_count": 0,
        }
        write_json(run / "transport-audit.json", forged)
        with mock.patch.object(builder, "audit_run", return_value=authoritative) as audit:
            with self.assertRaisesRegex(ValueError, "transport audit disagrees"):
                builder.build(self.raw, self.root / "compact-forged-audit")
        audit.assert_called_once_with(run)

    def test_summary_run_id_path_escape_is_rejected_before_run_paths(self):
        summary_path = self.raw / "improvement/summary.json"
        summary = json.loads(summary_path.read_text())
        summary["runs"][0]["run_id"] = "../escaped"
        write_json(summary_path, summary)
        escaped = self.raw / "escaped"
        with self.assertRaisesRegex(ValueError, "run inventory"):
            builder.build(self.raw, self.root / "compact-path-escape")
        self.assertFalse(escaped.exists())

    def test_video_bytes_tamper_is_rejected_by_builder(self):
        source_manifest, _ = self._render_manifest()
        video_path = source_manifest.with_name("video.mp4")
        video_path.write_bytes(b"not-an-mp4")
        manifest = json.loads(source_manifest.read_text())
        manifest["artifact_sha256"]["video.mp4"] = sha256_file(video_path)
        write_json(source_manifest, manifest)
        with self.assertRaisesRegex(ValueError, "ffprobe rejected"):
            builder.build(
                self.raw,
                self.root / "compact-invalid-video-bytes",
                [{"source_manifest": str(source_manifest)}],
            )

    def test_video_16x_contract_is_rejected_with_a_valid_mp4(self):
        source_manifest, _ = self._render_manifest()
        manifest = json.loads(source_manifest.read_text())
        manifest["rendering"]["base_speed"] = 16
        write_json(source_manifest, manifest)
        with self.assertRaisesRegex(ValueError, "fixed overview video contract"):
            builder.build(
                self.raw,
                self.root / "compact-invalid-video-speed",
                [{"source_manifest": str(source_manifest)}],
            )

    def test_unsafe_paths_and_undeclared_assets_are_rejected(self):
        for value in ("../outside", "/etc/passwd", "one/../../outside", "one\\two"):
            with self.assertRaises(ValueError):
                safe_path(self.output, value)
        self.assertIsNone(ImprovementAssets(self.output).asset("report.json"))

    def test_missing_bundle_returns_disabled_status(self):
        self.assertFalse(load_improvement_assets(self.root / "missing").info()["enabled"])


if __name__ == "__main__":
    unittest.main()
