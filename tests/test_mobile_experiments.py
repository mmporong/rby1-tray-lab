import json
from pathlib import Path
import tempfile
import unittest

from rby1_tray.experiments import (
    aggregate,
    apply_case,
    build_run_specs,
    paired_cooperation_counts,
    parse_names,
    parse_seeds,
    run_benchmark,
    wilson_interval,
)
from rby1_tray.scene import load_config


class MobileExperimentTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config("configs/mobile-native.json")

    def test_input_validation_rejects_unknown_duplicates_and_invalid_seeds(self):
        with self.assertRaisesRegex(ValueError, "unknown cases"):
            parse_names("nominal,unknown", {"nominal": None}, "cases")
        with self.assertRaisesRegex(ValueError, "duplicates"):
            parse_names("nominal,nominal", {"nominal": None}, "cases")
        with self.assertRaisesRegex(ValueError, "integers"):
            parse_seeds("0,nope")
        with self.assertRaisesRegex(ValueError, "nonnegative"):
            parse_seeds("-1")
        with self.assertRaisesRegex(ValueError, "duplicates"):
            parse_seeds("1,1")

    def test_cases_change_only_the_preregistered_factor(self):
        tray = apply_case(self.config, "tray-x-plus-0.01")
        self.assertAlmostEqual(tray["tray_initial_pos_m"][0], self.config["tray_initial_pos_m"][0] + .01)
        self.assertEqual(self.config["tray_initial_pos_m"][0], .65)
        payload = apply_case(self.config, "payload-mass-scale-1.2")
        self.assertEqual(payload["payload_mass_scale"], 1.2)
        noisy = apply_case(self.config, "depth-noise-0.003-dropout-0.03")
        self.assertEqual(noisy["sensor_noise"]["depth_std_m"], .003)
        self.assertEqual(noisy["sensor_noise"]["dropout_probability"], .03)
        high = apply_case(self.config, "target-height-1.25")
        self.assertEqual(high["target"]["support_z_m"], 1.25)

    def test_visual_physics_pose_and_latency_cases_use_separate_config_boundaries(self):
        gray = apply_case(self.config, "gray-tray")
        self.assertEqual(gray["tray_rgba"], [.55, .55, .55, 1.0])
        yawed = apply_case(self.config, "tray-yaw-0.035")
        self.assertEqual(yawed["tray_initial_yaw_rad"], .035)
        low_friction = apply_case(self.config, "low-friction-0.7")
        self.assertEqual(low_friction["tray_friction_scale"], .7)
        delayed = apply_case(self.config, "rgbd-delay-0.05")
        self.assertEqual(delayed["sensors"]["head"]["delivery_delay_s"], .05)
        stale = apply_case(self.config, "rgbd-stale-0.15")
        self.assertEqual(stale["sensors"]["head"]["delivery_delay_s"], .15)
        biased = apply_case(self.config, "calibration-bias-0.005")
        self.assertEqual(
            biased["sensors"]["head"]["calibration_translation_bias_m"],
            [.005, 0.0, 0.0],
        )
        self.assertNotIn("tray_rgba", self.config)
        self.assertNotIn("delivery_delay_s", self.config["sensors"]["head"])

    def test_stale_case_preregisters_and_reports_expected_perception_rejection(self):
        spec = build_run_specs(
            self.config, ["rgbd-stale-0.15"], [0], ["coop-depth-cad"]
        )[0]
        self.assertEqual(spec["expected_outcome"], "stale_rgbd_rejection")

        def stale_run(_config, output):
            output.mkdir()
            (output / "manifest.json").write_text('{"artifact_sha256": {}}\n')
            return {
                "success": False,
                "failure_reason": "perception_failed",
                "simulation_time_s": 4.0,
                "metrics": {"max_joint_torque_nm": 1.0},
                "mission": {
                    "failure": {
                        "reason": "perception_failed",
                        "detail": "stale RGB-D frame: age=0.150000s",
                    }
                },
            }

        with tempfile.TemporaryDirectory() as directory:
            summary = run_benchmark(
                self.config,
                Path(directory) / "benchmark",
                ["rgbd-stale-0.15"],
                [0],
                ["coop-depth-cad"],
                stale_run,
                source_snapshot=lambda: {"source.py": "fixed"},
                provenance={"commit": "test"},
            )
        row = summary["runs"][0]
        self.assertFalse(row["success"])
        self.assertTrue(row["expected_rejection_observed"])
        self.assertEqual(row["mission_failure"]["reason"], "perception_failed")
        self.assertEqual(summary["expected_rejections"]["sample_size"], 1)
        self.assertEqual(summary["expected_rejections"]["observed"], 1)

    def test_run_matrix_disables_monitoring_and_preserves_paired_identity(self):
        specs = build_run_specs(
            self.config,
            ["nominal"],
            [3, 7],
            ["coop-depth-cad", "no-coop-depth-cad"],
        )
        self.assertEqual(len(specs), 4)
        self.assertEqual(len({spec["run_id"] for spec in specs}), 4)
        for spec in specs:
            self.assertFalse(spec["config"]["monitoring"]["enabled"])
            self.assertEqual(spec["config"]["perception"]["estimator"], "depth_cad")
            self.assertEqual(spec["config"]["seed"], spec["seed"])

    def test_wilson_interval_and_aggregate_keep_sample_size(self):
        interval = wilson_interval(5, 10)
        self.assertAlmostEqual(interval["low"], 0.2365930905, places=9)
        self.assertAlmostEqual(interval["high"], 0.7634069095, places=9)
        rows = [
            {"success": True, "simulation_time_s": 10, "max_joint_torque_nm": 20, "highest_tray_center_z_m": 1.2},
            {"success": False, "simulation_time_s": 4, "max_joint_torque_nm": 99, "highest_tray_center_z_m": 0.2},
        ]
        summary = aggregate(rows)
        self.assertEqual(summary["sample_size"], 2)
        self.assertEqual(summary["execution_errors"], 0)
        self.assertEqual(summary["successes"], 1)
        self.assertEqual(summary["successful_run_metrics"]["max_joint_torque_nm"]["max"], 20)
        with self.assertRaises(ValueError):
            wilson_interval(1, 0)

    def test_paired_counts_require_matching_case_seed_and_estimator(self):
        base = {"case": "nominal", "seed": 0, "estimator": "depth_cad", "execution_status": "completed"}
        counts = paired_cooperation_counts([
            {**base, "cooperation_enabled": True, "success": True},
            {**base, "cooperation_enabled": False, "success": False},
            {**base, "seed": 1, "cooperation_enabled": True, "success": True},
        ])
        self.assertEqual(counts["complete_pairs"], 1)
        self.assertEqual(counts["cooperation_only_success"], 1)
        self.assertEqual(counts["missing_variant_pairs"], 1)

    def test_runner_errors_are_preserved_but_not_counted_as_physics_failures(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "benchmark"

            def fail_run(_config, _output):
                raise RuntimeError("synthetic runner outage")

            summary = run_benchmark(
                self.config,
                output,
                ["nominal"],
                [0],
                ["coop-depth-cad"],
                fail_run,
                source_snapshot=lambda: {"source.py": "fixed"},
                provenance={"commit": "test"},
            )
            self.assertFalse(summary["complete"])
            self.assertEqual(summary["overall"]["recorded_count"], 1)
            self.assertEqual(summary["overall"]["sample_size"], 0)
            self.assertEqual(summary["overall"]["execution_errors"], 1)
            self.assertIsNone(summary["overall"]["success_rate"])
            self.assertIn("synthetic runner outage", summary["runs"][0]["failure_reason"])

    def test_runner_writes_preregistration_summary_and_linked_manifests(self):
        calls = []

        def fake_run(config, output):
            output.mkdir()
            (output / "manifest.json").write_text('{"artifact_sha256": {}}\n')
            calls.append(config)
            success = bool(config["cooperation"]["enabled"])
            return {
                "success": success,
                "failure_reason": None if success else "test_failure",
                "simulation_time_s": 12.5,
                "highest_tray_center_z_m": config["target"]["support_z_m"] + .1,
                "metrics": {"max_joint_torque_nm": 42.0},
                "mission": {"cooperation_summary": {"enabled": success, "slowdown_events": 2}},
            }

        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "benchmark"
            summary = run_benchmark(
                self.config,
                output,
                ["target-height-1.10", "target-height-1.25"],
                [0],
                ["coop-depth-cad", "no-coop-depth-cad"],
                fake_run,
                source_snapshot=lambda: {"source.py": "fixed"},
                provenance={"commit": "test"},
            )
            self.assertTrue(summary["complete"])
            self.assertEqual(summary["expected_runs"], 4)
            self.assertEqual(summary["paired_cooperation"]["complete_pairs"], 2)
            self.assertEqual(
                summary["target_height_grid"]["maximum_successful_tested_height_m"], 1.25
            )
            self.assertEqual(summary["target_height_grid"]["scope"], "preregistered_tested_grid_only_not_global_reach")
            self.assertTrue(all(config["monitoring"]["enabled"] is False for config in calls))
            manifest = json.loads((output / "dataset-manifest.json").read_text())
            self.assertEqual(len(manifest["run_manifests"]), 4)
            self.assertTrue(all(row["run_manifest_sha256"] for row in summary["runs"]))
            self.assertTrue(all(row["cooperation_summary"] for row in summary["runs"]))
            preregistered = json.loads((output / "preregistered.json").read_text())
            self.assertEqual(preregistered["expected_runs"], 4)


if __name__ == "__main__":
    unittest.main()
