import gzip
import json
import math
from pathlib import Path
import tempfile
import unittest

from rby1_tray.improvement_metrics import extract_trace_metrics


def _row(time_s, phase, quaternion, payload_x):
    return {
        "time_s": time_s,
        "phase": phase,
        "tray": {"pos_m": [1.0, 2.0, 3.0], "quat_wxyz": quaternion},
        "payloads": [
            {"name": "payload_0", "pos_m": [1.0 + payload_x, 2.0, 3.1]}
        ],
    }


class ImprovementMetricTests(unittest.TestCase):
    def test_transport_metrics_use_grasp_hold_reference_in_tray_frame(self):
        half_angle = math.pi / 8
        pitch_quaternion = [math.cos(half_angle), 0.0, math.sin(half_angle), 0.0]
        rows = [
            _row(1.0, "grasp_hold", [1.0, 0.0, 0.0, 0.0], .10),
            _row(2.0, "lift", [1.0, 0.0, 0.0, 0.0], .13),
            _row(3.0, "raise_high", pitch_quaternion, .10),
            _row(4.0, "return_home", [1.0, 0.0, 0.0, 0.0], 2.0),
        ]
        with tempfile.TemporaryDirectory() as directory:
            trace = Path(directory) / "trace.jsonl"
            trace.write_text("".join(json.dumps(row) + "\n" for row in rows))
            result = extract_trace_metrics(Path(directory))
        self.assertIsNone(result["trace_metrics_missing_reason"])
        metrics = result["trace_metrics"]
        self.assertEqual(metrics["transport_samples"], 2)
        self.assertEqual(metrics["valid_tray_pose_samples"], 2)
        self.assertEqual(metrics["payload_displacement_samples"], 2)
        self.assertEqual(metrics["transport_start_time_s"], 2.0)
        self.assertEqual(metrics["transport_end_time_s"], 3.0)
        self.assertEqual(metrics["payload_baseline_phase"], "grasp_hold")
        self.assertEqual(metrics["payload_baseline_time_s"], 1.0)
        self.assertEqual(
            metrics["payload_displacement_reference"],
            "first valid grasp_hold sample in tray frame",
        )
        self.assertAlmostEqual(metrics["max_abs_tray_pitch_rad"], math.pi / 4)
        self.assertAlmostEqual(metrics["max_tray_tilt_rad"], math.pi / 4)
        self.assertGreaterEqual(metrics["max_payload_to_tray_displacement_m"], .03)
        self.assertLess(metrics["max_payload_to_tray_displacement_m"], 2.0)

    def test_compressed_trace_is_supported(self):
        rows = [
            _row(1.0, "grasp_hold", [1.0, 0.0, 0.0, 0.0], .10),
            _row(2.0, "lift", [1.0, 0.0, 0.0, 0.0], .10),
        ]
        with tempfile.TemporaryDirectory() as directory:
            trace = Path(directory) / "trace.jsonl.gz"
            with gzip.open(trace, "wt") as handle:
                handle.write("".join(json.dumps(row) + "\n" for row in rows))
            result = extract_trace_metrics(Path(directory))
        self.assertEqual(result["trace_metrics"]["transport_samples"], 1)

    def test_missing_or_incomplete_trace_reports_reason_instead_of_zero(self):
        with tempfile.TemporaryDirectory() as directory:
            missing = extract_trace_metrics(Path(directory))
            self.assertIsNone(missing["trace_metrics"])
            self.assertIn("not found", missing["trace_metrics_missing_reason"])
            (Path(directory) / "trace.jsonl").write_text(
                json.dumps(_row(1.0, "home_start", [1.0, 0.0, 0.0, 0.0], .1))
                + "\n"
            )
            incomplete = extract_trace_metrics(Path(directory))
            self.assertIsNone(incomplete["trace_metrics"])
            self.assertIn("no transport-phase", incomplete["trace_metrics_missing_reason"])

    def test_invalid_pose_is_not_reported_as_zero_data(self):
        invalid_payload = _row(2.0, "lift", [0.0, 0.0, 0.0, 0.0], .10)
        rows = [
            invalid_payload,
            _row(3.0, "raise_high", [0.0, 0.0, 0.0, 0.0], .10),
            _row(4.0, "drive_high", [0.0, 0.0, 0.0, 0.0], .13),
        ]
        with tempfile.TemporaryDirectory() as directory:
            trace = Path(directory) / "trace.jsonl"
            trace.write_text("".join(json.dumps(row) + "\n" for row in rows))
            result = extract_trace_metrics(Path(directory))
        metrics = result["trace_metrics"]
        self.assertEqual(metrics["valid_tray_pose_samples"], 0)
        self.assertIsNone(metrics["max_abs_tray_roll_rad"])
        self.assertIsNone(metrics["max_abs_tray_pitch_rad"])
        self.assertIsNone(metrics["max_tray_tilt_rad"])
        self.assertIsNone(metrics["payload_baseline_phase"])
        self.assertIsNone(metrics["max_payload_to_tray_displacement_m"])
        self.assertIn("no valid finite tray quaternion", result["trace_metrics_missing_reason"])
        self.assertIn("no valid finite payload", result["trace_metrics_missing_reason"])

    def test_nonfinite_payload_is_filtered_and_fallback_origin_is_explicit(self):
        invalid_payload = _row(2.0, "lift", [1.0, 0.0, 0.0, 0.0], .10)
        invalid_payload["payloads"][0]["pos_m"][0] = float("nan")
        rows = [
            invalid_payload,
            _row(3.0, "raise_high", [1.0, 0.0, 0.0, 0.0], .10),
            _row(4.0, "drive_high", [1.0, 0.0, 0.0, 0.0], .13),
        ]
        with tempfile.TemporaryDirectory() as directory:
            trace = Path(directory) / "trace.jsonl"
            trace.write_text("".join(json.dumps(row) + "\n" for row in rows))
            result = extract_trace_metrics(Path(directory))
        metrics = result["trace_metrics"]
        self.assertEqual(metrics["payload_baseline_phase"], "raise_high")
        self.assertEqual(metrics["payload_baseline_time_s"], 3.0)
        self.assertEqual(
            metrics["payload_displacement_reference"],
            "first valid transport sample in tray frame",
        )
        self.assertEqual(metrics["payload_displacement_samples"], 2)
        self.assertAlmostEqual(metrics["max_payload_to_tray_displacement_m"], .03)
        self.assertIsNone(result["trace_metrics_missing_reason"])


if __name__ == "__main__":
    unittest.main()
