import gzip
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest


TOOLS = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

from audit_transport_contacts import audit_run
from render_study_video import _validate_replay


def _json_bytes(value):
    return (json.dumps(value, sort_keys=True) + "\n").encode()


class TransportAuditTests(unittest.TestCase):
    def test_recorded_raise_high_collision_and_manifest_validation(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            config_bytes = _json_bytes({"record_dt_s": 0.02})
            (run / "config.json").write_bytes(config_bytes)
            (run / "scene.xml").write_text(
                """<mujoco><worldbody>
                <body name="tray"><freejoint name="tray_free"/>
                  <geom name="tray_floor" type="box" size=".18 .27 .012"/>
                  <geom name="tray_xrim_-1" type="box" pos="-.18 0 .037" size=".006 .27 .025"/>
                  <geom name="tray_xrim_1" type="box" pos=".18 0 .037" size=".006 .27 .025"/>
                  <geom name="tray_yrim_-1" type="box" pos="0 -.27 .037" size=".18 .006 .025"/>
                  <geom name="tray_yrim_1" type="box" pos="0 .27 .037" size=".18 .006 .025"/>
                </body>
                <geom name="container_xwall_-1" type="box" pos=".926 0 1.235" size=".012 .384 .035"/>
                <geom name="container_xwall_1" type="box" pos="1.474 0 1.235" size=".012 .384 .035"/>
                <geom name="container_ywall_-1" type="box" pos="1.2 -.372 1.235" size=".25 .012 .035"/>
                <geom name="container_ywall_1" type="box" pos="1.2 .372 1.235" size=".25 .012 .035"/>
                </worldbody></mujoco>""",
                encoding="utf-8",
            )
            rows = [
                {"time_s": 1.0, "phase": "drive_loaded", "tray": {"pos_m": [0.74, 0, 1.2], "quat_wxyz": [1, 0, 0, 0]}},
                {"time_s": 2.0, "phase": "raise_high", "tray": {"pos_m": [0.75, 0, 1.2], "quat_wxyz": [1, 0, 0, 0]}},
            ]
            trace_bytes = b"".join(_json_bytes(row) for row in rows)
            with gzip.open(run / "trace.jsonl.gz", "wb") as stream:
                stream.write(trace_bytes)
            (run / "ik.json").write_bytes(
                _json_bytes(
                    [
                        {
                            "phase": "raise_high",
                            "transport_clearance": {"minimum_preflight_clearance_m": 0.03},
                        }
                    ]
                )
            )
            manifest = {
                "commit": "abc123",
                "artifact_sha256": {
                    "config.json": hashlib.sha256(config_bytes).hexdigest(),
                    "trace.jsonl": hashlib.sha256(trace_bytes).hexdigest(),
                    "scene.xml": hashlib.sha256((run / "scene.xml").read_bytes()).hexdigest(),
                    "ik.json": hashlib.sha256((run / "ik.json").read_bytes()).hexdigest(),
                },
            }
            (run / "manifest.json").write_bytes(_json_bytes(manifest))

            report = audit_run(run)

            self.assertEqual(report["scope"], "recorded_raise_high_frames_only")
            self.assertEqual(report["frame_count"], 1)
            self.assertEqual(report["contact_frame_count"], 1)
            self.assertEqual(report["first_contact_time_s"], 2.0)
            self.assertLess(report["minimum_contact_distance_m"], 0.0)
            self.assertEqual(report["minimum_preflight_clearance_m"], 0.03)
            self.assertEqual(report["source_trace_sha256"], hashlib.sha256(trace_bytes).hexdigest())
            (run / "ik.json").write_text("[]")
            with self.assertRaisesRegex(RuntimeError, "ik.json does not match"):
                audit_run(run)

    def test_replay_contract_rejects_metric_mismatch(self):
        expected = {
            "success": False,
            "failure_reason": "payload_lost",
            "simulation_time_s": 10.0,
            "failure_time_s": 10.0,
            "metrics": {"max_joint_torque_nm": 12.0},
        }
        _validate_replay(expected, json.loads(json.dumps(expected)))
        changed = json.loads(json.dumps(expected))
        changed["metrics"]["max_joint_torque_nm"] += 1e-6
        with self.assertRaisesRegex(RuntimeError, "metrics.max_joint_torque_nm"):
            _validate_replay(expected, changed)


if __name__ == "__main__":
    unittest.main()
