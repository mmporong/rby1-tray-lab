import gzip
import hashlib
import json
import math
from pathlib import Path
import tempfile
import unittest

from rby1_tray.playback import (
    PlaybackAssets,
    StagedOutput,
    canonical_config_sha256,
    nearest_step_count,
)


def _write_json(path: Path, value):
    path.write_text(json.dumps(value, sort_keys=True, allow_nan=False) + "\n")


class PlaybackBundleFixture:
    def __init__(self, root: Path, config: dict):
        self.root = root
        self.config = config
        (root / "sensors").mkdir(parents=True)
        for name in ("overview.mp4", "pickup.mp4", "placement.mp4"):
            (root / name).write_bytes(f"video:{name}".encode())
        sensor_rows = []
        observations = (
            ("head_rgbd", 1.0),
            ("left_wrist_rgbd", 2.0),
            ("right_wrist_rgbd", 2.0),
            ("left_wrist_rgbd", 3.0),
            ("right_wrist_rgbd", 3.0),
        )
        for index, (camera_name, timestamp) in enumerate(observations):
            paths = {}
            for kind in ("rgb", "depth"):
                relative = f"sensors/{index:02d}_{camera_name}_{kind}.png"
                (root / relative).write_bytes(f"{kind}:{timestamp}".encode())
                paths[kind] = relative
            sensor_rows.append(
                {
                    "camera_name": camera_name,
                    "timestamp_sim_s": timestamp,
                    "rgb_path": paths["rgb"],
                    "depth_path": paths["depth"],
                }
            )
        _write_json(root / "sensors.json", {"schema_version": 1, "frames": sensor_rows})
        _write_json(root / "config.json", config)
        result = {"terminal": True, "success": True, "failure_reason": None}
        _write_json(root / "result.json", result)
        self.states = [
            {
                "frame_index": 0,
                "target_time_s": 0.0,
                "actual_time_s": 0.0,
                "terminal_frame": False,
                "state": {"time_s": 0.0, "result": {"terminal": False, "success": False}},
            },
            {
                "frame_index": 1,
                "target_time_s": 8 / 60,
                "actual_time_s": 0.134,
                "terminal_frame": False,
                "state": {"time_s": 0.134, "result": {"terminal": False, "success": False}},
            },
            {
                "frame_index": 2,
                "target_time_s": 16 / 60,
                "actual_time_s": 0.201,
                "terminal_frame": True,
                "state": {"time_s": 0.201, "result": result},
            },
        ]
        self.write_states()
        self.manifest = {
            "schema_version": 1,
            "base_speed": 8,
            "fps": 60,
            "frame_count": len(self.states),
            "duration_sim_s": 0.201,
            "width": 960,
            "height": 720,
            "physics_dt_s": 0.002,
            "source_commit": "a" * 40,
            "source_dirty": False,
            "config_sha256": canonical_config_sha256(config),
            "result": result,
        }
        self.refresh_manifest()

    def write_states(self):
        with (self.root / "states.json.gz").open("wb") as raw:
            with gzip.GzipFile(fileobj=raw, mode="wb", mtime=0) as stream:
                stream.write(json.dumps(self.states, allow_nan=False).encode())

    def refresh_manifest(self):
        self.manifest["artifact_sha256"] = {
            path.relative_to(self.root).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted(self.root.rglob("*"))
            if path.is_file() and path.name != "manifest.json"
        }
        _write_json(self.root / "manifest.json", self.manifest)


class PlaybackAssetsTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name) / "bundle"
        self.config = {"base_mode": "mobile", "physics_dt_s": 0.002, "seed": 7}
        self.fixture = PlaybackBundleFixture(self.root, self.config)

    def tearDown(self):
        self.temporary.cleanup()

    def test_valid_bundle_exposes_verified_playback_contract(self):
        assets = PlaybackAssets(self.root, self.config)

        info = assets.info()
        self.assertTrue(info["enabled"])
        self.assertEqual(info["speed_options"], [1, 1.5, 2])
        self.assertEqual(info["simulation_speed_options"], [8, 12, 16])
        self.assertEqual(info["frame_count"], 3)
        self.assertEqual(info["videos"]["placement"], "/playback/placement.mp4")
        path, mime, encoding = assets.asset("states.json.gz")
        self.assertEqual(path, self.root / "states.json.gz")
        self.assertEqual(mime, "application/json")
        self.assertEqual(encoding, "gzip")
        self.assertIsNone(assets.asset("../manifest.json"))

    def test_artifact_hash_tampering_is_rejected(self):
        (self.root / "overview.mp4").write_bytes(b"tampered")

        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            PlaybackAssets(self.root, self.config)

    def test_configuration_mismatch_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "configuration hash"):
            PlaybackAssets(self.root, {**self.config, "seed": 8})

    def test_sensor_lookup_never_returns_a_future_frame(self):
        assets = PlaybackAssets(self.root, self.config)

        self.assertIsNone(assets.sensor_png("head_rgbd", "rgb", 0.999))
        payload, timestamp = assets.sensor_png("head_rgbd", "rgb", 1.5)
        self.assertEqual(timestamp, 1.0)
        self.assertEqual(payload, b"rgb:1.0")
        payload, timestamp = assets.sensor_png("right_wrist_rgbd", "depth", 99.0)
        self.assertEqual(timestamp, 3.0)
        self.assertEqual(payload, b"depth:3.0")
        with self.assertRaisesRegex(ValueError, "finite"):
            assets.sensor_png("head_rgbd", "rgb", math.nan)

    def test_noncontiguous_frame_index_is_rejected_even_with_valid_hash(self):
        self.fixture.states[1]["frame_index"] = 4
        self.fixture.write_states()
        self.fixture.refresh_manifest()

        with self.assertRaisesRegex(ValueError, "contiguous"):
            PlaybackAssets(self.root, self.config)

    def test_sampling_rounds_to_nearest_physics_step(self):
        self.assertEqual(nearest_step_count(0.0, 8 / 60, 0.002), 67)
        self.assertEqual(nearest_step_count(0.134, 16 / 60, 0.002), 66)
        self.assertEqual(nearest_step_count(1.0, 0.5, 0.002), 0)
        with self.assertRaises(ValueError):
            nearest_step_count(0.0, 1.0, 0.0)

    def test_invalid_or_mismatched_manifest_physics_step_is_rejected(self):
        for value in (None, math.nan, math.inf, 0.0, -0.002, 0.004):
            with self.subTest(value=value):
                self.fixture.manifest["physics_dt_s"] = value
                (self.root / "manifest.json").write_text(
                    json.dumps(self.fixture.manifest, sort_keys=True) + "\n"
                )
                with self.assertRaisesRegex(ValueError, "physics_dt_s"):
                    PlaybackAssets(self.root, self.config)
        self.fixture.manifest["physics_dt_s"] = 0.002

    def test_invalid_active_physics_step_is_rejected_as_value_error(self):
        for value in (None, math.nan, math.inf, 0.0):
            with self.subTest(value=value):
                with self.assertRaisesRegex(ValueError, "physics_dt_s"):
                    PlaybackAssets(self.root, {**self.config, "physics_dt_s": value})

    def test_staged_output_cleans_only_its_partial_build(self):
        destination = Path(self.temporary.name) / "retryable"
        staged = StagedOutput(destination)
        partial = staged.create()
        (partial / "partial.mp4").write_bytes(b"incomplete")

        staged.cleanup()

        self.assertFalse(destination.exists())
        self.assertFalse(partial.exists())
        existing = Path(self.temporary.name) / "existing"
        existing.mkdir()
        sentinel = existing / "owner-file.txt"
        sentinel.write_text("keep")
        with self.assertRaises(FileExistsError):
            StagedOutput(existing).create()
        self.assertEqual(sentinel.read_text(), "keep")

    def test_sensor_index_rejects_unknown_camera_and_missing_coverage(self):
        document = json.loads((self.root / "sensors.json").read_text())
        document["frames"][0]["camera_name"] = "unknown_rgbd"
        _write_json(self.root / "sensors.json", document)
        self.fixture.refresh_manifest()
        with self.assertRaisesRegex(ValueError, "camera_name"):
            PlaybackAssets(self.root, self.config)

        document["frames"][0]["camera_name"] = "head_rgbd"
        for row in document["frames"]:
            if row["camera_name"] == "right_wrist_rgbd":
                row["camera_name"] = "left_wrist_rgbd"
        _write_json(self.root / "sensors.json", document)
        self.fixture.refresh_manifest()
        with self.assertRaisesRegex(ValueError, "cover"):
            PlaybackAssets(self.root, self.config)

    def test_source_change_prevents_staging_publication(self):
        destination = Path(self.temporary.name) / "source-frozen"
        staged = StagedOutput(destination)
        partial = staged.create()
        (partial / "manifest.json").write_text("{}")

        try:
            with self.assertRaisesRegex(RuntimeError, "source changed"):
                staged.commit_if_sources_match({"source.py": "before"}, {"source.py": "after"})
        finally:
            staged.cleanup()

        self.assertFalse(destination.exists())
        self.assertFalse(partial.exists())


if __name__ == "__main__":
    unittest.main()
