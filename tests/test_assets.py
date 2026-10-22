import subprocess
import tempfile
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from rby1_tray.assets import (
    FINGER_PROXY_CENTER_M,
    FINGER_PROXY_HALF_EXTENTS_M,
    REPOSITORY,
    REVISION,
    asset_directory,
    finger_proxy_comparison,
    provenance,
    validate_checkout,
)
from rby1_tray.scene import build_scene, load_config


class AssetIntegrityTests(unittest.TestCase):
    def test_cached_checkout_and_license_are_verified(self):
        path = asset_directory()
        result = provenance(path)
        self.assertEqual(result["repository"], REPOSITORY)
        self.assertEqual(result["revision"], REVISION)
        self.assertTrue(result["worktree_clean"])
        self.assertEqual(result["license"], "Apache-2.0")
        self.assertIn("rby1a_1.2.xml", result["files"])
        self.assertIn("LICENSE", result["files"])
        self.assertGreater(len(result["files"]), 10)
        self.assertTrue(all(len(digest) == 64 for digest in result["files"].values()))

    def test_dirty_checkout_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            checkout = Path(temporary)
            subprocess.run(["git", "init", "-q", str(checkout)], check=True)
            subprocess.run(["git", "-C", str(checkout), "config", "user.email", "test@example.invalid"], check=True)
            subprocess.run(["git", "-C", str(checkout), "config", "user.name", "Asset Test"], check=True)
            (checkout / "tracked.txt").write_text("clean\n")
            subprocess.run(["git", "-C", str(checkout), "add", "tracked.txt"], check=True)
            subprocess.run(["git", "-C", str(checkout), "commit", "-qm", "fixture"], check=True)
            subprocess.run(["git", "-C", str(checkout), "remote", "add", "origin", REPOSITORY], check=True)
            revision = subprocess.check_output(
                ["git", "-C", str(checkout), "rev-parse", "HEAD"], text=True
            ).strip()
            validate_checkout(checkout, expected_revision=revision)
            (checkout / "untracked.txt").write_text("unexpected\n")
            with self.assertRaisesRegex(ValueError, "local changes"):
                validate_checkout(checkout, expected_revision=revision)
            (checkout / "untracked.txt").unlink()
            (checkout / "tracked.txt").write_text("changed\n")
            with self.assertRaisesRegex(ValueError, "local changes"):
                validate_checkout(checkout, expected_revision=revision)

    def test_proxy_measurement_matches_generated_scene(self):
        comparison = finger_proxy_comparison(asset_directory())
        self.assertEqual(comparison["proxy"]["center_m"], FINGER_PROXY_CENTER_M)
        self.assertEqual(
            comparison["proxy"]["half_extents_m"], FINGER_PROXY_HALF_EXTENTS_M
        )
        self.assertEqual(
            comparison["source_meshes"]["EE_FINGER_0.obj"]["extent_m"],
            (0.003, 0.032, 0.06),
        )
        self.assertEqual(comparison["contact_mesh_extent_ratio"], (2.0, 0.875, 0.833333333))
        self.assertEqual(comparison["contact_mesh_center_offset_m"], (0.0015, 0.0, 0.0035))
        _, xml = build_scene(load_config())
        root = ET.fromstring(xml)
        proxies = [
            geom for geom in root.findall(".//geom")
            if geom.get("name", "").endswith("_proxy")
        ]
        self.assertEqual(len(proxies), 4)
        for proxy in proxies:
            self.assertEqual(
                tuple(float(value) for value in proxy.get("pos").split()),
                FINGER_PROXY_CENTER_M,
            )
            self.assertEqual(
                tuple(float(value) for value in proxy.get("size").split()),
                FINGER_PROXY_HALF_EXTENTS_M,
            )


if __name__ == "__main__":
    unittest.main()
