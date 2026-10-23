import hashlib
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch
from http.server import HTTPServer
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from PIL import Image

from rby1_tray.structural_view import StructuralAssets
from rby1_tray.viewer import _make_handler


class FakeSession:
    def snapshot(self):
        return {"time_s": 0.0, "phase": "home_start", "finished": False}


def write_structural_fixture(directory: Path) -> dict:
    hashes = {}
    for name, color in (("surface.png", "red"), ("cutaway.png", "blue")):
        path = directory / name
        Image.new("RGB", (8, 6), color).save(path, format="PNG")
        hashes[name] = hashlib.sha256(path.read_bytes()).hexdigest()
    report = {
        "schema_version": 1,
        "title": "RB-Y1 flange static stress",
        "snapshot_time_s": 71.704,
        "phase": "grasp_hold",
        "material": {
            "youngs_modulus_pa": 69e9,
            "poisson_ratio": 0.33,
            "source": "assumed",
        },
        "metrics": {
            "max_displacement_m": 0.00012,
            "peak_von_mises_pa": 42e6,
            "volume_p95_von_mises_pa": 19e6,
            "force_balance_relative": 1e-8,
            "moment_balance_relative": 2e-8,
            "mesh_converged": True,
        },
        "assumptions": [
            "재료 물성은 해석 가정",
            "체결면은 고정 경계",
            "MuJoCo 하중을 단방향 정적 해석에 적용",
        ],
        "artifact_sha256": hashes,
    }
    (directory / "report.json").write_text(
        json.dumps(report, ensure_ascii=False), encoding="utf-8"
    )
    return report


def replace_image(directory: Path, report: dict, name: str, size: tuple[int, int]) -> None:
    path = directory / name
    Image.new("1", size).save(path, format="PNG")
    report["artifact_sha256"][name] = hashlib.sha256(path.read_bytes()).hexdigest()
    (directory / "report.json").write_text(json.dumps(report), encoding="utf-8")


class StructuralAssetsTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.directory = Path(self.temporary.name)
        self.report = write_structural_fixture(self.directory)

    def tearDown(self):
        self.temporary.cleanup()

    def test_valid_report_exposes_only_summary_and_fixed_image_urls(self):
        assets = StructuralAssets(self.directory)

        info = assets.info()

        self.assertTrue(info["enabled"])
        self.assertEqual(info["summary"]["snapshot_time_s"], 71.704)
        self.assertEqual(info["summary"]["metrics"]["peak_von_mises_pa"], 42e6)
        self.assertNotIn("artifact_sha256", info["summary"])
        self.assertEqual(
            info["image_urls"],
            {
                "surface": "/structural/surface.png",
                "cutaway": "/structural/cutaway.png",
            },
        )
        payload = assets.asset("surface.png")
        self.assertIsNotNone(payload)
        assert payload is not None
        self.assertEqual(
            hashlib.sha256(payload).hexdigest(),
            self.report["artifact_sha256"]["surface.png"],
        )
        self.assertIsNone(assets.asset("../report.json"))
        self.assertIsNone(assets.asset("report.json"))

    def test_hash_mismatch_disables_the_entire_structural_result(self):
        (self.directory / "surface.png").write_bytes(b"changed")

        with self.assertRaisesRegex(ValueError, "SHA-256"):
            StructuralAssets(self.directory)

    def test_symlinked_directory_or_asset_is_rejected(self):
        with tempfile.TemporaryDirectory() as other:
            directory_link = self.directory.parent / "structural-link"
            directory_link.symlink_to(self.directory, target_is_directory=True)
            try:
                with self.assertRaisesRegex(ValueError, "regular directory"):
                    StructuralAssets(directory_link)
            finally:
                directory_link.unlink()

        surface = self.directory / "surface.png"
        target = self.directory / "surface-real.png"
        surface.rename(target)
        surface.symlink_to(target.name)
        with self.assertRaisesRegex(ValueError, "regular file"):
            StructuralAssets(self.directory)

    def test_invalid_units_or_schema_values_are_rejected(self):
        for field, value in (
            ("snapshot_time_s", "71.704"),
            ("snapshot_time_s", float("nan")),
        ):
            report = dict(self.report)
            report[field] = value
            (self.directory / "report.json").write_text(
                json.dumps(report), encoding="utf-8"
            )
            with self.assertRaises(ValueError):
                StructuralAssets(self.directory)

        report = dict(self.report)
        report["metrics"] = {**self.report["metrics"], "mesh_converged": 1}
        (self.directory / "report.json").write_text(
            json.dumps(report), encoding="utf-8"
        )
        with self.assertRaisesRegex(ValueError, "must be boolean"):
            StructuralAssets(self.directory)

        report = dict(self.report)
        report["material"] = {**self.report["material"], "youngs_modulus_pa": 70e9}
        (self.directory / "report.json").write_text(
            json.dumps(report), encoding="utf-8"
        )
        with self.assertRaisesRegex(ValueError, "69 GPa"):
            StructuralAssets(self.directory)

    def test_nonconverged_result_remains_visible_as_a_finding(self):
        report = dict(self.report)
        report["metrics"] = {**self.report["metrics"], "mesh_converged": False}
        (self.directory / "report.json").write_text(
            json.dumps(report), encoding="utf-8"
        )

        info = StructuralAssets(self.directory).info()

        self.assertTrue(info["enabled"])
        self.assertFalse(info["summary"]["metrics"]["mesh_converged"])

    def test_png_dimensions_and_total_pixels_are_bounded(self):
        for size in ((8193, 1), (5657, 5657)):
            with self.subTest(size=size):
                report = write_structural_fixture(self.directory)
                replace_image(self.directory, report, "surface.png", size)

                with self.assertRaisesRegex(ValueError, "8192 px / 32 MP"):
                    StructuralAssets(self.directory)

    def test_pillow_decompression_bomb_warning_and_error_are_normalized(self):
        for exception in (
            Image.DecompressionBombWarning("suspicious pixel count"),
            Image.DecompressionBombError("unsafe pixel count"),
        ):
            with self.subTest(exception=type(exception).__name__):
                with patch(
                    "rby1_tray.structural_view.Image.open", side_effect=exception
                ):
                    with self.assertRaisesRegex(ValueError, "not a valid PNG image"):
                        StructuralAssets(self.directory)


class StructuralHttpTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.directory = Path(self.temporary.name)
        write_structural_fixture(self.directory)
        self.assets = StructuralAssets(self.directory)
        self.server = HTTPServer(
            ("127.0.0.1", 0), _make_handler(FakeSession(), structural=self.assets)
        )
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.base_url = f"http://127.0.0.1:{self.server.server_port}"

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)
        self.temporary.cleanup()

    def test_api_and_exact_png_routes_are_read_only(self):
        with urlopen(f"{self.base_url}/api/structural", timeout=3) as response:
            info = json.load(response)
        self.assertTrue(info["enabled"])
        self.assertEqual(info["summary"]["phase"], "grasp_hold")

        for name in ("surface.png", "cutaway.png"):
            with urlopen(f"{self.base_url}/structural/{name}", timeout=3) as response:
                self.assertEqual(response.headers.get_content_type(), "image/png")
                self.assertEqual(
                    hashlib.sha256(response.read()).hexdigest(),
                    self.report_hash(name),
                )
            request = Request(
                f"{self.base_url}/structural/{name}", method="HEAD"
            )
            with urlopen(request, timeout=3) as response:
                self.assertGreater(int(response.headers["Content-Length"]), 0)
                self.assertEqual(response.read(), b"")

    def report_hash(self, name: str) -> str:
        return hashlib.sha256((self.directory / name).read_bytes()).hexdigest()

    def test_undeclared_and_traversal_paths_are_not_served(self):
        for path in (
            "/structural/report.json",
            "/structural/unknown.png",
            "/structural/%2e%2e/report.json",
        ):
            with self.assertRaises(HTTPError) as caught:
                urlopen(f"{self.base_url}{path}", timeout=3)
            self.assertEqual(caught.exception.code, 404)

    def test_absent_structural_assets_report_disabled_without_affecting_state(self):
        server = HTTPServer(("127.0.0.1", 0), _make_handler(FakeSession()))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            base = f"http://127.0.0.1:{server.server_port}"
            with urlopen(f"{base}/api/structural", timeout=3) as response:
                self.assertEqual(json.load(response), {"enabled": False})
            with urlopen(f"{base}/api/state", timeout=3) as response:
                self.assertEqual(json.load(response)["phase"], "home_start")
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)

    def test_decompression_bomb_isolated_before_server_start(self):
        with patch(
            "rby1_tray.structural_view.Image.open",
            side_effect=Image.DecompressionBombError("unsafe pixel count"),
        ):
            try:
                structural = StructuralAssets(self.directory)
            except ValueError:
                structural = None

        server = HTTPServer(
            ("127.0.0.1", 0), _make_handler(FakeSession(), structural=structural)
        )
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            base = f"http://127.0.0.1:{server.server_port}"
            with urlopen(f"{base}/api/structural", timeout=3) as response:
                self.assertEqual(json.load(response), {"enabled": False})
            with urlopen(f"{base}/api/state", timeout=3) as response:
                self.assertEqual(json.load(response)["phase"], "home_start")
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)


if __name__ == "__main__":
    unittest.main()
