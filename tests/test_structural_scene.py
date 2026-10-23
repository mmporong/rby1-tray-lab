import hashlib
import json
import math
from pathlib import Path
import struct
import subprocess
import tempfile
import threading
import unittest
from http.server import HTTPServer
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from rby1_tray.structural_scene import ASSET_NAMES, StructuralSceneAssets
from rby1_tray.viewer import _UnavailableStructuralScene, _make_handler


STATE_HASH = "a" * 64


class FakePlayback:
    manifest = {"artifact_sha256": {"states.json.gz": STATE_HASH}}

    def info(self):
        return {"enabled": True, "frame_count": 2, "fps": 60, "base_speed": 8}


class FakeSession:
    def snapshot(self):
        return {"time_s": 0.0, "phase": "home_start", "finished": False}


def floats(*values):
    return struct.pack(f"<{len(values)}f", *values)


def fixture(directory: Path):
    geometry = floats(
        0, 0, 0, 0, 0, 1,
        .1, 0, 0, 0, 0, 1,
        0, .1, 0, 0, 0, 1,
    )
    part_frame = [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 5, 0, 10, 0, 1, 0, 1e6]
    frames = floats(*(part_frame + part_frame + part_frame + part_frame))
    fem_vertex = [0, 0, 0, 0, 0, 1] + [0] * 36
    fem_vertex[6] = 1
    fem = floats(*(fem_vertex * 3))
    payloads = {
        "geometry.bin": geometry,
        "frames.bin": frames,
        "fem-surface.bin": fem,
        "fem-cutaway.bin": fem,
    }
    for name, payload in payloads.items():
        (directory / name).write_bytes(payload)
    manifest = {
        "schema_version": 1,
        "fps": 60,
        "frame_count": 2,
        "base_speed": 8,
        "reference_states_sha256": STATE_HASH,
        "frame_times_s": [0.0, 1 / 60],
        "frame_stride_floats": 38,
        "part_stride_floats": 19,
        "geometry_stride_floats": 6,
        "max_von_mises_pa": 1e6,
        "parts": [
            {
                "id": "torso",
                "label": "몸통",
                "kind": "section",
                "analysis_status": "section_assumption",
                "vertex_offset": 0,
                "vertex_count": 3,
                "section": {
                    "origin_body_m": [0, 0, 0],
                    "basis_body": [1, 0, 0, 0, 1, 0, 0, 0, 1],
                    "area_m2": .01,
                    "I_m4": 1e-5,
                    "J_m4": 2e-5,
                },
                "assumptions": ["실제 외형에 등가 단면을 대응"],
            },
            {
                "id": "left_flange_fem",
                "label": "왼쪽 플랜지",
                "kind": "fem",
                "vertex_offset": 3,
                "vertex_count": 0,
                "assumptions": ["준정적 선형 탄성 가정"],
            },
        ],
        "fem_part_id": "left_flange_fem",
        "fem_stride_floats": 42,
        "fem_surface_vertex_count": 3,
        "fem_cutaway_vertex_count": 3,
        "assets": {name: hashlib.sha256(payload).hexdigest() for name, payload in payloads.items()},
        "scope": "기록된 하중을 이용한 동기 구조 응력 표시",
        "assumptions": ["전신은 등가 단면 추정", "플랜지는 CAD FEM"],
    }
    (directory / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    return manifest


class StructuralSceneAssetsTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.directory = Path(self.temporary.name)
        self.manifest = fixture(self.directory)

    def tearDown(self):
        self.temporary.cleanup()

    def write_manifest(self, manifest):
        (self.directory / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")

    def test_valid_scene_exposes_summary_and_only_fixed_asset_urls(self):
        assets = StructuralSceneAssets(self.directory, FakePlayback())
        info = assets.info()
        self.assertTrue(info["enabled"])
        self.assertEqual(info["frame_count"], 2)
        self.assertEqual(info["parts"][0]["label"], "몸통")
        self.assertNotIn("assets", info)
        self.assertEqual(set(info["asset_urls"]), set(ASSET_NAMES))
        self.assertIsNotNone(assets.asset("geometry.bin"))
        self.assertIsNone(assets.asset("../manifest.json"))
        self.assertIsNone(assets.asset("manifest.json"))

    def add_load_diagnostics(self):
        terms = [0, 0, 4, 0, 0, 0, 1, 0, 6, 0, .5, 0, 4, 0, 0, 0, .5, 0]
        payload = floats(*(terms * 4))
        (self.directory / "diagnostics.bin").write_bytes(payload)
        digest = hashlib.sha256(payload).hexdigest()
        self.manifest["assets"]["diagnostics.bin"] = digest
        self.manifest["load_diagnostics"] = {
            "asset": "diagnostics.bin", "sha256": digest,
            "part_stride_floats": 18, "frame_stride_floats": 36,
            "parts": [{"id": part["id"], "parent_label": "부모", "source_body_label": "하부",
                       "action": "child_on_parent" if part["kind"] == "fem" else "parent_on_child", "subtree_mass_kg": 1.0,
                       "origin_body_m": [0, 0, 0], "basis_body": [1, 0, 0, 0, 1, 0, 0, 0, 1]}
                      for part in self.manifest["parts"]],
        }
        self.write_manifest(self.manifest)

    def test_optional_load_terms_must_reconstruct_same_frame_wrench(self):
        self.add_load_diagnostics()
        assets = StructuralSceneAssets(self.directory, FakePlayback())
        self.assertEqual(assets.info()["load_diagnostics"]["frame_stride_floats"], 36)
        self.assertIn("diagnostics.bin", assets.info()["asset_urls"])
        served = assets.asset("diagnostics.bin")
        assert served is not None
        self.assertEqual(len(served), 2 * 2 * 18 * 4)
        # A self-consistent byte hash cannot hide a wrong force decomposition.
        payload = bytearray((self.directory / "diagnostics.bin").read_bytes())
        struct.pack_into('<f', payload, 0, 100)
        (self.directory / "diagnostics.bin").write_bytes(payload)
        digest = hashlib.sha256(payload).hexdigest()
        self.manifest["assets"]["diagnostics.bin"] = digest
        self.manifest["load_diagnostics"]["sha256"] = digest
        self.write_manifest(self.manifest)
        with self.assertRaisesRegex(ValueError, "reconstruct"):
            StructuralSceneAssets(self.directory, FakePlayback())

    def test_load_metadata_rejects_mismatched_frame_mass_and_sources(self):
        self.add_load_diagnostics()
        original = json.loads(json.dumps(self.manifest))
        for change in ("origin", "mass", "source", "order"):
            self.manifest = json.loads(json.dumps(original))
            part = self.manifest["load_diagnostics"]["parts"][0]
            if change == "origin": part["origin_body_m"] = [1, 0, 0]
            elif change == "mass": part["subtree_mass_kg"] = -1
            elif change == "source": part["source_body_per_frame"] = ["one"]
            else: part["id"] = "wrong"
            self.write_manifest(self.manifest)
            with self.assertRaises(ValueError, msg=change):
                StructuralSceneAssets(self.directory, FakePlayback())

    def test_cut_action_and_varying_base_mass_cannot_be_mislabelled(self):
        self.add_load_diagnostics()
        original = json.loads(json.dumps(self.manifest))
        for index in (0, 1):
            self.manifest = json.loads(json.dumps(original))
            part = self.manifest["load_diagnostics"]["parts"][index]
            part["action"] = "parent_on_child" if index == 1 else "child_on_parent"
            self.write_manifest(self.manifest)
            with self.assertRaisesRegex(ValueError, "cut convention"):
                StructuralSceneAssets(self.directory, FakePlayback())
        self.manifest = json.loads(json.dumps(original))
        self.manifest["parts"][0]["id"] = "base"
        meta = self.manifest["load_diagnostics"]["parts"][0]
        meta.update(id="base", action="child_on_parent", subtree_mass_kg=None,
                    source_body_per_frame=["torso", "wheel"],
                    subtree_mass_kg_by_source={"torso": 66.0, "wheel": 2.5},
                    source_labels={"torso": "몸통", "wheel": "휠"})
        self.write_manifest(self.manifest)
        clean = StructuralSceneAssets(self.directory, FakePlayback()).info()["load_diagnostics"]["parts"][0]
        self.assertEqual(clean["source_body_per_frame"], ["torso", "wheel"])
        self.assertIsNone(clean["subtree_mass_kg"])
        meta["subtree_mass_kg"] = 66.0
        self.write_manifest(self.manifest)
        with self.assertRaisesRegex(ValueError, "no scalar mass"):
            StructuralSceneAssets(self.directory, FakePlayback())

    def test_playback_identity_and_shape_mismatches_are_rejected(self):
        for field, value, message in (
            ("frame_count", 3, "frame_count"),
            ("fps", 30, "fps"),
            ("base_speed", 16, "base_speed"),
            ("reference_states_sha256", "b" * 64, "reference states"),
            ("frame_stride_floats", 37, "part count"),
        ):
            with self.subTest(field=field):
                manifest={**self.manifest,field:value};self.write_manifest(manifest)
                with self.assertRaisesRegex(ValueError, message):
                    StructuralSceneAssets(self.directory, FakePlayback())
                self.write_manifest(self.manifest)

    def test_visual_only_status_is_exposed_and_invalid_status_is_rejected(self):
        manifest=json.loads(json.dumps(self.manifest));manifest["parts"][0]["analysis_status"]="visual_only";self.write_manifest(manifest)
        self.assertEqual(StructuralSceneAssets(self.directory,FakePlayback()).info()["parts"][0]["analysis_status"],"visual_only")
        manifest["parts"][0]["analysis_status"]="certified";self.write_manifest(manifest)
        with self.assertRaisesRegex(ValueError,"analysis_status"):
            StructuralSceneAssets(self.directory,FakePlayback())

    def test_invalid_section_geometry_and_nonfinite_buffers_are_rejected(self):
        manifest=json.loads(json.dumps(self.manifest));manifest["parts"][0]["section"]["area_m2"]=0;self.write_manifest(manifest)
        with self.assertRaisesRegex(ValueError, "area_m2"):
            StructuralSceneAssets(self.directory, FakePlayback())
        self.write_manifest(self.manifest)
        manifest=json.loads(json.dumps(self.manifest));manifest["parts"][0]["section"]["basis_body"]=[1,0,0,1,0,0,0,0,1];self.write_manifest(manifest)
        with self.assertRaisesRegex(ValueError, "not unit length|not orthogonal"):
            StructuralSceneAssets(self.directory, FakePlayback())
        self.write_manifest(self.manifest)
        payload=bytearray((self.directory/"frames.bin").read_bytes());payload[:4]=struct.pack("<f",math.nan);(self.directory/"frames.bin").write_bytes(payload)
        manifest={**self.manifest,"assets":{**self.manifest["assets"],"frames.bin":hashlib.sha256(payload).hexdigest()}};self.write_manifest(manifest)
        with self.assertRaisesRegex(ValueError, "non-finite"):
            StructuralSceneAssets(self.directory, FakePlayback())

    def test_each_recorded_pose_rotation_must_be_orthonormal(self):
        payload=bytearray((self.directory/"frames.bin").read_bytes())
        payload[3*4:4*4]=struct.pack("<f",2.0)
        (self.directory/"frames.bin").write_bytes(payload)
        manifest={**self.manifest,"assets":{**self.manifest["assets"],"frames.bin":hashlib.sha256(payload).hexdigest()}}
        self.write_manifest(manifest)
        with self.assertRaisesRegex(ValueError,"pose at frame 0, part 0"):
            StructuralSceneAssets(self.directory,FakePlayback())

    def test_recorded_peak_stress_is_nonnegative_and_matches_registry_maximum(self):
        manifest={**self.manifest,"max_von_mises_pa":1.1e6};self.write_manifest(manifest)
        with self.assertRaisesRegex(ValueError,"does not match recorded part peaks"):
            StructuralSceneAssets(self.directory,FakePlayback())
        self.write_manifest(self.manifest)
        payload=bytearray((self.directory/"frames.bin").read_bytes());payload[18*4:19*4]=struct.pack("<f",-1.0);(self.directory/"frames.bin").write_bytes(payload)
        manifest={**self.manifest,"assets":{**self.manifest["assets"],"frames.bin":hashlib.sha256(payload).hexdigest()}};self.write_manifest(manifest)
        with self.assertRaisesRegex(ValueError,"peak stress is negative"):
            StructuralSceneAssets(self.directory,FakePlayback())

    def test_hash_truncation_symlinks_and_extra_asset_declarations_are_rejected(self):
        (self.directory/"geometry.bin").write_bytes(b"broken")
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            StructuralSceneAssets(self.directory, FakePlayback())
        fixture(self.directory)
        target=self.directory/"real.bin";(self.directory/"geometry.bin").rename(target);(self.directory/"geometry.bin").symlink_to(target.name)
        with self.assertRaisesRegex(ValueError, "regular file"):
            StructuralSceneAssets(self.directory, FakePlayback())
        (self.directory/"geometry.bin").unlink();target.rename(self.directory/"geometry.bin")
        manifest={**self.manifest,"assets":{**self.manifest["assets"],"secret.bin":"c"*64}};self.write_manifest(manifest)
        with self.assertRaisesRegex(ValueError, "supported binary assets"):
            StructuralSceneAssets(self.directory, FakePlayback())


class StructuralSceneHttpTests(unittest.TestCase):
    def setUp(self):
        self.temporary=tempfile.TemporaryDirectory();self.directory=Path(self.temporary.name);fixture(self.directory)
        self.assets=StructuralSceneAssets(self.directory,FakePlayback())
        self.server=HTTPServer(("127.0.0.1",0),_make_handler(FakeSession(),structural_scene=self.assets));self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start();self.base=f"http://127.0.0.1:{self.server.server_port}"

    def tearDown(self):
        self.server.shutdown();self.server.server_close();self.thread.join(timeout=2);self.temporary.cleanup()

    def test_api_script_and_exact_binary_routes(self):
        with urlopen(self.base+"/api/structural-scene",timeout=3) as response:self.assertTrue(json.load(response)["enabled"])
        with urlopen(self.base+"/structural-scene.js",timeout=3) as response:
            self.assertEqual(response.headers.get_content_type(),"text/javascript");self.assertIn(b"window.StructuralScene",response.read())
        for name in ASSET_NAMES:
            with urlopen(self.base+f"/structural-scene/{name}",timeout=3) as response:
                self.assertEqual(response.headers.get_content_type(),"application/octet-stream");self.assertEqual(response.read(),self.assets.asset(name))
            with urlopen(Request(self.base+f"/structural-scene/{name}",method="HEAD"),timeout=3) as response:self.assertEqual(response.read(),b"")

    def test_unknown_and_traversal_scene_routes_are_rejected(self):
        for path in ("/structural-scene/manifest.json","/structural-scene/unknown.bin","/structural-scene/%2e%2e/manifest.json"):
            with self.assertRaises(HTTPError) as caught:urlopen(self.base+path,timeout=3)
            self.assertEqual(caught.exception.code,404)

    def test_page_csp_allows_same_origin_external_script(self):
        with urlopen(self.base+"/",timeout=3) as response:
            self.assertIn("script-src 'self'",response.headers["Content-Security-Policy"])

    def test_configured_invalid_scene_reports_error_without_disabling_primary_state(self):
        server=HTTPServer(("127.0.0.1",0),_make_handler(FakeSession(),structural_scene=_UnavailableStructuralScene()))
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start();base=f"http://127.0.0.1:{server.server_port}"
        try:
            with urlopen(base+"/api/structural-scene",timeout=3) as response:info=json.load(response)
            self.assertEqual(info["enabled"],False);self.assertEqual(info["configured"],True);self.assertIn("검증",info["error"])
            with urlopen(base+"/api/state",timeout=3) as response:self.assertEqual(json.load(response)["phase"],"home_start")
            with self.assertRaises(HTTPError) as caught:urlopen(base+"/structural-scene/geometry.bin",timeout=3)
            self.assertEqual(caught.exception.code,404)
        finally:
            server.shutdown();server.server_close();thread.join(timeout=2)


class StructuralSceneJavascriptTests(unittest.TestCase):
    def test_javascript_syntax_and_frame_contract(self):
        root=Path(__file__).resolve().parents[1];script=root/"src/rby1_tray/web/structural-scene.js";page=(root/"src/rby1_tray/web/index.html").read_text()
        subprocess.run(["node","--check",str(script)],check=True,capture_output=True,text=True)
        self.assertIn("structuralScene?.seekFrame(index,selectedTime)",page)
        self.assertIn("ui.video.addEventListener('seeked',()=>{syncRecordedState(true,ui.video.currentTime)",page)
        self.assertIn("ui.video.addEventListener('pause',()=>",page)
        self.assertIn("maximum=this.frames[frameBase+offset+18]",script.read_text())
        self.assertIn("'현재 전체 FEM 최대'",script.read_text())
        self.assertNotIn("maximum=result.maximum",script.read_text())
        self.assertIn("this.el.canvas.dataset.frameIndex=String(bounded)",script.read_text())
        self.assertIn("dataset.syncVideoTime=Number(videoTime).toFixed(6)",script.read_text())
        self.assertIn("canvas.dataset.lastRenderMs=elapsed.toFixed(3)",script.read_text())
        self.assertIn("Math.floor(selectedTime*playbackInfo.fps+1e-3)",page)
        self.assertIn("requestVideoFrameCallback",page)
        self.assertIn("metadata.mediaTime",page)


if __name__ == "__main__":
    unittest.main()
