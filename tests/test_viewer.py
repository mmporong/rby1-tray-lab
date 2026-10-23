import json
from io import BytesIO
from pathlib import Path
import tempfile
import threading
import unittest
from http.server import HTTPServer
from types import SimpleNamespace
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import numpy as np
from PIL import Image

from rby1_tray.viewer import _byte_range, _make_handler, _view_value


class FakeSession:
    def __init__(self):
        self.finished = True
        self.reset_calls = 0
        self.render_calls = []
        self.sensor_frames = []
        self.monitor_frames = []

    def snapshot(self):
        return {"time_s": 0.0, "phase": "approach", "finished": self.finished, "result": {}}

    def reset(self):
        self.reset_calls += 1
        self.finished = False

    def render(self, **kwargs):
        self.render_calls.append(kwargs)
        return np.zeros((720, 960, 3), dtype=np.uint8)


class ViewerContractTests(unittest.TestCase):
    def setUp(self):
        self.session = FakeSession()
        self.directory = tempfile.TemporaryDirectory()
        video = Path(self.directory.name) / 'overview.mp4'
        video.write_bytes(b'0123456789')
        image = BytesIO()
        Image.new('RGB', (2, 2), 'blue').save(image, format='PNG')
        self.recorded_sensor = image.getvalue()
        class FakePlayback:
            def info(inner):
                return {'enabled': True, 'base_speed': 8, 'fps': 60,
                        'speed_options': [1, 1.5, 2]}

            def asset(inner, name):
                return (video, 'video/mp4', None) if name == 'overview.mp4' else None

            def sensor_png(inner, camera, kind, time_s):
                metadata = {
                    "sensor_profile": "test_rgbd",
                    "observation_role": "monitor_only",
                    "depth_range_m": {"minimum": 0.1, "maximum": 4.0},
                    "calibration_source": "test",
                    "clock_domain": "simulation_time",
                }
                return (self.recorded_sensor, 1.0, metadata) if time_s >= 1.0 else None
        class FakeImprovement:
            def info(self):
                return {"enabled": True, "schema_version": 1, "studies": {}}

            def asset(self, name):
                return (video, "video/mp4", None) if name == "videos/test.mp4" else None
        self.server = HTTPServer(("127.0.0.1", 0), _make_handler(
            self.session, FakePlayback(), improvement=FakeImprovement()
        ))
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.base_url = f"http://127.0.0.1:{self.server.server_port}"

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)
        self.directory.cleanup()

    def test_playback_capability_reports_native_rates(self):
        with urlopen(f'{self.base_url}/api/playback', timeout=3) as response:
            info = json.load(response)
        self.assertEqual(info['speed_options'], [1, 1.5, 2])
        self.assertEqual(info['base_speed'], 8)
        self.assertEqual(info['fps'], 60)

    def test_improvement_endpoint_and_media_are_independent_of_original_playback(self):
        with urlopen(f'{self.base_url}/api/improvement', timeout=3) as response:
            self.assertEqual(json.load(response)["schema_version"], 1)
        request = Request(f'{self.base_url}/improvement/videos/test.mp4', headers={"Range": "bytes=2-5"})
        with urlopen(request, timeout=3) as response:
            self.assertEqual(response.status, 206)
            self.assertEqual(response.read(), b"2345")
        request = Request(f'{self.base_url}/improvement/videos/test.mp4', method="HEAD")
        with urlopen(request, timeout=3) as response:
            self.assertEqual(response.headers["Accept-Ranges"], "bytes")
            self.assertEqual(response.read(), b"")
        with self.assertRaises(HTTPError) as caught:
            urlopen(f'{self.base_url}/improvement/overview.mp4', timeout=3)
        self.assertEqual(caught.exception.code, 404)
        self.assertEqual(self.session.reset_calls, 0)

    def test_video_range_and_head_support_native_seeking(self):
        request = Request(f'{self.base_url}/playback/overview.mp4', headers={'Range': 'bytes=2-5'})
        with urlopen(request, timeout=3) as response:
            self.assertEqual(response.status, 206)
            self.assertEqual(response.headers['Content-Range'], 'bytes 2-5/10')
            self.assertEqual(response.read(), b'2345')
        request = Request(f'{self.base_url}/playback/overview.mp4', method='HEAD')
        with urlopen(request, timeout=3) as response:
            self.assertEqual(response.headers['Content-Length'], '10')
            self.assertEqual(response.headers['Accept-Ranges'], 'bytes')
            self.assertEqual(response.read(), b'')

    def test_invalid_video_range_and_unknown_asset_are_rejected(self):
        request = Request(f'{self.base_url}/playback/overview.mp4', headers={'Range': 'bytes=15-'})
        with self.assertRaises(HTTPError) as caught:
            urlopen(request, timeout=3)
        self.assertEqual(caught.exception.code, 416)
        self.assertEqual(caught.exception.headers['Content-Range'], 'bytes */10')
        with self.assertRaises(HTTPError) as caught:
            urlopen(f'{self.base_url}/playback/not-a-declared-file', timeout=3)
        self.assertEqual(caught.exception.code, 404)

    def test_recorded_sensor_observations_follow_playback_time(self):
        for time_s, status in [('0.5', 404), ('NaN', 400), ('-1', 400)]:
            with self.assertRaises(HTTPError) as caught:
                urlopen(f'{self.base_url}/sensor.png?camera=head_rgbd&kind=rgb&time_s={time_s}', timeout=3)
            self.assertEqual(caught.exception.code, status)
        with urlopen(f'{self.base_url}/sensor.png?camera=head_rgbd&kind=rgb&time_s=2', timeout=3) as response:
            self.assertEqual(response.read(), self.recorded_sensor)
            self.assertEqual(response.headers['X-Sensor-Timestamp-S'], '1')

    def test_byte_ranges_support_suffix_open_end_and_reject_multiple(self):
        self.assertEqual(_byte_range('bytes=-3', 10), (7, 9))
        self.assertEqual(_byte_range('bytes=7-', 10), (7, 9))
        self.assertEqual(_byte_range('bytes=0-999', 10), (0, 9))
        for value in ['bytes=0-1,3-5', 'bytes=-0', 'bytes=-', 'other=0-1']:
            with self.assertRaises(ValueError):
                _byte_range(value, 10)

    def test_replay_endpoint_resets_a_finished_session(self):
        request = Request(f"{self.base_url}/api/replay", data=b"{}", method="POST")
        with urlopen(request, timeout=3) as response:
            state = json.load(response)

        self.assertEqual(self.session.reset_calls, 1)
        self.assertFalse(state["finished"])

    def test_frame_endpoint_passes_named_view_to_session(self):
        with urlopen(f"{self.base_url}/frame.png?view=placement&azimuth=120", timeout=3) as response:
            self.assertEqual(response.headers.get_content_type(), "image/png")

        self.assertEqual(self.session.render_calls[-1]["view"], "placement")
        self.assertEqual(self.session.render_calls[-1]["azimuth"], 120.0)

    def test_unknown_view_falls_back_to_overview(self):
        self.assertEqual(_view_value({"view": ["unknown"]}), "overview")
        self.assertEqual(_view_value({}), "overview")

    def test_sensor_endpoint_returns_latest_stored_rgb_frame(self):
        old = np.full((2, 3, 3), 10, dtype=np.uint8)
        latest = np.full((2, 3, 3), 220, dtype=np.uint8)
        for timestamp, rgb in ((1.0, old), (2.5, latest)):
            self.session.sensor_frames.append(SimpleNamespace(
                rgb=rgb,
                depth_m=np.ones((2, 3), dtype=float),
                timestamp_sim_s=timestamp,
                metadata={
                    "camera_name": "head_rgbd",
                    "depth_range_m": {"minimum": 0.1, "maximum": 4.0},
                },
            ))

        with urlopen(
            f"{self.base_url}/sensor.png?camera=head_rgbd&kind=rgb", timeout=3
        ) as response:
            image = np.asarray(Image.open(response))
            timestamp = response.headers["X-Sensor-Timestamp-S"]

        self.assertTrue(np.all(image == 220))
        self.assertEqual(timestamp, "2.5")

    def test_sensor_depth_preview_marks_invalid_pixels_black(self):
        self.session.monitor_frames.append(SimpleNamespace(
            rgb=np.zeros((2, 2, 3), dtype=np.uint8),
            depth_m=np.array([[0.1, 0.3], [np.nan, 0.2]]),
            timestamp_sim_s=4.0,
            metadata={
                "camera_name": "left_wrist_rgbd",
                "sensor_profile": "D405",
                "observation_role": "monitor_only",
                "depth_range_m": {"minimum": 0.1, "maximum": 0.3},
                "calibration_source": "test_mount",
                "clock_domain": "simulation_time",
            },
        ))

        with urlopen(
            f"{self.base_url}/sensor.png?camera=left_wrist_rgbd&kind=depth",
            timeout=3,
        ) as response:
            image = np.asarray(Image.open(response))

        self.assertTrue(np.all(image[1, 0] == [230, 55, 135]))
        self.assertTrue(np.all(image[0, 0] == 255))
        self.assertTrue(np.all(image[0, 1] == 1))

    def test_sensor_endpoint_selects_latest_across_monitor_and_controller_frames(self):
        common = {
            "camera_name": "head_rgbd",
            "sensor_profile": "ZED X Mini",
            "depth_range_m": {"minimum": 0.2, "maximum": 8.0},
            "calibration_source": "synthetic_mount_design",
            "clock_domain": "simulation_time",
        }
        self.session.sensor_frames.append(SimpleNamespace(
            rgb=np.full((2, 2, 3), 20, dtype=np.uint8),
            depth_m=np.ones((2, 2)), timestamp_sim_s=1.0,
            metadata={**common, "observation_role": "controller"},
        ))
        self.session.monitor_frames.append(SimpleNamespace(
            rgb=np.full((2, 2, 3), 230, dtype=np.uint8),
            depth_m=np.ones((2, 2)), timestamp_sim_s=2.0,
            metadata={
                **common,
                "observation_role": "monitor_only",
                "capture_batch_id": "monitor-2.000000",
            },
        ))
        for camera in ("left_wrist_rgbd", "right_wrist_rgbd"):
            self.session.monitor_frames.append(SimpleNamespace(
                rgb=np.full((2, 2, 3), 100, dtype=np.uint8),
                depth_m=np.ones((2, 2)), timestamp_sim_s=2.0,
                metadata={
                    **common,
                    "camera_name": camera,
                    "observation_role": "monitor_only",
                    "capture_batch_id": "monitor-2.000000",
                },
            ))

        with urlopen(
            f"{self.base_url}/sensor.png?camera=head_rgbd&kind=rgb", timeout=3
        ) as response:
            image = np.asarray(Image.open(response))
            headers = response.headers

        self.assertTrue(np.all(image == 230))
        self.assertEqual(headers["X-Sensor-Timestamp-S"], "2")
        self.assertEqual(headers["X-Sensor-Observation-Role"], "monitor_only")
        self.assertEqual(headers["X-Sensor-Capture-Batch-Id"], "monitor-2.000000")
        self.assertEqual(headers["X-Sensor-Depth-Min-M"], "0.2")
        self.assertEqual(headers["X-Sensor-Depth-Max-M"], "8")

    def test_sensor_endpoint_reports_missing_and_invalid_requests(self):
        for path, status in (
            ("/sensor.png?camera=right_wrist_rgbd&kind=rgb", 404),
            ("/sensor.png?camera=unknown&kind=rgb", 400),
            ("/sensor.png?camera=head_rgbd&kind=pointcloud", 400),
        ):
            with self.assertRaises(HTTPError) as caught:
                urlopen(f"{self.base_url}{path}", timeout=3)
            self.assertEqual(caught.exception.code, status)


if __name__ == "__main__":
    unittest.main()
