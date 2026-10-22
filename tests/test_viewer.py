import json
import threading
import unittest
from http.server import HTTPServer
from types import SimpleNamespace
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import numpy as np
from PIL import Image

from rby1_tray.viewer import _make_handler, _view_value


class FakeSession:
    def __init__(self):
        self.finished = True
        self.reset_calls = 0
        self.render_calls = []
        self.sensor_frames = []

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
        self.server = HTTPServer(("127.0.0.1", 0), _make_handler(self.session))
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.base_url = f"http://127.0.0.1:{self.server.server_port}"

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)

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
                metadata={"camera_name": "head_rgbd"},
            ))

        with urlopen(
            f"{self.base_url}/sensor.png?camera=head_rgbd&kind=rgb", timeout=3
        ) as response:
            image = np.asarray(Image.open(response))
            timestamp = response.headers["X-Sensor-Timestamp-S"]

        self.assertTrue(np.all(image == 220))
        self.assertEqual(timestamp, "2.5")

    def test_sensor_depth_preview_marks_invalid_pixels_black(self):
        self.session.sensor_frames.append(SimpleNamespace(
            rgb=np.zeros((2, 2, 3), dtype=np.uint8),
            depth_m=np.array([[0.1, 0.3], [np.nan, 0.2]]),
            timestamp_sim_s=4.0,
            metadata={"camera_name": "left_wrist_rgbd"},
        ))

        with urlopen(
            f"{self.base_url}/sensor.png?camera=left_wrist_rgbd&kind=depth",
            timeout=3,
        ) as response:
            image = np.asarray(Image.open(response))

        self.assertEqual(image[1, 0], 0)
        self.assertEqual(image[0, 0], 255)
        self.assertEqual(image[0, 1], 1)

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
