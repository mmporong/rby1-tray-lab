import unittest

import numpy as np

from rby1_tray.camera import NamedRGBDCamera
from rby1_tray.mission import MobileSession
from rby1_tray.scene import load_config


class SensorIntegrationTests(unittest.TestCase):
    def test_render_preserves_live_dynamics_and_solver_state(self):
        config = load_config("configs/mobile-native.json")
        config["monitoring"]["enabled"] = False
        session = MobileSession(config)
        camera = NamedRGBDCamera(session.model, "head_rgbd", width=160, height=100)
        names = ("qpos", "qvel", "qacc_warmstart", "sensordata", "efc_force")
        before = {name: getattr(session.data, name).copy() for name in names}
        try:
            frame = camera.capture(session.data)
            self.assertEqual(frame.rgb.shape, (100, 160, 3))
            for name in names:
                np.testing.assert_array_equal(getattr(session.data, name), before[name], err_msg=name)
        finally:
            camera.close()
            session.close()

    def test_monitors_share_capture_batch_and_exclude_gt_calibration(self):
        config = load_config("configs/mobile-native.json")
        config["monitoring"]["width_px"] = 160
        session = MobileSession(config)
        try:
            session.step()
            self.assertEqual(len(session.monitor_frames), 3)
            self.assertEqual(len({f.metadata["capture_batch_id"] for f in session.monitor_frames}), 1)
            self.assertEqual(len({f.timestamp_sim_s for f in session.monitor_frames}), 1)
            for frame in session.monitor_frames:
                self.assertEqual(frame.metadata["observation_role"], "monitor_only")
                self.assertNotIn("model_calibration", frame.metadata)
            self.assertFalse(session.snapshot()["audit"]["runtime_state_overwrite"])
        finally:
            session.close()


if __name__ == "__main__":
    unittest.main()
