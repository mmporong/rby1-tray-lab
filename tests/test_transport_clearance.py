import unittest

import numpy as np

from rby1_tray.mission import MobileSession
from rby1_tray.perception import GraspEndpoint, TrayEstimate
from rby1_tray.scene import build_scene, load_config
from rby1_tray.transport_clearance import (
    carried_tray_wall_clearance_m,
    container_near_wall_outer_x_m,
    geometry_clearance_safe_x_m,
    rotated_tray_x_support_m,
)


class TransportClearanceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_config("configs/mobile-native.json")
        cls.model, _ = build_scene(cls.config)

    def test_compiled_geometry_support_and_wall_face(self):
        self.assertAlmostEqual(container_near_wall_outer_x_m(self.model), 0.926, places=12)
        self.assertAlmostEqual(rotated_tray_x_support_m(self.model, 0.0), 0.186, places=12)
        self.assertAlmostEqual(
            rotated_tray_x_support_m(self.model, 0.035), 0.195334, places=6
        )
        self.assertAlmostEqual(
            rotated_tray_x_support_m(self.model, -0.035), 0.195334, places=6
        )

    def test_legacy_reference_and_geometry_clearance_safe_x(self):
        legacy_safe_x = 0.95 - self.config["tray_half_extents_m"][0] - 0.03
        self.assertAlmostEqual(legacy_safe_x, 0.740, places=12)
        safe_x = geometry_clearance_safe_x_m(self.model, 0.035, 0.75, 0.03)
        self.assertAlmostEqual(safe_x, 0.700666, places=6)

    def test_candidate_config_changes_only_transport_planning(self):
        baseline = load_config("configs/mobile-native.json")
        candidate = load_config("configs/mobile-native-improved.json")
        transport = candidate.pop("transport_planning")
        self.assertEqual(candidate, baseline)
        self.assertEqual(transport["profile"], "geometry_clearance")
        self.assertEqual(transport["clearance_m"], 0.03)

    def test_mobile_session_wires_legacy_and_candidate_profiles(self):
        session = MobileSession.__new__(MobileSession)
        session.model = self.model
        session._detected_tray = TrayEstimate(
            center_world_m=(0.65, 0.0, 0.012),
            yaw_rad=0.035,
            dimensions_m=(0.372, 0.552),
            covariance_xyzyaw=tuple(tuple(row) for row in np.eye(4)),
            plane_normal_world=(0.0, 0.0, 1.0),
            pixel_count=100,
            quality={},
            grasps=(
                GraspEndpoint("left", (.65, .27, .05), .035),
                GraspEndpoint("right", (.65, -.27, .05), .035),
            ),
        )
        session.config = load_config("configs/mobile-native.json")
        safe_x, metrics = session._raise_high_safe_x(0.75)
        self.assertAlmostEqual(safe_x, 0.740, places=12)
        self.assertIsNone(metrics)
        session.config = load_config("configs/mobile-native-improved.json")
        safe_x, metrics = session._raise_high_safe_x(0.75)
        self.assertAlmostEqual(safe_x, 0.700666, places=6)
        assert metrics is not None
        self.assertEqual(metrics["profile"], "geometry_clearance")
        self.assertEqual(
            metrics["preflight_center_source"],
            "commanded_tray_center_linear_interpolation_not_fk",
        )

    def test_vertical_raise_preserves_requested_clearance_for_both_yaw_signs(self):
        for yaw in (0.0, 0.035, -0.035):
            safe_x = geometry_clearance_safe_x_m(self.model, yaw, 0.75, 0.03)
            samples = np.linspace(
                [safe_x, 0.0, 0.32],
                [safe_x, 0.0, 1.312],
                200,
            )
            finite = [
                value
                for point in samples
                if (value := carried_tray_wall_clearance_m(self.model, point, yaw))
                is not None
            ]
            self.assertTrue(finite)
            self.assertGreaterEqual(min(finite), 0.03 - 1e-12)

    def test_malformed_profile_cannot_silently_select_legacy(self):
        session = MobileSession.__new__(MobileSession)
        for transport in (None, [], {}, {"clearance_m": .03},
                          {"profile": "unknown"}, {"profile": "geometry_clearance"}):
            with self.subTest(transport=transport):
                session.config = {"transport_planning": transport}
                with self.assertRaises(ValueError):
                    session._raise_high_safe_x(.75)

    def test_old_waypoint_exposes_yaw_collision_risk(self):
        clearance = carried_tray_wall_clearance_m(
            self.model, (0.74, 0.0, 1.23), 0.035
        )
        self.assertIsNotNone(clearance)
        assert clearance is not None
        self.assertLess(clearance, 0.0)


if __name__ == "__main__":
    unittest.main()
