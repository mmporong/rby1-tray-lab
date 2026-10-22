import unittest

import numpy as np

from rby1_tray.camera import apply_depth_range
from rby1_tray.perception import (
    PerceptionError,
    TrayCAD,
    backproject_pixels,
    estimate_tray_pose,
)


class PerceptionTests(unittest.TestCase):
    def setUp(self):
        self.height, self.width = 480, 640
        self.K = np.array([[500.0, 0, 319.5], [0, 500.0, 239.5], [0, 0, 1]])
        self.T = np.eye(4)

    def synthetic_tray(self, center=(0.0, 0.0), yaw=0.0, surface_z=1.0):
        rows, cols = np.indices((self.height, self.width))
        x = (cols - self.K[0, 2]) * surface_z / self.K[0, 0]
        y = (rows - self.K[1, 2]) * surface_z / self.K[1, 1]
        c, s = np.cos(yaw), np.sin(yaw)
        dx, dy = x - center[0], y - center[1]
        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy
        inside = (np.abs(local_x) <= .186) & (np.abs(local_y) <= .276)
        rgb = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        rgb[:] = [35, 39, 44]
        rgb[inside] = [38, 108, 164]
        depth = np.full((self.height, self.width), 2.5, dtype=float)
        depth[inside] = surface_z
        return rgb, depth

    def test_backprojection_round_trip(self):
        points = np.array([[0.1, -0.2, 1.2], [-0.3, 0.15, 2.0]])
        pixels = np.column_stack([
            self.K[0, 0] * points[:, 0] / points[:, 2] + self.K[0, 2],
            self.K[1, 1] * points[:, 1] / points[:, 2] + self.K[1, 2],
        ])
        recovered = backproject_pixels(pixels, points[:, 2], self.K, self.T)
        np.testing.assert_allclose(recovered, points, atol=1e-12)

    def test_estimates_rotated_rectangle_from_pixels_and_depth(self):
        rgb, depth = self.synthetic_tray(center=(.08, -.06), yaw=np.deg2rad(27))
        result = estimate_tray_pose(rgb, depth, self.K, self.T)
        np.testing.assert_allclose(result.center_world_m[:2], [.08, -.06], atol=.004)
        self.assertAlmostEqual(result.center_world_m[2], .988, delta=.002)
        self.assertAlmostEqual(result.yaw_rad, np.deg2rad(27), delta=np.deg2rad(1.2))
        # Pixel quantisation and morphology erode roughly one border pixel.
        np.testing.assert_allclose(result.dimensions_m, [.372, .552], atol=.015)
        self.assertEqual(result.quality["source"], "rgbd_observation")
        self.assertGreater(result.quality["score_0_to_1"], .8)
        self.assertEqual(
            result.quality["uncertainty_semantics"],
            "heuristic_uncertainty_proxy_not_calibrated_covariance",
        )
        self.assertIn("CAD_dimension_residual", result.quality["uncertainty_sources"])
        self.assertIn("camera_extrinsic_bias", result.quality["uncertainty_excludes"])
        self.assertIn("depth_scale_bias", result.quality["uncertainty_excludes"])
        self.assertGreater(result.pixel_count, 1000)

    def test_world_transform_changes_estimate_without_ground_truth(self):
        rgb, depth = self.synthetic_tray(center=(.0, .0), yaw=np.deg2rad(-18))
        transform = np.eye(4)
        transform[:3, 3] = [.4, -.2, .3]
        result = estimate_tray_pose(rgb, depth, self.K, transform)
        np.testing.assert_allclose(result.center_world_m, [.4, -.2, 1.288], atol=.004)
        self.assertAlmostEqual(result.yaw_rad, np.deg2rad(-18), delta=np.deg2rad(1.2))

    def test_grasps_use_native_rim_without_handle_offset(self):
        rgb, depth = self.synthetic_tray(yaw=0)
        result = estimate_tray_pose(rgb, depth, self.K, self.T, cad=TrayCAD())
        left, right = result.grasps
        self.assertEqual(left.contact_surface, "native_tray_rim")
        np.testing.assert_allclose(left.position_world_m, [0, .27, 1.043], atol=.004)
        np.testing.assert_allclose(right.position_world_m, [0, -.27, 1.043], atol=.004)
        self.assertAlmostEqual(left.yaw_world_rad, -np.pi / 2)
        self.assertIn("yaw+pi", result.yaw_ambiguity)

    def test_no_blue_object_fails_instead_of_using_pose_fallback(self):
        rgb = np.zeros((100, 120, 3), dtype=np.uint8)
        depth = np.ones((100, 120), dtype=float)
        with self.assertRaisesRegex(PerceptionError, "no blue tray"):
            estimate_tray_pose(rgb, depth, self.K, self.T)

    def test_wrong_dimensions_fail_cad_gate(self):
        rgb, depth = self.synthetic_tray()
        # Retain only a narrow connected blue strip.
        rgb[:, :280] = [35, 39, 44]
        rgb[:, 360:] = [35, 39, 44]
        with self.assertRaisesRegex(PerceptionError, "geometry gates"):
            estimate_tray_pose(rgb, depth, self.K, self.T)

    def test_low_saturation_blue_grey_floor_does_not_hide_tray(self):
        rgb, depth = self.synthetic_tray(center=(.08, -.06), yaw=np.deg2rad(12))
        tray = np.any(rgb != [35, 39, 44], axis=2)
        # This floor is larger and passes the original B/delta gates, but its
        # saturation is only 0.40.  The tray saturation is about 0.77.
        rgb[~tray] = [90, 120, 150]
        result = estimate_tray_pose(rgb, depth, self.K, self.T)
        np.testing.assert_allclose(result.center_world_m[:2], [.08, -.06], atol=.004)
        self.assertAlmostEqual(result.yaw_rad, np.deg2rad(12), delta=np.deg2rad(1.2))
        self.assertIn("S>=0.450", result.quality["mask_parameters"])

    def test_raised_rim_and_payload_holes_do_not_bias_plane_or_yaw(self):
        rgb, depth = self.synthetic_tray(center=(.04, -.03), yaw=np.deg2rad(21))
        tray = np.any(rgb != [35, 39, 44], axis=2)
        rows, cols = np.nonzero(tray)
        # Elevated blue rim points are valid observations, while two payloads
        # occlude asymmetric parts of the blue floor.
        edge = (
            (rows <= rows.min() + 5) | (rows >= rows.max() - 5)
            | (cols <= cols.min() + 5) | (cols >= cols.max() - 5)
        )
        depth[rows[edge], cols[edge]] += .05
        rgb[190:235, 300:345] = [220, 60, 40]
        rgb[260:305, 385:425] = [230, 210, 40]
        result = estimate_tray_pose(rgb, depth, self.K, self.T)
        np.testing.assert_allclose(result.center_world_m[:2], [.04, -.03], atol=.006)
        self.assertAlmostEqual(result.center_world_m[2], .988, delta=.003)
        self.assertAlmostEqual(result.yaw_rad, np.deg2rad(21), delta=np.deg2rad(2.0))
        self.assertLess(result.quality["plane_rms_m"], .002)
        self.assertGreater(result.quality["plane_inlier_ratio"], .55)

    def test_non_largest_component_can_win_cad_geometry_gate(self):
        rgb, depth = self.synthetic_tray(center=(.22, .12), yaw=np.deg2rad(-9))
        # A larger, saturated-blue rectangle is a distractor with the wrong CAD
        # dimensions.  Candidate selection must score geometry, not pixel count.
        rgb[5:225, 5:250] = [35, 100, 170]
        depth[5:225, 5:250] = 1.2
        result = estimate_tray_pose(rgb, depth, self.K, self.T)
        np.testing.assert_allclose(result.center_world_m[:2], [.22, .12], atol=.006)
        self.assertAlmostEqual(result.yaw_rad, np.deg2rad(-9), delta=np.deg2rad(1.0))
        self.assertGreaterEqual(result.quality["candidate_count"], 2)

    def test_image_boundary_truncation_is_rejected(self):
        rgb, depth = self.synthetic_tray(center=(-.55, 0), yaw=0)
        with self.assertRaisesRegex(PerceptionError, "touches image boundary"):
            estimate_tray_pose(rgb, depth, self.K, self.T)

    def test_depth_profile_clips_out_of_range_samples(self):
        depth = np.array([[.05, .07, .2, .5, .51, np.inf, -1]])
        clipped = apply_depth_range(depth, .07, .5)
        np.testing.assert_allclose(clipped[0, 1:4], [.07, .2, .5])
        self.assertTrue(np.all(np.isnan(clipped[0, [0, 4, 5, 6]])))
        with self.assertRaisesRegex(ValueError, "maximum depth"):
            apply_depth_range(depth, .5, .07)

    def test_invalid_inputs_are_rejected(self):
        rgb, depth = self.synthetic_tray()
        with self.assertRaisesRegex(PerceptionError, "uint8"):
            estimate_tray_pose(rgb.astype(float), depth, self.K, self.T)
        with self.assertRaisesRegex(PerceptionError, "match"):
            estimate_tray_pose(rgb, depth[:-1], self.K, self.T)
        invalid_transform = self.T.copy()
        invalid_transform[0, 0] = 2
        with self.assertRaisesRegex(PerceptionError, "proper"):
            estimate_tray_pose(rgb, depth, self.K, invalid_transform)
        with self.assertRaisesRegex(ValueError, "half extents"):
            TrayCAD(half_extents_m=(.18, -.27, .012))


if __name__ == "__main__":
    unittest.main()
