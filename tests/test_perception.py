import json
import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np

from rby1_tray.camera import apply_depth_range
from rby1_tray.perception import (
    PerceptionError,
    TrayCAD,
    backproject_pixels,
    confirm_grasp_rim_from_depth,
    estimate_tray_pose,
    estimate_tray_pose_from_depth,
    validate_tray_estimate_update,
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

    def recorded_pregrasp(self):
        repository = Path(__file__).resolve().parents[1]
        controller_data = repository / "evidence/mobile-native/controller-data"
        frames_index = json.loads((controller_data / "frames.json").read_text())
        config = json.loads(
            (repository / "evidence/mobile-native/config.json").read_text()
        )
        cad = TrayCAD(
            half_extents_m=tuple(config["tray_half_extents_m"]),
            grasp_height_m=float(config["grasp"]["tcp_height_m"]),
        )
        wrist_entries = {
            side: min(
                (
                    entry for entry in frames_index
                    if entry["camera_name"] == f"{side}_wrist_rgbd"
                ),
                key=lambda entry: float(entry["timestamp_sim_s"]),
            )
            for side in ("left", "right")
        }
        first_wrist_time = min(
            float(entry["timestamp_sim_s"]) for entry in wrist_entries.values()
        )
        head_entry = max(
            (
                entry for entry in frames_index
                if entry["camera_name"] == "head_rgbd"
                and float(entry["timestamp_sim_s"]) < first_wrist_time
            ),
            key=lambda entry: float(entry["timestamp_sim_s"]),
        )
        with np.load(controller_data / head_entry["path"]) as frame:
            estimate = estimate_tray_pose(
                frame["rgb"], frame["depth_m"], frame["K"],
                frame["T_world_optical"], cad=cad,
            )
        frames = {}
        for side, entry in wrist_entries.items():
            with np.load(controller_data / entry["path"]) as frame:
                frames[side] = {key: frame[key].copy() for key in (
                    "depth_m", "K", "T_world_optical",
                )}
        return estimate, frames, cad

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

    def test_depth_cad_estimator_ignores_tray_colour(self):
        rgb, depth = self.synthetic_tray(center=(.08, -.06), yaw=np.deg2rad(27))
        rgb[np.any(rgb != [35, 39, 44], axis=2)] = [170, 70, 45]
        with self.assertRaisesRegex(PerceptionError, "no blue tray"):
            estimate_tray_pose(rgb, depth, self.K, self.T)

        result = estimate_tray_pose_from_depth(depth, self.K, self.T)
        np.testing.assert_allclose(result.center_world_m[:2], [.08, -.06], atol=.004)
        self.assertAlmostEqual(result.yaw_rad, np.deg2rad(27), delta=np.deg2rad(1.2))
        self.assertEqual(result.quality["source"], "depth_cad_observation")
        self.assertEqual(result.quality["mask"], "world_height_connected_component")

    def test_depth_cad_estimator_tolerates_payload_occlusion(self):
        _, depth = self.synthetic_tray(center=(.04, -.03), yaw=np.deg2rad(16))
        depth[205:250, 300:345] = np.nan
        depth[270:300, 370:405] = np.nan
        result = estimate_tray_pose_from_depth(depth, self.K, self.T)
        np.testing.assert_allclose(result.center_world_m[:2], [.04, -.03], atol=.008)
        self.assertAlmostEqual(result.yaw_rad, np.deg2rad(16), delta=np.deg2rad(2.0))
        self.assertGreater(result.quality["footprint_hull_coverage"], .60)

    def test_depth_cad_estimator_rejects_boundary_truncation_and_missing_depth(self):
        _, truncated = self.synthetic_tray(center=(-.55, 0), yaw=0)
        with self.assertRaisesRegex(PerceptionError, "touches image boundary"):
            estimate_tray_pose_from_depth(truncated, self.K, self.T)
        with self.assertRaisesRegex(PerceptionError, "insufficient valid depth"):
            estimate_tray_pose_from_depth(
                np.full_like(truncated, np.nan), self.K, self.T
            )

    def test_depth_cad_estimator_rejects_wrong_size_distractor(self):
        _, depth = self.synthetic_tray(center=(.20, .12), yaw=np.deg2rad(-11))
        depth[30:130, 40:140] = 1.2
        result = estimate_tray_pose_from_depth(depth, self.K, self.T)
        np.testing.assert_allclose(result.center_world_m[:2], [.20, .12], atol=.006)
        self.assertAlmostEqual(result.yaw_rad, np.deg2rad(-11), delta=np.deg2rad(1.2))
        self.assertGreaterEqual(result.quality["candidate_count"], 2)

    def test_two_spatially_distinct_cad_trays_are_rejected_as_ambiguous(self):
        rows, cols = np.indices((self.height, self.width))
        x = (cols - self.K[0, 2]) / self.K[0, 0]
        y = (rows - self.K[1, 2]) / self.K[1, 1]
        first = (np.abs(x + .25) <= .186) & (np.abs(y) <= .276)
        second = (np.abs(x - .25) <= .186) & (np.abs(y) <= .276)
        rgb = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        rgb[:] = [35, 39, 44]
        rgb[first | second] = [38, 108, 164]
        depth = np.full((self.height, self.width), 2.5, dtype=float)
        depth[first | second] = 1.0

        with self.assertRaisesRegex(
            PerceptionError, "multiple spatial tray hypotheses"
        ):
            estimate_tray_pose_from_depth(depth, self.K, self.T)
        with self.assertRaisesRegex(
            PerceptionError, "selection_prior=none_ambiguity_reject"
        ):
            estimate_tray_pose(rgb, depth, self.K, self.T)

    def test_recorded_head_depth_merges_floor_and_rim_into_one_hypothesis(self):
        repository = Path(__file__).resolve().parents[1]
        evidence = repository / "evidence/mobile-native"
        controller_data = evidence / "controller-data"
        config = json.loads((evidence / "config.json").read_text())
        frames_index = json.loads((controller_data / "frames.json").read_text())
        first_wrist_time = min(
            float(entry["timestamp_sim_s"])
            for entry in frames_index
            if entry["camera_name"] in {"left_wrist_rgbd", "right_wrist_rgbd"}
        )
        head_entry = max(
            (
                entry for entry in frames_index
                if entry["camera_name"] == "head_rgbd"
                and float(entry["timestamp_sim_s"]) < first_wrist_time
            ),
            key=lambda entry: float(entry["timestamp_sim_s"]),
        )
        cad = TrayCAD(
            half_extents_m=tuple(config["tray_half_extents_m"]),
            grasp_height_m=float(config["grasp"]["tcp_height_m"]),
        )
        with np.load(controller_data / head_entry["path"]) as frame:
            depth_result = estimate_tray_pose_from_depth(
                frame["depth_m"], frame["K"], frame["T_world_optical"], cad=cad,
            )
            colour_result = estimate_tray_pose(
                frame["rgb"], frame["depth_m"], frame["K"],
                frame["T_world_optical"], cad=cad,
            )
        self.assertEqual(depth_result.quality["hypothesis_count"], 1)
        self.assertEqual(
            depth_result.quality["selection_prior"], "none_ambiguity_reject"
        )
        self.assertIsNone(depth_result.quality["score_margin"])
        known_station = np.asarray(config["tray_initial_pos_m"], dtype=float)
        depth_center = np.asarray(depth_result.center_world_m)
        colour_center = np.asarray(colour_result.center_world_m)
        self.assertLess(np.linalg.norm(depth_center - known_station), .015)
        self.assertLess(np.linalg.norm(depth_center - colour_center), .015)
        yaw_difference = abs(
            (depth_result.yaw_rad - colour_result.yaw_rad + np.pi / 2)
            % np.pi - np.pi / 2
        )
        self.assertLess(yaw_difference, .08)
        np.testing.assert_allclose(
            depth_result.dimensions_m, cad.outer_dimensions_m, rtol=.15
        )

    def test_reobservation_innovation_gate_accepts_consistent_pose(self):
        _, depth = self.synthetic_tray(center=(.02, -.03), yaw=np.deg2rad(8))
        previous = estimate_tray_pose_from_depth(depth, self.K, self.T)
        candidate = replace(
            previous,
            center_world_m=(.025, -.034, previous.center_world_m[2] + .002),
            yaw_rad=previous.yaw_rad + np.deg2rad(2),
        )
        result = validate_tray_estimate_update(previous, candidate)
        self.assertTrue(result["accepted"])
        self.assertLess(result["position_innovation_m"], .01)
        self.assertAlmostEqual(result["yaw_innovation_rad"], np.deg2rad(2))

    def test_reobservation_innovation_gate_rejects_biased_pose(self):
        _, depth = self.synthetic_tray(center=(.02, -.03), yaw=np.deg2rad(8))
        previous = estimate_tray_pose_from_depth(depth, self.K, self.T)
        shifted = replace(previous, center_world_m=(.10, -.03, .988))
        with self.assertRaisesRegex(PerceptionError, "position innovation"):
            validate_tray_estimate_update(previous, shifted)
        rotated = replace(previous, yaw_rad=previous.yaw_rad + np.deg2rad(12))
        with self.assertRaisesRegex(PerceptionError, "yaw innovation"):
            validate_tray_estimate_update(previous, rotated)

    def test_reobservation_yaw_gate_respects_rectangle_pi_symmetry(self):
        _, depth = self.synthetic_tray(yaw=np.deg2rad(-6))
        previous = estimate_tray_pose_from_depth(depth, self.K, self.T)
        equivalent = replace(previous, yaw_rad=previous.yaw_rad + np.pi)
        result = validate_tray_estimate_update(previous, equivalent)
        self.assertAlmostEqual(result["yaw_innovation_rad"], 0.0, delta=1e-12)

    def test_recorded_pregrasp_depth_confirms_both_native_rims(self):
        estimate, frames, cad = self.recorded_pregrasp()
        for grasp in estimate.grasps:
            frame = frames[grasp.side]
            result = confirm_grasp_rim_from_depth(
                frame["depth_m"], frame["K"], frame["T_world_optical"],
                grasp.position_world_m, estimate.center_world_m, estimate.yaw_rad,
                cad=cad,
            )
            self.assertTrue(result["passed"], grasp.side)
            self.assertGreater(result["rim_geometry_support_fraction"], .95)
            self.assertLess(result["rim_plane_rms_m"], .006)
            self.assertIn("single depth view", result["ambiguity"])

    def test_wrist_rim_confirmation_rejects_empty_and_finger_only_depth(self):
        estimate, frames, cad = self.recorded_pregrasp()
        grasp = estimate.grasps[0]
        frame = frames["left"]
        empty = np.full_like(frame["depth_m"], np.nan)
        result = confirm_grasp_rim_from_depth(
            empty, frame["K"], frame["T_world_optical"],
            grasp.position_world_m, estimate.center_world_m, estimate.yaw_rad,
            cad=cad,
        )
        self.assertFalse(result["passed"])
        self.assertIn("insufficient valid depth", result["rejection_reason"])

        optical = np.linalg.inv(frame["T_world_optical"]) @ np.append(
            grasp.position_world_m, 1.0
        )
        pixel = frame["K"] @ optical[:3]
        col, row = np.rint(pixel[:2] / pixel[2]).astype(int)
        finger_only = empty.copy()
        finger_only[row - 24: row + 25, col - 24: col + 25] = optical[2] - .03
        result = confirm_grasp_rim_from_depth(
            finger_only, frame["K"], frame["T_world_optical"],
            grasp.position_world_m, estimate.center_world_m, estimate.yaw_rad,
            cad=cad,
        )
        self.assertFalse(result["passed"])
        self.assertEqual(result["rim_geometry_support_pixels"], 0)

    def test_wrist_rim_confirmation_rejects_offset_expected_grasp(self):
        estimate, frames, cad = self.recorded_pregrasp()
        frame = frames["left"]
        offset = np.asarray(estimate.grasps[0].position_world_m) + [0, -.08, 0]
        result = confirm_grasp_rim_from_depth(
            frame["depth_m"], frame["K"], frame["T_world_optical"],
            offset, estimate.center_world_m, estimate.yaw_rad,
            cad=cad,
        )
        self.assertFalse(result["passed"])
        self.assertFalse(result["expected_grasp_on_known_rim"])
        self.assertIn("not on the known CAD rim", result["rejection_reason"])

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
