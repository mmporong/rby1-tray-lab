import unittest
from copy import deepcopy
from unittest import mock

import numpy as np
from scipy.spatial.transform import Rotation

from rby1_tray.planning import Robot, build_plan
from rby1_tray.scene import build_scene, load_config


class FixedTorsoPlanningTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_config()
        cls.model, _ = build_scene(cls.config)

    def test_arm_only_ablation_keeps_all_actuated_joints(self):
        robot = Robot(self.model, use_torso=False)

        self.assertEqual(len(robot.names), 20)
        self.assertEqual(len(robot.qadr), 20)
        self.assertEqual(len(robot.dadr), 20)
        self.assertEqual(len(robot.act), 20)
        np.testing.assert_array_equal(robot.active, np.arange(6, 20))

    def test_arm_only_plan_freezes_torso_at_shared_pickup_posture(self):
        config = dict(self.config)
        config["use_torso"] = False

        robot, plan, results = build_plan(self.model, config, lift_only=True)

        self.assertGreater(len(plan), 4)
        for waypoint in plan:
            self.assertEqual(waypoint.joints_rad.shape, (20,))
            np.testing.assert_allclose(
                waypoint.joints_rad[:6], robot.fixed_torso, rtol=0, atol=1e-12)
        self.assertTrue(all(result["forbidden_contact_count"] == 0 for result in results))

    def test_custom_fixed_torso_is_preserved_exactly(self):
        fixed = np.array([0, 1.516, -2.20, 1.50, 0, 0])
        robot = Robot(self.model, use_torso=False, fixed_torso_rad=fixed)
        targets = [
            np.array([.65, -.24, .132]),
            np.array([.65, .24, .132]),
        ]

        q, metrics = robot.solve(
            targets,
            posture=robot.crouched_pickup_posture(),
            gripper_m=0.0,
        )

        np.testing.assert_array_equal(q[:6], fixed)
        self.assertEqual(q.shape, (20,))
        self.assertLessEqual(metrics["gravity_torque_ratio"], robot.TORQUE_LIMIT_RATIO)
        self.assertEqual(set(metrics["elbow_flexion_rad"]), {"right", "left"})

    def test_solve_tracks_explicit_tool_rotations(self):
        robot = Robot(self.model)
        targets = np.array([
            [.65, -.24, .132],
            [.65, .24, .132],
        ])
        q, _ = robot.solve(
            targets,
            posture=robot.crouched_pickup_posture(),
            gripper_m=0.0,
        )
        robot._prepare(q, 0.0)
        rotations = np.array([
            robot.work.site_xmat[site].reshape(3, 3).copy()
            for site in robot.tcp
        ])

        solved, metrics = robot.solve(
            targets,
            seed=q,
            gripper_m=0.0,
            target_rotations=rotations,
        )

        self.assertEqual(solved.shape, (20,))
        self.assertLessEqual(
            metrics["orientation_error_rad"], robot.ORIENTATION_TOLERANCE_RAD)

    def test_rejects_invalid_tool_rotation(self):
        robot = Robot(self.model)
        targets = [[.65, -.24, .132], [.65, .24, .132]]
        invalid = np.repeat(np.eye(3)[None, :, :], 2, axis=0)
        invalid[1, 0, 0] = 2

        with self.assertRaisesRegex(ValueError, "proper rotation matrices"):
            robot.solve(targets, target_rotations=invalid)

    def test_rejects_gripper_reference_outside_upstream_limits(self):
        robot = Robot(self.model)
        targets = [[.65, -.24, .132], [.65, .24, .132]]

        with self.assertRaisesRegex(ValueError, "gripper_m must be within"):
            robot.solve(targets, gripper_m=.01)

    def test_plan_passes_configured_grasp_yaw_to_every_ik_request(self):
        config = deepcopy(self.config)
        config["grasp"] = {
            "side_offset_y_m": .27,
            "tcp_height_m": .047,
            "yaw_rad": np.pi / 2,
            "open_gripper_m": -.045,
            "closed_gripper_m": -.015,
        }
        expected = Rotation.from_euler("z", np.pi / 2).as_matrix()

        with mock.patch.object(Robot, "solve", autospec=True) as solve:
            solve.side_effect = lambda robot, targets, seed=None, **kwargs: (
                robot.crouched_pickup_posture(),
                {
                    "position_error_m": 0.0,
                    "orientation_error_rad": 0.0,
                    "gravity_torque_ratio": 0.0,
                    "elbow_flexion_rad": {"right": 0.0, "left": 0.0},
                    "forbidden_contact_count": 0,
                },
            )
            _, plan, results = build_plan(self.model, config, lift_only=True)

        self.assertGreater(solve.call_count, 0)
        for call in solve.call_args_list:
            rotations = call.kwargs["target_rotations"]
            np.testing.assert_allclose(rotations[0], expected, atol=1e-12)
            np.testing.assert_allclose(rotations[1], expected, atol=1e-12)
        self.assertTrue(
            all(waypoint.gripper_m == -.015 for waypoint in plan if waypoint.phase == "grasp"))
        self.assertTrue(all(len(result["target_tcp_positions_m"]) == 2 for result in results))
        self.assertTrue(all(len(result["target_tcp_rotations"]) == 2 for result in results))
        self.assertTrue(all("gripper_reference_m" in result for result in results))

    def test_detected_tray_pose_drives_world_grasp_positions_and_yaw(self):
        config = deepcopy(self.config)
        config["grasp"] = {
            "side_offset_y_m": .27,
            "tcp_height_m": .055,
            "yaw_rad": np.pi / 2,
        }
        tray_pose = {
            "center_world_m": [.5, .1, .02],
            "yaw_rad": .2,
        }
        tray_rotation = Rotation.from_euler("z", .2).as_matrix()
        expected_tool_rotation = Rotation.from_euler(
            "z", .2 + np.pi / 2).as_matrix()

        with mock.patch.object(Robot, "solve", autospec=True) as solve:
            solve.side_effect = lambda robot, targets, seed=None, **kwargs: (
                robot.crouched_pickup_posture(),
                {
                    "position_error_m": 0.0,
                    "orientation_error_rad": 0.0,
                    "gravity_torque_ratio": 0.0,
                    "elbow_flexion_rad": {"right": 0.0, "left": 0.0},
                    "forbidden_contact_count": 0,
                },
            )
            build_plan(
                self.model, config, lift_only=True, tray_pose=tray_pose)

        first_call = solve.call_args_list[0]
        expected_center = np.array([.5, .1, .12])
        expected_targets = np.array([
            expected_center + tray_rotation @ np.array([0, -.27, .055]),
            expected_center + tray_rotation @ np.array([0, .27, .055]),
        ])
        np.testing.assert_allclose(first_call.args[1], expected_targets, atol=1e-12)
        np.testing.assert_allclose(
            first_call.kwargs["target_rotations"][0],
            expected_tool_rotation,
            atol=1e-12,
        )

    def test_invalid_fixed_torso_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "six finite"):
            Robot(self.model, use_torso=False, fixed_torso_rad=[0, 1, 2])
        with self.assertRaisesRegex(ValueError, "joint limits"):
            Robot(self.model, use_torso=False, fixed_torso_rad=[0, 2, 0, 0, 0, 0])


if __name__ == "__main__":
    unittest.main()
