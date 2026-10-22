import unittest

import numpy as np

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

    def test_invalid_fixed_torso_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "six finite"):
            Robot(self.model, use_torso=False, fixed_torso_rad=[0, 1, 2])
        with self.assertRaisesRegex(ValueError, "joint limits"):
            Robot(self.model, use_torso=False, fixed_torso_rad=[0, 2, 0, 0, 0, 0])


if __name__ == "__main__":
    unittest.main()
