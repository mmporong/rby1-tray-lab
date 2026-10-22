import json
from dataclasses import replace
import unittest
import tempfile

import numpy as np

from rby1_tray.camera import RGBDFrame
from rby1_tray.mission import (
    ArmMotion,
    HIGH_CARRY_WARM_START_RAD,
    MobileSession,
    interpolate_joint_path,
    joint_path_limit_violation,
    tray_tcp_targets,
)
from rby1_tray.perception import PerceptionError, TrayEstimate, GraspEndpoint
from rby1_tray.scene import load_config


class MissionContractTests(unittest.TestCase):
    @staticmethod
    def _pickup_config():
        config = load_config("configs/mobile-native.json")
        config["base"]["initial_pose_xy_yaw"] = [0.0, 0.0, 0.0]
        return config

    @staticmethod
    def _head_frame(timestamp_sim_s=0.0):
        return RGBDFrame(
            rgb=np.zeros((400, 640, 3), dtype=np.uint8),
            depth_m=np.ones((400, 640)),
            rgb_raw=np.zeros((400, 640, 3), dtype=np.uint8),
            depth_raw_m=np.ones((400, 640)),
            K=np.array([[500.0, 0, 319.5], [0, 500.0, 199.5], [0, 0, 1]]),
            T_world_optical=np.eye(4),
            timestamp_sim_s=timestamp_sim_s,
            metadata={
                "camera_name": "head_rgbd",
                "calibration_source": "test",
                "optical_frame": "+X right, +Y down, +Z forward",
            },
        )

    @staticmethod
    def _wrist_frame(camera_name, timestamp_sim_s=0.0):
        return RGBDFrame(
            rgb=np.zeros((360, 640, 3), dtype=np.uint8),
            depth_m=np.full((360, 640), 0.1),
            rgb_raw=np.zeros((360, 640, 3), dtype=np.uint8),
            depth_raw_m=np.full((360, 640), 0.1),
            K=np.array([[500.0, 0, 319.5], [0, 500.0, 179.5], [0, 0, 1]]),
            T_world_optical=np.eye(4),
            timestamp_sim_s=timestamp_sim_s,
            metadata={
                "camera_name": camera_name,
                "calibration_source": "test",
                "optical_frame": "+X right, +Y down, +Z forward",
            },
        )

    def test_perceived_pose_defines_native_rim_targets(self):
        center = np.array([0.64, -0.03, 0.012])
        targets, rotation = tray_tcp_targets(center, np.pi / 2, .27, .055)
        np.testing.assert_allclose(
            targets,
            [[.91, -.03, .067], [.37, -.03, .067]],
            atol=1e-12,
        )
        np.testing.assert_allclose(rotation @ [0, 1, 0], [-1, 0, 0], atol=1e-12)

    def test_tcp_target_validation_rejects_invalid_geometry(self):
        with self.assertRaisesRegex(ValueError, "center_world"):
            tray_tcp_targets([0, 0], 0, .27, .055)
        with self.assertRaisesRegex(ValueError, "positive"):
            tray_tcp_targets([0, 0, 0], 0, 0, .055)

    def test_arm_motion_rejects_incomplete_joint_reference(self):
        with self.assertRaisesRegex(ValueError, "start_joints"):
            ArmMotion(
                "raise_high", 1.0, np.zeros(19), np.zeros(20), -.015, -.015,
                (0.7, 0, 1.312), (.25, 0, 0),
                (np.zeros(20), np.zeros(20)), {},
            )

    def test_cubic_joint_path_is_continuous_and_checks_exact_overshoot(self):
        path = [np.zeros(20), np.ones(20), np.zeros(20), np.zeros(20)]
        np.testing.assert_allclose(interpolate_joint_path(path, 0.0), path[0])
        np.testing.assert_allclose(interpolate_joint_path(path, 3.0), path[-1])
        epsilon = 1e-6
        left_velocity = (
            interpolate_joint_path(path, 1.0)
            - interpolate_joint_path(path, 1.0 - epsilon)
        ) / epsilon
        right_velocity = (
            interpolate_joint_path(path, 1.0 + epsilon)
            - interpolate_joint_path(path, 1.0)
        ) / epsilon
        np.testing.assert_allclose(left_velocity, right_velocity, atol=1e-5)
        self.assertEqual(
            joint_path_limit_violation(path, np.zeros(20), np.ones(20)), 0.0
        )
        path[1][0] = 1.1
        self.assertAlmostEqual(
            joint_path_limit_violation(path, np.zeros(20), np.ones(20)), 0.1
        )

    def test_scratch_task_pose_respects_grasp_state(self):
        session = MobileSession.__new__(MobileSession)
        session._detected_tray = TrayEstimate(
            center_world_m=(.65, 0, .012), yaw_rad=0.0,
            dimensions_m=(.372, .552), covariance_xyzyaw=tuple(
                tuple(row) for row in np.eye(4)
            ),
            plane_normal_world=(0, 0, 1), pixel_count=100,
            quality={}, grasps=(),
        )
        session._planned_tray_center = np.array([1.2, 0, 1.212])
        np.testing.assert_allclose(
            session._predicted_task_center("pickup_descend", [9, 9, 9]),
            [.65, 0, .012],
        )
        np.testing.assert_allclose(
            session._predicted_task_center("lift", [.65, 0, .082]),
            [.65, 0, .082],
        )
        np.testing.assert_allclose(
            session._predicted_task_center("withdraw", [1.2, 0, 1.312]),
            [1.2, 0, 1.212],
        )

    def test_seat_load_transfer_requires_observable_finite_baseline(self):
        expected_weight_n = 4.54203
        for baseline, settled in (
            (None, 0.0),
            (0.0, 0.0),
            (np.nan, 0.0),
            (4.3, np.inf),
        ):
            passed, minimum = MobileSession._load_transfer_valid(
                baseline, settled, expected_weight_n
            )
            self.assertFalse(passed)
            self.assertAlmostEqual(minimum, 0.25 * expected_weight_n)
        passed, _ = MobileSession._load_transfer_valid(4.319, 0.0, expected_weight_n)
        self.assertTrue(passed)

    def test_invalid_rgbd_terminates_without_ground_truth_fallback(self):
        frame = self._head_frame()

        def reject(*_args, **_kwargs):
            raise PerceptionError("test observation has no tray")

        session = MobileSession(observation_provider=lambda _data: frame, pose_estimator=reject)
        initial_tray = session.data.body("tray").xpos.copy()
        session._enter_phase("observe_close")
        result = session.step()
        self.assertTrue(result["finished"])
        self.assertEqual(result["mission"]["failure"]["reason"], "perception_failed")
        self.assertEqual(result["result"]["failure_reason"], "perception_failed")
        self.assertNotIn("detected_tray", result["mission"])
        np.testing.assert_allclose(session.data.body("tray").xpos, initial_tray, atol=1e-8)
        self.assertFalse(result["audit"]["runtime_state_overwrite"])
        session.close()

    def test_first_control_packet_declares_sensor_only_inputs(self):
        frame = self._head_frame()
        estimate = TrayEstimate(
            center_world_m=(.65, 0, .012), yaw_rad=0.0,
            dimensions_m=(.372, .552),
            covariance_xyzyaw=tuple(tuple(row) for row in np.eye(4) * 1e-5),
            plane_normal_world=(0, 0, 1), pixel_count=1000,
            quality={"source": "rgbd_observation", "score_0_to_1": .95},
            grasps=(
                GraspEndpoint("left", (.65, .27, .067), -np.pi / 2),
                GraspEndpoint("right", (.65, -.27, .067), -np.pi / 2),
            ),
        )
        session = MobileSession(
            config=self._pickup_config(),
            observation_provider=lambda data: replace(frame, timestamp_sim_s=float(data.time)),
            pose_estimator=lambda *_args, **_kwargs: estimate,
        )
        while session.phase in {"home_start", "prepare_pickup"} and not session.finished:
            session.step(100)
        self.assertFalse(session.finished)
        self.assertEqual(session.phase, "drive_approach")
        solve_seeds = []
        solve = session.robot.solve

        def capture_solve(*args, **kwargs):
            solve_seeds.append(np.asarray(kwargs["seed"], dtype=float).copy())
            return solve(*args, **kwargs)

        session.robot.solve = capture_solve
        session._enter_phase("observe_close")
        result = session.step()
        packet = result["mission"]["command"]
        self.assertEqual(result["phase"], "pickup_approach")
        self.assertEqual(packet["controller_inputs"]["tray_pose"], "rgbd_estimate")
        self.assertEqual(packet["controller_inputs"]["base_pose"], "wheel_encoder_odometry")
        self.assertFalse(packet["controller_inputs"]["simulator_object_pose_used"])
        self.assertEqual(len(packet["upper_body_position_reference_rad"]), 20)
        self.assertEqual(len(packet["wheel_velocity_reference_rad_s"]), 4)
        observation = packet["observation"]
        self.assertEqual(len(observation["upper_body_joint_position_rad"]), 20)
        self.assertEqual(len(observation["upper_body_joint_velocity_rad_s"]), 20)
        self.assertEqual(len(observation["wheel_encoder_angle_rad"]), 4)
        self.assertEqual(len(observation["wheel_encoder_velocity_rad_s"]), 4)
        self.assertFalse(observation["ground_truth_body_pose_or_contact_included"])
        self.assertEqual(set(observation["wrist_wrench_local"]), {"left", "right"})
        self.assertEqual(len(session.sensor_frames), 1)
        self.assertNotIn("model_calibration", session.sensor_frames[0].metadata)
        self.assertFalse(
            session.sensor_frames[0].metadata["ground_truth_extrinsics_in_controller_packet"]
        )
        self.assertFalse(result["audit"]["runtime_state_overwrite"])
        motion = session._active_motion
        self.assertIsNotNone(motion)
        self.assertEqual(len(motion.joint_path_rad), 3)
        self.assertFalse(any(
            np.array_equal(waypoint, HIGH_CARRY_WARM_START_RAD)
            for waypoint in motion.joint_path_rad
        ))
        np.testing.assert_allclose(solve_seeds[0], motion.start_joints_rad)
        np.testing.assert_allclose(solve_seeds[1], HIGH_CARRY_WARM_START_RAD)
        self.assertLess(
            float(np.max(np.abs(
                motion.joint_path_rad[1][:6] - motion.start_joints_rad[:6]
            ))),
            0.01,
        )
        pickup_curve = np.array([
            interpolate_joint_path(motion.joint_path_rad, segment + alpha)
            for segment in range(len(motion.joint_path_rad) - 1)
            for alpha in np.linspace(0.0, 1.0, 51)
        ])
        self.assertLess(
            float(np.max(np.abs(
                pickup_curve[:, :6] - motion.start_joints_rad[:6]
            ))),
            0.03,
        )
        self.assertEqual(
            motion.ik_metrics["seed_policy"],
            "crouched_arm_clearance_preapproach_then_high_branch_ik_seed_without_executed_waypoint",
        )
        self.assertEqual(
            motion.ik_metrics["pickup_preapproach_offset_m"], [0.05, 0.0, 0.11]
        )
        class FakeWristCamera:
            def __init__(self, supplied):
                self.supplied = supplied
                self.calibration_override = None

            def capture(self, _data):
                return self.supplied

            def close(self):
                pass

        session._wrist_cameras = {
            "left": FakeWristCamera(self._wrist_frame("left_wrist_rgbd", float(session.data.time))),
            "right": FakeWristCamera(self._wrist_frame("right_wrist_rgbd", float(session.data.time))),
        }
        session._contacts = lambda: self.fail(
            "controller wrist verification must not inspect contact geometry"
        )
        verification = session._contact_wrist_verifier(session)
        self.assertFalse(verification["contact_geometry_ids_used_by_controller"])
        self.assertIn("force_delta_n", verification["proprioception"]["left"])
        with tempfile.TemporaryDirectory() as directory:
            exported = session.export_sim2real_dataset(directory)
            self.assertTrue((exported / "observations/0000_head_rgbd.npz").is_file())
            self.assertTrue((exported / "commands.jsonl").is_file())
            schema = (exported / "schema.json").read_text()
            self.assertIn("task-object qpos", schema)
            self.assertIn("wrist six-axis force-torque", schema)
            limits = json.loads(schema)["limits"]
            self.assertEqual(len(limits["head_joint_lower_rad"]), 2)
            self.assertEqual(len(limits["head_joint_upper_rad"]), 2)
        session.close()

    def test_departure_uses_model_qpos0_and_return_targets_same_pose(self):
        session = MobileSession()
        try:
            self.assertEqual(session.phase, "home_start")
            np.testing.assert_array_equal(session.command_rad, session.model.qpos0[session.robot.qadr])
            self.assertEqual(session.command_gripper_m, float(session.model.qpos0[session.robot.grip_qadr[0]]))
            self.assertFalse(session.snapshot()["result"]["success"])
            self.assertIn("returned_home", session.snapshot()["result"]["milestones"])
            session._planned_tray_center = np.array([1.2, 0.0, 1.212])
            session.phase = "withdraw"
            session._advance_phase()
            self.assertEqual(session.phase, "drive_return")
            self.assertIsNone(session._drive_start_tray_center)
            self.assertEqual(session._drive_target(), session.config["base"]["initial_pose_xy_yaw"])
            self.assertFalse(session.finished)
            self.assertEqual(session.home_reference["joint_position_rad"], session.command_rad.tolist())
        finally:
            session.close()

    def test_stale_rgbd_terminates_before_pose_estimation(self):
        estimator_called = False

        def estimator(*_args, **_kwargs):
            nonlocal estimator_called
            estimator_called = True
            raise AssertionError("stale RGB-D must be rejected before estimation")

        session = MobileSession(
            observation_provider=lambda _data: self._head_frame(-1.0),
            pose_estimator=estimator,
        )
        session._enter_phase("observe_close")
        result = session.step()
        self.assertTrue(result["finished"])
        self.assertEqual(result["mission"]["failure"]["reason"], "perception_failed")
        self.assertIn("stale RGB-D frame", result["mission"]["failure"]["detail"])
        self.assertFalse(estimator_called)
        session.close()


if __name__ == "__main__":
    unittest.main()
