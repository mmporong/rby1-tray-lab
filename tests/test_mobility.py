import unittest
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from rby1_tray.mobility import (
    DriveConfig,
    DriveController,
    DriveSensors,
    WHEEL_ACTUATOR_NAMES,
)


MODEL_PATH = Path.home() / ".cache/rby1-tray-lab/menagerie/rainbow_robotics_rby1/rby1m_1.2.xml"


def mobile_model():
    root = ET.parse(MODEL_PATH).getroot()
    root.find("compiler").set("meshdir", str(MODEL_PATH.parent / "assets"))
    ET.SubElement(
        root,
        "option",
        timestep=".002",
        integrator="implicitfast",
        solver="Newton",
        iterations="50",
        cone="elliptic",
        impratio="20",
    )
    ET.SubElement(
        root.find("worldbody"),
        "geom",
        name="floor",
        type="plane",
        size="5 5 .1",
        friction="1 .01 .001",
        condim="3",
    )
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))


def hold_upper_body(model, data):
    wheel_ids = {model.actuator(name).id for name in WHEEL_ACTUATOR_NAMES}
    for actuator_id in range(model.nu):
        if actuator_id in wheel_ids:
            continue
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        qpos_id = int(model.jnt_qposadr[joint_id])
        dof_id = int(model.jnt_dofadr[joint_id])
        kp = float(model.actuator_gainprm[actuator_id, 0])
        command = data.qpos[qpos_id] + data.qfrc_bias[dof_id] / kp
        data.ctrl[actuator_id] = np.clip(
            command, *model.actuator_ctrlrange[actuator_id]
        )


class MobilityTests(unittest.TestCase):
    def setUp(self):
        self.model = mobile_model()
        self.data = mujoco.MjData(self.model)
        # Initial free-joint placement is an initialization operation, before
        # the physical rollout. Runtime motion remains mj_step-only.
        self.data.joint("world_j").qpos[:3] = (-0.4, 0.0, 0.0)
        self.controller = DriveController(
            self.model, DriveConfig(initial_pose_xy_yaw=(-0.4, 0.0, 0.0))
        )
        mujoco.mj_forward(self.model, self.data)
        self.controller.reset(self.model, self.data)

    def step(self, target=None, duration_s=0.02):
        physics_dt = self.model.opt.timestep
        control_steps = round(duration_s / physics_dt)
        for _ in range(control_steps):
            hold_upper_body(self.model, self.data)
            if target is not None:
                self.controller.update(self.model, self.data, target, physics_dt)
            else:
                self.controller.stop(self.model, self.data, physics_dt)
            mujoco.mj_step(self.model, self.data)

    def test_update_writes_only_wheel_controls_and_never_state(self):
        qpos = self.data.qpos.copy()
        qvel = self.data.qvel.copy()
        self.data.ctrl[:] = np.arange(self.model.nu) + 10.0
        before_ctrl = self.data.ctrl.copy()
        commands = self.controller.update(self.model, self.data, (0.0, 0.0, 0.0), 0.01)
        np.testing.assert_array_equal(self.data.qpos, qpos)
        np.testing.assert_array_equal(self.data.qvel, qvel)
        wheel_ids = self.controller.actuator_ids
        other_ids = np.setdiff1d(np.arange(self.model.nu), wheel_ids)
        np.testing.assert_array_equal(self.data.ctrl[other_ids], before_ctrl[other_ids])
        self.assertEqual(set(commands), set(WHEEL_ACTUATOR_NAMES))
        self.assertFalse(np.any(self.data.xfrc_applied))
        self.assertFalse(np.any(self.data.qfrc_applied))

    def test_original_wheel_mapping_and_explicit_simulation_gain(self):
        np.testing.assert_allclose(
            self.model.actuator_ctrlrange[self.controller.actuator_ids],
            [[-3.14, 3.14]] * 4,
        )
        np.testing.assert_allclose(
            self.model.actuator_gainprm[self.controller.actuator_ids, 0], 100.0
        )
        np.testing.assert_allclose(
            self.model.actuator_biasprm[self.controller.actuator_ids, 2], -100.0
        )

    def test_rejects_unvalidated_lateral_mecanum_target(self):
        with self.assertRaisesRegex(ValueError, "lateral"):
            self.controller.update(self.model, self.data, (-0.4, 0.2, 0.0), 0.01)

    def test_control_uses_encoder_odometry_not_ground_truth_pose(self):
        sensors = self.controller.read_sim_sensors(self.model, self.data)
        first = self.controller.update(
            self.model, self.data, (0.0, 0.0, 0.0), 0.01, sensors=sensors
        )
        first_values = np.array(list(first.values()))
        self.controller.reset(self.model, self.data, sensors=sensors)
        self.data.joint("world_j").qpos[0] = 2.0
        self.data.joint("world_j").qvel[0] = 5.0
        mujoco.mj_forward(self.model, self.data)
        second = self.controller.update(
            self.model, self.data, (0.0, 0.0, 0.0), 0.01, sensors=sensors
        )
        np.testing.assert_allclose(list(second.values()), first_values)

    def test_external_encoder_and_imu_sensor_packet_updates_odometry(self):
        initial = DriveSensors((0.0, 0.0, 0.0, 0.0), imu_yaw_rad=0.1)
        self.controller.reset(self.model, self.data, sensors=initial)
        advanced = DriveSensors((1.0, 1.0, 1.0, 1.0), imu_yaw_rad=0.1)
        self.controller.update(
            self.model, self.data, (-0.1, 0.0, 0.1), 0.01, sensors=advanced
        )
        odometry = self.controller.metrics(self.model, self.data)[
            "wheel_odometry_pose_xy_yaw"
        ]
        self.assertAlmostEqual(odometry[0], -0.4 + 0.076 * np.cos(0.1))
        self.assertAlmostEqual(odometry[1], 0.076 * np.sin(0.1))
        self.assertAlmostEqual(odometry[2], 0.1)

    def test_wrapped_encoder_angles_do_not_create_odometry_jump(self):
        near_wrap = DriveSensors((3.13, 3.13, 3.13, 3.13))
        self.controller.reset(self.model, self.data, sensors=near_wrap)
        wrapped = DriveSensors((-3.13, -3.13, -3.13, -3.13))
        self.controller.update(
            self.model, self.data, (-0.3, 0.0, 0.0), 0.01, sensors=wrapped
        )
        odometry_x = self.controller.metrics(self.model, self.data)[
            "wheel_odometry_pose_xy_yaw"
        ][0]
        self.assertGreater(odometry_x, -0.4)
        self.assertLess(odometry_x, -0.39)

    def test_physical_drive_reaches_40_cm_and_stops(self):
        qpos_initial = self.data.qpos.copy()
        # Closed-loop target tracking through wheel contacts only.
        self.step((0.0, 0.0, 0.0), duration_s=7.0)
        self.step(None, duration_s=2.0)
        metrics = self.controller.metrics(self.model, self.data)
        x, y, yaw = metrics["ground_truth_pose_xy_yaw"]
        speed = np.linalg.norm(metrics["base_linear_velocity_m_s"][:2])
        wheel_speed = max(abs(v) for v in metrics["wheel_velocity_rad_s"].values())
        self.assertGreater(x, -0.05)
        self.assertLess(x, 0.05)
        self.assertLess(abs(y), 0.01)
        self.assertLess(abs(yaw), 0.01)
        self.assertLess(speed, 0.01)
        self.assertLess(wheel_speed, 0.05)
        self.assertLess(metrics["base_tilt_rad"], 0.01)
        self.assertTrue(metrics["within_tilt_limit"])
        self.assertTrue(metrics["estimated_is_stopped"])
        self.assertTrue(metrics["evaluation_only_is_physically_stopped"])
        self.assertLess(
            abs(metrics["wheel_odometry_pose_xy_yaw"][0] - x), 0.01
        )
        self.assertLess(metrics["evaluation_only_odometry_error_xy_m"], 0.01)
        self.assertEqual(len(metrics["wheel_encoder_angle_rad"]), 4)
        self.assertEqual(len(metrics["wheel_actuator_torque_nm"]), 4)
        self.assertTrue(
            all(
                np.isfinite(value)
                for value in metrics["wheel_actuator_torque_nm"].values()
            )
        )
        # The base pose changed because mj_step integrated wheel contact forces.
        self.assertGreater(abs(self.data.qpos[0] - qpos_initial[0]), 0.35)


if __name__ == "__main__":
    unittest.main()
