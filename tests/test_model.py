import unittest

import mujoco

from rby1_tray.scene import audit_model, build_scene, load_config
from rby1_tray.planning import Robot


class ModelContractTests(unittest.TestCase):
    def setUp(self):
        self.model, self.xml = build_scene(load_config())

    def test_four_task_objects_have_free_joints_and_active_collision(self):
        result = audit_model(self.model)
        self.assertTrue(result["free_objects"])
        self.assertFalse(result["stability_claim"])
        for name in ["tray", "payload_0", "payload_1", "payload_2"]:
            body = self.model.body(name).id
            geoms = self.model.geom_bodyid == body
            self.assertTrue(any(self.model.geom_contype[geoms] | self.model.geom_conaffinity[geoms]))

    def test_kinematic_task_object_is_rejected(self):
        self.model.body_mocapid[self.model.body("tray").id] = 0
        with self.assertRaises(ValueError): audit_model(self.model)

    def test_robot_adapter_maps_all_twenty_position_actuators(self):
        robot = Robot(self.model)
        self.assertEqual(len(robot.names), 20)
        self.assertEqual(len(set(robot.act.tolist())), 20)
        for joint, actuator in zip(robot.jids, robot.act):
            self.assertEqual(self.model.actuator_trnid[actuator, 0], joint)
            self.assertGreater(self.model.actuator_gainprm[actuator, 0], 0)
            self.assertTrue(self.model.actuator_forcelimited[actuator])

    def test_fixture_has_no_robot_free_base_or_task_object_weld(self):
        self.assertNotIn('name="world_j"', self.xml)
        self.assertNotIn('<weld', self.xml)
        self.assertEqual(self.model.neq, 2)

    def test_connect_attachment_to_free_tray_is_rejected(self):
        self.model.eq_type[0] = mujoco.mjtEq.mjEQ_CONNECT
        self.model.eq_obj1id[0] = self.model.body("tray").id
        self.model.eq_obj2id[0] = self.model.body("base").id
        with self.assertRaises(ValueError):
            audit_model(self.model)


if __name__ == "__main__": unittest.main()
