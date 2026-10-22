import unittest
from pathlib import Path

import mujoco
import numpy as np

from rby1_tray.scene import audit_model, build_scene, load_config
from rby1_tray.planning import Robot
from rby1_tray.assets import asset_directory


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

    def test_target_has_collidable_floor_connected_support(self):
        deck = self.model.geom('stand_deck')
        container = self.model.geom('container_bottom')
        deck_top = deck.pos[2] + deck.size[2]
        container_bottom = container.pos[2] - container.size[2]
        self.assertAlmostEqual(deck_top, container_bottom)
        for sx in (-1, 1):
            for sy in (-1, 1):
                leg = self.model.geom(f'stand_leg_{sx}_{sy}')
                self.assertAlmostEqual(leg.pos[2] - leg.size[2], 0)
                self.assertAlmostEqual(leg.pos[2] + leg.size[2], deck.pos[2] - deck.size[2])
                self.assertTrue(leg.contype[0] or leg.conaffinity[0])
        self.assertTrue(container.contype[0] or container.conaffinity[0])

    def test_connect_attachment_to_free_tray_is_rejected(self):
        self.model.eq_type[0] = mujoco.mjtEq.mjEQ_CONNECT
        self.model.eq_obj1id[0] = self.model.body("tray").id
        self.model.eq_obj2id[0] = self.model.body("base").id
        with self.assertRaises(ValueError):
            audit_model(self.model)

    def test_mobile_ft_boundary_preserves_native_hand_kinematics(self):
        config = load_config(Path(__file__).resolve().parents[1] / 'configs/mobile-native.json')
        model, _ = build_scene(config)
        upstream = mujoco.MjModel.from_xml_path(str(asset_directory() / 'rby1m_1.2.xml'))
        current_data, upstream_data = mujoco.MjData(model), mujoco.MjData(upstream)
        mujoco.mj_forward(model, current_data)
        mujoco.mj_forward(upstream, upstream_data)
        for side, letter in [('left', 'l'), ('right', 'r')]:
            site = model.site(f'{side}_ft_frame')
            self.assertEqual(model.body(int(site.bodyid[0])).name, f'{side}_ft_mount')
            for name in (f'EE_BODY_{letter.upper()}', f'ee_finger_{letter}1', f'ee_finger_{letter}2'):
                np.testing.assert_allclose(current_data.body(name).xpos, upstream_data.body(name).xpos, atol=1e-12)
                np.testing.assert_allclose(current_data.body(name).xmat, upstream_data.body(name).xmat, atol=1e-12)
        self.assertFalse(any('handle' in (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or '')
                             for i in range(model.ngeom)))
        for i in range(model.ngeom):
            if '_pad_' in model.geom(i).name:
                self.assertEqual(model.geom_type[i], mujoco.mjtGeom.mjGEOM_MESH)
                self.assertGreaterEqual(model.geom_dataid[i], 0)
                np.testing.assert_allclose(model.geom_friction[i], [2, .05, .01])


if __name__ == "__main__": unittest.main()
