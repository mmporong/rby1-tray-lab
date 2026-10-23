import unittest
import numpy as np
from tools.build_load_view import load_colors, geom_joint_mapping
from rby1_tray.scene import build_scene,load_config
from rby1_tray.planning import Robot

class LoadViewTests(unittest.TestCase):
    def test_normalized_torque_palette_has_fixed_scale_and_rejects_invalid(self):
        colors=load_colors([0,.5,1,2])
        np.testing.assert_allclose(colors[0], [.12,.42,.96,1])
        np.testing.assert_allclose(colors[1], [1,.82,.1,1])
        np.testing.assert_allclose(colors[2],colors[3])
        with self.assertRaises(ValueError):load_colors([np.nan])
        with self.assertRaises(ValueError):load_colors([-.01])

    def test_link_mapping_excludes_tray_and_preserves_physical_model(self):
        model,_=build_scene(load_config('configs/mobile-native.json'));robot=Robot(model)
        before=(model.geom_rgba.copy(),model.body_mass.copy(),model.actuator_forcerange.copy())
        mapping=geom_joint_mapping(model,robot)
        self.assertTrue(mapping)
        self.assertNotIn(model.geom('tray_floor').id,mapping)
        self.assertNotIn(model.geom('floor').id,mapping)
        self.assertEqual(set(mapping.values()),set(range(20)))
        for name,expected in zip(('geom_rgba','body_mass','actuator_forcerange'),before):
            np.testing.assert_array_equal(getattr(model,name),expected)

if __name__=='__main__':unittest.main()
