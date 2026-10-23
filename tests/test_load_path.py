from types import SimpleNamespace
import unittest

import numpy as np

from rby1_tray.load_path import (
    external_support_wrench,
    gravity_support_wrench,
    load_path_components,
    residual_wrench,
    subtree_body_ids,
)


def fixture():
    model = SimpleNamespace(
        nbody=5,
        body_parentid=np.array([0, 0, 1, 2, 1]),
        body_rootid=np.array([0, 1, 1, 1, 1]),
        body_mass=np.array([0.0, 2.0, 3.0, 4.0, 5.0]),
        opt=SimpleNamespace(gravity=np.array([0.0, 0.0, -10.0])),
    )
    data = SimpleNamespace(
        xipos=np.array([[0, 0, 0], [0, 0, 0], [2, 0, 0], [2, 3, 0], [0, 4, 0]], dtype=float),
        subtree_com=np.array([[0, 0, 0], [1, 2, 3], [9, 9, 9], [9, 9, 9], [9, 9, 9]], dtype=float),
        cfrc_ext=np.zeros((5, 6), dtype=float),
    )
    return model, data


class LoadPathTests(unittest.TestCase):
    def test_subtree_membership_uses_parent_chain(self):
        model, _ = fixture()
        np.testing.assert_array_equal(subtree_body_ids(model, 1), [1, 2, 3, 4])
        np.testing.assert_array_equal(subtree_body_ids(model, 2), [2, 3])
        np.testing.assert_array_equal(subtree_body_ids(model, 4), [4])

    def test_gravity_support_includes_offset_moment(self):
        model, data = fixture()
        force, moment = gravity_support_wrench(
            model, data, [2, 3], [1, 0, 0], np.eye(3)
        )
        np.testing.assert_allclose(force, [0, 0, 70])
        # -sum((COM-origin) x m*g): y component -70, x component +120.
        np.testing.assert_allclose(moment, [120, -70, 0])

    def test_external_reaction_is_transported_from_root_c_frame(self):
        model, data = fixture()
        data.cfrc_ext[2] = [1, 2, 3, 4, 5, 6]
        data.cfrc_ext[3] = [7, 11, 13, 17, 19, 23]
        force, moment = external_support_wrench(
            model, data, [2, 3], [4, 2, 3], np.eye(3)
        )
        summed_force = np.array([21, 24, 29])
        summed_moment = np.array([8, 13, 16])
        expected_moment = -(summed_moment + np.cross([-3, 0, 0], summed_force))
        np.testing.assert_allclose(force, -summed_force)
        np.testing.assert_allclose(moment, expected_moment)

    def test_residual_closes_identity_and_opposite_action_flips_terms(self):
        model, data = fixture()
        data.cfrc_ext[2, 3:] = [1, 2, 3]
        parent = load_path_components(model, data, 2, [0, 0, 0], np.eye(3), action_sign=1)
        reaction = load_path_components(model, data, 2, [0, 0, 0], np.eye(3), action_sign=-1)
        np.testing.assert_allclose(reaction[0], -parent[0])
        np.testing.assert_allclose(reaction[1], -parent[1])
        total = np.array([2, 3, 5, 7, 11, 13], dtype=float)
        residual = residual_wrench(total, *parent)
        np.testing.assert_allclose(parent[0] + parent[1] + residual, total)

    def test_gravity_and_external_support_can_cancel_without_a_motion_share(self):
        model, data = fixture()
        # Bodies 2+3 weigh 70 N. A 70 N upward external force yields an equal
        # downward external-support reaction at the same cut.
        data.cfrc_ext[2, 5] = 70
        gravity = gravity_support_wrench(model, data, [2, 3], [0, 0, 0], np.eye(3))
        external = external_support_wrench(model, data, [2, 3], [0, 0, 0], np.eye(3))
        np.testing.assert_allclose(gravity[0] + external[0], np.zeros(3))
        total = np.concatenate((gravity[0] + external[0], gravity[1] + external[1]))
        np.testing.assert_allclose(
            residual_wrench(total, np.concatenate(gravity), np.concatenate(external)),
            np.zeros(6),
        )

    def test_fem_site_frame_and_base_source_conventions_are_representable(self):
        model, data = fixture()
        angle = np.pi / 2
        rotation = np.array([[np.cos(angle), -np.sin(angle), 0],
                             [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
        base_source = load_path_components(
            model, data, 4, [0.25, 0.0, 0.0], rotation, action_sign=-1
        )
        fem_site = load_path_components(
            model, data, 2, [0.0, 0.5, 0.0], rotation, action_sign=-1
        )
        self.assertEqual(base_source[0].shape, (6,))
        self.assertEqual(fem_site[0].shape, (6,))
        self.assertTrue(np.all(np.isfinite(np.concatenate((*base_source, *fem_site)))))
        self.assertLess(base_source[0][2], 0)
        self.assertLess(fem_site[0][2], 0)

    def test_invalid_frames_and_mixed_roots_fail_closed(self):
        model, data = fixture()
        with self.assertRaises(ValueError):
            gravity_support_wrench(model, data, [1], [0, 0, 0], np.zeros((3, 3)))
        model.body_rootid[4] = 4
        with self.assertRaises(ValueError):
            external_support_wrench(model, data, [2, 4], [0, 0, 0], np.eye(3))


if __name__ == "__main__":
    unittest.main()
