import unittest
from unittest.mock import patch
import numpy as np

from rby1_tray.session import Session


class RuntimeAuditTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.session = Session(lift_only=True)

    @classmethod
    def tearDownClass(cls):
        cls.session.close()

    def setUp(self):
        self.session.reset()

    def test_one_tick_forbidden_contact_is_not_lost_between_log_samples(self):
        original = self.session.snapshot
        def injected():
            value = original()
            value['contacts']['forbidden'] = ['one_tick_collision']
            return value
        with patch.object(self.session, 'snapshot', side_effect=injected):
            self.session.step(1)
        self.assertTrue(self.session.finished)
        self.assertEqual(self.session.evaluator.result()['failure_reason'], 'forbidden_contact')
        self.assertEqual(self.session.evaluator.result()['failure_time_s'], self.session.config['physics_dt_s'])
        self.assertEqual(self.session.log[-1]['contacts']['forbidden'], ['one_tick_collision'])

    def test_velocity_override_is_latched(self):
        self.session.data.joint('payload_0_free').qvel[0] += .1
        self.session.step()
        self.assertTrue(self.session.audit['runtime_state_overwrite'])

    def test_external_force_is_detected(self):
        self.session.data.xfrc_applied[self.session.model.body('tray').id, 0] = 1
        self.session.step()
        self.assertTrue(self.session.audit['runtime_state_overwrite'])

    def test_constraint_activation_change_is_detected(self):
        self.session.data.eq_active[0] = not self.session.data.eq_active[0]
        self.session.step()
        self.assertTrue(self.session.audit['runtime_state_overwrite'])

    def test_payload_radius_covers_rotated_box_vertices(self):
        state = self.session.snapshot()
        radius = state['payloads'][1]['radius_m']
        self.assertAlmostEqual(radius, np.linalg.norm([.025, .025, .024]))
        # This corner can face the tray boundary after a 3D rotation.
        self.assertGreater(radius, .042)

    def test_physics_alone_does_not_trigger_override(self):
        self.session.step(20)
        self.assertFalse(self.session.audit['runtime_state_overwrite'])
        self.assertTrue(np.all(np.isfinite(self.session.data.qpos)))


if __name__ == '__main__':
    unittest.main()
