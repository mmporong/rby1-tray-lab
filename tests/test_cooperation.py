import unittest

from rby1_tray.cooperation import CooperationConfig, CooperationSupervisor


class CooperationTests(unittest.TestCase):
    def test_asymmetric_static_load_is_baselined_and_pose_error_slows(self):
        supervisor = CooperationSupervisor(CooperationConfig())
        for index in range(40):
            result = supervisor.update(index * .02, .001, .005, [3, 1], active=True)
        self.assertEqual(result["speed_scale"], 1)
        for index in range(40, 60):
            result = supervisor.update(index * .02, .012, .005, [3, 1], active=True)
        self.assertLess(result["speed_scale"], .8)
        self.assertFalse(result["abort"])

    def test_force_change_slows_and_sustained_pose_fault_aborts(self):
        supervisor = CooperationSupervisor(CooperationConfig())
        supervisor.update(0, 0, 0, [2, 2], active=True)
        for index in range(1, 40):
            result = supervisor.update(index * .02, 0, 0, [4, 0], active=True)
        self.assertLess(result["speed_scale"], 1)
        self.assertTrue(result["abort"])
        supervisor = CooperationSupervisor(CooperationConfig())
        self.assertFalse(supervisor.update(0, .03, 0, [2, 2], active=True)["abort"])
        self.assertFalse(supervisor.update(.1, .001, 0, [2, 2], active=True)["abort"])
        self.assertFalse(supervisor.update(.2, .03, 0, [2, 2], active=True)["abort"])
        self.assertTrue(supervisor.update(.52, .03, 0, [2, 2], active=True)["abort"])

    def test_disabled_supervision_records_fault_without_intervention(self):
        supervisor = CooperationSupervisor(CooperationConfig(enabled=False))
        for index in range(30):
            result = supervisor.update(index * .02, .03, .2, [2, 0], active=True)
        self.assertEqual(result["speed_scale"], 1)
        self.assertFalse(result["abort"])
        self.assertGreater(supervisor.summary["max_relative_position_error_m"], .02)

    def test_invalid_observations_and_thresholds_are_rejected(self):
        with self.assertRaises(ValueError):
            CooperationConfig(position_warning_m=.1)
        supervisor = CooperationSupervisor(CooperationConfig())
        with self.assertRaises(ValueError):
            supervisor.update(0, 0, 0, [float("nan"), 2], active=True)
        self.assertEqual(supervisor.summary["samples"], 0)


if __name__ == "__main__":
    unittest.main()
