"""Pure helper tests; no renderer or MuJoCo installation required."""
import unittest
import numpy as np

from rby1_tray.pickup_timing import blend_seed, make_schedule, segmented_progress


class PickupTimingTests(unittest.TestCase):
    def setUp(self):
        self.path = np.zeros((3, 20), dtype=float)
        self.path[1, 0] = 0.7
        self.path[2, 0] = 1.9
        self.schedule = make_schedule(self.path, 10.0, 2.454371637500114, 0.70)

    def test_alpha_endpoints_and_interpolation(self):
        a, b = np.array([1., 2.]), np.array([5., 6.])
        np.testing.assert_allclose(blend_seed(a, b, 0), a)
        np.testing.assert_allclose(blend_seed(a, b, 1), b)
        np.testing.assert_allclose(blend_seed(a, b, 0.75), [4., 5.])
        for bad in (-0.1, 1.1, float('nan')):
            with self.assertRaises(ValueError):
                blend_seed(a, b, bad)

    def test_schedule_and_first_segment_preservation(self):
        self.assertEqual(self.schedule.first_s, 5.0)
        self.assertLess(self.schedule.second_s, 5.0)
        for t in np.linspace(0, 5, 51):
            u = t / 10.0
            self.assertAlmostEqual(segmented_progress(t, self.schedule),
                                   2*(3*u*u-2*u*u*u), places=12)

    def test_monotonic_and_c1(self):
        schedule = self.schedule
        values = [segmented_progress(t, schedule)
                  for t in np.linspace(0, schedule.duration_s, 401)]
        self.assertAlmostEqual(values[0], 0)
        self.assertAlmostEqual(values[-1], 2)
        self.assertAlmostEqual(segmented_progress(5, schedule), 1)
        self.assertTrue(all(b + 1e-10 >= a for a, b in zip(values, values[1:])))
        eps = 1e-5
        left = (segmented_progress(5, schedule) - segmented_progress(5-eps, schedule))/eps
        right = (segmented_progress(5+eps, schedule) - segmented_progress(5, schedule))/eps
        self.assertLess(abs(left-right), 1e-4)

    def test_invalid_or_unsuitable_schedule(self):
        for override in ({"second_factor": 0}, {"baseline_second_jump_rad": 0},
                         {"minimum_second_s": 6}):
            params = {"original_total_s": 10, "baseline_second_jump_rad": 2.454,
                      "second_factor": 0.7, "minimum_second_s": 1.5}
            params.update(override)
            with self.assertRaises(ValueError):
                make_schedule(self.path, **params)
        with self.assertRaises(ValueError):
            make_schedule(self.path[:2], 10, 2.454, 0.7)


if __name__ == "__main__":
    unittest.main()
