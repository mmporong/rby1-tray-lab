import math
import unittest

from rby1_tray.evaluation import Evaluator


def config():
    return {
        "target": {
            "center_xy_m": [0.5, 0.0],
            "inner_half_xy_m": [0.31, 0.26],
            "support_z_m": 1.0,
        },
        "tray_half_extents_m": [0.25, 0.20, 0.02],
        "initial_tray_bottom_z_m": 0.0,
        "expected_payload_names": ["ball"],
        "payload_max_height_m": 0.25,
        "thresholds": {
            "grasp_hold_s": 0.15,
            "settle_hold_s": 0.40,
            "withdraw_clearance_m": 0.08,
            "linear_speed_m_s": 0.03,
            "angular_speed_rad_s": 0.10,
            "lift_clearance_m": 0.03,
            "payload_margin_m": 0.005,
            "containment_tolerance_m": 0.005,
        },
    }


def snapshot(time_s, *, pos=(0.0, 0.0, 0.02), quat=(1.0, 0.0, 0.0, 0.0),
             pads=(False, False, False, False), opposed=(True, True),
             pad_forces=None, payload_local=(0.0, 0.0, 0.07),
             payload_supported=True, payload_speed=0.0, container=False,
             robot_tray_contact=None, clearance=(0.0, 0.0), audit=None,
             phase="ignored"):
    # Identity is sufficient for most sequences. Rotation-specific tests provide
    # world coordinates explicitly after applying the desired quaternion.
    payload_world = [pos[i] + payload_local[i] for i in range(3)]
    return {
        "time_s": time_s,
        "phase": phase,
        "tray": {
            "pos_m": list(pos),
            "quat_wxyz": list(quat),
            "linear_velocity_m_s": [0.0, 0.0, 0.0],
            "angular_velocity_rad_s": [0.0, 0.0, 0.0],
        },
        "payloads": [{
            "name": "ball",
            "pos_m": payload_world,
            "radius_m": 0.05,
            "supported_by_tray": payload_supported,
            "linear_speed_m_s": payload_speed,
        }],
        "contacts": {
            "left_fingers": list(pads[:2]),
            "right_fingers": list(pads[2:]),
            "left_opposed": opposed[0],
            "right_opposed": opposed[1],
            "robot_tray_contact": (
                any(pads) if robot_tray_contact is None else robot_tray_contact
            ),
            "container_bottom": container,
            "forbidden": [],
        },
        "grasp_force_n": {"left": 1.0, "right": 1.0},
        "pad_forces_n": {
            "left": list((pad_forces or tuple(1.0 if pad else 0.0 for pad in pads))[:2]),
            "right": list((pad_forces or tuple(1.0 if pad else 0.0 for pad in pads))[2:]),
        },
        "hand_clearance_m": {"left": clearance[0], "right": clearance[1]},
        "joint_limit_violation": False,
        "audit": audit or {
            "free_objects": True,
            "object_attachment": False,
            "runtime_state_overwrite": False,
        },
    }


def establish_grasp(evaluator):
    evaluator.update(snapshot(0.00, pads=(True,) * 4))
    result = evaluator.update(snapshot(0.16, pads=(True,) * 4))
    assert result["milestones"]["grasped"]


class EvaluatorTests(unittest.TestCase):
    def test_complete_success_is_phase_independent(self):
        evaluator = Evaluator(config())
        establish_grasp(evaluator)
        evaluator.update(snapshot(0.30, pos=(0.0, 0.0, 0.20), pads=(True,) * 4,
                                  phase="release"))
        evaluator.update(snapshot(0.50, pos=(0.5, 0.0, 1.02), pads=(True,) * 4,
                                  container=True, phase="approach"))
        evaluator.update(snapshot(0.91, pos=(0.5, 0.0, 1.02), pads=(True,) * 4,
                                  container=True, phase="lift"))
        evaluator.update(snapshot(1.00, pos=(0.5, 0.0, 1.02), pads=(False,) * 4,
                                  container=True, clearance=(0.08, 0.12)))
        result = evaluator.update(snapshot(1.41, pos=(0.5, 0.0, 1.02), pads=(False,) * 4,
                                           container=True, clearance=(0.08, 0.12)))
        self.assertTrue(result["success"])
        self.assertTrue(all(result["milestones"].values()))

    def test_lift_without_all_four_pad_contacts_fails(self):
        evaluator = Evaluator(config())
        evaluator.update(snapshot(0.00, pads=(True, True, True, False)))
        result = evaluator.update(snapshot(0.20, pos=(0.0, 0.0, 0.20),
                                           pads=(True, True, True, False)))
        self.assertEqual(result["failure_reason"], "lift_without_secure_grasp")
        self.assertFalse(result["milestones"]["grasped"])

    def test_same_side_contacts_without_opposition_cannot_grasp(self):
        evaluator = Evaluator(config())
        evaluator.update(snapshot(0.00, pads=(True,) * 4, opposed=(False, True)))
        result = evaluator.update(snapshot(0.20, pos=(0.0, 0.0, 0.20),
                                           pads=(True,) * 4, opposed=(False, True)))
        self.assertEqual(result["failure_reason"], "lift_without_secure_grasp")
        self.assertFalse(result["checks"]["secure_grasp"])

    def test_zero_pad_force_cannot_grasp(self):
        evaluator = Evaluator(config())
        forces = (1.0, 1.0, 1.0, 0.0)
        evaluator.update(snapshot(0.00, pads=(True,) * 4, pad_forces=forces))
        result = evaluator.update(snapshot(0.20, pos=(0.0, 0.0, 0.20),
                                           pads=(True,) * 4, pad_forces=forces))
        self.assertEqual(result["failure_reason"], "lift_without_secure_grasp")
        self.assertFalse(result["checks"]["pad_force_sufficient"])

    def test_payload_loss_is_irreversible(self):
        evaluator = Evaluator(config())
        establish_grasp(evaluator)
        lost = snapshot(0.30, pos=(0.0, 0.0, 0.20), pads=(True,) * 4,
                        payload_local=(0.40, 0.0, 0.07))
        result = evaluator.update(lost)
        self.assertEqual(result["failure_reason"], "payload_lost")
        recovered = evaluator.update(snapshot(1.0, pos=(0.5, 0.0, 1.02),
                                               pads=(True,) * 4, container=True))
        self.assertEqual(recovered["failure_reason"], "payload_lost")
        self.assertFalse(recovered["success"])

    def test_contact_with_wrong_container_location_does_not_seat(self):
        evaluator = Evaluator(config())
        establish_grasp(evaluator)
        evaluator.update(snapshot(0.30, pos=(0.0, 0.0, 0.20), pads=(True,) * 4))
        evaluator.update(snapshot(0.50, pos=(1.0, 0.0, 1.02), pads=(True,) * 4,
                                  container=True))
        evaluator.update(snapshot(1.00, pos=(1.0, 0.0, 1.02), pads=(True,) * 4,
                                  container=True))
        result = evaluator.update(snapshot(1.10, pos=(1.0, 0.0, 1.02),
                                           pads=(False,) * 4, container=True))
        self.assertEqual(result["failure_reason"], "release_before_seat")

    def test_release_before_settle_hold_fails(self):
        evaluator = Evaluator(config())
        establish_grasp(evaluator)
        evaluator.update(snapshot(0.30, pos=(0.0, 0.0, 0.20), pads=(True,) * 4))
        evaluator.update(snapshot(0.50, pos=(0.5, 0.0, 1.02), pads=(True,) * 4,
                                  container=True))
        result = evaluator.update(snapshot(0.70, pos=(0.5, 0.0, 1.02),
                                           pads=(False,) * 4, container=True))
        self.assertEqual(result["failure_reason"], "release_before_seat")

    def test_release_without_withdrawal_is_not_success(self):
        evaluator = Evaluator(config())
        establish_grasp(evaluator)
        evaluator.update(snapshot(0.30, pos=(0.0, 0.0, 0.20), pads=(True,) * 4))
        evaluator.update(snapshot(0.50, pos=(0.5, 0.0, 1.02), pads=(True,) * 4,
                                  container=True))
        evaluator.update(snapshot(0.91, pos=(0.5, 0.0, 1.02), pads=(True,) * 4,
                                  container=True))
        result = evaluator.update(snapshot(1.00, pos=(0.5, 0.0, 1.02),
                                           pads=(False,) * 4, container=True,
                                           clearance=(0.02, 0.20)))
        self.assertTrue(result["milestones"]["released"])
        self.assertFalse(result["success"])
        self.assertFalse(result["terminal"])

    def test_partial_finger_release_is_not_a_release(self):
        evaluator = Evaluator(config())
        establish_grasp(evaluator)
        evaluator.update(snapshot(0.30, pos=(0.0, 0.0, 0.20), pads=(True,) * 4))
        evaluator.update(snapshot(0.50, pos=(0.5, 0.0, 1.02), pads=(True,) * 4,
                                  container=True))
        evaluator.update(snapshot(0.91, pos=(0.5, 0.0, 1.02), pads=(True,) * 4,
                                  container=True))
        result = evaluator.update(snapshot(1.00, pos=(0.5, 0.0, 1.02),
                                           pads=(True, False, False, False),
                                           container=True, clearance=(0.20, 0.20)))
        self.assertFalse(result["milestones"]["released"])
        self.assertFalse(result["success"])

    def test_palm_contact_prevents_whole_hand_release(self):
        evaluator = Evaluator(config())
        establish_grasp(evaluator)
        evaluator.update(snapshot(0.30, pos=(0.0, 0.0, 0.20), pads=(True,) * 4))
        evaluator.update(snapshot(0.50, pos=(0.5, 0.0, 1.02), pads=(True,) * 4,
                                  container=True))
        evaluator.update(snapshot(0.91, pos=(0.5, 0.0, 1.02), pads=(True,) * 4,
                                  container=True))
        result = evaluator.update(snapshot(
            1.00,
            pos=(0.5, 0.0, 1.02),
            pads=(False,) * 4,
            robot_tray_contact=True,
            container=True,
            clearance=(0.20, 0.20),
        ))
        self.assertFalse(result["checks"]["fully_released"])
        self.assertFalse(result["milestones"]["released"])

    def test_instant_withdrawal_does_not_succeed(self):
        evaluator = Evaluator(config())
        establish_grasp(evaluator)
        evaluator.update(snapshot(0.30, pos=(0.0, 0.0, 0.20), pads=(True,) * 4))
        evaluator.update(snapshot(0.50, pos=(0.5, 0.0, 1.02), pads=(True,) * 4,
                                  container=True))
        evaluator.update(snapshot(0.91, pos=(0.5, 0.0, 1.02), pads=(True,) * 4,
                                  container=True))
        result = evaluator.update(snapshot(1.00, pos=(0.5, 0.0, 1.02),
                                           pads=(False,) * 4, container=True,
                                           clearance=(0.20, 0.20)))
        self.assertTrue(result["milestones"]["released"])
        self.assertFalse(result["milestones"]["withdrawn"])

    def test_withdrawal_hold_resets_after_target_exit(self):
        evaluator = Evaluator(config())
        establish_grasp(evaluator)
        evaluator.update(snapshot(0.30, pos=(0.0, 0.0, 0.20), pads=(True,) * 4))
        evaluator.update(snapshot(0.50, pos=(0.5, 0.0, 1.02), pads=(True,) * 4,
                                  container=True))
        evaluator.update(snapshot(0.91, pos=(0.5, 0.0, 1.02), pads=(True,) * 4,
                                  container=True))
        evaluator.update(snapshot(1.00, pos=(0.5, 0.0, 1.02), pads=(False,) * 4,
                                  container=True, clearance=(0.20, 0.20)))
        evaluator.update(snapshot(1.30, pos=(0.8, 0.0, 1.02), pads=(False,) * 4,
                                  container=True, clearance=(0.20, 0.20)))
        evaluator.update(snapshot(1.40, pos=(0.5, 0.0, 1.02), pads=(False,) * 4,
                                  container=True, clearance=(0.20, 0.20)))
        result = evaluator.update(snapshot(1.51, pos=(0.5, 0.0, 1.02), pads=(False,) * 4,
                                           container=True, clearance=(0.20, 0.20)))
        self.assertFalse(result["success"])

    def test_missing_one_of_three_expected_payloads_fails(self):
        cfg = config()
        cfg["expected_payload_names"] = ["payload_0", "payload_1", "payload_2"]
        evaluator = Evaluator(cfg)
        snap = snapshot(0.0)
        snap["payloads"] = [
            {"name": "payload_0", "pos_m": [0.0, 0.0, 0.09], "radius_m": 0.05,
             "supported_by_tray": True, "linear_speed_m_s": 0.0},
            {"name": "payload_2", "pos_m": [0.0, 0.0, 0.09], "radius_m": 0.05,
             "supported_by_tray": True, "linear_speed_m_s": 0.0},
        ]
        result = evaluator.update(snap)
        self.assertEqual(result["failure_reason"], "payload_identity_mismatch")

    def test_contact_at_target_before_grasp_cannot_seat(self):
        evaluator = Evaluator(config())
        evaluator.update(snapshot(0.00, pos=(0.5, 0.0, 1.02), container=True))
        result = evaluator.update(snapshot(0.50, pos=(0.5, 0.0, 1.02), container=True))
        self.assertFalse(result["milestones"]["seated"])

    def test_payload_vertical_escape_fails(self):
        evaluator = Evaluator(config())
        result = evaluator.update(snapshot(0.0, pads=(True,) * 4,
                                           payload_local=(0.0, 0.0, 0.40)))
        self.assertEqual(result["failure_reason"], "payload_lost")

    def test_payload_center_below_floor_top_fails_even_when_sphere_overlaps(self):
        evaluator = Evaluator(config())
        result = evaluator.update(snapshot(0.0, payload_local=(0.0, 0.0, 0.0)))
        self.assertEqual(result["failure_reason"], "payload_lost")

    def test_airborne_payload_cannot_complete_seating(self):
        evaluator = Evaluator(config())
        establish_grasp(evaluator)
        evaluator.update(snapshot(0.30, pos=(0.0, 0.0, 0.20), pads=(True,) * 4))
        evaluator.update(snapshot(0.50, pos=(0.5, 0.0, 1.02), pads=(True,) * 4,
                                  container=True, payload_supported=False))
        result = evaluator.update(snapshot(1.00, pos=(0.5, 0.0, 1.02), pads=(True,) * 4,
                                           container=True, payload_supported=False))
        self.assertFalse(result["milestones"]["seated"])
        self.assertFalse(result["checks"]["payloads_supported"])

    def test_moving_payload_cannot_complete_seating(self):
        evaluator = Evaluator(config())
        establish_grasp(evaluator)
        evaluator.update(snapshot(0.30, pos=(0.0, 0.0, 0.20), pads=(True,) * 4))
        evaluator.update(snapshot(0.50, pos=(0.5, 0.0, 1.02), pads=(True,) * 4,
                                  container=True, payload_speed=0.20))
        result = evaluator.update(snapshot(1.00, pos=(0.5, 0.0, 1.02), pads=(True,) * 4,
                                           container=True, payload_speed=0.20))
        self.assertFalse(result["milestones"]["seated"])
        self.assertFalse(result["checks"]["payloads_settled"])

    def test_tilted_tray_on_support_cannot_complete_seating(self):
        evaluator = Evaluator(config())
        establish_grasp(evaluator)
        evaluator.update(snapshot(0.30, pos=(0.0, 0.0, 0.20), pads=(True,) * 4))
        angle = 0.06
        quat = (math.cos(angle / 2), math.sin(angle / 2), 0.0, 0.0)
        pos = (0.5, 0.0, 1.0 + 0.20 * math.sin(angle) + 0.02 * math.cos(angle))

        def tilted_snap(time_s):
            snap = snapshot(time_s, pos=pos, quat=quat, pads=(True,) * 4,
                            container=True)
            local_z = 0.07
            snap["payloads"][0]["pos_m"] = [
                pos[0],
                pos[1] - local_z * math.sin(angle),
                pos[2] + local_z * math.cos(angle),
            ]
            return snap

        evaluator.update(tilted_snap(0.50))
        result = evaluator.update(tilted_snap(1.00))
        self.assertTrue(result["checks"]["tray_on_support"])
        self.assertFalse(result["checks"]["tray_upright"])
        self.assertFalse(result["milestones"]["seated"])

    def test_attachment_audit_fails_and_latches(self):
        evaluator = Evaluator(config())
        bad_audit = {
            "free_objects": True,
            "object_attachment": True,
            "runtime_state_overwrite": False,
        }
        result = evaluator.update(snapshot(0.0, audit=bad_audit))
        self.assertEqual(result["failure_reason"], "object_attachment")
        self.assertEqual(evaluator.update(snapshot(1.0))["failure_reason"], "object_attachment")

    def test_nonfinite_and_nonmonotonic_time_fail(self):
        for invalid_time, expected in ((math.nan, "nonfinite_time"),
                                       (math.inf, "nonfinite_time")):
            with self.subTest(invalid_time=invalid_time):
                result = Evaluator(config()).update(snapshot(invalid_time))
                self.assertEqual(result["failure_reason"], expected)
        evaluator = Evaluator(config())
        evaluator.update(snapshot(1.0))
        result = evaluator.update(snapshot(0.5))
        self.assertEqual(result["failure_reason"], "nonmonotonic_time")
        self.assertEqual(result["failure_time_s"], 0.5)
        self.assertEqual(result["last_time_s"], 1.0)

    def test_failure_event_advances_last_time_and_update_count(self):
        evaluator = Evaluator(config())
        evaluator.update(snapshot(0.0))
        result = evaluator.update(snapshot(0.5, audit={
            "free_objects": True,
            "object_attachment": True,
            "runtime_state_overwrite": False,
        }))
        self.assertEqual(result["failure_time_s"], 0.5)
        self.assertEqual(result["last_time_s"], 0.5)
        self.assertEqual(result["updates"], 2)
        self.assertEqual(result["failure_event"]["reason"], "object_attachment")

    def test_finalize_latches_timeout_at_actual_deadline(self):
        evaluator = Evaluator(config())
        evaluator.update(snapshot(0.0))
        result = evaluator.finalize(5.0)
        self.assertTrue(result["terminal"])
        self.assertFalse(result["success"])
        self.assertEqual(result["failure_reason"], "episode_timeout")
        self.assertEqual(result["failure_time_s"], 5.0)
        self.assertEqual(result["last_time_s"], 5.0)
        self.assertEqual(result["updates"], 2)

    def test_finalize_accepts_same_time_as_last_snapshot(self):
        evaluator = Evaluator(config())
        evaluator.update(snapshot(5.0))
        result = evaluator.finalize(5.0)
        self.assertEqual(result["failure_reason"], "episode_timeout")
        self.assertEqual(result["failure_time_s"], 5.0)
        self.assertEqual(result["last_time_s"], 5.0)

    def test_finalize_preserves_existing_terminal_result(self):
        evaluator = Evaluator(config())
        failed = evaluator.update(snapshot(0.0, audit={
            "free_objects": False,
            "object_attachment": False,
            "runtime_state_overwrite": False,
        }))
        finalized = evaluator.finalize(10.0)
        self.assertEqual(finalized, failed)

    def test_payload_containment_uses_full_3d_orientation(self):
        evaluator = Evaluator(config())
        # +90 degrees about Y: local tray x maps toward world -z. A payload
        # centered inside local XY remains contained despite its world offset.
        quat = (math.sqrt(0.5), 0.0, math.sqrt(0.5), 0.0)
        snap = snapshot(0.0, quat=quat, pads=(True,) * 4)
        local = [0.10, 0.0, 0.07]
        # R_y(+90) * local = [z, y, -x].
        snap["payloads"][0]["pos_m"] = [local[2], local[1], 0.02 - local[0]]
        result = evaluator.update(snap)
        self.assertTrue(result["checks"]["payloads_contained"])

    def test_all_eight_tray_corners_must_fit_target_xy(self):
        evaluator = Evaluator(config())
        establish_grasp(evaluator)
        evaluator.update(snapshot(0.30, pos=(0.0, 0.0, 0.20), pads=(True,) * 4))
        # Rotating a 0.25 x 0.20 tray by 45 degrees gives a 0.318 m XY
        # half-span, larger than the 0.31 m target half-width.
        quat = (math.cos(math.pi / 8), 0.0, 0.0, math.sin(math.pi / 8))
        result = evaluator.update(snapshot(0.50, pos=(0.5, 0.0, 1.02), quat=quat,
                                           pads=(True,) * 4, container=True))
        self.assertFalse(result["checks"]["tray_in_target_xy"])

    def test_handle_envelope_outside_target_rejects_floor_only_false_pass(self):
        cfg = config()
        cfg["tray_containment_half_extents_m"] = [0.25, 0.20, 0.10]
        cfg["tray_containment_center_m"] = [0.08, 0.0, 0.08]
        evaluator = Evaluator(cfg)
        establish_grasp(evaluator)
        evaluator.update(snapshot(0.30, pos=(0.0, 0.0, 0.20), pads=(True,) * 4))
        # The 0.25 m floor fits x=[0.19, 0.81], while the handle/body
        # envelope reaches x=0.83 and must prevent seating.
        evaluator.update(snapshot(0.50, pos=(0.5, 0.0, 1.02), pads=(True,) * 4,
                                  container=True))
        result = evaluator.update(snapshot(1.00, pos=(0.5, 0.0, 1.02), pads=(True,) * 4,
                                           container=True))
        self.assertFalse(result["checks"]["tray_in_target_xy"])
        self.assertFalse(result["milestones"]["seated"])


if __name__ == "__main__":
    unittest.main()
