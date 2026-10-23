import importlib
import json
from http.server import HTTPServer
from pathlib import Path
from types import SimpleNamespace
import threading
import unittest
from urllib.request import urlopen

import numpy as np

from rby1_tray.joint_overlay import build_joint_load_metadata, joint_load_metadata
from rby1_tray.viewer import _make_handler


mujoco = importlib.import_module("mujoco")


def toy_session():
    body_open = []
    body_close = []
    motors = []
    names = []
    for index in range(20):
        name = f"torso_{index}" if index < 6 else (
            f"right_arm_{index - 6}" if index < 13 else f"left_arm_{index - 13}"
        )
        names.append(name)
        body_open.append(
            f'<body name="link_{index}" pos="0.03 0 0">'
            f'<joint name="{name}" type="hinge" axis="0 0 1" pos="0.01 0.02 0"/>'
            '<geom type="sphere" size="0.005" mass="0.1"/>'
        )
        body_close.append("</body>")
        motors.append(
            f'<motor name="act_{index}" joint="{name}" gear="1" forcerange="-{index + 2} {index + 2}" ctrllimited="false"/>'
        )
    xml = (
        '<mujoco><option gravity="0 0 0"/><worldbody>'
        + "".join(body_open)
        + "".join(reversed(body_close))
        + '</worldbody><actuator>'
        + "".join(motors)
        + '</actuator></mujoco>'
    )
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    data.qpos[:] = np.linspace(0.01, 0.2, 20)
    mujoco.mj_forward(model, data)
    joint_ids = np.array([model.joint(name).id for name in names])
    actuator_ids = np.array([model.actuator(f"act_{index}").id for index in range(20)])
    robot = SimpleNamespace(
        names=names,
        jids=joint_ids,
        act=actuator_ids,
        force_limit=np.arange(2, 22, dtype=float),
    )
    return SimpleNamespace(model=model, data=data, robot=robot)


class JointOverlayTests(unittest.TestCase):
    def test_toy_hinge_metadata_anchors_follow_nonzero_pose(self):
        session = toy_session()
        metadata = build_joint_load_metadata(session)
        self.assertTrue(metadata["available"])
        self.assertEqual(metadata["joint_names_order"], session.robot.names)
        self.assertEqual(len(metadata["joints"]), 20)
        for item, joint_id, actuator_id in zip(
            metadata["joints"], session.robot.jids, session.robot.act
        ):
            body_id = int(session.model.jnt_bodyid[joint_id])
            rotation = session.data.xmat[body_id].reshape(3, 3)
            anchor_world = session.data.xpos[body_id] + rotation @ item["anchor_body_m"]
            np.testing.assert_allclose(anchor_world, session.data.xanchor[joint_id], atol=1e-12)
            expected_limit = np.max(np.abs(session.model.actuator_forcerange[actuator_id]))
            self.assertEqual(item["limit_nm"], expected_limit)
            self.assertEqual(item["part_id"], session.model.body(body_id).name)

    def test_invalid_actuator_contract_and_name_order_fail_closed(self):
        for mutation in (
            "gear",
            "limit",
            "name",
            "transmission",
            "unlimited",
            "asymmetric_range",
        ):
            session = toy_session()
            if mutation == "gear":
                session.model.actuator_gear[session.robot.act[3], 0] = 2
            elif mutation == "limit":
                session.robot.force_limit[4] += 1
            elif mutation == "name":
                session.robot.names[5], session.robot.names[6] = (
                    session.robot.names[6], session.robot.names[5]
                )
            elif mutation == "transmission":
                session.model.actuator_trntype[session.robot.act[7]] = int(
                    mujoco.mjtTrn.mjTRN_SITE
                )
            elif mutation == "unlimited":
                session.model.actuator_forcelimited[session.robot.act[8]] = 0
            else:
                session.model.actuator_forcerange[session.robot.act[9]] = [-10, 11]
            with self.assertRaises(ValueError):
                build_joint_load_metadata(session)
            unavailable = joint_load_metadata(session)
            self.assertFalse(unavailable["available"])
            self.assertIn("검증하지 못했습니다", unavailable["reason"])

    def test_actual_rb_y1_order_and_limits_match_recorded_load_evidence(self):
        from rby1_tray.planning import Robot
        from rby1_tray.scene import build_scene, load_config

        root = Path(__file__).parents[1]
        model, _ = build_scene(load_config(root / "configs/mobile-native.json"))
        metadata = build_joint_load_metadata(SimpleNamespace(model=model, robot=Robot(model)))
        evidence = json.loads(
            (root / "evidence/mobile-native/load-view.json").read_text(encoding="utf-8")
        )
        self.assertEqual(metadata["joint_names_order"], evidence["joint_names"])
        self.assertEqual(
            [joint["limit_nm"] for joint in metadata["joints"]],
            evidence["model_torque_limits_nm"],
        )
        self.assertEqual(len({joint["part_id"] for joint in metadata["joints"]}), 20)
        self.assertTrue(all(len(joint["anchor_body_m"]) == 3 for joint in metadata["joints"]))

    def test_structural_scene_api_adds_cached_metadata_without_mutating_scene_info(self):
        scene_info = {"enabled": True, "configured": True, "parts": []}
        structural_scene = SimpleNamespace(info=lambda: scene_info)
        session = SimpleNamespace(snapshot=lambda: {})
        joint_load = {"available": True, "scope": "test", "joints": []}
        server = HTTPServer(
            ("127.0.0.1", 0),
            _make_handler(session, structural_scene=structural_scene, joint_load=joint_load),
        )
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            with urlopen(
                f"http://127.0.0.1:{server.server_port}/api/structural-scene", timeout=3
            ) as response:
                payload = json.load(response)
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)
        self.assertEqual(payload["joint_load"], joint_load)
        self.assertNotIn("joint_load", scene_info)


if __name__ == "__main__":
    unittest.main()
