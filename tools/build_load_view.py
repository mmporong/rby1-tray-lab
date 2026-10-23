#!/usr/bin/env python3
"""Render joint-load colors during a repeat matched to recorded physics states."""
from __future__ import annotations
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
for location in (ROOT, ROOT / 'src'):
    if str(location) not in sys.path:
        sys.path.insert(0, str(location))

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from rby1_tray.cli import rendering_environment
from rby1_tray.playback import StagedOutput, nearest_step_count, sha256_file
from tools.build_playback import _VideoEncoder, _verify_video


def load_colors(fractions):
    values = np.asarray(fractions, dtype=float)
    if not np.all(np.isfinite(values)) or np.any(values < 0):
        raise ValueError('load fractions must be finite and nonnegative')
    values = np.clip(values, 0, 1)
    blue, yellow, red = np.array([.12, .42, .96]), np.array([1., .82, .10]), np.array([.95, .12, .10])
    colors = np.where((values <= .5)[..., None],
                      blue + (yellow - blue) * (values * 2)[..., None],
                      yellow + (red - yellow) * ((values - .5) * 2)[..., None])
    return np.concatenate((colors, np.ones(values.shape + (1,))), axis=-1)


def geom_joint_mapping(model, robot):
    """Inherit the closest actuated ancestor joint for each rendered link."""
    joint_bodies = {int(model.jnt_bodyid[j]): i for i, j in enumerate(robot.jids)}
    mapping = {}
    for geom in range(model.ngeom):
        body = int(model.geom_bodyid[geom])
        while body:
            if body in joint_bodies:
                mapping[geom] = joint_bodies[body]
                break
            body = int(model.body_parentid[body])
    return mapping


def physical_vector(state):
    robot = state['robot_state']
    result = [*robot['position_rad'], *robot['head_position_rad'],
              *robot['gripper_position_m'], *robot['base_pose_xy_yaw'],
              *state['tray']['pos_m'], *state['tray']['quat_wxyz']]
    for item in state['payloads']:
        result.extend(item['pos_m'])
        result.append(item['linear_speed_m_s'])
    result.extend(state['joint_state']['actuator_force'])
    return np.asarray(result)


def build(bundle: Path, destination: Path):
    rendering_environment()
    import mujoco
    from rby1_tray.mission import MobileSession
    from rby1_tray.scene import load_config
    config = load_config(bundle / 'config.json')
    with gzip.open(bundle / 'states.json.gz', 'rt') as f:
        reference = json.load(f)
    if not reference:
        raise ValueError('reference physics states are empty')
    staging = StagedOutput(destination)
    output = staging.create()
    session = MobileSession(config)
    encoder = None
    maximum_difference = 0.0
    try:
        if not np.allclose(session.model.actuator_gear[session.robot.act, 0], 1):
            raise ValueError('load view requires unit-gear revolute joint actuators')
        encoder = _VideoEncoder(output / 'load.mp4', shutil.which('ffmpeg') or 'ffmpeg')
        mapping = geom_joint_mapping(session.model, session.robot)
        model_colors = session.model.geom_rgba.copy()
        font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 16)
        for index, row in enumerate(reference):
            steps = nearest_step_count(float(session.data.time), row['actual_time_s'], config['physics_dt_s'])
            session.step(steps)
            actual = session.snapshot()
            error = float(np.max(np.abs(physical_vector(actual) - physical_vector(row['state']))))
            maximum_difference = max(maximum_difference, error)
            if error > 1e-9 or actual['phase'] != row['state']['phase']:
                raise RuntimeError(f'load replay diverged at frame {index}: {error}')
            # Reuse the validated overview composition. Only MjvScene colors
            # change; model materials, masses, forces and physics state do not.
            session.render(960, 720, 105, -20, 'overview')
            fractions = np.abs(session.data.actuator_force[session.robot.act]) / session.robot.force_limit
            colors = load_colors(fractions)
            for geom in session.renderer.scene.geoms[:session.renderer.scene.ngeom]:
                if geom.objtype == mujoco.mjtObj.mjOBJ_GEOM and geom.objid in mapping:
                    geom.rgba[:] = colors[mapping[geom.objid]]
                    geom.matid = -1
                    geom.texid = -1
            pixels = session.renderer.render().copy()
            image = Image.fromarray(pixels)
            draw = ImageDraw.Draw(image)
            draw.rectangle((0, 0, 960, 58), fill='#101820')
            draw.text((16, 9), 'JOINT TORQUE / MODEL LIMIT   |   not structural stress', font=font, fill='white')
            draw.text((16, 32), f"{actual['time_s']:.3f} s   |   {actual['phase']}   |   max {100 * max(fractions):.1f}%", font=font, fill='#bdcbd4')
            draw.rectangle((0, 680, 960, 720), fill='#101820')
            for x, color in enumerate(load_colors(np.linspace(0, 1, 300))):
                draw.line((20+x, 690, 20+x, 704), fill=tuple(np.rint(color[:3]*255).astype(int)))
            draw.text((340, 687), '0% blue   /   50% yellow   /   100% red', font=font, fill='white')
            encoder.write(np.asarray(image))
            if index % 150 == 0:
                print(json.dumps({'frame':index, 'time_s':actual['time_s'], 'phase':actual['phase']}), flush=True)
        encoder.finish()
        if not np.array_equal(model_colors, session.model.geom_rgba):
            raise RuntimeError('visualization changed physical model materials')
        _verify_video(output / 'load.mp4', shutil.which('ffprobe') or 'ffprobe', len(reference))
        metadata = {
            'physics_source_commit': json.loads((bundle/'manifest.json').read_text())['source_commit'],
            'visualization_script': 'tools/build_load_view.py',
            'visualization_script_sha256': sha256_file(Path(__file__)),
            'reference_states_sha256': sha256_file(bundle/'states.json.gz'),
            'frame_count': len(reference), 'fps':60, 'base_speed':8,
            'maximum_matched_physical_state_difference': maximum_difference,
            'state_comparison_tolerance':1e-9,
            'compared_fields':'20 joint positions/torques, head, grippers, base xy/yaw, tray pose, payload positions/speeds; payload orientations absent from reference',
            'model_materials_unchanged':True,
            'joint_names':session.robot.names,
            'model_torque_limits_nm':session.robot.force_limit.tolist(),
            'encoding':'unit-gear revolute actuator force in Nm; link inherits closest actuator ancestor',
            'scope':'rigid-body joint load visualization; not FEM stress, deformation, fatigue or hardware safety rating',
            'video_sha256':sha256_file(output/'load.mp4'),
        }
        (output/'load-view.json').write_text(json.dumps(metadata,indent=2)+'\n')
        staging.commit()
        return metadata
    finally:
        if encoder is not None:
            encoder.terminate()
        session.close()
        staging.cleanup()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle',type=Path,default=ROOT/'evidence/playback-native')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    print(json.dumps(build(args.bundle,args.output),indent=2))

if __name__=='__main__':
    main()
