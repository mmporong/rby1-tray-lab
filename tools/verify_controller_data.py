"""Validate saved sensor/command alignment and replay RGB-D perception offline."""
import argparse
import gzip
import json
from pathlib import Path

import numpy as np

from rby1_tray.perception import TrayCAD, estimate_tray_pose, validate_calibration


def finite_vector(value, size, label):
    if isinstance(value, dict):
        value = list(value.values())
    array = np.asarray(value, dtype=float)
    if array.shape != (size,) or not np.all(np.isfinite(array)):
        raise ValueError(f'Invalid {label}: expected {size} finite values')


def verify(run_directory, require_complete=True):
    run = Path(run_directory)
    data = run / 'controller-data'
    config = json.loads((run / 'config.json').read_text())
    schema = json.loads((data / 'schema.json').read_text())
    if schema['schema_version'] != 1:
        raise ValueError('Unknown controller data schema')
    order = schema['array_order']
    if len(set(order['upper_body_joint_position_and_velocity'])) != 20 or order['head_joint_position_and_velocity'] != ['head_0', 'head_1']:
        raise ValueError('Joint identity/order is missing')
    joint_low = np.asarray(schema['limits']['upper_body_joint_lower_rad'])
    joint_high = np.asarray(schema['limits']['upper_body_joint_upper_rad'])
    head_low = np.asarray(schema['limits']['head_joint_lower_rad'])
    head_high = np.asarray(schema['limits']['head_joint_upper_rad'])
    grip_low, grip_high = schema['limits']['gripper_position_m']
    index = json.loads((data / 'frames.json').read_text())
    seen = set()
    head_estimates = []
    for row in index:
        relative = Path(row['path'])
        if relative.is_absolute() or '..' in relative.parts:
            raise ValueError('Unsafe frame path')
        with np.load(data / relative, allow_pickle=False) as frame:
            rgb, depth = frame['rgb'], frame['depth_m']
            if rgb.dtype != np.uint8 or rgb.shape != (*depth.shape, 3):
                raise ValueError('RGB/depth shape or dtype mismatch')
            validate_calibration(frame['K'], frame['T_world_optical'])
            if not np.isclose(float(frame['timestamp_sim_s']), row['timestamp_sim_s']):
                raise ValueError('Frame index timestamp mismatch')
            metadata = json.loads(str(frame['metadata_json']))
            if metadata['camera_name'] != row['camera_name']:
                raise ValueError('Camera identity mismatch')
            if metadata.get('ground_truth_extrinsics_in_controller_packet') is not False:
                raise ValueError('Controller calibration provenance is missing')
            if 'model_calibration' in metadata:
                raise ValueError('Renderer ground-truth calibration leaked into controller packet')
            profile = next(p for p in config['sensors'].values()
                           if p['camera_name'] == row['camera_name'])
            if metadata['sensor_profile'] != profile['model'] or metadata['clock_domain'] != 'simulation_time':
                raise ValueError('Sensor model or clock provenance differs')
            if metadata.get('raw_channels_role') != 'evaluation_only_ideal_render':
                raise ValueError('Ideal raw renderer channels need an explicit evaluation-only role')
            valid = depth[np.isfinite(depth)]
            if not valid.size or valid.min() < profile['range_m'][0] or valid.max() > profile['range_m'][1]:
                raise ValueError('Depth operating range violation')
            if row['camera_name'] == 'head_rgbd':
                estimate = estimate_tray_pose(
                    rgb, depth, frame['K'], frame['T_world_optical'],
                    cad=TrayCAD(half_extents_m=tuple(config['tray_half_extents_m']),
                                grasp_height_m=config['grasp']['tcp_height_m']),
                )
                head_estimates.append(estimate.to_dict())
            seen.add(row['camera_name'])
    if require_complete and seen != {'head_rgbd', 'left_wrist_rgbd', 'right_wrist_rgbd'}:
        raise ValueError(f'Incomplete camera observations: {sorted(seen)}')
    commands = data / 'commands.jsonl'
    opener = open if commands.exists() else gzip.open
    commands = commands if commands.exists() else commands.with_suffix('.jsonl.gz')
    previous_time = None
    count = 0
    with opener(commands, 'rt') as stream:
        for line in stream:
            row = json.loads(line)
            now = float(row['time_s'])
            if not np.isfinite(now) or (previous_time is not None and not np.isclose(
                    now - previous_time, config['control_dt_s'], rtol=0, atol=1e-8)):
                raise ValueError('Control timestamps are nonfinite or have a missing/duplicated tick')
            observation = row['observation']
            if not np.isclose(now, observation['timestamp_sim_s'], rtol=0, atol=1e-9):
                raise ValueError('Command and encoder timestamps differ')
            if row['controller_inputs']['simulator_object_pose_used'] is not False or observation['ground_truth_body_pose_or_contact_included'] is not False:
                raise ValueError('Controller ground-truth exclusion declaration violated')
            for key, size in [('upper_body_position_reference_rad', 20),
                              ('upper_body_velocity_reference_rad_s', 20),
                              ('head_position_reference_rad', 2),
                              ('wheel_velocity_reference_rad_s', 4)]:
                finite_vector(row[key], size, key)
            grip_reference = np.asarray(row['gripper_position_reference_m'], dtype=float)
            if grip_reference.shape != () or not np.isfinite(grip_reference) or not grip_low <= grip_reference <= grip_high:
                raise ValueError(f'Invalid gripper reference at {now}')
            for key, size in [('upper_body_joint_position_rad', 20),
                              ('upper_body_joint_velocity_rad_s', 20),
                              ('head_joint_position_rad', 2),
                              ('head_joint_velocity_rad_s', 2),
                              ('wheel_encoder_angle_rad', 4),
                              ('wheel_encoder_velocity_rad_s', 4),
                              ('wheel_odometry_pose_xy_yaw', 3)]:
                finite_vector(observation[key], size, key)
            reference = np.asarray(row['upper_body_position_reference_rad'])
            if np.any(reference < joint_low - 1e-8) or np.any(reference > joint_high + 1e-8):
                raise ValueError(f'Joint reference is outside the declared planning limits at {now}')
            head_reference = np.asarray(row['head_position_reference_rad'])
            if np.any(head_reference < head_low - 1e-8) or np.any(head_reference > head_high + 1e-8):
                raise ValueError(f'Head reference is outside the declared joint limits at {now}')
            wheel_reference = row['wheel_velocity_reference_rad_s']
            if set(wheel_reference) != set(order['wheel_command']):
                raise ValueError('Wheel command names differ from schema')
            for side in ('left', 'right'):
                for key in ('force_n', 'torque_nm'):
                    finite_vector(observation['wrist_wrench_local'][side][key], 3, key)
            for side in ('l', 'r'):
                fingers = observation['gripper_position_m'][side]
                finite_vector(fingers, 2, f'{side} finger encoders')
                # MuJoCo soft joint limits permit a small physical deflection.
                # This is an integrity tolerance, not a hardware accuracy claim.
                if np.any(np.asarray(fingers) < grip_low - 0.002) or np.any(np.asarray(fingers) > grip_high + 0.002):
                    raise ValueError(f'Finger encoders exceed model range plus 2 mm at {now}')
            previous_time = now
            count += 1
    if not count:
        raise ValueError('No controller packets')
    result = {'commands': count, 'cameras': sorted(seen), 'head_estimates': head_estimates,
              'control_dt_s': config['control_dt_s'], 'status': 'PASS',
              'scope': 'saved packet integrity and offline perception replay; not hardware validation'}
    print(json.dumps(result, ensure_ascii=False, allow_nan=False))
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_directory')
    parser.add_argument('--allow-incomplete', action='store_true')
    args = parser.parse_args()
    verify(args.run_directory, require_complete=not args.allow_incomplete)
