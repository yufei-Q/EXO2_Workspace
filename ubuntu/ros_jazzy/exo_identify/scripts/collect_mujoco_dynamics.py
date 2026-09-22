#!/usr/bin/env python3

"""Collect noisy forward-dynamics identification data in MuJoCo.

The excitation CSV contains position and velocity commands, matching the
``JointState`` command used by the real-hardware experiment.  This collector
drives the generated MuJoCo model with a configurable virtual position/
velocity servo, advances it with ``mj_step``, and records the resulting state
and actuator torque.  Every sample written to disk comes from that simulated
forward integration.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import json
from pathlib import Path
import time

from common import (
    RESULTS_ROOT,
    default_urdf_path,
    load_config,
    require_csv_columns,
    save_processed,
)
from mujoco_model import convert_urdf_to_mjcf
import numpy as np


def trajectory_columns(joint_count: int):
    return tuple(
        ['t']
        + [f'q{joint + 1}' for joint in range(joint_count)]
        + [f'dq{joint + 1}' for joint in range(joint_count)]
        + [f'ddq{joint + 1}' for joint in range(joint_count)]
    )


def read_trajectory(path: Path, joint_count: int):
    """Read and validate a desired position/velocity trajectory CSV."""
    values = np.atleast_1d(np.genfromtxt(path, delimiter=',', names=True))
    require_csv_columns(values, trajectory_columns(joint_count), path)
    result = tuple(
        np.column_stack([
            np.asarray(values[f'{prefix}{joint + 1}'], dtype=float)
            for joint in range(joint_count)
        ])
        for prefix in ('q', 'dq', 'ddq')
    )
    time_values = np.asarray(values['t'], dtype=float)
    if time_values.size < 2 or not np.all(np.isfinite(time_values)):
        raise ValueError(f'{path} must contain at least two finite time samples')
    if np.any(np.diff(time_values) <= 0.0):
        raise ValueError(f'{path} time must be strictly increasing')
    if not all(np.all(np.isfinite(item)) for item in result):
        raise ValueError(f'{path} contains NaN or infinite trajectory values')
    return time_values, *result


def joint_addresses(mujoco, model, joint_count: int):
    """Return MuJoCo qpos and velocity addresses for joint1..jointN."""
    qpos = []
    dof = []
    for index in range(joint_count):
        joint_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, f'joint{index + 1}')
        if joint_id < 0:
            raise ValueError(f'MuJoCo model is missing joint{index + 1}')
        qpos.append(int(model.jnt_qposadr[joint_id]))
        dof.append(int(model.jnt_dofadr[joint_id]))
    return np.asarray(qpos, dtype=int), np.asarray(dof, dtype=int)


def _joint_values(values, joint_count: int, name: str, minimum=0.0):
    """Validate a scalar or one-value-per-joint simulation setting."""
    array = np.asarray(values, dtype=float).reshape(-1)
    if array.size == 1:
        array = np.full(joint_count, float(array[0]), dtype=float)
    if array.shape != (joint_count,):
        raise ValueError(f'{name} must contain {joint_count} values')
    if not np.all(np.isfinite(array)) or np.any(array < minimum):
        raise ValueError(
            f'{name} must contain finite values >= {minimum:g}')
    return array


def _simulation_settings(config: dict, joint_count: int):
    simulation = config.get('simulation', {})
    controller = simulation.get('controller', {})
    noise = simulation.get('noise', {})
    settings = {
        'position_kp': _joint_values(
            controller.get('position_kp', [1.0] * joint_count),
            joint_count, 'simulation.controller.position_kp'),
        'velocity_kd': _joint_values(
            controller.get('velocity_kd', [0.1] * joint_count),
            joint_count, 'simulation.controller.velocity_kd'),
        'torque_limits': _joint_values(
            controller.get('torque_limits', [5.0] * joint_count),
            joint_count, 'simulation.controller.torque_limits', minimum=1e-9),
        'command_torque_std': _joint_values(
            noise.get('command_torque_std', [0.0] * joint_count),
            joint_count, 'simulation.noise.command_torque_std'),
        'external_torque_std': _joint_values(
            noise.get('external_torque_std', [0.0] * joint_count),
            joint_count, 'simulation.noise.external_torque_std'),
        'position_std': _joint_values(
            noise.get('position_std', [0.0] * joint_count),
            joint_count, 'simulation.noise.position_std'),
        'velocity_std': _joint_values(
            noise.get('velocity_std', [0.0] * joint_count),
            joint_count, 'simulation.noise.velocity_std'),
        'acceleration_std': _joint_values(
            noise.get('acceleration_std', [0.0] * joint_count),
            joint_count, 'simulation.noise.acceleration_std'),
        'torque_measurement_std': _joint_values(
            noise.get('torque_measurement_std', [0.0] * joint_count),
            joint_count, 'simulation.noise.torque_measurement_std'),
    }
    return settings


@contextmanager
def live_viewer(mjcf_path: Path, enabled: bool):
    """Open a passive MuJoCo viewer for the active forward simulation."""
    if not enabled:
        yield None
        return
    try:
        import mujoco
        import mujoco.viewer
    except ImportError as error:
        raise RuntimeError(
            'MuJoCo and its viewer are required for --viewer; '
            'install requirements-mujoco.txt') from error

    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    data = mujoco.MjData(model)
    with mujoco.viewer.launch_passive(model, data) as active_viewer:
        yield {
            'mujoco': mujoco,
            'model': model,
            'data': data,
            'viewer': active_viewer,
            'camera_configured': False,
        }


def update_live_viewer(viewer_state, source_data, elapsed, speed):
    """Copy the current forward-simulation state into the live viewer."""
    if viewer_state is None:
        return
    active_viewer = viewer_state['viewer']
    if not active_viewer.is_running():
        return
    mujoco = viewer_state['mujoco']
    model = viewer_state['model']
    data = viewer_state['data']
    data.qpos[:] = source_data.qpos
    data.qvel[:] = source_data.qvel
    data.qacc[:] = source_data.qacc
    mujoco.mj_forward(model, data)
    if not viewer_state['camera_configured']:
        body_positions = np.asarray(data.xpos[1:], dtype=float)
        if body_positions.size:
            active_viewer.cam.lookat[:] = 0.5 * (
                np.min(body_positions, axis=0) + np.max(body_positions, axis=0))
            active_viewer.cam.distance = max(
                0.75, 2.2 * float(np.max(np.ptp(body_positions, axis=0))))
            active_viewer.cam.azimuth = 135.0
            active_viewer.cam.elevation = -18.0
        viewer_state['camera_configured'] = True
    active_viewer.sync()
    target_time = viewer_state['wall_start'] + elapsed / speed
    remaining = target_time - time.perf_counter()
    if remaining > 0.0:
        time.sleep(remaining)


def _controller_torque(
        mujoco, model, data, qpos_address, dof_address,
        q_command, dq_command, settings, rng):
    """Compute a virtual position/velocity-drive command."""
    mujoco.mj_forward(model, data)
    q_actual = np.asarray(data.qpos[qpos_address], dtype=float)
    dq_actual = np.asarray(data.qvel[dof_address], dtype=float)
    torque = (
        settings['position_kp'] * (q_command - q_actual)
        + settings['velocity_kd'] * (dq_command - dq_actual)
    )
    torque += rng.normal(0.0, settings['command_torque_std'])
    return np.clip(
        torque,
        -settings['torque_limits'],
        settings['torque_limits'],
    )


def _measured_sample(
        data, qpos_address, dof_address, settings, rng, initial=False):
    """Read simulated sensors and add configurable measurement noise."""
    q = np.asarray(data.qpos[qpos_address], dtype=float).copy()
    dq = np.asarray(data.qvel[dof_address], dtype=float).copy()
    ddq = np.asarray(data.qacc[dof_address], dtype=float).copy()
    torque = np.asarray(data.qfrc_actuator[dof_address], dtype=float).copy()
    if not initial:
        q += rng.normal(0.0, settings['position_std'])
        dq += rng.normal(0.0, settings['velocity_std'])
        ddq += rng.normal(0.0, settings['acceleration_std'])
        torque += rng.normal(0.0, settings['torque_measurement_std'])
    return q, dq, ddq, torque


def collect_forward_dataset(
        mjcf_path: Path,
        time_values: np.ndarray,
        q_command: np.ndarray,
        dq_command: np.ndarray,
        output_path: Path,
        settings: dict,
        seed: int = 0,
        viewer_state=None,
        realtime_speed: float = 1.0):
    """Run MuJoCo forward dynamics and save noisy measured joint data."""
    try:
        import mujoco
    except ImportError as error:
        raise RuntimeError(
            'MuJoCo Python is required; install requirements-mujoco.txt') from error

    if realtime_speed <= 0.0 or not np.isfinite(realtime_speed):
        raise ValueError('realtime_speed must be finite and positive')
    if q_command.shape != dq_command.shape:
        raise ValueError('q_command and dq_command must have the same shape')

    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    joint_count = q_command.shape[1]
    if model.nq != joint_count or model.nv != joint_count or model.nu != joint_count:
        raise ValueError(
            f'MuJoCo dimensions (nq={model.nq}, nv={model.nv}, nu={model.nu}) '
            f'do not match trajectory joint count {joint_count}')
    qpos_address, dof_address = joint_addresses(
        mujoco, model, joint_count)
    model_dt = float(model.opt.timestep)
    sample_dt = np.diff(time_values)
    substeps = np.rint(sample_dt / model_dt).astype(int)
    if np.any(substeps < 1) or np.any(
            np.abs(substeps * model_dt - sample_dt) > 1e-7):
        raise ValueError(
            'trajectory sample intervals must be integer multiples of '
            f'MuJoCo timestep ({model_dt:g} s)')

    rng = np.random.default_rng(seed)
    data = mujoco.MjData(model)
    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    data.qacc[:] = 0.0
    data.qpos[qpos_address] = q_command[0]
    data.qvel[dof_address] = dq_command[0]
    data.ctrl[:] = 0.0
    data.qfrc_applied[:] = 0.0
    mujoco.mj_forward(model, data)
    if viewer_state is not None:
        viewer_state['wall_start'] = time.perf_counter()

    measured = np.zeros((time_values.size, 1 + 4 * joint_count), dtype=float)
    q0, dq0, ddq0, tau0 = _measured_sample(
        data, qpos_address, dof_address, settings, rng, initial=True)
    measured[0] = np.r_[time_values[0], q0, dq0, ddq0, tau0]
    for index in range(time_values.size - 1):
        for substep in range(int(substeps[index])):
            alpha = float(substep + 1) / float(substeps[index])
            q_target = (
                (1.0 - alpha) * q_command[index]
                + alpha * q_command[index + 1]
            )
            dq_target = (
                (1.0 - alpha) * dq_command[index]
                + alpha * dq_command[index + 1]
            )
            command_torque = _controller_torque(
                mujoco, model, data, qpos_address, dof_address,
                q_target, dq_target, settings, rng)
            data.ctrl[:] = command_torque
            data.qfrc_applied[:] = 0.0
            data.qfrc_applied[dof_address] = rng.normal(
                0.0, settings['external_torque_std'])
            mujoco.mj_step(model, data)
        q, dq, ddq, torque = _measured_sample(
            data, qpos_address, dof_address, settings, rng)
        measured[index + 1] = np.r_[time_values[index + 1], q, dq, ddq, torque]
        if viewer_state is not None:
            update_live_viewer(
                viewer_state, data, float(time_values[index + 1]),
                realtime_speed)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    save_processed(output_path, measured)
    return measured


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(
        description='Collect noisy forward-dynamics data in MuJoCo')
    parser.add_argument('--config', type=Path)
    parser.add_argument('--urdf', type=Path)
    parser.add_argument(
        '--trajectory-file', type=Path,
        default=RESULTS_ROOT / 'seven_dof/excitation/excitation_id.csv')
    parser.add_argument(
        '--validation-trajectory-file', type=Path,
        default=RESULTS_ROOT / 'seven_dof/excitation/excitation_validation.csv')
    parser.add_argument(
        '--output-dir', type=Path,
        default=RESULTS_ROOT / 'seven_dof/simulation_data',
        help='directory for simulated measured CSV data')
    parser.add_argument(
        '--model-dir', type=Path,
        default=RESULTS_ROOT / 'seven_dof/simulation_model',
        help='directory for the MJCF model used by this simulation')
    parser.add_argument(
        '--torque-noise-std', type=float, default=None,
        help=(
            'override per-joint torque measurement noise in N.m; '
            'default comes from simulation.noise'))
    parser.add_argument(
        '--viewer', action='store_true',
        help='show the current forward-simulation state in a live MuJoCo viewer')
    parser.add_argument(
        '--viewer-speed', type=float, default=1.0,
        help='real-time display speed multiplier when --viewer is enabled')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_arguments(argv)
    config = load_config(args.config)
    urdf_path = (args.urdf or default_urdf_path()).expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    model_dir = args.model_dir.expanduser().resolve()
    model_dir.mkdir(parents=True, exist_ok=True)
    joint_count = len(config['excitation_joint_lower'])
    trajectory_path = args.trajectory_file.expanduser().resolve()
    validation_trajectory_path = (
        args.validation_trajectory_file.expanduser().resolve())
    time_values, q, dq, _ = read_trajectory(trajectory_path, joint_count)
    validation_time, validation_q, validation_dq, _ = read_trajectory(
        validation_trajectory_path, joint_count)

    model_config = config.get('simulation', {})
    settings = _simulation_settings(config, joint_count)
    if args.torque_noise_std is not None:
        if args.torque_noise_std < 0.0 or not np.isfinite(args.torque_noise_std):
            raise ValueError('torque_noise_std must be finite and non-negative')
        settings['torque_measurement_std'] = np.full(
            joint_count, float(args.torque_noise_std), dtype=float)

    mjcf_path = model_dir / 'exo7_sim.xml'
    id_path = output_dir / 'dynamics_id_sim.csv'
    validation_path = output_dir / 'dynamics_validation_sim.csv'

    convert_urdf_to_mjcf(
        urdf_path,
        mjcf_path,
        gravity=config['gravity'],
        timestep=float(model_config.get('timestep', 0.002)),
        base_position=model_config.get('base_position', (0.0, 0.0, 0.0)),
        base_orientation_rpy=model_config.get(
            'base_orientation_rpy', (0.0, 0.0, 0.0)),
        joint_damping=model_config.get('joint_damping'),
        joint_armature=model_config.get('joint_armature'),
        joint_friction_loss=model_config.get('joint_friction_loss'),
    )
    with live_viewer(mjcf_path, args.viewer) as viewer_state:
        if viewer_state is not None:
            print(
                'Live MuJoCo forward simulation enabled: '
                f'{args.viewer_speed:g}x real time')
        collect_forward_dataset(
            mjcf_path, time_values, q, dq, id_path, settings,
            seed=171, viewer_state=viewer_state,
            realtime_speed=args.viewer_speed)
        if viewer_state is not None:
            print('Live MuJoCo viewer: displaying validation trajectory')
        collect_forward_dataset(
            mjcf_path, validation_time, validation_q, validation_dq,
            validation_path, settings, seed=191,
            viewer_state=viewer_state,
            realtime_speed=args.viewer_speed)

    metadata = {
        'simulation_model': str(mjcf_path),
        'trajectory_file': str(trajectory_path),
        'validation_trajectory_file': str(validation_trajectory_path),
        'base_position': model_config.get('base_position', [0.0, 0.0, 0.0]),
        'base_orientation_rpy': model_config.get(
            'base_orientation_rpy', [0.0, 0.0, 0.0]),
        'joint_damping': model_config.get('joint_damping'),
        'joint_armature': model_config.get('joint_armature'),
        'joint_friction_loss': model_config.get('joint_friction_loss'),
        'collection_method': (
            'MuJoCo forward dynamics: position/velocity commands are applied '
            'through a configurable virtual servo, mj_step advances the model, '
            'and qfrc_actuator is recorded as the simulated joint torque.'),
        'controller': {
            'position_kp': settings['position_kp'].tolist(),
            'velocity_kd': settings['velocity_kd'].tolist(),
            'torque_limits_Nm': settings['torque_limits'].tolist(),
        },
        'noise': {
            'command_torque_std_Nm': settings['command_torque_std'].tolist(),
            'external_torque_std_Nm': settings['external_torque_std'].tolist(),
            'position_std_rad': settings['position_std'].tolist(),
            'velocity_std_rad_s': settings['velocity_std'].tolist(),
            'acceleration_std_rad_s2': settings[
                'acceleration_std'].tolist(),
            'torque_measurement_std_Nm': settings[
                'torque_measurement_std'].tolist(),
        },
        'data_definition': (
            'Measured q, dq, qacc and actuator torque from a MuJoCo '
            'forward simulation with configurable controller and noise.'),
    }
    with (output_dir / 'simulation_metadata_sim.json').open(
            'w', encoding='utf-8') as stream:
        json.dump(metadata, stream, ensure_ascii=False, indent=2)
    print(f'Simulation collection outputs: {output_dir}')


if __name__ == '__main__':
    main()
