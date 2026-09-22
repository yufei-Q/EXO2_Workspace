#!/usr/bin/env python3

"""Validate the identified gravity/friction compensation in MuJoCo."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from collect_mujoco_dynamics import joint_addresses, read_trajectory
from common import (
    RESULTS_ROOT,
    build_estimator_model,
    default_urdf_path,
    load_base_set,
    load_config,
    torque_regressor,
)
from identify_parameters import metrics
import numpy as np


def identified_compensation(
        config, urdf_path, base_set_path, parameters_path, q, dq):
    """Evaluate the same gravity/friction model used by deployment."""
    model, data = build_estimator_model(config, urdf_path)
    columns, _ = load_base_set(base_set_path)
    with np.load(parameters_path) as identified:
        rigid_beta = np.asarray(
            identified['rigid_beta']
            if 'rigid_beta' in identified else identified['beta'][:len(columns)],
            dtype=float,
        )
        include_friction = bool(
            np.asarray(identified['include_friction']).item()
        ) if 'include_friction' in identified else False
        friction_beta = np.asarray(
            identified['friction_beta'], dtype=float
        ) if include_friction and 'friction_beta' in identified else np.zeros(
            2 * model.nv)
    if rigid_beta.shape != (len(columns),):
        raise ValueError('identified rigid parameters do not match base set')
    if friction_beta.shape != (2 * model.nv,):
        raise ValueError('identified friction parameters do not match the model')
    transition_velocity = np.asarray(
        config['friction']['transition_velocity_rad_s'], dtype=float)
    if transition_velocity.shape != (model.nv,) or np.any(transition_velocity <= 0.0):
        raise ValueError('friction transition velocities do not match the model')

    zero_velocity = np.zeros(model.nv)
    gravity = np.vstack([
        torque_regressor(model, data, position, zero_velocity, zero_velocity)
        [:, columns] @ rigid_beta
        for position in q
    ])
    if include_friction:
        viscous = friction_beta[:model.nv]
        coulomb = friction_beta[model.nv:]
        friction = viscous[None, :] * dq + coulomb[None, :] * np.tanh(
            dq / transition_velocity[None, :])
    else:
        friction = np.zeros_like(dq)
    return gravity, friction


def mujoco_compensation_reference(mjcf_path, q, dq):
    """Return MuJoCo gravity and passive-friction compensation terms."""
    try:
        import mujoco
    except ImportError as error:
        raise RuntimeError(
            'MuJoCo Python is required; install requirements-mujoco.txt') from error

    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    data = mujoco.MjData(model)
    qpos_address, dof_address = joint_addresses(mujoco, model, q.shape[1])
    gravity = np.zeros_like(q)
    friction = np.zeros_like(q)
    for index in range(q.shape[0]):
        data.qpos[:] = 0.0
        data.qvel[:] = 0.0
        data.qacc[:] = 0.0
        data.ctrl[:] = 0.0
        data.qfrc_applied[:] = 0.0
        data.qpos[qpos_address] = q[index]
        mujoco.mj_forward(model, data)
        gravity[index] = data.qfrc_bias[dof_address]

        data.qvel[dof_address] = dq[index]
        mujoco.mj_forward(model, data)
        friction[index] = (
            -data.qfrc_passive[dof_address]
            -data.qfrc_constraint[dof_address]
        )
    return gravity, friction


def export_compensation_validation(
        path, t, gravity_reference, gravity_predicted,
        friction_reference, friction_predicted):
    joint_count = gravity_reference.shape[1]
    total_reference = gravity_reference + friction_reference
    total_predicted = gravity_predicted + friction_predicted
    columns = ['t']
    values = [t]
    for name, reference, predicted in (
            ('gravity', gravity_reference, gravity_predicted),
            ('friction', friction_reference, friction_predicted),
            ('compensation', total_reference, total_predicted)):
        columns.extend(
            f'{name}{joint + 1}_reference' for joint in range(joint_count))
        columns.extend(
            f'{name}{joint + 1}_predicted' for joint in range(joint_count))
        columns.extend(
            f'{name}{joint + 1}_error' for joint in range(joint_count))
        values.extend((reference, predicted, predicted - reference))
    np.savetxt(
        path,
        np.column_stack(values),
        delimiter=',',
        header=','.join(columns),
        comments='',
    )


def validate_compensation(
        mjcf_path, config, urdf_path, base_set_path, parameters_path,
        trajectory_path, output_dir):
    """Compare deployment-model compensation with MuJoCo references."""
    joint_count = len(config['excitation_joint_lower'])
    time_values, q, dq, _ = read_trajectory(trajectory_path, joint_count)
    gravity_reference, friction_reference = mujoco_compensation_reference(
        mjcf_path, q, dq)
    gravity_predicted, friction_predicted = identified_compensation(
        config, urdf_path, base_set_path, parameters_path, q, dq)

    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / 'compensation_validation_sim.csv'
    export_compensation_validation(
        output_path, time_values, gravity_reference, gravity_predicted,
        friction_reference, friction_predicted)

    total_reference = gravity_reference + friction_reference
    total_predicted = gravity_predicted + friction_predicted
    report = {
        'model': str(parameters_path.resolve()),
        'reference': (
            'MuJoCo gravity at dq=0 plus negative qfrc_passive and '
            'qfrc_constraint at the validation velocity'),
        'friction_model': 'Fv*dq + Fc*tanh(dq/vs)',
        'validation_data': str(trajectory_path.resolve()),
        'validation_samples': int(q.shape[0]),
        'gravity_metrics': metrics(gravity_reference, gravity_predicted),
        'friction_metrics': metrics(friction_reference, friction_predicted),
        'total_compensation_metrics': metrics(
            total_reference, total_predicted),
    }
    report_path = output_dir / 'compensation_report_sim.json'
    with report_path.open('w', encoding='utf-8') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
    return report


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(
        description='Validate identified MuJoCo gravity/friction compensation')
    parser.add_argument('--config', type=Path)
    parser.add_argument('--urdf', type=Path)
    parser.add_argument(
        '--mjcf', type=Path,
        default=RESULTS_ROOT / 'seven_dof/simulation_model/exo7_sim.xml')
    parser.add_argument(
        '--base-set', type=Path,
        default=RESULTS_ROOT / 'seven_dof/excitation/base_parameter_set.npz')
    parser.add_argument(
        '--parameters', type=Path,
        default=RESULTS_ROOT / 'seven_dof/dynamics_identification/'
        'identified_parameters_sim.npz')
    parser.add_argument(
        '--trajectory-file', type=Path,
        default=RESULTS_ROOT / 'seven_dof/excitation/excitation_validation.csv')
    parser.add_argument(
        '--output-dir', type=Path,
        default=RESULTS_ROOT / 'seven_dof/compensation_validation')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_arguments(argv)
    config = load_config(args.config)
    urdf_path = (args.urdf or default_urdf_path()).expanduser().resolve()
    report = validate_compensation(
        args.mjcf.expanduser().resolve(),
        config,
        urdf_path,
        args.base_set.expanduser().resolve(),
        args.parameters.expanduser().resolve(),
        args.trajectory_file.expanduser().resolve(),
        args.output_dir.expanduser().resolve(),
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
