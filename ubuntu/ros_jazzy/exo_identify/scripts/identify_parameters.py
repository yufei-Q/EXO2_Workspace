#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
from pathlib import Path

from common import (
    build_estimator_model,
    friction_parameter_labels_for_count,
    friction_regressor,
    load_base_set,
    load_config,
    require_csv_columns,
    RESULTS_ROOT,
    torque_regressor,
)
import numpy as np


def signal_columns(joint_count: int):
    return (
        ['t']
        + [f'q{joint + 1}' for joint in range(joint_count)]
        + [f'dq{joint + 1}' for joint in range(joint_count)]
        + [f'ddq{joint + 1}' for joint in range(joint_count)]
        + [f'tau{joint + 1}' for joint in range(joint_count)]
    )


def read_dataset(path: Path, joint_count: int):
    values = np.genfromtxt(path, delimiter=',', names=True)
    columns = tuple(signal_columns(joint_count))
    require_csv_columns(values, columns, path)
    values = np.atleast_1d(values)
    result = (
        np.asarray(values['t'], dtype=float),
        np.column_stack([values[f'q{joint + 1}'] for joint in range(joint_count)]),
        np.column_stack([values[f'dq{joint + 1}'] for joint in range(joint_count)]),
        np.column_stack([values[f'ddq{joint + 1}'] for joint in range(joint_count)]),
        np.column_stack([values[f'tau{joint + 1}'] for joint in range(joint_count)]),
    )
    if not all(np.all(np.isfinite(item)) for item in result):
        raise ValueError(f'{path} contains NaN or infinite values')
    return result


def build_matrix(model, data, q, dq, ddq, columns, config, include_friction):
    blocks = []
    for index in range(q.shape[0]):
        rigid = torque_regressor(model, data, q[index], dq[index], ddq[index])[:, columns]
        blocks.append(
            np.column_stack((rigid, friction_regressor(dq[index], config)))
            if include_friction
            else rigid
        )
    return np.vstack(blocks)


def metrics(measured, predicted):
    error = predicted - measured
    scale = np.maximum(np.ptp(measured, axis=0), 1e-12)
    rmse = np.sqrt(np.mean(error**2, axis=0))
    return {
        'rmse_per_joint_Nm': rmse.tolist(),
        'max_abs_error_per_joint_Nm': np.max(np.abs(error), axis=0).tolist(),
        'normalized_rmse_per_joint': (rmse / scale).tolist(),
    }


def export_prediction(path, t, measured, predicted):
    joint_count = measured.shape[1]
    header = (
        ['t']
        + [f'tau{joint + 1}_measured' for joint in range(joint_count)]
        + [f'tau{joint + 1}_predicted' for joint in range(joint_count)]
        + [f'error{joint + 1}' for joint in range(joint_count)]
    )
    np.savetxt(
        path,
        np.column_stack((t, measured, predicted, predicted - measured)),
        delimiter=',',
        header=','.join(header),
        comments='',
    )


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description='Identify rigid-body and friction parameters')
    parser.add_argument(
        '--id-data', type=Path,
        default=RESULTS_ROOT / 'seven_dof/simulation_data/dynamics_id_sim.csv')
    parser.add_argument(
        '--validation-data', type=Path,
        default=RESULTS_ROOT / 'seven_dof/simulation_data/'
        'dynamics_validation_sim.csv')
    parser.add_argument(
        '--base-set', type=Path,
        default=RESULTS_ROOT / 'seven_dof/excitation/base_parameter_set.npz')
    parser.add_argument('--config', type=Path)
    parser.add_argument('--urdf', type=Path)
    parser.add_argument(
        '--output-dir', type=Path,
        default=RESULTS_ROOT / 'seven_dof/dynamics_identification')
    parser.add_argument('--friction', choices=('auto', 'on', 'off'), default='auto')
    parser.add_argument(
        '--suffix', default='sim',
        help='suffix appended to generated files, for example "_sim"')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_arguments(argv)
    config = load_config(args.config)
    include_friction = (
        bool(config['identification']['include_friction'])
        if args.friction == 'auto'
        else args.friction == 'on'
    )
    model, data = build_estimator_model(config, args.urdf)
    columns, rigid_labels = load_base_set(args.base_set)
    t, q, dq, ddq, tau = read_dataset(args.id_data, model.nv)
    matrix = build_matrix(model, data, q, dq, ddq, columns, config, include_friction)
    target = tau.reshape(-1)
    ridge = float(config['ridge'])
    normal = matrix.T @ matrix + ridge * np.eye(matrix.shape[1])
    parameters = np.linalg.solve(normal, matrix.T @ target)
    rigid_count = len(columns)
    rigid_beta = parameters[:rigid_count]
    friction_beta = parameters[rigid_count:]
    predicted = (matrix @ parameters).reshape(-1, model.nv)

    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    suffix = str(args.suffix)
    if suffix and not suffix.startswith('_'):
        suffix = f'_{suffix}'
    export_prediction(
        output / f'prediction_id{suffix}.csv', t, tau, predicted)
    np.savez(
        output / f'identified_parameters{suffix}.npz',
        beta=parameters,
        rigid_beta=rigid_beta,
        friction_beta=friction_beta,
        base_columns=columns,
        include_friction=np.asarray(include_friction),
    )
    report = {
        'input_data': str(args.id_data.resolve()),
        'coordinate_requirement': (
            'q/dq/ddq/tau must already be mapped to URDF joint1..jointN '
            'signs, units, zero offsets, and output-shaft quantities.'
        ),
        'friction_columns_enabled': include_friction,
        'friction_model': 'Fv*dq + Fc*tanh(dq/vs)',
        'identified_base_parameters': [
            {'label': label, 'value': float(value)}
            for label, value in zip(rigid_labels, rigid_beta)
        ],
        'identified_friction_parameters': [
            {'label': label, 'value': float(value)}
            for label, value in zip(
                friction_parameter_labels_for_count(model.nv)
                if include_friction else [], friction_beta
            )
        ],
        'regressor_shape': list(matrix.shape),
        'regressor_condition_number': float(np.linalg.cond(matrix)),
        'identification_metrics': metrics(tau, predicted),
    }
    if args.validation_data:
        tv, qv, dqv, ddqv, tauv = read_dataset(args.validation_data, model.nv)
        validation_matrix = build_matrix(
            model, data, qv, dqv, ddqv, columns, config, include_friction
        )
        predicted_v = (validation_matrix @ parameters).reshape(-1, model.nv)
        export_prediction(
            output / f'prediction_validation{suffix}.csv',
            tv, tauv, predicted_v)
        report['validation_data'] = str(args.validation_data.resolve())
        report['independent_validation_metrics'] = metrics(tauv, predicted_v)
    with (output / f'identification_report{suffix}.json').open(
            'w', encoding='utf-8') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
