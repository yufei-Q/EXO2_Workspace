#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
from pathlib import Path

from common import build_estimator_model, load_base_set, load_config, torque_regressor
import numpy as np


def frequencies(order=2):
    return np.asarray(
        [
            (k1, k2)
            for k1 in range(-order, order + 1)
            for k2 in range(-order, order + 1)
            if k1 > 0 or (k1 == 0 and k2 > 0)
        ],
        dtype=int,
    )


def basis(q, frequency_vectors):
    phase = q @ frequency_vectors.T
    values = [np.ones(q.shape[0])]
    for index in range(frequency_vectors.shape[0]):
        values.extend((np.cos(phase[:, index]), np.sin(phase[:, index])))
    return np.column_stack(values)


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(
        description='Export identified gravity as a trigonometric formula'
    )
    parser.add_argument('--parameters', type=Path, required=True)
    parser.add_argument('--base-set', type=Path, required=True)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--urdf', type=Path)
    parser.add_argument(
        '--output',
        type=Path,
        default=Path('identify_output/gravity_formula.json'),
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_arguments(argv)
    config = load_config(args.config)
    model, data = build_estimator_model(config, args.urdf)
    columns, _ = load_base_set(args.base_set)
    with np.load(args.parameters) as identified:
        if (
            'base_columns' in identified
            and not np.array_equal(
                columns, np.asarray(identified['base_columns'], dtype=int)
            )
        ):
            raise ValueError(
                'base parameter set does not match the identified parameters; '
                'rerun identification with the current base set'
            )
        beta = (
            identified['rigid_beta']
            if 'rigid_beta' in identified
            else identified['beta'][: len(columns)]
        )
        include_friction = bool(
            np.asarray(identified['include_friction']).item()
        ) if 'include_friction' in identified else False
        friction_beta = (
            np.asarray(identified['friction_beta'], dtype=float)
            if include_friction and 'friction_beta' in identified
            else np.asarray([], dtype=float)
        )
    if include_friction and friction_beta.shape != (4,):
        raise ValueError(
            'identified friction parameters must contain [Fv1, Fv2, Fc1, Fc2]'
        )
    lower = np.asarray(config['gravity_formula_joint_lower'], dtype=float)
    upper = np.asarray(config['gravity_formula_joint_upper'], dtype=float)
    mesh1, mesh2 = np.meshgrid(
        np.linspace(lower[0], upper[0], 41),
        np.linspace(lower[1], upper[1], 41),
        indexing='ij',
    )
    q = np.column_stack((mesh1.ravel(), mesh2.ravel()))
    torque = np.vstack(
        [
            torque_regressor(model, data, position, np.zeros(2), np.zeros(2))[
                :, columns
            ]
            @ beta
            for position in q
        ]
    )
    frequency_vectors = frequencies(order=2)
    design = basis(q, frequency_vectors)
    coefficients, *_ = np.linalg.lstsq(design, torque, rcond=None)
    residual = design @ coefficients - torque
    formula = {
        'definition': (
            'phi(q)=[1,cos(k1*q1+k2*q2),sin(k1*q1+k2*q2),...]; '
            'tau_g_hat=phi(q)*coefficient_matrix'
        ),
        'frequency_vectors': frequency_vectors.tolist(),
        'coefficient_matrix': coefficients.tolist(),
        'max_formula_error_Nm': float(np.max(np.abs(residual))),
        'rmse_formula_error_Nm': float(np.sqrt(np.mean(residual**2))),
        'friction': {
            'enabled': include_friction,
            'definition': 'tau_f=Fv*dq+Fc*tanh(dq/vs)',
            'viscous_coefficients_Nm_per_rad_s': (
                friction_beta[:2].tolist() if include_friction else [0.0, 0.0]
            ),
            'coulomb_coefficients_Nm': (
                friction_beta[2:].tolist() if include_friction else [0.0, 0.0]
            ),
            'transition_velocity_rad_s': config['friction'][
                'transition_velocity_rad_s'
            ],
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('w', encoding='utf-8') as stream:
        json.dump(formula, stream, ensure_ascii=False, indent=2)
    print(json.dumps(formula, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
