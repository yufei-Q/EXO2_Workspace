#!/usr/bin/env python3

"""Export a compact, model-dimension gravity approximation."""

from __future__ import annotations

import argparse
from itertools import product
import json
from pathlib import Path

from common import (
    build_estimator_model,
    load_base_set,
    load_config,
    RESULTS_ROOT,
    torque_regressor,
)
import numpy as np
from scipy.stats import qmc


def frequencies(joint_count: int, order: int = 1) -> np.ndarray:
    if order != 1:
        raise ValueError('serial rigid-body gravity needs Fourier order 1')
    vectors = []
    for values in product((-1, 0, 1), repeat=joint_count):
        if not any(values):
            continue
        first_nonzero = next(value for value in values if value)
        if first_nonzero > 0:
            vectors.append(values)
    return np.asarray(vectors, dtype=int)


def basis(q: np.ndarray, frequency_vectors: np.ndarray) -> np.ndarray:
    angles = np.asarray(q, dtype=float)
    phase = angles @ frequency_vectors.T
    values = [np.ones(angles.shape[0])]
    for index in range(frequency_vectors.shape[0]):
        values.extend((np.cos(phase[:, index]), np.sin(phase[:, index])))
    return np.column_stack(values)


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(
        description='Export a model-dimension gravity formula')
    parser.add_argument(
        '--parameters', type=Path,
        default=RESULTS_ROOT / 'seven_dof/dynamics_identification/'
        'identified_parameters_sim.npz')
    parser.add_argument(
        '--base-set', type=Path,
        default=RESULTS_ROOT / 'seven_dof/excitation/base_parameter_set.npz')
    parser.add_argument('--config', type=Path)
    parser.add_argument('--urdf', type=Path)
    parser.add_argument('--order', type=int, default=1, choices=(1,))
    parser.add_argument(
        '--output', type=Path,
        default=RESULTS_ROOT / 'seven_dof/dynamics_identification/'
        'gravity_formula_sim.json')
    parser.add_argument('--quiet', action='store_true')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_arguments(argv)
    config = load_config(args.config)
    model, data = build_estimator_model(config, args.urdf)
    joint_count = model.nv
    columns, _ = load_base_set(args.base_set)
    with np.load(args.parameters) as identified:
        if (
            'base_columns' in identified
            and not np.array_equal(
                columns, np.asarray(identified['base_columns'], dtype=int))
        ):
            raise ValueError(
                'base parameter set does not match the identified parameters; '
                'rerun identification with the current base set')
        beta = (
            np.asarray(identified['rigid_beta'], dtype=float)
            if 'rigid_beta' in identified
            else np.asarray(identified['beta'][:len(columns)], dtype=float)
        )
        include_friction = bool(
            np.asarray(identified['include_friction']).item()
        ) if 'include_friction' in identified else False
        friction_beta = (
            np.asarray(identified['friction_beta'], dtype=float)
            if include_friction and 'friction_beta' in identified
            else np.zeros(2 * joint_count)
        )
    if include_friction and friction_beta.shape != (2 * joint_count,):
        raise ValueError(
            f'identified friction parameters must contain {2 * joint_count} values')

    grid_shape = (3,) * joint_count
    grid_indices = np.asarray(
        list(product(range(3), repeat=joint_count)), dtype=int)
    q_grid = 2.0 * np.pi * grid_indices / 3.0
    torque_grid = np.vstack([
        torque_regressor(model, data, position, np.zeros(joint_count),
                         np.zeros(joint_count))[:, columns] @ beta
        for position in q_grid
    ])
    frequency_vectors = frequencies(joint_count, args.order)
    samples = torque_grid.reshape((*grid_shape, joint_count))
    spectrum = np.fft.fftn(
        samples, axes=tuple(range(joint_count))) / (3 ** joint_count)
    coefficients = [np.real(spectrum[(0,) * joint_count])]
    for vector in frequency_vectors:
        index = tuple(0 if value == 0 else 1 if value == 1 else 2
                      for value in vector)
        complex_coefficient = spectrum[index]
        coefficients.extend((
            2.0 * np.real(complex_coefficient),
            -2.0 * np.imag(complex_coefficient),
        ))
    coefficients = np.asarray(coefficients, dtype=float)

    lower = np.asarray(config['gravity_formula_joint_lower'], dtype=float)
    upper = np.asarray(config['gravity_formula_joint_upper'], dtype=float)
    if lower.shape != (joint_count,) or upper.shape != (joint_count,):
        raise ValueError('gravity formula bounds must contain one value per joint')
    q_check = qmc.scale(
        qmc.Sobol(d=joint_count, scramble=True, seed=23).random_base2(10),
        lower, upper)
    torque_check = np.vstack([
        torque_regressor(model, data, position, np.zeros(joint_count),
                         np.zeros(joint_count))[:, columns] @ beta
        for position in q_check
    ])
    residual = basis(q_check, frequency_vectors) @ coefficients - torque_check
    formula = {
        'joint_count': joint_count,
        'definition': (
            'phi(q)=[1,cos(k dot q),sin(k dot q),...]; '
            'tau_g_hat=phi(q)*coefficient_matrix'
        ),
        'frequency_vectors': frequency_vectors.tolist(),
        'coefficient_matrix': coefficients.tolist(),
        'fit_samples': int(q_grid.shape[0]),
        'check_samples': int(q_check.shape[0]),
        'max_formula_error_Nm': float(np.max(np.abs(residual))),
        'rmse_formula_error_Nm': float(np.sqrt(np.mean(residual ** 2))),
        'friction': {
            'enabled': include_friction,
            'definition': 'tau_f=Fv*dq+Fc*tanh(dq/vs)',
            'viscous_coefficients_Nm_per_rad_s': (
                friction_beta[:joint_count].tolist()
                if include_friction else [0.0] * joint_count),
            'coulomb_coefficients_Nm': (
                friction_beta[joint_count:].tolist()
                if include_friction else [0.0] * joint_count),
            'transition_velocity_rad_s': config['friction'][
                'transition_velocity_rad_s'],
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('w', encoding='utf-8') as stream:
        json.dump(formula, stream, ensure_ascii=False, indent=2)
    if not args.quiet:
        print(json.dumps(formula, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
