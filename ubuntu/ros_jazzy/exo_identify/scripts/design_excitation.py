#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
from pathlib import Path

from collision import (
    collision_model_fingerprint,
    collision_settings,
    CollisionChecker,
)
from common import (
    build_estimator_model,
    friction_regressor,
    load_config,
    RESULTS_ROOT,
    torque_regressor,
)
import numpy as np


PARAMETER_NAMES = [
    'm', 'mx', 'my', 'mz', 'Ixx', 'Ixy', 'Iyy', 'Ixz', 'Iyz', 'Izz'
]
DEFAULT_MJCF = RESULTS_ROOT / 'seven_dof/simulation_model/exo7_sim.xml'


def evaluate(
        coefficients: np.ndarray, t: np.ndarray, omega: float,
        position_offset=None):
    njoints, harmonics, _ = coefficients.shape
    q = np.zeros((t.size, njoints))
    dq = np.zeros_like(q)
    ddq = np.zeros_like(q)
    for joint in range(njoints):
        for harmonic in range(1, harmonics + 1):
            a, b = coefficients[joint, harmonic - 1]
            phase = harmonic * omega * t
            q[:, joint] += a * np.sin(phase) + b * np.cos(phase)
            dq[:, joint] += harmonic * omega * (
                a * np.cos(phase) - b * np.sin(phase)
            )
            ddq[:, joint] -= (harmonic * omega) ** 2 * (
                a * np.sin(phase) + b * np.cos(phase)
            )
    if position_offset is not None:
        q += np.asarray(position_offset, dtype=float)
    return q, dq, ddq


def feasible(q, dq, ddq, config) -> bool:
    return bool(
        np.all(q >= np.asarray(config['excitation_joint_lower']))
        and np.all(q <= np.asarray(config['excitation_joint_upper']))
        and np.all(np.abs(dq) <= np.asarray(config['velocity_limit']))
        and np.all(np.abs(ddq) <= np.asarray(config['acceleration_limit']))
        and np.all(
            np.ptp(q, axis=0)
            >= np.asarray(config['minimum_position_peak_to_peak'])
        )
    )


def stack_regressor(model, data, q, dq, ddq, config, include_friction, stride=5):
    rigid_blocks = []
    design_blocks = []
    for index in range(0, q.shape[0], stride):
        rigid = torque_regressor(model, data, q[index], dq[index], ddq[index])
        rigid_blocks.append(rigid)
        design_blocks.append(
            np.column_stack((rigid, friction_regressor(dq[index], config)))
            if include_friction
            else rigid
        )
    return np.vstack(rigid_blocks), np.vstack(design_blocks)


def information_metrics(matrix: np.ndarray):
    norms = np.linalg.norm(matrix, axis=0)
    active = norms > 1e-12
    normalized = matrix[:, active] / norms[active]
    singular = np.linalg.svd(normalized, compute_uv=False)
    rank = np.count_nonzero(singular > singular[0] * 1e-9)
    condition = float(singular[0] / singular[rank - 1])
    strength = float(np.exp(np.mean(np.log(norms[active] + 1e-15))))
    score = float(np.log(condition) - 0.15 * np.log(strength + 1e-15))
    return score, condition, strength


def independent_columns(matrix: np.ndarray, relative_tolerance: float):
    residual = matrix.copy()
    initial_norm = float(np.max(np.linalg.norm(residual, axis=0)))
    selected = []
    basis = []
    while True:
        norms = np.linalg.norm(residual, axis=0)
        if selected:
            norms[np.asarray(selected, dtype=int)] = -np.inf
        pivot = int(np.argmax(norms))
        if norms[pivot] <= initial_norm * relative_tolerance:
            break
        vector = residual[:, pivot] / norms[pivot]
        for existing in basis:
            vector -= existing * (existing @ vector)
        vector_norm = np.linalg.norm(vector)
        if vector_norm <= relative_tolerance:
            break
        vector /= vector_norm
        basis.append(vector)
        selected.append(pivot)
        residual -= np.outer(vector, vector @ residual)
    return np.asarray(selected, dtype=int)


def random_coefficients(rng, njoints, harmonics):
    coefficients = rng.normal(size=(njoints, harmonics, 2))
    decay = np.arange(1, harmonics + 1, dtype=float)[None, :, None] ** 2.3
    coefficients /= decay
    coefficients[:, :, 1] *= 0.8
    return coefficients


def scale_to_range(coefficients, t, omega, target_coverage, target_center):
    q, _, _ = evaluate(coefficients, t, omega)
    coverage = np.ptp(q, axis=0)
    scaled = coefficients.copy()
    for joint in range(coefficients.shape[0]):
        if coverage[joint] > 1e-12:
            scaled[joint] *= target_coverage[joint] / coverage[joint]
    scaled_q, _, _ = evaluate(scaled, t, omega)
    range_center = 0.5 * (
        np.min(scaled_q, axis=0) + np.max(scaled_q, axis=0))
    offset = np.asarray(target_center, dtype=float) - range_center
    return scaled, offset


def shift_start_near_center(coefficients, t, omega, position_offset, center):
    q, _, _ = evaluate(coefficients, t, omega, position_offset)
    # The final sample duplicates the start for an integer number of periods.
    search = q[:-1] if q.shape[0] > 1 else q
    index = int(np.argmin(np.linalg.norm(search - center, axis=1)))
    time_shift = float(t[index])
    shifted = coefficients.copy()
    for harmonic in range(1, coefficients.shape[1] + 1):
        phase = harmonic * omega * time_shift
        cosine = np.cos(phase)
        sine = np.sin(phase)
        a = coefficients[:, harmonic - 1, 0]
        b = coefficients[:, harmonic - 1, 1]
        shifted[:, harmonic - 1, 0] = a * cosine - b * sine
        shifted[:, harmonic - 1, 1] = a * sine + b * cosine
    return shifted, time_shift


def save_trajectory(path, t, q, dq, ddq):
    np.savetxt(
        path,
        np.column_stack([t, q, dq, ddq]),
        delimiter=',',
        header=','.join([
            't',
            *(f'q{joint + 1}' for joint in range(q.shape[1])),
            *(f'dq{joint + 1}' for joint in range(q.shape[1])),
            *(f'ddq{joint + 1}' for joint in range(q.shape[1])),
        ]),
        comments='',
    )


def validation_trajectory(coefficients, t, omega, target_center):
    validation = 0.65 * coefficients.copy()
    validation[:, :, [0, 1]] = validation[:, :, [1, 0]]
    q_uncentered, _, _ = evaluate(validation, t, 0.83 * omega)
    offset = np.asarray(target_center, dtype=float) - 0.5 * (
        np.min(q_uncentered, axis=0) + np.max(q_uncentered, axis=0))
    validation, time_shift = shift_start_near_center(
        validation, t, 0.83 * omega, offset, target_center)
    q_uncentered, _, _ = evaluate(validation, t, 0.83 * omega)
    offset = np.asarray(target_center, dtype=float) - 0.5 * (
        np.min(q_uncentered, axis=0) + np.max(q_uncentered, axis=0))
    q, dq, ddq = evaluate(validation, t, 0.83 * omega, offset)
    return validation, offset, time_shift, q, dq, ddq


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(
        description='Design a model-dimension Fourier excitation trajectory')
    parser.add_argument('--config', type=Path)
    parser.add_argument('--urdf', type=Path)
    parser.add_argument(
        '--mjcf', type=Path, default=DEFAULT_MJCF,
        help='unified contact-enabled MJCF generated by mujoco_model.py')
    parser.add_argument(
        '--output-dir', type=Path,
        default=RESULTS_ROOT / 'seven_dof/excitation')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_arguments(argv)
    config = load_config(args.config)
    model, data = build_estimator_model(config, args.urdf)
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    dt = float(config['sample_time'])
    t = np.arange(0.0, float(config['duration']) + 0.5 * dt, dt)
    omega = 2.0 * np.pi * float(config['base_frequency'])
    harmonics = int(config['harmonics'])
    rng = np.random.default_rng(int(config['random_seed']))
    include_friction = bool(config['identification']['include_friction'])
    collision = collision_settings(config)
    checker = None
    if collision['enabled']:
        checker = CollisionChecker(
            args.mjcf, model.nv, collision['minimum_clearance_m'])
    target_center = np.asarray(
        config.get('target_position_center', [0.0] * model.nv), dtype=float)
    if target_center.shape != (model.nv,):
        raise ValueError('target_position_center must contain one value per joint')

    candidates = []
    feasible_count = 0
    collision_rejected_count = 0
    for _ in range(int(config['trajectory_candidates'])):
        coefficients, position_offset = scale_to_range(
            random_coefficients(rng, model.nv, harmonics),
            t,
            omega,
            np.asarray(config['target_position_peak_to_peak'], dtype=float),
            target_center,
        )
        q, dq, ddq = evaluate(coefficients, t, omega, position_offset)
        if not feasible(q, dq, ddq, config):
            continue
        feasible_count += 1
        if checker is not None:
            coarse_result = checker.check_positions(q, t)
            if not coarse_result.collision_free:
                collision_rejected_count += 1
                continue
        rigid, design = stack_regressor(
            model, data, q, dq, ddq, config, include_friction
        )
        score, condition, strength = information_metrics(design)
        candidates.append({
            'score': score,
            'condition': condition,
            'strength': strength,
            'coefficients': coefficients,
            'position_offset': position_offset,
        })
    if not candidates:
        if feasible_count and collision_rejected_count == feasible_count:
            raise RuntimeError(
                f'All {feasible_count} trajectories satisfying the existing '
                'angle/velocity/acceleration conditions violate the configured '
                'collision clearance. Keep the constraints unchanged only if '
                'more or differently parameterized candidates can produce a '
                'collision-free path.')
        raise RuntimeError(
            'No feasible trajectory; relax limits or increase candidates')

    candidates.sort(key=lambda item: item['score'])
    dense_t = None
    selected = None
    dense_rejected_count = 0
    if checker is not None:
        dense_t = np.arange(
            0.0, float(config['duration']) + 0.5 * checker.timestep,
            checker.timestep)
    for rank, candidate in enumerate(candidates, start=1):
        coefficients, id_time_shift = shift_start_near_center(
            candidate['coefficients'], t, omega,
            candidate['position_offset'], target_center)
        position_offset = candidate['position_offset']
        q, dq, ddq = evaluate(
            coefficients, t, omega, position_offset)
        (validation, validation_offset, validation_time_shift,
         qv, dqv, ddqv) = validation_trajectory(
            coefficients, t, omega, target_center)
        id_collision = None
        validation_collision = None
        if checker is not None:
            q_dense, _, _ = evaluate(
                coefficients, dense_t, omega, position_offset)
            id_collision = checker.check_positions(q_dense, dense_t)
            qv_dense, _, _ = evaluate(
                validation, dense_t, 0.83 * omega, validation_offset)
            validation_collision = checker.check_positions(qv_dense, dense_t)
            if (not id_collision.collision_free
                    or not validation_collision.collision_free):
                dense_rejected_count += 1
                continue
        selected = {
            **candidate,
            'rank': rank,
            'coefficients': coefficients,
            'position_offset': position_offset,
            'id_time_shift': id_time_shift,
            'q': q,
            'dq': dq,
            'ddq': ddq,
            'validation': validation,
            'validation_offset': validation_offset,
            'validation_time_shift': validation_time_shift,
            'qv': qv,
            'dqv': dqv,
            'ddqv': ddqv,
            'id_collision': id_collision,
            'validation_collision': validation_collision,
        }
        break
    if selected is None:
        raise RuntimeError(
            f'All {len(candidates)} collision-prefiltered candidates failed '
            'the dense identification/validation trajectory check')

    score = selected['score']
    condition = selected['condition']
    strength = selected['strength']
    coefficients = selected['coefficients']
    position_offset = selected['position_offset']
    id_time_shift = selected['id_time_shift']
    q, dq, ddq = selected['q'], selected['dq'], selected['ddq']
    validation = selected['validation']
    validation_offset = selected['validation_offset']
    validation_time_shift = selected['validation_time_shift']
    qv, dqv, ddqv = selected['qv'], selected['dqv'], selected['ddqv']
    rigid, _ = stack_regressor(
        model, data, q, dq, ddq, config, include_friction)
    columns = independent_columns(rigid, float(config['svd_relative_tolerance']))
    labels = [
        f'{model.names[body]}:{name}'
        for body in range(1, model.njoints)
        for name in PARAMETER_NAMES
    ]
    base_labels = [labels[index] for index in columns]
    np.savez(
        output / 'base_parameter_set.npz',
        base_columns=columns,
        full_parameter_labels=np.asarray(labels),
        base_parameter_labels=np.asarray(base_labels),
    )
    save_trajectory(output / 'excitation_id.csv', t, q, dq, ddq)

    save_trajectory(output / 'excitation_validation.csv', t, qv, dqv, ddqv)
    report = {
        'warning': (
            'Trajectory is in simulation joint coordinates; do not command '
            'hardware before mapping, limits, and signs are verified.'
        ),
        'base_parameter_count': int(columns.size),
        'base_columns_zero_based': columns.tolist(),
        'base_parameter_labels': base_labels,
        'feasible_candidates': feasible_count,
        'collision_prefilter_rejected_candidates': collision_rejected_count,
        'collision_prefilter_survivors': len(candidates),
        'dense_collision_rejected_candidates': dense_rejected_count,
        'selected_collision_free_rank': selected['rank'],
        'selected_objective': score,
        'selected_normalized_condition_number': condition,
        'selected_geometric_column_strength': strength,
        'position_peak_to_peak_rad': np.ptp(q, axis=0).tolist(),
        'position_minimum_rad': np.min(q, axis=0).tolist(),
        'position_maximum_rad': np.max(q, axis=0).tolist(),
        'initial_position_rad': q[0].tolist(),
        'maximum_absolute_velocity_rad_s': np.max(np.abs(dq), axis=0).tolist(),
        'maximum_absolute_acceleration_rad_s2': np.max(np.abs(ddq), axis=0).tolist(),
        'position_offset_rad': position_offset.tolist(),
        'start_time_shift_s': id_time_shift,
        'trajectory_coefficients': coefficients.tolist(),
        'validation_position_minimum_rad': np.min(qv, axis=0).tolist(),
        'validation_position_maximum_rad': np.max(qv, axis=0).tolist(),
        'validation_initial_position_rad': qv[0].tolist(),
        'validation_maximum_absolute_velocity_rad_s': np.max(
            np.abs(dqv), axis=0).tolist(),
        'validation_maximum_absolute_acceleration_rad_s2': np.max(
            np.abs(ddqv), axis=0).tolist(),
        'validation_start_time_shift_s': validation_time_shift,
        'collision_check': {
            'enabled': collision['enabled'],
            'mjcf': str(args.mjcf.expanduser().resolve()),
            'model_sha256': collision_model_fingerprint(args.mjcf),
            'minimum_clearance_m': collision['minimum_clearance_m'],
            'dense_check_timestep_s': (
                checker.timestep if checker is not None else None),
            'identification_trajectory': (
                selected['id_collision'].to_dict()
                if selected['id_collision'] is not None else None),
            'validation_trajectory': (
                selected['validation_collision'].to_dict()
                if selected['validation_collision'] is not None else None),
        },
    }
    with (output / 'excitation_report.json').open('w', encoding='utf-8') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
