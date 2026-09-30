#!/usr/bin/env python3

"""Orchestrate the independent MuJoCo dynamics-identification stages.

The actual work lives in one script per stage:

``mujoco_model.py``
    Generate the single contact-enabled MJCF used by every MuJoCo stage.
``design_excitation.py``
    Design the shared excitation and validation trajectories.
``collect_mujoco_dynamics.py``
    Run MuJoCo forward dynamics and record simulated sensor/actuator data.
``identify_parameters.py``
    Identify rigid-body and friction parameters from CSV data.
``export_gravity_formula.py``
    Export a compact gravity formula from the identified model.
``validate_mujoco_compensation.py``
    Compare the deployed model with MuJoCo gravity and friction terms.

This file deliberately only dispatches those stages. It provides a convenient
one-command simulation run without hiding any stage that can also be run on its
own. Real-hardware collection and compensation remain separate ROS 2 steps.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from collect_mujoco_dynamics import main as collect_main
from collision import (
    collision_model_fingerprint,
    collision_settings,
    validate_collision_model,
)
from common import load_config, RESULTS_ROOT
from design_excitation import main as design_main
from export_gravity_formula import main as export_formula_main
from identify_parameters import main as identify_main
from mujoco_model import main as model_main
from validate_mujoco_compensation import main as validate_main


def _optional_model_arguments(args):
    values = []
    if args.config is not None:
        values.extend(['--config', str(args.config)])
    if args.urdf is not None:
        values.extend(['--urdf', str(args.urdf)])
    return values


def _mjcf_path(args):
    return args.mjcf or args.model_dir / 'exo7_sim.xml'


def _collision_model_error(args):
    path = _mjcf_path(args)
    if not path.is_file():
        return f'MJCF does not exist: {path}'
    collision = collision_settings(load_config(args.config))
    if not collision['enabled']:
        return None
    try:
        import mujoco
        model = mujoco.MjModel.from_xml_path(str(path))
        validate_collision_model(
            mujoco, model, collision['minimum_clearance_m'])
    except (ImportError, OSError, RuntimeError, ValueError) as error:
        return str(error)
    return None


def _model(args, force=False):
    path = _mjcf_path(args)
    error = None if force else _collision_model_error(args)
    if error is None and not force:
        return False
    if args.mjcf is not None and not force:
        raise ValueError(
            f'The explicitly supplied MJCF is not a valid collision model: '
            f'{path}: {error}. Regenerate it explicitly with the model stage '
            'or --regenerate-model.')
    model_arguments = [
        '--no-viewer',
        '--output', str(path),
        *_optional_model_arguments(args),
    ]
    model_main(model_arguments)
    generated_error = _collision_model_error(args)
    if generated_error is not None:
        raise RuntimeError(
            f'Generated MJCF failed collision validation: {generated_error}')
    return True


def _trajectory_collision_record_is_current(args):
    if not (args.trajectory_file.is_file()
            and args.validation_trajectory_file.is_file()):
        return False
    collision = collision_settings(load_config(args.config))
    if not collision['enabled']:
        return True
    report_path = args.trajectory_file.parent / 'excitation_report.json'
    try:
        with report_path.open('r', encoding='utf-8') as stream:
            report = json.load(stream)
        check = report['collision_check']
        report_fingerprint = check['model_sha256']
        clearance = float(check['minimum_clearance_m'])
        identification = check['identification_trajectory']
        validation = check['validation_trajectory']
    except (KeyError, OSError, TypeError, ValueError, json.JSONDecodeError):
        return False
    if not isinstance(identification, dict) or not isinstance(validation, dict):
        return False
    return (
        bool(check.get('enabled'))
        and report_fingerprint == collision_model_fingerprint(_mjcf_path(args))
        and clearance + 1e-12 >= collision['minimum_clearance_m']
        and bool(identification.get('collision_free'))
        and bool(validation.get('collision_free'))
    )


def _design(args, force=False):
    trajectory_path = args.trajectory_file.expanduser().resolve()
    validation_path = args.validation_trajectory_file.expanduser().resolve()
    if not force and trajectory_path.is_file() and validation_path.is_file():
        return
    if trajectory_path.parent != validation_path.parent:
        raise ValueError(
            'trajectory-file and validation-trajectory-file must share a '
            'directory when the pipeline designs both files')
    design_arguments = [
        '--output-dir', str(trajectory_path.parent),
        '--mjcf', str(_mjcf_path(args)),
        *_optional_model_arguments(args),
    ]
    design_main(design_arguments)


def _collect(args):
    collect_arguments = [
        '--trajectory-file', str(args.trajectory_file),
        '--validation-trajectory-file', str(args.validation_trajectory_file),
        '--output-dir', str(args.output_dir),
        '--model-dir', str(args.model_dir),
    ]
    if args.config is not None:
        collect_arguments.extend(['--config', str(args.config)])
    collect_arguments.extend(['--mjcf', str(_mjcf_path(args))])
    if args.torque_noise_std is not None:
        collect_arguments.extend(
            ['--torque-noise-std', str(args.torque_noise_std)])
    if args.viewer:
        collect_arguments.extend([
            '--viewer', '--viewer-speed', str(args.viewer_speed)])
    collect_main(collect_arguments)


def _identify(args):
    identify_arguments = [
        '--id-data', str(args.output_dir / 'dynamics_id_sim.csv'),
        '--validation-data', str(args.output_dir / 'dynamics_validation_sim.csv'),
        '--base-set', str(args.base_set),
        '--output-dir', str(args.identification_output_dir),
        '--friction', 'on',
        '--suffix', 'sim',
        *_optional_model_arguments(args),
    ]
    identify_main(identify_arguments)


def _export(args):
    export_arguments = [
        '--parameters', str(
            args.identification_output_dir / 'identified_parameters_sim.npz'),
        '--base-set', str(args.base_set),
        '--output', str(
            args.identification_output_dir / 'gravity_formula_sim.json'),
        '--quiet',
        *_optional_model_arguments(args),
    ]
    export_formula_main(export_arguments)


def _validate(args):
    validate_arguments = [
        '--mjcf', str(_mjcf_path(args)),
        '--base-set', str(args.base_set),
        '--parameters', str(
            args.identification_output_dir / 'identified_parameters_sim.npz'),
        '--trajectory-file', str(args.validation_trajectory_file),
        '--output-dir', str(args.validation_output_dir),
        *_optional_model_arguments(args),
    ]
    validate_main(validate_arguments)


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(
        description='Run the staged seven-DOF MuJoCo dynamics workflow')
    parser.add_argument(
        'stage', nargs='?', default='all',
        choices=(
            'model', 'design', 'collect', 'identify', 'export', 'validate',
            'all'),
        help='one stage, or all simulation stages in order')
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
        help='directory for MuJoCo forward-simulation CSV data')
    parser.add_argument(
        '--model-dir', type=Path,
        default=RESULTS_ROOT / 'seven_dof/simulation_model',
        help='directory containing exo7_sim.xml when --mjcf is not set')
    parser.add_argument(
        '--mjcf', type=Path, default=None,
        help='existing MuJoCo MJCF file generated by mujoco_model.py')
    parser.add_argument(
        '--identification-output-dir', type=Path,
        default=RESULTS_ROOT / 'seven_dof/dynamics_identification')
    parser.add_argument(
        '--validation-output-dir', type=Path,
        default=RESULTS_ROOT / 'seven_dof/compensation_validation',
        help='directory for gravity/friction compensation validation')
    parser.add_argument(
        '--base-set', type=Path,
        default=RESULTS_ROOT / 'seven_dof/excitation/base_parameter_set.npz')
    parser.add_argument('--torque-noise-std', type=float, default=None)
    parser.add_argument(
        '--viewer', action='store_true',
        help='show the live MuJoCo collection window during the collect stage')
    parser.add_argument(
        '--viewer-speed', type=float, default=1.0,
        help='real-time display speed multiplier for --viewer')
    parser.add_argument(
        '--regenerate-model', action='store_true',
        help='regenerate the canonical MJCF before an all-stage run')
    parser.add_argument(
        '--regenerate-trajectory', action='store_true',
        help='regenerate trajectories before an all-stage run')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_arguments(argv)
    args.trajectory_file = args.trajectory_file.expanduser().resolve()
    args.validation_trajectory_file = (
        args.validation_trajectory_file.expanduser().resolve())
    args.output_dir = args.output_dir.expanduser().resolve()
    args.model_dir = args.model_dir.expanduser().resolve()
    if args.mjcf is not None:
        args.mjcf = args.mjcf.expanduser().resolve()
    args.identification_output_dir = (
        args.identification_output_dir.expanduser().resolve())
    args.validation_output_dir = (
        args.validation_output_dir.expanduser().resolve())
    args.base_set = args.base_set.expanduser().resolve()
    if args.config is not None:
        args.config = args.config.expanduser().resolve()
    if args.urdf is not None:
        args.urdf = args.urdf.expanduser().resolve()

    if args.stage == 'model':
        _model(args, force=True)
    elif args.stage == 'design':
        _design(args, force=True)
    elif args.stage == 'collect':
        _collect(args)
    elif args.stage == 'identify':
        _identify(args)
    elif args.stage == 'export':
        _export(args)
    elif args.stage == 'validate':
        _validate(args)
    else:
        model_changed = _model(args, force=args.regenerate_model)
        trajectory_is_current = _trajectory_collision_record_is_current(args)
        _design(
            args,
            force=(
                args.regenerate_trajectory
                or model_changed
                or not trajectory_is_current),
        )
        _collect(args)
        _identify(args)
        _export(args)
        _validate(args)
    print(f'Simulation pipeline stage completed: {args.stage}')


if __name__ == '__main__':
    main()
