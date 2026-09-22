from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from scipy.signal import butter, sosfiltfilt
from scipy.spatial.transform import Rotation


PACKAGE_NAME = 'exo_identify'
_local_root = Path(__file__).resolve().parents[1]
if (_local_root / 'config').is_dir():
    SOURCE_ROOT = _local_root
else:
    try:
        from ament_index_python.packages import get_package_share_directory

        SOURCE_ROOT = Path(get_package_share_directory(PACKAGE_NAME))
    except Exception:
        SOURCE_ROOT = _local_root
RESULTS_ROOT = SOURCE_ROOT / 'results'
HARDWARE_MAPPING_KEYS = (
    'motor_indices',
    'joint_directions',
    'motor_zero_positions',
    'motor_position_per_joint_radian',
    'joint_torque_per_motor_torque',
)


def signal_header(joint_count: int) -> str:
    """Return the canonical CSV header for an arbitrary joint count."""
    names = [f'q{joint + 1}' for joint in range(joint_count)]
    names += [f'dq{joint + 1}' for joint in range(joint_count)]
    names += [f'ddq{joint + 1}' for joint in range(joint_count)]
    names += [f'tau{joint + 1}' for joint in range(joint_count)]
    return ','.join(['t', *names])


@dataclass(frozen=True)
class ProcessingConfig:
    filter_rate: float = 500.0
    output_rate: float = 100.0
    cutoff_frequency: float = 0.5
    filter_order: int = 4
    edge_trim: float = 2.0

    def validate(self):
        if self.filter_rate <= 0.0 or self.output_rate <= 0.0:
            raise ValueError('filter_rate and output_rate must be positive')
        if not 0.0 < self.cutoff_frequency < 0.5 * self.output_rate:
            raise ValueError('cutoff_frequency must be below output Nyquist frequency')
        if self.output_rate > self.filter_rate:
            raise ValueError('output_rate cannot exceed filter_rate')
        if self.filter_order < 1:
            raise ValueError('filter_order must be positive')
        if self.edge_trim < 0.0:
            raise ValueError('edge_trim cannot be negative')


def _validate_experiment_signals(t, q, dq, tau):
    time = np.asarray(t, dtype=float).reshape(-1)
    signals = [np.asarray(item, dtype=float) for item in (q, dq, tau)]
    if time.size < 10:
        raise ValueError('at least ten raw samples are required')
    if any(item.ndim != 2 or item.shape[0] != time.size for item in signals):
        raise ValueError('q, dq and tau must have shape (samples, joints)')
    if not signals or any(item.shape[1] != signals[0].shape[1] for item in signals):
        raise ValueError('q, dq and tau must have the same joint count')
    if not all(np.all(np.isfinite(item)) for item in (time, *signals)):
        raise ValueError('raw data contains NaN or infinite values')

    order = np.argsort(time)
    time = time[order]
    signals = [item[order] for item in signals]
    unique = np.r_[True, np.diff(time) > 1e-6]
    time = time[unique]
    signals = [item[unique] for item in signals]
    if time.size < 10 or np.any(np.diff(time) <= 0.0):
        raise ValueError('raw timestamps do not contain enough unique samples')
    return time, signals


def _interpolate_columns(target_time, source_time, values):
    return np.column_stack([
        np.interp(target_time, source_time, values[:, joint])
        for joint in range(values.shape[1])
    ])


def process_signals(t, q, dq, tau, config=None):
    settings = config or ProcessingConfig()
    settings.validate()
    time, (position, velocity, torque) = _validate_experiment_signals(
        t, q, dq, tau)

    filter_dt = 1.0 / settings.filter_rate
    filter_time = np.arange(
        time[0], time[-1] + 0.5 * filter_dt, filter_dt)
    uniform = [
        _interpolate_columns(filter_time, time, item)
        for item in (position, velocity, torque)
    ]
    sos = butter(
        settings.filter_order,
        settings.cutoff_frequency,
        btype='lowpass',
        fs=settings.filter_rate,
        output='sos',
    )
    position_filtered, velocity_filtered, torque_filtered = [
        sosfiltfilt(sos, item, axis=0) for item in uniform
    ]
    acceleration_filtered = np.gradient(
        velocity_filtered, filter_dt, axis=0, edge_order=2)

    output_start = time[0] + settings.edge_trim
    output_end = time[-1] - settings.edge_trim
    if output_end <= output_start:
        raise ValueError('edge_trim removes the entire recording')
    output_dt = 1.0 / settings.output_rate
    output_time_absolute = np.arange(
        output_start, output_end + 0.5 * output_dt, output_dt)
    output_signals = [
        _interpolate_columns(output_time_absolute, filter_time, item)
        for item in (
            position_filtered,
            velocity_filtered,
            acceleration_filtered,
            torque_filtered,
        )
    ]
    output_time = output_time_absolute - output_time_absolute[0]
    processed = np.column_stack((output_time, *output_signals))
    report = {
        'settings': asdict(settings),
        'raw_samples': int(time.size),
        'raw_duration_seconds': float(time[-1] - time[0]),
        'raw_mean_rate_hz': float(1.0 / np.mean(np.diff(time))),
        'raw_maximum_gap_seconds': float(np.max(np.diff(time))),
        'processed_samples': int(processed.shape[0]),
        'processed_duration_seconds': float(output_time[-1]),
        'position_range_rad': [
            [float(np.min(output_signals[0][:, joint])),
             float(np.max(output_signals[0][:, joint]))]
            for joint in range(output_signals[0].shape[1])
        ],
        'maximum_absolute_velocity_rad_s': np.max(
            np.abs(output_signals[1]), axis=0).tolist(),
        'maximum_absolute_acceleration_rad_s2': np.max(
            np.abs(output_signals[2]), axis=0).tolist(),
        'maximum_absolute_torque_Nm': np.max(
            np.abs(output_signals[3]), axis=0).tolist(),
    }
    return processed, report


def save_processed(path, values):
    np.savetxt(
        path,
        values,
        delimiter=',',
        header=signal_header((values.shape[1] - 1) // 4),
        comments='',
    )


def package_share() -> Path:
    """Locate installed package data, with a source-tree fallback."""
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory(PACKAGE_NAME))
    except Exception:
        return SOURCE_ROOT


def default_config_path() -> Path:
    source = SOURCE_ROOT / 'config' / 'identification.json'
    installed = package_share() / 'config' / 'identification.json'
    return source if source.exists() else installed


def default_hardware_mapping_path() -> Path:
    return SOURCE_ROOT / 'config' / 'hardware_mapping.yaml'


def load_hardware_mapping(path: Path | str | None = None) -> dict:
    mapping_path = (
        Path(path).expanduser()
        if path else default_hardware_mapping_path()
    ).resolve()
    if not mapping_path.is_file():
        raise FileNotFoundError(
            f'hardware mapping file does not exist: {mapping_path}')
    with mapping_path.open('r', encoding='utf-8') as stream:
        document = yaml.safe_load(stream) or {}
    mapping = document.get('hardware_mapping')
    if not isinstance(mapping, dict):
        raise ValueError(
            f'{mapping_path} must contain a hardware_mapping mapping')
    missing = [key for key in HARDWARE_MAPPING_KEYS if key not in mapping]
    if missing:
        raise ValueError(
            f'{mapping_path} is missing hardware mapping keys: {", ".join(missing)}')
    return mapping


def default_urdf_path() -> Path:
    source = (
        SOURCE_ROOT / 'urdf' / 'exo.SLDASM' / 'urdf'
        / '装配体.SLDASM.urdf'
    )
    installed = (
        package_share() / 'urdf' / 'exo.SLDASM' / 'urdf'
        / '装配体.SLDASM.urdf'
    )
    return source if source.exists() else installed


def load_config(path: Path | str | None = None) -> dict:
    config_path = Path(path) if path else default_config_path()
    with config_path.open('r', encoding='utf-8') as stream:
        return json.load(stream)


def build_estimator_model(
    config: dict, urdf_path: Path | str | None = None
) -> tuple[Any, Any]:
    try:
        import pinocchio as pin
    except ImportError as error:
        raise RuntimeError(
            'Pinocchio is required for parameter identification; '
            'the MuJoCo collector itself does not require it.') from error
    path = Path(urdf_path) if urdf_path else default_urdf_path()
    model = pin.buildModelFromUrdf(str(path))
    gravity_world = np.asarray(config['gravity'], dtype=float).reshape(-1)
    if gravity_world.shape != (3,) or not np.all(np.isfinite(gravity_world)):
        raise ValueError('gravity must contain three finite values')
    simulation = config.get('simulation', {})
    base_orientation_rpy = np.asarray(
        simulation.get('base_orientation_rpy', (0.0, 0.0, 0.0)),
        dtype=float,
    ).reshape(-1)
    if (base_orientation_rpy.shape != (3,)
            or not np.all(np.isfinite(base_orientation_rpy))):
        raise ValueError(
            'simulation.base_orientation_rpy must contain three finite values')
    # MuJoCo applies gravity in the world frame after rotating the URDF root
    # by base_orientation_rpy.  Pinocchio's fixed-base URDF model keeps the
    # root frame unrotated, so express the same world gravity in that root
    # frame before building all regressors used for identification.
    base_rotation = Rotation.from_euler(
        'xyz', base_orientation_rpy).as_matrix()
    model.gravity.linear = base_rotation.T @ gravity_world
    if model.nv < 1:
        raise RuntimeError(f'URDF contains no actuated joints (model.nv={model.nv})')
    return model, model.createData()


def configuration_from_joint_angles(
    model: Any, joint_angles: np.ndarray
) -> np.ndarray:
    """
    Convert physical angles to Pinocchio's joint configuration.

    Continuous URDF joints use [cos(q), sin(q)] internally. CSV files and the
    hardware adapter remain in unwrapped physical joint angles.
    """
    import pinocchio as pin

    angles = np.asarray(joint_angles, dtype=float).reshape(-1)
    if angles.size != model.nv:
        raise ValueError(f'Expected {model.nv} joint angles, got {angles.size}')
    return np.asarray(
        pin.integrate(model, pin.neutral(model), angles), dtype=float
    )


def torque_regressor(model, data, q, dq, ddq) -> np.ndarray:
    import pinocchio as pin

    configuration = configuration_from_joint_angles(model, q)
    return np.asarray(
        pin.computeJointTorqueRegressor(
            model,
            data,
            configuration,
            np.asarray(dq, dtype=float),
            np.asarray(ddq, dtype=float),
        ),
        dtype=float,
    ).copy()


def friction_regressor(dq: np.ndarray, config: dict) -> np.ndarray:
    velocity = np.asarray(dq, dtype=float).reshape(-1)
    joint_count = velocity.size
    if joint_count < 1:
        raise ValueError('At least one joint velocity is required')
    scale = np.asarray(
        config['friction']['transition_velocity_rad_s'], dtype=float
    )
    if scale.shape != (joint_count,) or np.any(scale <= 0.0):
        raise ValueError(
            'friction.transition_velocity_rad_s must contain one positive value per joint'
        )
    matrix = np.zeros((joint_count, 2 * joint_count), dtype=float)
    indices = np.arange(joint_count)
    matrix[indices, indices] = velocity
    matrix[indices, joint_count + indices] = np.tanh(velocity / scale)
    return matrix


def friction_parameter_labels() -> list[str]:
    # Kept for compatibility with callers that do not have a model handy.
    return []


def friction_parameter_labels_for_count(joint_count: int) -> list[str]:
    return (
        [f'joint{joint + 1}:Fv' for joint in range(joint_count)]
        + [f'joint{joint + 1}:Fc' for joint in range(joint_count)]
    )


def load_base_set(path: Path | str) -> tuple[np.ndarray, list[str]]:
    with np.load(Path(path)) as metadata:
        return (
            metadata['base_columns'].astype(int),
            metadata['base_parameter_labels'].astype(str).tolist(),
        )


def require_csv_columns(values: np.ndarray, required: tuple[str, ...], path: Path):
    names = values.dtype.names or ()
    missing = [name for name in required if name not in names]
    if missing:
        raise ValueError(
            f"{path} is missing CSV columns: {', '.join(missing)}"
        )
