from __future__ import annotations

import json
from pathlib import Path
import re

import numpy as np


def feature_names(joint_count):
    angles = []
    for joint in range(joint_count):
        angles.extend((f'sin_q{joint + 1}', f'cos_q{joint + 1}'))
    return tuple(
        angles
        + [f'dq{joint + 1}' for joint in range(joint_count)]
        + [f'ddq{joint + 1}' for joint in range(joint_count)]
    )


def data_columns(joint_count):
    return tuple(
        ['t']
        + [f'q{joint + 1}' for joint in range(joint_count)]
        + [f'dq{joint + 1}' for joint in range(joint_count)]
        + [f'ddq{joint + 1}' for joint in range(joint_count)]
        + [f'tau{joint + 1}' for joint in range(joint_count)]
    )


def _infer_joint_count(names):
    indices = sorted(
        int(match.group(1))
        for name in names
        if (match := re.fullmatch(r'q([1-9][0-9]*)', name))
    )
    if not indices or indices != list(range(1, indices[-1] + 1)):
        raise ValueError('dataset must contain contiguous q1..qN columns')
    return indices[-1]


def load_dataset(path, joint_count=None):
    source = Path(path).expanduser().resolve()
    values = np.genfromtxt(source, delimiter=',', names=True)
    names = values.dtype.names or ()
    count = int(joint_count or _infer_joint_count(names))
    required = data_columns(count)
    missing = [name for name in required if name not in names]
    if missing:
        raise ValueError(f'{source} is missing columns: {", ".join(missing)}')
    arrays = {
        name: np.atleast_1d(np.asarray(values[name], dtype=np.float64))
        for name in required
    }
    lengths = {array.size for array in arrays.values()}
    if len(lengths) != 1 or next(iter(lengths)) < 10:
        raise ValueError(f'{source} does not contain enough aligned samples')
    if not all(np.all(np.isfinite(array)) for array in arrays.values()):
        raise ValueError(f'{source} contains NaN or infinite values')
    q = np.column_stack([
        arrays[f'q{joint + 1}'] for joint in range(count)])
    dq = np.column_stack([
        arrays[f'dq{joint + 1}'] for joint in range(count)])
    ddq = np.column_stack([
        arrays[f'ddq{joint + 1}'] for joint in range(count)])
    tau = np.column_stack([
        arrays[f'tau{joint + 1}'] for joint in range(count)])
    return arrays['t'], q, dq, ddq, tau


def make_features(q, dq, ddq):
    position = np.asarray(q, dtype=np.float64)
    velocity = np.asarray(dq, dtype=np.float64)
    acceleration = np.asarray(ddq, dtype=np.float64)
    one_sample = position.ndim == 1
    position = np.atleast_2d(position)
    velocity = np.atleast_2d(velocity)
    acceleration = np.atleast_2d(acceleration)
    if not (
        position.shape == velocity.shape == acceleration.shape
        and position.shape[1] >= 1
    ):
        raise ValueError(
            'q, dq and ddq must have matching shape (samples, joints)')
    angle_features = []
    for joint in range(position.shape[1]):
        angle_features.extend((
            np.sin(position[:, joint]), np.cos(position[:, joint])))
    features = np.column_stack((*angle_features, velocity, acceleration))
    return features[0] if one_sample else features


class NeuralDynamicsModel:
    """Small NumPy MLP for model-dimension joint-torque prediction."""

    FORMAT_VERSION = 2

    def __init__(
        self,
        weights,
        biases,
        input_mean,
        input_std,
        output_mean,
        output_std,
        domain_min,
        domain_max,
        metadata=None,
    ):
        self.weights = [np.asarray(item, dtype=np.float64) for item in weights]
        self.biases = [np.asarray(item, dtype=np.float64) for item in biases]
        self.input_mean = np.asarray(input_mean, dtype=np.float64)
        self.input_std = np.asarray(input_std, dtype=np.float64)
        self.output_mean = np.asarray(output_mean, dtype=np.float64)
        self.output_std = np.asarray(output_std, dtype=np.float64)
        self.domain_min = np.asarray(domain_min, dtype=np.float64)
        self.domain_max = np.asarray(domain_max, dtype=np.float64)
        self.metadata = dict(metadata or {})
        self.joint_count = int(self.output_mean.size)
        self.feature_names = feature_names(self.joint_count)
        self._validate()

    def _validate(self):
        if self.joint_count < 1:
            raise ValueError('model must predict at least one joint')
        if not self.weights or len(self.weights) != len(self.biases):
            raise ValueError('model must contain matching weights and biases')
        expected_input = len(self.feature_names)
        for index, (weight, bias) in enumerate(zip(self.weights, self.biases)):
            if weight.ndim != 2 or bias.shape != (weight.shape[1],):
                raise ValueError(f'invalid neural-network layer {index}')
            if weight.shape[0] != expected_input:
                raise ValueError(f'incompatible neural-network layer {index}')
            expected_input = weight.shape[1]
        if expected_input != self.joint_count:
            raise ValueError(
                'neural-network output does not match the joint count')
        if self.input_mean.shape != (len(self.feature_names),):
            raise ValueError('invalid input normalization shape')
        if self.input_std.shape != self.input_mean.shape:
            raise ValueError('invalid input standard-deviation shape')
        if self.output_mean.shape != (self.joint_count,) or (
                self.output_std.shape != (self.joint_count,)):
            raise ValueError('invalid output normalization shape')
        if self.domain_min.shape != (3 * self.joint_count,) or (
                self.domain_max.shape != (3 * self.joint_count,)):
            raise ValueError('training domain must describe q, dq and ddq')
        arrays = (
            *self.weights, *self.biases, self.input_mean, self.input_std,
            self.output_mean, self.output_std, self.domain_min, self.domain_max,
        )
        if not all(np.all(np.isfinite(item)) for item in arrays):
            raise ValueError('model contains NaN or infinite values')
        if np.any(self.input_std <= 0.0) or np.any(self.output_std <= 0.0):
            raise ValueError('normalization standard deviations must be positive')

    def _predict_normalized(self, normalized_features):
        activation = np.asarray(normalized_features, dtype=np.float64)
        for weight, bias in zip(self.weights[:-1], self.biases[:-1]):
            activation = np.tanh(activation @ weight + bias)
        return activation @ self.weights[-1] + self.biases[-1]

    def predict_features(self, features):
        values = np.asarray(features, dtype=np.float64)
        one_sample = values.ndim == 1
        values = np.atleast_2d(values)
        if values.shape[1] != len(self.feature_names):
            raise ValueError(
                f'expected {len(self.feature_names)} input features')
        normalized = (values - self.input_mean) / self.input_std
        torque = (
            self._predict_normalized(normalized) * self.output_std
            + self.output_mean
        )
        return torque[0] if one_sample else torque

    def predict(self, q, dq, ddq):
        return self.predict_features(make_features(q, dq, ddq))

    def outside_training_domain(self, q, dq, ddq, tolerance=0.05):
        state = np.concatenate((
            np.asarray(q, dtype=float).reshape(self.joint_count),
            np.asarray(dq, dtype=float).reshape(self.joint_count),
            np.asarray(ddq, dtype=float).reshape(self.joint_count),
        ))
        span = np.maximum(self.domain_max - self.domain_min, 1e-9)
        margin = float(tolerance) * span
        return bool(np.any(state < self.domain_min - margin) or np.any(
            state > self.domain_max + margin))

    def save(self, path):
        target = Path(path).expanduser().resolve()
        target.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            'format_version': np.asarray(self.FORMAT_VERSION, dtype=np.int64),
            'joint_count': np.asarray(self.joint_count, dtype=np.int64),
            'feature_names': np.asarray(self.feature_names),
            'layer_count': np.asarray(len(self.weights), dtype=np.int64),
            'input_mean': self.input_mean,
            'input_std': self.input_std,
            'output_mean': self.output_mean,
            'output_std': self.output_std,
            'domain_min': self.domain_min,
            'domain_max': self.domain_max,
            'metadata_json': np.asarray(json.dumps(self.metadata)),
        }
        for index, (weight, bias) in enumerate(zip(self.weights, self.biases)):
            payload[f'weight_{index}'] = weight
            payload[f'bias_{index}'] = bias
        np.savez(target, **payload)

    @classmethod
    def load(cls, path):
        source = Path(path).expanduser().resolve()
        with np.load(source, allow_pickle=False) as values:
            version = int(np.asarray(values['format_version']).item())
            if version != cls.FORMAT_VERSION:
                raise ValueError(f'unsupported model format version {version}')
            joint_count = int(np.asarray(values['joint_count']).item())
            names = tuple(np.asarray(values['feature_names']).astype(str))
            if names != feature_names(joint_count):
                raise ValueError('model input features are incompatible')
            count = int(np.asarray(values['layer_count']).item())
            weights = [values[f'weight_{index}'].copy() for index in range(count)]
            biases = [values[f'bias_{index}'].copy() for index in range(count)]
            metadata = json.loads(str(np.asarray(values['metadata_json']).item()))
            return cls(
                weights, biases, values['input_mean'].copy(),
                values['input_std'].copy(), values['output_mean'].copy(),
                values['output_std'].copy(), values['domain_min'].copy(),
                values['domain_max'].copy(), metadata)
