#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
from pathlib import Path

from neural_dynamics_model import (
    FEATURE_NAMES,
    load_dataset,
    make_features,
    NeuralDynamicsModel,
)
import numpy as np


def default_config_path():
    try:
        from ament_index_python.packages import get_package_share_directory

        installed = (
            Path(get_package_share_directory('exo_identify'))
            / 'neural_network' / 'training_config.json'
        )
        if installed.is_file():
            return installed
    except Exception:
        pass
    return Path(__file__).resolve().parent / 'training_config.json'


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(
        description='Train a NumPy MLP to predict total two-joint torque')
    parser.add_argument('--id-data', type=Path, nargs='+', required=True)
    parser.add_argument('--validation-data', type=Path, required=True)
    parser.add_argument(
        '--config', type=Path,
        default=default_config_path())
    parser.add_argument(
        '--output-dir', type=Path, default=Path('neural_identify_result'))
    return parser.parse_args(argv)


def load_config(path):
    with Path(path).expanduser().resolve().open('r', encoding='utf-8') as stream:
        values = json.load(stream)
    hidden = [int(value) for value in values['hidden_sizes']]
    if not hidden or any(value <= 0 for value in hidden):
        raise ValueError('hidden_sizes must contain positive layer sizes')
    for name in ('epochs', 'batch_size', 'patience', 'print_every'):
        if int(values[name]) <= 0:
            raise ValueError(f'{name} must be positive')
    for name in ('learning_rate', 'min_delta'):
        if float(values[name]) <= 0.0:
            raise ValueError(f'{name} must be positive')
    if float(values['l2_weight_decay']) < 0.0:
        raise ValueError('l2_weight_decay cannot be negative')
    for name in ('deployment_max_normalized_rmse', 'deployment_min_r2'):
        threshold = np.asarray(values[name], dtype=float)
        if threshold.shape != (2,) or not np.all(np.isfinite(threshold)):
            raise ValueError(f'{name} must contain two finite values')
    return values


def initialize_layers(rng, sizes):
    weights = []
    biases = []
    for fan_in, fan_out in zip(sizes[:-1], sizes[1:]):
        limit = np.sqrt(6.0 / (fan_in + fan_out))
        weights.append(rng.uniform(-limit, limit, size=(fan_in, fan_out)))
        biases.append(np.zeros(fan_out, dtype=np.float64))
    return weights, biases


def forward(features, weights, biases, cache=False):
    activation = features
    activations = [activation]
    for weight, bias in zip(weights[:-1], biases[:-1]):
        activation = np.tanh(activation @ weight + bias)
        activations.append(activation)
    output = activation @ weights[-1] + biases[-1]
    if cache:
        return output, activations
    return output


def gradients(features, target, weights, biases, weight_decay):
    prediction, activations = forward(features, weights, biases, cache=True)
    delta = 2.0 * (prediction - target) / prediction.size
    weight_gradients = [None] * len(weights)
    bias_gradients = [None] * len(biases)
    weight_gradients[-1] = (
        activations[-1].T @ delta + weight_decay * weights[-1])
    bias_gradients[-1] = np.sum(delta, axis=0)
    delta = delta @ weights[-1].T
    for index in range(len(weights) - 2, -1, -1):
        hidden = activations[index + 1]
        delta *= 1.0 - hidden**2
        weight_gradients[index] = (
            activations[index].T @ delta + weight_decay * weights[index])
        bias_gradients[index] = np.sum(delta, axis=0)
        if index:
            delta = delta @ weights[index].T
    return weight_gradients, bias_gradients


class Adam:
    def __init__(self, parameters, learning_rate):
        self.parameters = parameters
        self.learning_rate = float(learning_rate)
        self.first = [np.zeros_like(value) for value in parameters]
        self.second = [np.zeros_like(value) for value in parameters]
        self.step = 0

    def update(self, gradients):
        self.step += 1
        beta1 = 0.9
        beta2 = 0.999
        for index, (parameter, gradient) in enumerate(
                zip(self.parameters, gradients)):
            self.first[index] = (
                beta1 * self.first[index] + (1.0 - beta1) * gradient)
            self.second[index] = (
                beta2 * self.second[index] + (1.0 - beta2) * gradient**2)
            first = self.first[index] / (1.0 - beta1**self.step)
            second = self.second[index] / (1.0 - beta2**self.step)
            parameter -= self.learning_rate * first / (np.sqrt(second) + 1e-8)


def physical_metrics(measured, predicted):
    error = predicted - measured
    rmse = np.sqrt(np.mean(error**2, axis=0))
    scale = np.maximum(np.ptp(measured, axis=0), 1e-12)
    total = np.sum((measured - np.mean(measured, axis=0))**2, axis=0)
    residual = np.sum(error**2, axis=0)
    return {
        'rmse_per_joint_Nm': rmse.tolist(),
        'mae_per_joint_Nm': np.mean(np.abs(error), axis=0).tolist(),
        'max_abs_error_per_joint_Nm': np.max(np.abs(error), axis=0).tolist(),
        'normalized_rmse_per_joint': (rmse / scale).tolist(),
        'r2_per_joint': (1.0 - residual / np.maximum(total, 1e-12)).tolist(),
    }


def export_prediction(path, t, measured, predicted):
    error = predicted - measured
    np.savetxt(
        path,
        np.column_stack((t, measured, predicted, error)),
        delimiter=',',
        header=(
            't,tau1_measured,tau2_measured,tau1_predicted,tau2_predicted,'
            'error1,error2'
        ),
        comments='',
    )


def training_domain(q, dq, ddq):
    state = np.column_stack((q, dq, ddq))
    return np.min(state, axis=0), np.max(state, axis=0)


def main(argv=None):
    args = parse_arguments(argv)
    config = load_config(args.config)
    if any(
        path.resolve() == args.validation_data.resolve()
        for path in args.id_data
    ):
        raise ValueError('training and validation data must be different files')

    training_sets = [load_dataset(path) for path in args.id_data]
    train_t, train_q, train_dq, train_ddq, train_tau = (
        np.concatenate(items, axis=0) for items in zip(*training_sets)
    )
    valid_t, valid_q, valid_dq, valid_ddq, valid_tau = load_dataset(
        args.validation_data)
    train_features = make_features(train_q, train_dq, train_ddq)
    valid_features = make_features(valid_q, valid_dq, valid_ddq)

    input_mean = np.mean(train_features, axis=0)
    input_std = np.maximum(np.std(train_features, axis=0), 1e-8)
    output_mean = np.mean(train_tau, axis=0)
    output_std = np.maximum(np.std(train_tau, axis=0), 1e-8)
    train_x = (train_features - input_mean) / input_std
    valid_x = (valid_features - input_mean) / input_std
    train_y = (train_tau - output_mean) / output_std
    valid_y = (valid_tau - output_mean) / output_std

    rng = np.random.default_rng(int(config['random_seed']))
    sizes = [len(FEATURE_NAMES), *config['hidden_sizes'], 2]
    weights, biases = initialize_layers(rng, sizes)
    optimizer = Adam(
        [*weights, *biases], float(config['learning_rate']))
    batch_size = int(config['batch_size'])
    patience = int(config['patience'])
    min_delta = float(config['min_delta'])
    weight_decay = float(config['l2_weight_decay'])
    best_loss = np.inf
    best_epoch = 0
    best_weights = None
    best_biases = None
    epochs_without_improvement = 0
    history = []

    for epoch in range(1, int(config['epochs']) + 1):
        order = rng.permutation(train_x.shape[0])
        for start in range(0, train_x.shape[0], batch_size):
            indices = order[start:start + batch_size]
            weight_gradients, bias_gradients = gradients(
                train_x[indices], train_y[indices], weights, biases,
                weight_decay)
            optimizer.update([*weight_gradients, *bias_gradients])
        train_prediction = forward(train_x, weights, biases)
        valid_prediction = forward(valid_x, weights, biases)
        train_loss = float(np.mean((train_prediction - train_y)**2))
        valid_loss = float(np.mean((valid_prediction - valid_y)**2))
        history.append((epoch, train_loss, valid_loss))
        if valid_loss < best_loss - min_delta:
            best_loss = valid_loss
            best_epoch = epoch
            best_weights = [value.copy() for value in weights]
            best_biases = [value.copy() for value in biases]
            epochs_without_improvement = 0
        else:
            epochs_without_improvement += 1
        if epoch == 1 or epoch % int(config['print_every']) == 0:
            print(
                f'epoch={epoch} train_mse={train_loss:.8f} '
                f'validation_mse={valid_loss:.8f} best_epoch={best_epoch}',
                flush=True,
            )
        if epochs_without_improvement >= patience:
            print(f'early stopping at epoch {epoch}', flush=True)
            break

    if best_weights is None:
        raise RuntimeError('training failed to produce a finite model')
    domain_min, domain_max = training_domain(train_q, train_dq, train_ddq)
    metadata = {
        'model_type': 'MLP',
        'activation': 'tanh',
        'hidden_sizes': list(config['hidden_sizes']),
        'input_definition': (
            '[sin(q1),cos(q1),sin(q2),cos(q2),dq1,dq2,ddq1,ddq2]'
        ),
        'output_definition': '[tau1,tau2] in URDF joint coordinates and N.m',
        'coordinate_requirement': (
            'q/dq/ddq/tau use URDF joint signs, SI units, zero offsets, and '
            'output-shaft quantities'
        ),
        'training_data': [str(path.resolve()) for path in args.id_data],
        'validation_data': str(args.validation_data.resolve()),
        'best_epoch': best_epoch,
    }
    model = NeuralDynamicsModel(
        best_weights,
        best_biases,
        input_mean,
        input_std,
        output_mean,
        output_std,
        domain_min,
        domain_max,
        metadata,
    )
    train_prediction = model.predict(train_q, train_dq, train_ddq)
    valid_prediction = model.predict(valid_q, valid_dq, valid_ddq)
    output = args.output_dir.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    model.save(output / 'neural_dynamics_model.npz')
    export_prediction(
        output / 'prediction_id.csv', train_t, train_tau, train_prediction)
    export_prediction(
        output / 'prediction_validation.csv', valid_t, valid_tau,
        valid_prediction)
    np.savetxt(
        output / 'training_history.csv', np.asarray(history), delimiter=',',
        header='epoch,train_normalized_mse,validation_normalized_mse',
        comments='')
    training_metrics = physical_metrics(train_tau, train_prediction)
    validation_metrics = physical_metrics(valid_tau, valid_prediction)
    maximum_nrmse = np.asarray(
        config['deployment_max_normalized_rmse'], dtype=float)
    minimum_r2 = np.asarray(config['deployment_min_r2'], dtype=float)
    deployment_recommended = bool(
        np.all(np.asarray(
            validation_metrics['normalized_rmse_per_joint']) <= maximum_nrmse)
        and np.all(np.asarray(
            validation_metrics['r2_per_joint']) >= minimum_r2)
    )
    report = {
        **metadata,
        'training_samples': int(train_t.size),
        'validation_samples': int(valid_t.size),
        'feature_names': list(FEATURE_NAMES),
        'training_domain': {
            'labels': ['q1', 'q2', 'dq1', 'dq2', 'ddq1', 'ddq2'],
            'minimum': domain_min.tolist(),
            'maximum': domain_max.tolist(),
        },
        'best_validation_normalized_mse': best_loss,
        'epochs_completed': int(history[-1][0]),
        'training_metrics': training_metrics,
        'validation_metrics': validation_metrics,
        'deployment_gate': {
            'maximum_normalized_rmse_per_joint': maximum_nrmse.tolist(),
            'minimum_r2_per_joint': minimum_r2.tolist(),
            'deployment_recommended': deployment_recommended,
            'warning': (
                'Do not use this model for real-time torque control unless '
                'deployment_recommended is true and hardware safety tests pass.'
            ),
        },
    }
    with (output / 'training_report.json').open(
            'w', encoding='utf-8') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
