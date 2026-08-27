#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
from pathlib import Path

from neural_dynamics_model import load_dataset, NeuralDynamicsModel
import numpy as np


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(
        description='Evaluate a trained neural inverse-dynamics model')
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--data', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--report', type=Path)
    return parser.parse_args(argv)


def metrics(measured, predicted):
    error = predicted - measured
    rmse = np.sqrt(np.mean(error**2, axis=0))
    scale = np.maximum(np.ptp(measured, axis=0), 1e-12)
    return {
        'rmse_per_joint_Nm': rmse.tolist(),
        'max_abs_error_per_joint_Nm': np.max(np.abs(error), axis=0).tolist(),
        'normalized_rmse_per_joint': (rmse / scale).tolist(),
    }


def main(argv=None):
    args = parse_arguments(argv)
    model = NeuralDynamicsModel.load(args.model)
    t, q, dq, ddq, measured = load_dataset(args.data)
    predicted = model.predict(q, dq, ddq)
    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(
        output,
        np.column_stack((t, measured, predicted, predicted - measured)),
        delimiter=',',
        header=(
            't,tau1_measured,tau2_measured,tau1_predicted,tau2_predicted,'
            'error1,error2'
        ),
        comments='',
    )
    report = {
        'model': str(args.model.expanduser().resolve()),
        'data': str(args.data.expanduser().resolve()),
        'samples': int(t.size),
        'metrics': metrics(measured, predicted),
    }
    if args.report:
        report_path = args.report.expanduser().resolve()
        report_path.parent.mkdir(parents=True, exist_ok=True)
        with report_path.open('w', encoding='utf-8') as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2)
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
