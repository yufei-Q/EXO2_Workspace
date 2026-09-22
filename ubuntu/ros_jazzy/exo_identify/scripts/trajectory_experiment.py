#!/usr/bin/env python3

from __future__ import annotations

import csv
from datetime import datetime
import json
from pathlib import Path

from common import (
    RESULTS_ROOT,
    default_hardware_mapping_path,
    load_hardware_mapping,
    process_signals,
    ProcessingConfig,
    save_processed,
)
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


MOTOR_COUNT = 7


def trajectory_columns(joint_count):
    return (
        ['t']
        + [f'q{joint + 1}' for joint in range(joint_count)]
        + [f'dq{joint + 1}' for joint in range(joint_count)]
        + [f'ddq{joint + 1}' for joint in range(joint_count)]
    )


class TrajectoryExperimentNode(Node):
    """Track a seven-joint excitation and record synchronized feedback."""

    def __init__(self):
        super().__init__('exo_trajectory_experiment')
        self._declare_parameters()
        self._load_parameters()
        self.trajectory = self._load_trajectory(
            self.trajectory_file, self.joint_count)
        self._scale_trajectory()
        self._validate_trajectory()

        self.feedback = None
        self.feedback_receive_time = None
        self.last_feedback_warning = None
        self.state = 'idle'
        self.hold_q = np.zeros(self.joint_count)
        self.desired_q = np.zeros(self.joint_count)
        self.desired_dq = np.zeros(self.joint_count)
        self.desired_ddq = np.zeros(self.joint_count)
        self.segment_start = None
        self.segment_coefficients = None
        self.excitation_start = None
        self.records = []
        self.disable_deadline = None

        self.command_pub = self.create_publisher(
            JointState, '/dm_motor_usb/command', 1)
        self.enable_pub = self.create_publisher(
            Bool, '/dm_motor_usb/enable', 1)
        self.create_subscription(
            JointState, '/dm_motor_usb/feedback', self.feedback_callback, 20)
        self.create_service(
            Trigger, '/exo_identify/prepare', self.prepare_callback)
        self.create_service(
            Trigger, '/exo_identify/start', self.start_callback)
        self.create_service(
            Trigger, '/exo_identify/stop', self.stop_callback)
        self.create_service(
            Trigger, '/exo_identify/emergency_stop',
            self.emergency_stop_callback)
        self.timer = self.create_timer(1.0 / self.command_rate, self.timer_callback)

        self.get_logger().info(
            f'Loaded {self.trajectory_file} ({self.trajectory["t"][-1]:.2f} s); '
            f'joint motor indices={self.motor_indices}')

    def _declare_parameters(self):
        self.declare_parameter(
            'trajectory_file',
            str(RESULTS_ROOT / 'seven_dof/excitation/excitation_id.csv'))
        self.declare_parameter(
            'output_directory',
            str(RESULTS_ROOT / 'seven_dof/hardware_experiments'))
        self.declare_parameter(
            'hardware_mapping_file', str(default_hardware_mapping_path()))
        self.declare_parameter('joint_lower_limits', [-3.1415926536] * MOTOR_COUNT)
        self.declare_parameter('joint_upper_limits', [3.1415926536] * MOTOR_COUNT)
        self.declare_parameter('joint_velocity_limits', [2.5] * MOTOR_COUNT)
        self.declare_parameter('motor_position_limits', [12.0] * MOTOR_COUNT)
        self.declare_parameter('excitation_amplitude_scale', [0.2] * MOTOR_COUNT)
        self.declare_parameter('excitation_time_scale', 4.0)
        self.declare_parameter('command_rate', 500.0)
        self.declare_parameter('transition_duration', 5.0)
        self.declare_parameter('stop_duration', 2.0)
        self.declare_parameter('hold_before_start', 0.5)
        self.declare_parameter('hold_after_stop', 0.5)
        self.declare_parameter('feedback_timeout', 0.1)
        self.declare_parameter('processing_filter_rate', 500.0)
        self.declare_parameter('processing_output_rate', 100.0)
        self.declare_parameter('processing_cutoff_frequency', 0.5)
        self.declare_parameter('processing_filter_order', 4)
        self.declare_parameter('processing_edge_trim', 2.0)
        self.declare_parameter('auto_enable_on_prepare', False)
        self.declare_parameter('disable_on_finish', True)

    def _array_parameter(self, name, length=MOTOR_COUNT, dtype=float):
        values = list(self.get_parameter(name).value)
        if len(values) != length:
            raise ValueError(f'{name} must contain exactly {length} values')
        return np.asarray(values, dtype=dtype)

    @staticmethod
    def _mapping_array(mapping, name, length=MOTOR_COUNT, dtype=float):
        values = list(mapping[name])
        if len(values) != length:
            raise ValueError(
                f'hardware_mapping.{name} must contain exactly {length} values')
        try:
            result = np.asarray(values, dtype=dtype)
        except (TypeError, ValueError) as error:
            raise ValueError(
                f'hardware_mapping.{name} must contain numeric values') from error
        if not np.all(np.isfinite(result)):
            raise ValueError(f'hardware_mapping.{name} contains non-finite values')
        return result

    def _load_parameters(self):
        path = str(self.get_parameter('trajectory_file').value)
        if not path:
            raise ValueError('trajectory_file must be an absolute or relative CSV path')
        self.trajectory_file = Path(path).expanduser().resolve()
        output_directory = Path(
            str(self.get_parameter('output_directory').value)
        ).expanduser()
        if not output_directory.is_absolute():
            output_directory = RESULTS_ROOT / output_directory
        self.output_directory = output_directory.resolve()
        mapping = load_hardware_mapping(
            self.get_parameter('hardware_mapping_file').value)
        self.motor_indices = self._mapping_array(
            mapping, 'motor_indices', dtype=int).tolist()
        self.joint_count = len(self.motor_indices)
        if self.joint_count != MOTOR_COUNT or len(set(self.motor_indices)) != MOTOR_COUNT or any(
                index < 0 or index >= MOTOR_COUNT for index in self.motor_indices):
            raise ValueError('motor_indices must contain all seven distinct values from 0 to 6')
        self.directions = self._mapping_array(mapping, 'joint_directions')
        if not np.all(np.isin(self.directions, (-1.0, 1.0))):
            raise ValueError('joint_directions values must be +1.0 or -1.0')
        self.motor_zero = self._mapping_array(mapping, 'motor_zero_positions')
        self.position_scale = self._mapping_array(
            mapping, 'motor_position_per_joint_radian')
        self.torque_scale = self._mapping_array(
            mapping, 'joint_torque_per_motor_torque')
        self.joint_lower = self._array_parameter('joint_lower_limits')
        self.joint_upper = self._array_parameter('joint_upper_limits')
        self.velocity_limits = self._array_parameter('joint_velocity_limits')
        self.motor_position_limits = self._array_parameter('motor_position_limits')
        self.excitation_amplitude_scale = self._array_parameter(
            'excitation_amplitude_scale')
        self.excitation_time_scale = float(
            self.get_parameter('excitation_time_scale').value)
        if np.any(self.position_scale <= 0.0) or np.any(self.torque_scale <= 0.0):
            raise ValueError('position and torque scale values must be positive')
        if np.any(self.joint_lower >= self.joint_upper):
            raise ValueError('each joint lower limit must be below its upper limit')
        if np.any(self.velocity_limits <= 0.0):
            raise ValueError('joint_velocity_limits must be positive')
        if (np.any(self.excitation_amplitude_scale <= 0.0)
                or np.any(self.excitation_amplitude_scale > 1.0)):
            raise ValueError('excitation_amplitude_scale must be in (0, 1]')
        if self.excitation_time_scale < 1.0:
            raise ValueError('excitation_time_scale must be at least 1.0')

        self.command_rate = float(self.get_parameter('command_rate').value)
        self.transition_duration = float(
            self.get_parameter('transition_duration').value)
        self.stop_duration = float(self.get_parameter('stop_duration').value)
        self.hold_before_start = float(
            self.get_parameter('hold_before_start').value)
        self.hold_after_stop = float(
            self.get_parameter('hold_after_stop').value)
        self.feedback_timeout = float(
            self.get_parameter('feedback_timeout').value)
        self.processing_config = ProcessingConfig(
            filter_rate=float(
                self.get_parameter('processing_filter_rate').value),
            output_rate=float(
                self.get_parameter('processing_output_rate').value),
            cutoff_frequency=float(
                self.get_parameter('processing_cutoff_frequency').value),
            filter_order=int(
                self.get_parameter('processing_filter_order').value),
            edge_trim=float(
                self.get_parameter('processing_edge_trim').value),
        )
        self.processing_config.validate()
        self.auto_enable = bool(
            self.get_parameter('auto_enable_on_prepare').value)
        self.disable_on_finish = bool(
            self.get_parameter('disable_on_finish').value)
        if self.command_rate <= 0.0 or self.transition_duration <= 0.0:
            raise ValueError('command_rate and transition_duration must be positive')
        if self.stop_duration <= 0.0:
            raise ValueError('stop_duration must be positive')

    @staticmethod
    def _load_trajectory(path, joint_count):
        values = np.genfromtxt(path, delimiter=',', names=True)
        names = values.dtype.names or ()
        required = trajectory_columns(joint_count)
        missing = [name for name in required if name not in names]
        if missing:
            raise ValueError(f'{path} is missing columns: {", ".join(missing)}')
        result = {
            name: np.atleast_1d(np.asarray(values[name], dtype=float))
            for name in required
        }
        if len(result['t']) < 2:
            raise ValueError('trajectory must contain at least two samples')
        if not all(np.all(np.isfinite(item)) for item in result.values()):
            raise ValueError('trajectory contains NaN or infinite values')
        if np.any(np.diff(result['t']) <= 0.0):
            raise ValueError('trajectory time must be strictly increasing')
        result['t'] -= result['t'][0]
        return result

    def _scale_trajectory(self):
        scale = self.excitation_amplitude_scale
        time_scale = self.excitation_time_scale
        self.trajectory['t'] *= time_scale
        for joint in range(self.joint_count):
            suffix = str(joint + 1)
            self.trajectory[f'q{suffix}'] *= scale[joint]
            self.trajectory[f'dq{suffix}'] *= scale[joint] / time_scale
            self.trajectory[f'ddq{suffix}'] *= scale[joint] / time_scale**2

    def _validate_trajectory(self):
        q = np.column_stack([
            self.trajectory[f'q{joint + 1}']
            for joint in range(self.joint_count)
        ])
        dq = np.column_stack([
            self.trajectory[f'dq{joint + 1}']
            for joint in range(self.joint_count)
        ])
        if (np.any(q < self.joint_lower)
                or np.any(q > self.joint_upper)):
            raise ValueError('trajectory exceeds configured joint position limits')
        if np.any(np.abs(dq) > self.velocity_limits):
            raise ValueError('trajectory exceeds configured joint velocity limits')
        motor_q = self.motor_zero + self.directions * self.position_scale * q
        if np.any(np.abs(motor_q) > self.motor_position_limits):
            raise ValueError('mapped trajectory exceeds motor position limits')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _feedback_is_fresh(self):
        return (
            self.feedback_receive_time is not None
            and self._now() - self.feedback_receive_time <= self.feedback_timeout
        )

    def feedback_callback(self, message):
        if min(len(message.position), len(message.velocity), len(message.effort)) < MOTOR_COUNT:
            self.get_logger().warning('Ignoring incomplete motor feedback')
            return
        feedback_arrays = (
            np.asarray(message.position[:MOTOR_COUNT], dtype=float),
            np.asarray(message.velocity[:MOTOR_COUNT], dtype=float),
            np.asarray(message.effort[:MOTOR_COUNT], dtype=float),
        )
        if not all(np.all(np.isfinite(item)) for item in feedback_arrays):
            now = self._now()
            if (self.last_feedback_warning is None
                    or now - self.last_feedback_warning > 1.0):
                self.last_feedback_warning = now
                self.get_logger().warning(
                    'Ignoring motor feedback containing NaN or infinite values')
            return
        self.feedback = message
        self.feedback_receive_time = self._now()
        if self.state == 'excitation':
            self._record_feedback(message)

    def _joint_feedback(self, message=None):
        message = message or self.feedback
        indices = self.motor_indices
        motor_q = np.asarray([message.position[index] for index in indices])
        motor_dq = np.asarray([message.velocity[index] for index in indices])
        motor_tau = np.asarray([message.effort[index] for index in indices])
        q = self.directions * (motor_q - self.motor_zero) / self.position_scale
        dq = self.directions * motor_dq / self.position_scale
        tau = self.directions * self.torque_scale * motor_tau
        return motor_q, motor_dq, motor_tau, q, dq, tau

    def _publish_enable(self, enabled):
        message = Bool()
        message.data = bool(enabled)
        self.enable_pub.publish(message)

    def _command_violation(self, q, dq):
        position = np.asarray(q, dtype=float).reshape(-1)
        velocity = np.asarray(dq, dtype=float).reshape(-1)
        if position.shape != (self.joint_count,) or velocity.shape != (
                self.joint_count,):
            return 'command has an invalid joint dimension'
        if not np.all(np.isfinite(position)) or not np.all(np.isfinite(velocity)):
            return 'command contains NaN or infinite values'
        if np.any(position < self.joint_lower) or np.any(
                position > self.joint_upper):
            return 'command exceeded a joint position limit'
        if np.any(np.abs(velocity) > self.velocity_limits):
            return 'command exceeded a joint velocity limit'
        motor_q = (
            self.motor_zero + self.directions * self.position_scale * position)
        if np.any(np.abs(motor_q) > self.motor_position_limits):
            return 'command exceeded a motor position limit'
        return None

    def _emergency_disable(self, reason):
        self._publish_enable(False)
        self.state = 'idle'
        self.segment_coefficients = None
        self.segment_start = None
        self.disable_deadline = None
        self.desired_dq.fill(0.0)
        self.desired_ddq.fill(0.0)
        self.get_logger().error(f'{reason}; motors disabled immediately')

    def _publish_joint_command(self, q, dq):
        violation = self._command_violation(q, dq)
        if violation is not None:
            self._emergency_disable(violation)
            return False
        motor_q = self.motor_zero + self.directions * self.position_scale * q
        motor_dq = self.directions * self.position_scale * dq
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.position = [0.0] * MOTOR_COUNT
        message.velocity = [0.0] * MOTOR_COUNT
        message.effort = [0.0] * MOTOR_COUNT
        for joint in range(self.joint_count):
            motor = self.motor_indices[joint]
            message.position[motor] = float(motor_q[joint])
            message.velocity[motor] = float(motor_dq[joint])
        self.command_pub.publish(message)
        return True

    def prepare_callback(self, _request, response):
        if not self._feedback_is_fresh():
            response.success = False
            response.message = 'no fresh /dm_motor_usb/feedback'
            return response
        if self.state not in ('idle', 'ready'):
            response.success = False
            response.message = f'cannot prepare while state={self.state}'
            return response
        _, _, _, q, dq, _ = self._joint_feedback()
        violation = self._command_violation(q, dq)
        if violation is not None:
            self._emergency_disable(violation)
            response.success = False
            response.message = violation
            return response
        self.hold_q = q.copy()
        self.desired_q = q.copy()
        self.desired_dq.fill(0.0)
        self.desired_ddq.fill(0.0)
        self.state = 'ready'
        if not self._publish_joint_command(
                self.hold_q, np.zeros(self.joint_count)):
            response.success = False
            response.message = 'current pose cannot be held within safety limits'
            return response
        if self.auto_enable:
            self._publish_enable(True)
        response.success = True
        response.message = 'holding current pose; enable motors, then call start'
        return response

    def start_callback(self, _request, response):
        if self.state != 'ready':
            response.success = False
            response.message = 'call /exo_identify/prepare first'
            return response
        if not self._feedback_is_fresh():
            response.success = False
            response.message = 'motor feedback is stale'
            return response
        _, _, _, q, dq, _ = self._joint_feedback()
        target_q, target_dq, target_ddq = self._trajectory_sample(0.0)
        coefficients = self._quintic(
            q, dq, np.zeros(self.joint_count), target_q, target_dq, target_ddq,
            self.transition_duration)
        violation = self._segment_violation(
            coefficients, self.transition_duration)
        if violation is not None:
            self._emergency_disable(
                f'unsafe start transition: {violation}')
            response.success = False
            response.message = f'start transition rejected: {violation}'
            return response
        self.segment_coefficients = coefficients
        self.segment_start = self._now() + self.hold_before_start
        self.records = []
        self.state = 'transition_wait'
        response.success = True
        response.message = 'smooth transition and excitation scheduled'
        return response

    def stop_callback(self, _request, response):
        if self.state in ('idle', 'ready', 'hold'):
            self._publish_enable(False)
            self.state = 'idle'
            response.success = True
            response.message = 'motors disabled'
            return response
        response.success = self._begin_stop()
        response.message = (
            'smooth stop started' if response.success
            else 'smooth stop was unsafe; motors disabled immediately')
        return response

    def emergency_stop_callback(self, _request, response):
        self._emergency_disable('emergency stop requested')
        response.success = True
        response.message = 'motors disabled immediately'
        return response

    @staticmethod
    def _quintic(q0, dq0, ddq0, q1, dq1, ddq1, duration):
        joint_count = np.asarray(q0).size
        coefficients = np.zeros((joint_count, 6))
        coefficients[:, 0] = q0
        coefficients[:, 1] = dq0
        coefficients[:, 2] = 0.5 * ddq0
        t = duration
        matrix = np.asarray([
            [t**3, t**4, t**5],
            [3 * t**2, 4 * t**3, 5 * t**4],
            [6 * t, 12 * t**2, 20 * t**3],
        ])
        for joint in range(joint_count):
            rhs = np.asarray([
                q1[joint] - coefficients[joint, 0]
                - coefficients[joint, 1] * t
                - coefficients[joint, 2] * t**2,
                dq1[joint] - coefficients[joint, 1]
                - 2 * coefficients[joint, 2] * t,
                ddq1[joint] - 2 * coefficients[joint, 2],
            ])
            coefficients[joint, 3:] = np.linalg.solve(matrix, rhs)
        return coefficients

    @staticmethod
    def _evaluate_quintic(coefficients, elapsed):
        powers = np.asarray([1, elapsed, elapsed**2, elapsed**3,
                             elapsed**4, elapsed**5])
        derivative = np.asarray([0, 1, 2 * elapsed, 3 * elapsed**2,
                                 4 * elapsed**3, 5 * elapsed**4])
        second = np.asarray([0, 0, 2, 6 * elapsed, 12 * elapsed**2,
                             20 * elapsed**3])
        return (
            coefficients @ powers,
            coefficients @ derivative,
            coefficients @ second,
        )

    def _segment_violation(self, coefficients, duration):
        sample_count = max(2, int(np.ceil(duration * self.command_rate)) + 1)
        for elapsed in np.linspace(0.0, duration, sample_count):
            q, dq, _ = self._evaluate_quintic(coefficients, elapsed)
            violation = self._command_violation(q, dq)
            if violation is not None:
                return f'{violation} at t={elapsed:.6f} s'
        return None

    def _trajectory_sample(self, elapsed):
        t = self.trajectory['t']
        return tuple(
            np.asarray([
                np.interp(
                    elapsed, t, self.trajectory[f'{prefix}{joint + 1}'])
                for joint in range(self.joint_count)
            ])
            for prefix in ('q', 'dq', 'ddq')
        )

    def timer_callback(self):
        now = self._now()
        if self.state in ('idle',):
            return
        if not self._feedback_is_fresh():
            self._emergency_disable('feedback timeout')
            return
        if self.state == 'ready':
            if not self._publish_joint_command(
                    self.hold_q, np.zeros(self.joint_count)):
                return
            return
        if self.state == 'transition_wait':
            if not self._publish_joint_command(
                    self.hold_q, np.zeros(self.joint_count)):
                return
            if now >= self.segment_start:
                self.state = 'transition'
            return
        if self.state in ('transition', 'stopping'):
            elapsed = max(0.0, now - self.segment_start)
            duration = (
                self.transition_duration if self.state == 'transition'
                else self.stop_duration)
            q, dq, ddq = self._evaluate_quintic(
                self.segment_coefficients, min(elapsed, duration))
            self.desired_q, self.desired_dq, self.desired_ddq = q, dq, ddq
            if not self._publish_joint_command(q, dq):
                return
            if elapsed >= duration:
                if self.state == 'transition':
                    self.excitation_start = now
                    self.state = 'excitation'
                    self.get_logger().info('Excitation started')
                else:
                    self._finish_hold(q)
            return
        if self.state == 'excitation':
            elapsed = now - self.excitation_start
            duration = self.trajectory['t'][-1]
            q, dq, ddq = self._trajectory_sample(min(elapsed, duration))
            self.desired_q, self.desired_dq, self.desired_ddq = q, dq, ddq
            if not self._publish_joint_command(q, dq):
                return
            if elapsed >= duration:
                self._begin_stop()
            return
        if self.state == 'hold':
            if not self._publish_joint_command(
                    self.hold_q, np.zeros(self.joint_count)):
                return
            if self.disable_deadline is not None and now >= self.disable_deadline:
                if self.disable_on_finish:
                    self._publish_enable(False)
                self.state = 'idle'
                self.get_logger().info('Experiment finished')

    def _begin_stop(self):
        if self.state == 'stopping':
            return True
        if self.state in ('hold', 'idle'):
            return False
        target = self.desired_q + 0.5 * self.stop_duration * self.desired_dq
        target = np.minimum(np.maximum(target, self.joint_lower), self.joint_upper)
        coefficients = self._quintic(
            self.desired_q, self.desired_dq, self.desired_ddq,
            target, np.zeros(self.joint_count), np.zeros(self.joint_count),
            self.stop_duration)
        violation = self._segment_violation(coefficients, self.stop_duration)
        if violation is not None:
            self._emergency_disable(f'unsafe smooth stop: {violation}')
            return False
        self.segment_coefficients = coefficients
        self.segment_start = self._now()
        self.state = 'stopping'
        self.get_logger().info('Smooth stop started')
        return True

    def _finish_hold(self, q):
        self.hold_q = q.copy()
        self.desired_q = q.copy()
        self.desired_dq.fill(0.0)
        self.desired_ddq.fill(0.0)
        self.state = 'hold'
        self.disable_deadline = self._now() + self.hold_after_stop
        self._write_records()

    def _record_feedback(self, message):
        motor_q, motor_dq, motor_tau, q, dq, tau = self._joint_feedback(message)
        self.records.append([
            self._now() - self.excitation_start,
            *self.desired_q, *self.desired_dq, *self.desired_ddq,
            *motor_q, *motor_dq, *motor_tau, *q, *dq, *tau,
        ])

    def _write_records(self):
        if len(self.records) < 3:
            self.get_logger().warning('Too few excitation samples to save')
            return
        values = np.asarray(self.records, dtype=float)
        keep = np.r_[True, np.diff(values[:, 0]) > 1e-6]
        values = values[keep]
        run_directory = self.output_directory / datetime.now().strftime(
            'run_%Y%m%d_%H%M%S')
        run_directory.mkdir(parents=True, exist_ok=False)
        raw_header = ['t']
        raw_header += [f'q{joint + 1}_des' for joint in range(self.joint_count)]
        raw_header += [f'dq{joint + 1}_des' for joint in range(self.joint_count)]
        raw_header += [f'ddq{joint + 1}_des' for joint in range(self.joint_count)]
        raw_header += [f'motor{joint + 1}_q' for joint in range(self.joint_count)]
        raw_header += [f'motor{joint + 1}_dq' for joint in range(self.joint_count)]
        raw_header += [f'motor{joint + 1}_tau' for joint in range(self.joint_count)]
        raw_header += [f'q{joint + 1}' for joint in range(self.joint_count)]
        raw_header += [f'dq{joint + 1}' for joint in range(self.joint_count)]
        raw_header += [f'tau{joint + 1}' for joint in range(self.joint_count)]
        with (run_directory / 'experiment_raw.csv').open(
                'w', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream)
            writer.writerow(raw_header)
            writer.writerows(values)

        t = values[:, 0]
        signal_start = 1 + 6 * self.joint_count
        actual_q = values[:, signal_start:signal_start + self.joint_count]
        actual_dq = values[
            :, signal_start + self.joint_count:signal_start + 2 * self.joint_count]
        actual_tau = values[
            :, signal_start + 2 * self.joint_count:signal_start + 3 * self.joint_count]
        range_summary = {}
        for joint in range(self.joint_count):
            position = actual_q[:, joint]
            range_summary[f'joint{joint + 1}'] = {
                'minimum_rad': float(np.min(position)),
                'maximum_rad': float(np.max(position)),
                'peak_to_peak_rad': float(np.ptp(position)),
                'minimum_deg': float(np.rad2deg(np.min(position))),
                'maximum_deg': float(np.rad2deg(np.max(position))),
            }

        processed, processing_report = process_signals(
            t,
            actual_q,
            actual_dq,
            actual_tau,
            self.processing_config,
        )
        save_processed(run_directory / 'measured_id.csv', processed)
        metadata = {
            'trajectory_file': str(self.trajectory_file),
            'motor_indices_zero_based': self.motor_indices,
            'joint_directions': self.directions.tolist(),
            'motor_zero_positions': self.motor_zero.tolist(),
            'motor_position_per_joint_radian': self.position_scale.tolist(),
            'joint_torque_per_motor_torque': self.torque_scale.tolist(),
            'samples': int(len(values)),
            'excitation_amplitude_scale': self.excitation_amplitude_scale.tolist(),
            'excitation_time_scale': self.excitation_time_scale,
            'actual_range_summary': range_summary,
            'processing': processing_report,
        }
        with (run_directory / 'metadata.json').open('w', encoding='utf-8') as stream:
            json.dump(metadata, stream, ensure_ascii=False, indent=2)
        self.get_logger().info(f'Experiment data saved to {run_directory}')

    def destroy_node(self):
        if rclpy.ok():
            self._publish_enable(False)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TrajectoryExperimentNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (OSError, ValueError) as error:
        if node is None:
            print(f'experiment node startup failed: {error}')
        else:
            node.get_logger().fatal(str(error))
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
