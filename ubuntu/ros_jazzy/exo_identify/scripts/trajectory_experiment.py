#!/usr/bin/env python3

from __future__ import annotations

import csv
from datetime import datetime
import json
from pathlib import Path

from common import process_signals, ProcessingConfig, save_processed
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


MOTOR_COUNT = 7
TRAJECTORY_COLUMNS = ('t', 'q1', 'q2', 'dq1', 'dq2', 'ddq1', 'ddq2')


class TrajectoryExperimentNode(Node):
    """Track a two-joint excitation and record synchronized motor feedback."""

    def __init__(self):
        super().__init__('exo_trajectory_experiment')
        self._declare_parameters()
        self._load_parameters()
        self.trajectory = self._load_trajectory(self.trajectory_file)
        self._scale_trajectory()
        self._validate_trajectory()

        self.feedback = None
        self.feedback_receive_time = None
        self.state = 'idle'
        self.hold_q = np.zeros(2)
        self.desired_q = np.zeros(2)
        self.desired_dq = np.zeros(2)
        self.desired_ddq = np.zeros(2)
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
        self.timer = self.create_timer(1.0 / self.command_rate, self.timer_callback)

        self.get_logger().info(
            f'Loaded {self.trajectory_file} ({self.trajectory["t"][-1]:.2f} s); '
            f'joint motor indices={self.motor_indices}')

    def _declare_parameters(self):
        self.declare_parameter('trajectory_file', '')
        self.declare_parameter('output_directory', 'experiment_output')
        self.declare_parameter('motor_indices', [0, 6])
        self.declare_parameter('joint_directions', [1.0, 1.0])
        self.declare_parameter('motor_zero_positions', [0.0, 0.0])
        self.declare_parameter('motor_position_per_joint_radian', [1.0, 1.0])
        self.declare_parameter('joint_torque_per_motor_torque', [1.0, 1.0])
        self.declare_parameter('joint_lower_limits', [-3.1415926536] * 2)
        self.declare_parameter('joint_upper_limits', [3.1415926536] * 2)
        self.declare_parameter('joint_velocity_limits', [2.5, 2.5])
        self.declare_parameter('motor_position_limits', [12.0, 12.0])
        self.declare_parameter('excitation_amplitude_scale', [0.2, 0.2])
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

    def _array_parameter(self, name, length=2, dtype=float):
        values = list(self.get_parameter(name).value)
        if len(values) != length:
            raise ValueError(f'{name} must contain exactly {length} values')
        return np.asarray(values, dtype=dtype)

    def _load_parameters(self):
        path = str(self.get_parameter('trajectory_file').value)
        if not path:
            raise ValueError('trajectory_file must be an absolute or relative CSV path')
        self.trajectory_file = Path(path).expanduser().resolve()
        self.output_directory = Path(
            str(self.get_parameter('output_directory').value)
        ).expanduser().resolve()
        self.motor_indices = self._array_parameter(
            'motor_indices', dtype=int).tolist()
        if len(set(self.motor_indices)) != 2 or any(
                index < 0 or index >= MOTOR_COUNT for index in self.motor_indices):
            raise ValueError('motor_indices must be two different values from 0 to 6')
        self.directions = self._array_parameter('joint_directions')
        if not np.all(np.isin(self.directions, (-1.0, 1.0))):
            raise ValueError('joint_directions values must be +1.0 or -1.0')
        self.motor_zero = self._array_parameter('motor_zero_positions')
        self.position_scale = self._array_parameter(
            'motor_position_per_joint_radian')
        self.torque_scale = self._array_parameter(
            'joint_torque_per_motor_torque')
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
    def _load_trajectory(path):
        values = np.genfromtxt(path, delimiter=',', names=True)
        names = values.dtype.names or ()
        missing = [name for name in TRAJECTORY_COLUMNS if name not in names]
        if missing:
            raise ValueError(f'{path} is missing columns: {", ".join(missing)}')
        result = {
            name: np.atleast_1d(np.asarray(values[name], dtype=float))
            for name in TRAJECTORY_COLUMNS
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
        for joint in range(2):
            suffix = str(joint + 1)
            self.trajectory[f'q{suffix}'] *= scale[joint]
            self.trajectory[f'dq{suffix}'] *= scale[joint] / time_scale
            self.trajectory[f'ddq{suffix}'] *= scale[joint] / time_scale**2

    def _validate_trajectory(self):
        q = np.column_stack((self.trajectory['q1'], self.trajectory['q2']))
        dq = np.column_stack((self.trajectory['dq1'], self.trajectory['dq2']))
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

    def _publish_joint_command(self, q, dq):
        motor_q = self.motor_zero + self.directions * self.position_scale * q
        motor_dq = self.directions * self.position_scale * dq
        if np.any(np.abs(motor_q) > self.motor_position_limits):
            self.get_logger().error('Command exceeded motor position limit; stopping')
            self._begin_stop()
            return
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.position = [0.0] * MOTOR_COUNT
        message.velocity = [0.0] * MOTOR_COUNT
        message.effort = [0.0] * MOTOR_COUNT
        for joint in range(2):
            motor = self.motor_indices[joint]
            message.position[motor] = float(motor_q[joint])
            message.velocity[motor] = float(motor_dq[joint])
        self.command_pub.publish(message)

    def prepare_callback(self, _request, response):
        if not self._feedback_is_fresh():
            response.success = False
            response.message = 'no fresh /dm_motor_usb/feedback'
            return response
        if self.state not in ('idle', 'ready'):
            response.success = False
            response.message = f'cannot prepare while state={self.state}'
            return response
        _, _, _, q, _, _ = self._joint_feedback()
        self.hold_q = q.copy()
        self.desired_q = q.copy()
        self.desired_dq.fill(0.0)
        self.desired_ddq.fill(0.0)
        self.state = 'ready'
        self._publish_joint_command(self.hold_q, np.zeros(2))
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
        self.segment_coefficients = self._quintic(
            q, dq, np.zeros(2), target_q, target_dq, target_ddq,
            self.transition_duration)
        self.segment_start = self._now() + self.hold_before_start
        self.records = []
        self.state = 'transition_wait'
        response.success = True
        response.message = 'smooth transition and excitation scheduled'
        return response

    def stop_callback(self, _request, response):
        if self.state in ('idle', 'ready'):
            self._publish_enable(False)
            self.state = 'idle'
            response.success = True
            response.message = 'motors disabled'
            return response
        self._begin_stop()
        response.success = True
        response.message = 'smooth stop started'
        return response

    @staticmethod
    def _quintic(q0, dq0, ddq0, q1, dq1, ddq1, duration):
        coefficients = np.zeros((2, 6))
        coefficients[:, 0] = q0
        coefficients[:, 1] = dq0
        coefficients[:, 2] = 0.5 * ddq0
        t = duration
        matrix = np.asarray([
            [t**3, t**4, t**5],
            [3 * t**2, 4 * t**3, 5 * t**4],
            [6 * t, 12 * t**2, 20 * t**3],
        ])
        for joint in range(2):
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

    def _trajectory_sample(self, elapsed):
        t = self.trajectory['t']
        return tuple(
            np.asarray([
                np.interp(elapsed, t, self.trajectory[f'{prefix}1']),
                np.interp(elapsed, t, self.trajectory[f'{prefix}2']),
            ])
            for prefix in ('q', 'dq', 'ddq')
        )

    def timer_callback(self):
        now = self._now()
        if self.state in ('idle',):
            return
        if not self._feedback_is_fresh():
            if self.state not in ('ready', 'hold'):
                self.get_logger().error('Feedback timeout; disabling motors')
                self._publish_enable(False)
                self.state = 'idle'
            return
        if self.state == 'ready':
            self._publish_joint_command(self.hold_q, np.zeros(2))
            return
        if self.state == 'transition_wait':
            self._publish_joint_command(self.hold_q, np.zeros(2))
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
            self._publish_joint_command(q, dq)
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
            self._publish_joint_command(q, dq)
            if elapsed >= duration:
                self._begin_stop()
            return
        if self.state == 'hold':
            self._publish_joint_command(self.hold_q, np.zeros(2))
            if self.disable_deadline is not None and now >= self.disable_deadline:
                if self.disable_on_finish:
                    self._publish_enable(False)
                self.state = 'idle'
                self.get_logger().info('Experiment finished')

    def _begin_stop(self):
        if self.state in ('stopping', 'hold', 'idle'):
            return
        target = self.desired_q + 0.5 * self.stop_duration * self.desired_dq
        target = np.minimum(np.maximum(target, self.joint_lower), self.joint_upper)
        self.segment_coefficients = self._quintic(
            self.desired_q, self.desired_dq, self.desired_ddq,
            target, np.zeros(2), np.zeros(2), self.stop_duration)
        self.segment_start = self._now()
        self.state = 'stopping'
        self.get_logger().info('Smooth stop started')

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
        raw_header = [
            't', 'q1_des', 'q2_des', 'dq1_des', 'dq2_des',
            'ddq1_des', 'ddq2_des', 'motor1_q', 'motor2_q',
            'motor1_dq', 'motor2_dq', 'motor1_tau', 'motor2_tau',
            'q1', 'q2', 'dq1', 'dq2', 'tau1', 'tau2',
        ]
        with (run_directory / 'experiment_raw.csv').open(
                'w', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream)
            writer.writerow(raw_header)
            writer.writerows(values)

        t = values[:, 0]
        actual_q = values[:, 13:15]
        range_summary = {}
        for joint in range(2):
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
            values[:, 15:17],
            values[:, 17:19],
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
