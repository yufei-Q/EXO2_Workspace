#!/usr/bin/env python3

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


MOTOR_COUNT = 7


class GravityFormula:
    def __init__(self, path):
        with Path(path).expanduser().resolve().open(
                'r', encoding='utf-8') as stream:
            values = json.load(stream)
        self.frequencies = np.asarray(values['frequency_vectors'], dtype=float)
        self.coefficients = np.asarray(values['coefficient_matrix'], dtype=float)
        expected_rows = 1 + 2 * self.frequencies.shape[0]
        if self.frequencies.ndim != 2 or self.frequencies.shape[1] != 2:
            raise ValueError('gravity frequency_vectors must have shape (N, 2)')
        if self.coefficients.shape != (expected_rows, 2):
            raise ValueError(
                'gravity coefficient_matrix has an incompatible shape')
        if not np.all(np.isfinite(self.coefficients)):
            raise ValueError('gravity coefficients contain non-finite values')

        friction = values.get('friction', {})
        self.friction_enabled = bool(friction.get('enabled', False))
        self.viscous_friction = np.asarray(
            friction.get('viscous_coefficients_Nm_per_rad_s', [0.0, 0.0]),
            dtype=float,
        )
        self.coulomb_friction = np.asarray(
            friction.get('coulomb_coefficients_Nm', [0.0, 0.0]), dtype=float
        )
        self.transition_velocity = np.asarray(
            friction.get('transition_velocity_rad_s', [0.05, 0.05]), dtype=float
        )
        if any(
            item.shape != (2,)
            for item in (
                self.viscous_friction,
                self.coulomb_friction,
                self.transition_velocity,
            )
        ):
            raise ValueError('friction formula arrays must each contain two values')
        if not all(
            np.all(np.isfinite(item))
            for item in (
                self.viscous_friction,
                self.coulomb_friction,
                self.transition_velocity,
            )
        ):
            raise ValueError('friction formula contains non-finite values')
        if np.any(self.transition_velocity <= 0.0):
            raise ValueError('friction transition velocities must be positive')

    def evaluate(self, q):
        phase = self.frequencies @ np.asarray(q, dtype=float)
        basis = [1.0]
        for value in phase:
            basis.extend((np.cos(value), np.sin(value)))
        return np.asarray(basis) @ self.coefficients

    def evaluate_friction(self, dq):
        if not self.friction_enabled:
            return np.zeros(2)
        velocity = np.asarray(dq, dtype=float)
        return (
            self.viscous_friction * velocity
            + self.coulomb_friction
            * np.tanh(velocity / self.transition_velocity)
        )


class GravityCompensationNode(Node):
    def __init__(self):
        super().__init__('exo_gravity_compensation')
        self._declare_parameters()
        self._load_parameters()

        self.feedback = None
        self.feedback_time = None
        self.state = 'idle'
        self.current_scale = 0.0
        self.ramp_start = None
        self.ramp_start_scale = 0.0
        self.ramp_target_scale = 0.0
        self.last_limit_warning = None

        self.command_pub = self.create_publisher(
            JointState, '/dm_motor_usb/command', 1)
        self.enable_pub = self.create_publisher(
            Bool, '/dm_motor_usb/enable', 1)
        self.create_subscription(
            JointState, '/dm_motor_usb/feedback', self.feedback_callback, 20)
        self.create_service(
            Trigger, '/exo_identify/gravity/prepare', self.prepare_callback)
        self.create_service(
            Trigger, '/exo_identify/gravity/start', self.start_callback)
        self.create_service(
            Trigger, '/exo_identify/gravity/stop', self.stop_callback)
        self.timer = self.create_timer(
            1.0 / self.command_rate, self.timer_callback)

        self.get_logger().info(
            f'Gravity/friction compensation loaded; motors={self.motor_indices}, '
            f'directions={self.directions.tolist()}, '
            f'gravity scale={self.gravity_compensation_scale:.3f}, '
            f'friction scales={self.friction_compensation_scale.tolist()}, '
            f'gravity limits={self.max_gravity_torque.tolist()} N.m, '
            f'friction limits={self.max_friction_torque.tolist()} N.m')

    def _declare_parameters(self):
        self.declare_parameter('formula_file', '')
        self.declare_parameter('motor_indices', [0, 6])
        self.declare_parameter('joint_directions', [1.0, 1.0])
        self.declare_parameter('motor_zero_positions', [0.0, 0.0])
        self.declare_parameter(
            'motor_position_per_joint_radian', [1.0, 1.0])
        self.declare_parameter(
            'joint_torque_per_motor_torque', [1.0, 1.0])
        self.declare_parameter('joint_velocity_limits', [0.5, 0.5])
        self.declare_parameter('gravity_compensation_scale', 0.1)
        self.declare_parameter('friction_compensation_scale', [0.1, 0.1])
        self.declare_parameter('max_gravity_torque', [0.15, 0.05])
        self.declare_parameter('max_friction_torque', [0.05, 0.02])
        self.declare_parameter('ramp_duration', 3.0)
        self.declare_parameter('command_rate', 200.0)
        self.declare_parameter('feedback_timeout', 0.1)

    def _array_parameter(self, name, dtype=float):
        values = list(self.get_parameter(name).value)
        if len(values) != 2:
            raise ValueError(f'{name} must contain exactly two values')
        return np.asarray(values, dtype=dtype)

    def _load_parameters(self):
        formula_path = str(self.get_parameter('formula_file').value)
        if not formula_path:
            raise ValueError('formula_file is required')
        self.formula = GravityFormula(formula_path)
        self.motor_indices = self._array_parameter(
            'motor_indices', dtype=int).tolist()
        if len(set(self.motor_indices)) != 2 or any(
                index < 0 or index >= MOTOR_COUNT
                for index in self.motor_indices):
            raise ValueError('motor_indices must be different values from 0 to 6')
        self.directions = self._array_parameter('joint_directions')
        if not np.all(np.isin(self.directions, (-1.0, 1.0))):
            raise ValueError('joint_directions values must be +1.0 or -1.0')
        self.motor_zero = self._array_parameter('motor_zero_positions')
        self.position_scale = self._array_parameter(
            'motor_position_per_joint_radian')
        self.torque_scale = self._array_parameter(
            'joint_torque_per_motor_torque')
        self.velocity_limits = self._array_parameter('joint_velocity_limits')
        self.gravity_compensation_scale = float(
            self.get_parameter('gravity_compensation_scale').value)
        self.friction_compensation_scale = self._array_parameter(
            'friction_compensation_scale')
        self.max_gravity_torque = self._array_parameter(
            'max_gravity_torque')
        self.max_friction_torque = self._array_parameter(
            'max_friction_torque')
        self.ramp_duration = float(
            self.get_parameter('ramp_duration').value)
        self.command_rate = float(self.get_parameter('command_rate').value)
        self.feedback_timeout = float(
            self.get_parameter('feedback_timeout').value)

        if np.any(self.position_scale <= 0.0) or np.any(self.torque_scale <= 0.0):
            raise ValueError('position and torque scales must be positive')
        if np.any(self.velocity_limits <= 0.0):
            raise ValueError('joint velocity limits must be positive')
        if not 0.0 <= self.gravity_compensation_scale <= 1.0:
            raise ValueError('gravity_compensation_scale must be in [0, 1]')
        if np.any(self.friction_compensation_scale < 0.0) or np.any(
            self.friction_compensation_scale > 1.0
        ):
            raise ValueError(
                'friction_compensation_scale values must be in [0, 1]')
        if np.any(self.max_gravity_torque <= 0.0):
            raise ValueError('max_gravity_torque values must be positive')
        if np.any(self.max_friction_torque < 0.0):
            raise ValueError('max_friction_torque values cannot be negative')
        if (
            np.any(self.friction_compensation_scale > 0.0)
            and np.any(self.max_friction_torque > 0.0)
            and not self.formula.friction_enabled
        ):
            raise ValueError(
                'friction compensation requested, but formula has no friction parameters'
            )
        if self.ramp_duration <= 0.0 or self.command_rate <= 0.0:
            raise ValueError('ramp_duration and command_rate must be positive')
        if self.feedback_timeout <= 0.0:
            raise ValueError('feedback_timeout must be positive')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _feedback_is_fresh(self):
        return (
            self.feedback_time is not None
            and self._now() - self.feedback_time <= self.feedback_timeout
        )

    def feedback_callback(self, message):
        if min(len(message.position), len(message.velocity)) < MOTOR_COUNT:
            return
        self.feedback = message
        self.feedback_time = self._now()

    def _joint_state(self):
        motor_q = np.asarray([
            self.feedback.position[index] for index in self.motor_indices])
        motor_dq = np.asarray([
            self.feedback.velocity[index] for index in self.motor_indices])
        q = self.directions * (
            motor_q - self.motor_zero) / self.position_scale
        dq = self.directions * motor_dq / self.position_scale
        return q, dq

    def _publish_enable(self, enabled):
        message = Bool()
        message.data = bool(enabled)
        self.enable_pub.publish(message)

    def _publish_torque(self, joint_torque):
        motor_torque = (
            self.directions * np.asarray(joint_torque) / self.torque_scale)
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.position = [0.0] * MOTOR_COUNT
        message.velocity = [0.0] * MOTOR_COUNT
        message.effort = [0.0] * MOTOR_COUNT
        for joint, motor in enumerate(self.motor_indices):
            message.effort[motor] = float(motor_torque[joint])
        self.command_pub.publish(message)

    def _fault(self, reason):
        self.current_scale = 0.0
        self.state = 'idle'
        self._publish_torque(np.zeros(2))
        self._publish_enable(False)
        self.get_logger().error(f'{reason}; zero torque sent and motors disabled')

    def prepare_callback(self, _request, response):
        if not self._feedback_is_fresh():
            response.success = False
            response.message = 'no fresh /dm_motor_usb/feedback'
            return response
        if self.state != 'idle':
            response.success = False
            response.message = f'cannot prepare while state={self.state}'
            return response
        q, dq = self._joint_state()
        if np.any(np.abs(dq) > self.velocity_limits):
            response.success = False
            response.message = 'current joint velocity exceeds configured limits'
            return response
        predicted = self.formula.evaluate(q)
        predicted_friction = self.formula.evaluate_friction(dq)
        self._publish_torque(np.zeros(2))
        self.state = 'ready'
        response.success = True
        response.message = (
            f'zero torque ready; q={q.tolist()}, full gravity='
            f'{predicted.tolist()} N.m, current friction='
            f'{predicted_friction.tolist()} N.m; enable motors, then call start')
        return response

    def start_callback(self, _request, response):
        if self.state != 'ready':
            response.success = False
            response.message = 'call /exo_identify/gravity/prepare first'
            return response
        if not self._feedback_is_fresh():
            response.success = False
            response.message = 'motor feedback is stale'
            return response
        self.ramp_start = self._now()
        self.ramp_start_scale = 0.0
        self.ramp_target_scale = 1.0
        self.current_scale = 0.0
        self.state = 'ramping_up'
        response.success = True
        response.message = (
            'gravity/friction compensation ramping to 100.0%')
        return response

    def stop_callback(self, _request, response):
        if self.state in ('idle', 'ready'):
            self._publish_torque(np.zeros(2))
            self._publish_enable(False)
            self.current_scale = 0.0
            self.state = 'idle'
            response.success = True
            response.message = 'zero torque sent and motors disabled'
            return response
        self.ramp_start = self._now()
        self.ramp_start_scale = self.current_scale
        self.ramp_target_scale = 0.0
        self.state = 'ramping_down'
        response.success = True
        response.message = 'gravity compensation ramping down'
        return response

    def _update_ramp(self, now):
        progress = min(max((now - self.ramp_start) / self.ramp_duration, 0.0), 1.0)
        self.current_scale = (
            self.ramp_start_scale
            + progress * (self.ramp_target_scale - self.ramp_start_scale))
        if progress >= 1.0:
            if self.state == 'ramping_up':
                self.state = 'active'
                self.get_logger().info(
                    f'Gravity compensation active at '
                    f'{100.0 * self.current_scale:.1f}%')
            else:
                self.current_scale = 0.0
                self.state = 'idle'
                self._publish_enable(False)
                self.get_logger().info(
                    'Gravity compensation stopped; motors disabled')

    def timer_callback(self):
        if self.state == 'idle':
            self._publish_torque(np.zeros(2))
            return
        if not self._feedback_is_fresh():
            self._fault('feedback timeout')
            return
        q, dq = self._joint_state()
        if np.any(np.abs(dq) > self.velocity_limits):
            self._fault('joint velocity limit exceeded')
            return
        if self.state == 'ready':
            self._publish_torque(np.zeros(2))
            return
        if self.state in ('ramping_up', 'ramping_down'):
            self._update_ramp(self._now())

        gravity_requested = (
            self.current_scale
            * self.gravity_compensation_scale
            * self.formula.evaluate(q)
        )
        friction_requested = (
            self.current_scale
            * self.friction_compensation_scale
            * self.formula.evaluate_friction(dq)
        )
        gravity_limited = np.clip(
            gravity_requested,
            -self.max_gravity_torque,
            self.max_gravity_torque,
        )
        friction_limited = np.clip(
            friction_requested,
            -self.max_friction_torque,
            self.max_friction_torque,
        )
        if (
            np.any(np.abs(gravity_requested - gravity_limited) > 1e-9)
            or np.any(np.abs(friction_requested - friction_limited) > 1e-9)
        ):
            now = self._now()
            if self.last_limit_warning is None or now - self.last_limit_warning > 1.0:
                self.last_limit_warning = now
                self.get_logger().warning(
                    'Compensation torque limited: '
                    f'gravity requested={gravity_requested.tolist()}, '
                    f'applied={gravity_limited.tolist()}; '
                    f'friction requested={friction_requested.tolist()}, '
                    f'applied={friction_limited.tolist()} N.m')
        self._publish_torque(gravity_limited + friction_limited)

    def destroy_node(self):
        if rclpy.ok():
            self._publish_torque(np.zeros(2))
            self._publish_enable(False)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GravityCompensationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (OSError, ValueError) as error:
        if node is None:
            print(f'gravity compensation startup failed: {error}')
        else:
            node.get_logger().fatal(str(error))
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
