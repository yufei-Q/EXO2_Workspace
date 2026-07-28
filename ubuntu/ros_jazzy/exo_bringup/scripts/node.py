#!/usr/bin/env python3

import threading
import time

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray, UInt8MultiArray
from std_srvs.srv import Trigger

from scripts.protocol import (
    FLAG_CLEAR_ERROR,
    FLAG_ENABLE,
    FLAG_SET_ZERO,
    JOINT_NAMES,
    MOTOR_COUNT,
    FeedbackStreamParser,
    build_command_frame,
)


class DmMotorUsbNode(Node):
    def __init__(self):
        super().__init__('dm_motor_usb')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('rate', 500.0)
        self.declare_parameter('kp', [0.0] * MOTOR_COUNT)
        self.declare_parameter('kd', [0.0] * MOTOR_COUNT)

        port = self.get_parameter('port').value
        rate = float(self.get_parameter('rate').value)
        if rate <= 0.0:
            raise ValueError('rate must be greater than zero')

        self.kp = self._read_gain('kp')
        self.kd = self._read_gain('kd')
        self.serial = serial.Serial(
            port=port,
            baudrate=115200,
            timeout=0.01,
            write_timeout=0.02,
        )

        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.parser = FeedbackStreamParser()
        self.sequence = 0
        self.enabled = False
        self.one_shot_flags = 0
        self.closed = False
        self.last_log_time = {}

        self.position = [0.0] * MOTOR_COUNT
        self.velocity = [0.0] * MOTOR_COUNT
        self.torque = [0.0] * MOTOR_COUNT

        self.feedback_pub = self.create_publisher(
            JointState, '/dm_motor_usb/feedback', 10)
        self.status_pub = self.create_publisher(
            UInt8MultiArray, '/dm_motor_usb/status', 10)
        self.temperature_pub = self.create_publisher(
            Float32MultiArray, '/dm_motor_usb/temperature', 10)

        self.create_subscription(
            JointState, '/dm_motor_usb/command', self.command_callback, 1)
        self.create_subscription(
            Bool, '/dm_motor_usb/enable', self.enable_callback, 1)
        self.create_service(
            Trigger, '/dm_motor_usb/clear_error', self.clear_error_callback)
        self.create_service(
            Trigger, '/dm_motor_usb/set_zero', self.set_zero_callback)

        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()
        self.timer = self.create_timer(1.0 / rate, self.send_command)

        self.get_logger().info(
            f'Opened {port}; sending {MOTOR_COUNT}-motor commands at {rate:.1f} Hz')

    def _read_gain(self, name):
        values = self.get_parameter(name).value
        if len(values) != MOTOR_COUNT:
            raise ValueError(
                f'{name} must contain exactly {MOTOR_COUNT} values')
        return [float(value) for value in values]

    def _log_throttled(self, level, key, message):
        now = time.monotonic()
        if now - self.last_log_time.get(key, 0.0) < 1.0:
            return
        self.last_log_time[key] = now
        getattr(self.get_logger(), level)(message)

    def command_callback(self, message):
        with self.lock:
            if len(message.position) >= MOTOR_COUNT:
                self.position = list(message.position[:MOTOR_COUNT])
            if len(message.velocity) >= MOTOR_COUNT:
                self.velocity = list(message.velocity[:MOTOR_COUNT])
            if len(message.effort) >= MOTOR_COUNT:
                self.torque = list(message.effort[:MOTOR_COUNT])

        if (len(message.position) < MOTOR_COUNT or
                len(message.velocity) < MOTOR_COUNT or
                len(message.effort) < MOTOR_COUNT):
            self._log_throttled(
                'warning',
                'short_command',
                'JointState arrays should all contain 7 values; short fields keep their previous target')

    def enable_callback(self, message):
        with self.lock:
            self.enabled = bool(message.data)

    def clear_error_callback(self, _request, response):
        with self.lock:
            self.one_shot_flags |= FLAG_CLEAR_ERROR
        response.success = True
        response.message = 'clear-error queued for all motors'
        return response

    def set_zero_callback(self, _request, response):
        with self.lock:
            self.one_shot_flags |= FLAG_SET_ZERO
        response.success = True
        response.message = 'set-zero queued for all motors'
        return response

    def _make_command_frame(self):
        with self.lock:
            one_shot_flags = self.one_shot_flags
            flags = one_shot_flags
            if self.enabled:
                flags |= FLAG_ENABLE

            frame = build_command_frame(
                self.sequence,
                flags,
                self.position,
                self.velocity,
                self.kp,
                self.kd,
                self.torque,
            )
            self.sequence = (self.sequence + 1) & 0xFFFF
            self.one_shot_flags = 0
        return frame, one_shot_flags

    def send_command(self):
        if self.closed:
            return

        frame, one_shot_flags = self._make_command_frame()
        try:
            self.serial.write(frame)
        except (serial.SerialException, serial.SerialTimeoutException) as error:
            if one_shot_flags:
                with self.lock:
                    self.one_shot_flags |= one_shot_flags
            self._log_throttled(
                'error', 'write', f'USB write failed: {error}')

    def read_loop(self):
        while not self.stop_event.is_set():
            try:
                data = self.serial.read(256)
            except serial.SerialException as error:
                if not self.stop_event.is_set():
                    self._log_throttled(
                        'error', 'read', f'USB read failed: {error}')
                    self.stop_event.wait(0.1)
                continue

            if not data:
                continue

            for sequence, feedback in self.parser.feed(data):
                self.publish_feedback(feedback, sequence)

    def publish_feedback(self, feedback, _sequence):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = list(JOINT_NAMES)
        joint_state.position = [item.position for item in feedback]
        joint_state.velocity = [item.velocity for item in feedback]
        joint_state.effort = [item.torque for item in feedback]

        status = UInt8MultiArray()
        status.data = [item.status for item in feedback]

        temperature = Float32MultiArray()
        for item in feedback:
            temperature.data.extend([
                float(item.mos_temperature),
                float(item.rotor_temperature),
            ])

        self.feedback_pub.publish(joint_state)
        self.status_pub.publish(status)
        self.temperature_pub.publish(temperature)

    def shutdown_serial(self):
        if self.closed:
            return

        if hasattr(self, 'timer'):
            self.timer.cancel()

        with self.lock:
            self.enabled = False
            self.one_shot_flags = 0
            self.position = [0.0] * MOTOR_COUNT
            self.velocity = [0.0] * MOTOR_COUNT
            self.torque = [0.0] * MOTOR_COUNT

        for _ in range(3):
            self.send_command()
            time.sleep(0.005)

        self.closed = True
        self.stop_event.set()
        if self.reader.is_alive():
            self.reader.join(timeout=0.2)
        self.serial.close()

    def destroy_node(self):
        self.shutdown_serial()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DmMotorUsbNode()
        rclpy.spin(node)
    except (serial.SerialException, ValueError) as error:
        if node is not None:
            node.get_logger().fatal(str(error))
        else:
            print(f'dm_motor_usb startup failed: {error}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
