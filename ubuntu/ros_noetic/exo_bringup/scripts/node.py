#!/usr/bin/env python3

import os
import sys
import threading
import time

import rospy
import serial
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray, UInt8, UInt8MultiArray
from std_srvs.srv import Trigger, TriggerResponse

# Catkin may execute this file through a devel-space relay script.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from protocol import (
    FLAG_CLEAR_ERROR,
    FLAG_ENABLE,
    FLAG_SET_ZERO,
    FLAG_VELOCITY_MODE,
    JOINT_NAMES,
    MODE_MIT,
    MODE_VELOCITY,
    MOTOR_COUNT,
    FeedbackStreamParser,
    build_command_frame,
)


class DmMotorUsbNode:
    def __init__(self):
        port = rospy.get_param('~port', '/dev/ttyACM0')
        rate = float(rospy.get_param('~rate', 500.0))
        if rate <= 0.0:
            raise rospy.ROSInitException('rate must be greater than zero')

        self.kp = self._read_gain('~kp')
        self.kd = self._read_gain('~kd')
        self.serial = serial.Serial(
            port=port,
            baudrate=115200,
            timeout=0.01,
            write_timeout=0.02,
        )

        self.lock = threading.Lock()
        self.parser = FeedbackStreamParser()
        self.sequence = 0
        self.enabled = False
        self.control_mode = MODE_MIT
        self.one_shot_flags = 0
        self.closed = False

        self.position = [0.0] * MOTOR_COUNT
        self.velocity = [0.0] * MOTOR_COUNT
        self.torque = [0.0] * MOTOR_COUNT

        self.feedback_pub = rospy.Publisher(
            '/dm_motor_usb/feedback', JointState, queue_size=10)
        self.status_pub = rospy.Publisher(
            '/dm_motor_usb/status', UInt8MultiArray, queue_size=10)
        self.temperature_pub = rospy.Publisher(
            '/dm_motor_usb/temperature', Float32MultiArray, queue_size=10)

        rospy.Subscriber(
            '/dm_motor_usb/command', JointState,
            self.command_callback, queue_size=1)
        rospy.Subscriber(
            '/dm_motor_usb/enable', Bool,
            self.enable_callback, queue_size=1)
        rospy.Subscriber(
            '/dm_motor_usb/control_mode', UInt8,
            self.control_mode_callback, queue_size=1)
        rospy.Service(
            '/dm_motor_usb/clear_error', Trigger,
            self.clear_error_callback)
        rospy.Service(
            '/dm_motor_usb/set_zero', Trigger,
            self.set_zero_callback)

        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / rate), self.send_command)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo(
            'Opened %s; sending %d-motor commands at %.1f Hz',
            port, MOTOR_COUNT, rate)

    @staticmethod
    def _read_gain(name):
        values = rospy.get_param(name, [0.0] * MOTOR_COUNT)
        if isinstance(values, (int, float)):
            values = [float(values)] * MOTOR_COUNT
        if len(values) != MOTOR_COUNT:
            raise rospy.ROSInitException(
                '{} must contain exactly {} values'.format(
                    name, MOTOR_COUNT))
        return [float(value) for value in values]

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
            rospy.logwarn_throttle(
                1.0,
                'JointState arrays should all contain 7 values; '
                'short fields keep their previous target')

    def enable_callback(self, message):
        with self.lock:
            self.enabled = bool(message.data)

    def control_mode_callback(self, message):
        mode = int(message.data)
        if mode not in (MODE_MIT, MODE_VELOCITY):
            rospy.logwarn_throttle(
                1.0, 'control_mode must be 0 (MIT) or 1 (velocity)')
            return

        with self.lock:
            if mode == self.control_mode:
                return
            self.control_mode = mode
            self.enabled = False
            self.position = [0.0] * MOTOR_COUNT
            self.velocity = [0.0] * MOTOR_COUNT
            self.torque = [0.0] * MOTOR_COUNT

        mode_name = 'velocity' if mode == MODE_VELOCITY else 'MIT'
        rospy.loginfo(
            'Control mode changed to %s; motors were disabled and targets cleared',
            mode_name)

    def clear_error_callback(self, _request):
        with self.lock:
            self.one_shot_flags |= FLAG_CLEAR_ERROR
        return TriggerResponse(True, 'clear-error queued for all motors')

    def set_zero_callback(self, _request):
        with self.lock:
            self.one_shot_flags |= FLAG_SET_ZERO
        return TriggerResponse(True, 'set-zero queued for all motors')

    def _make_command_frame(self):
        with self.lock:
            one_shot_flags = self.one_shot_flags
            flags = one_shot_flags
            if self.enabled:
                flags |= FLAG_ENABLE
            if self.control_mode == MODE_VELOCITY:
                flags |= FLAG_VELOCITY_MODE

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

    def send_command(self, _event=None):
        if self.closed:
            return

        frame, one_shot_flags = self._make_command_frame()
        try:
            self.serial.write(frame)
        except (serial.SerialException, serial.SerialTimeoutException) as error:
            if one_shot_flags:
                with self.lock:
                    self.one_shot_flags |= one_shot_flags
            rospy.logerr_throttle(1.0, 'USB write failed: %s', error)

    def read_loop(self):
        while not rospy.is_shutdown() and not self.closed:
            try:
                data = self.serial.read(256)
            except serial.SerialException as error:
                if not self.closed:
                    rospy.logerr_throttle(1.0, 'USB read failed: %s', error)
                    time.sleep(0.1)
                continue

            if not data:
                continue

            for sequence, feedback in self.parser.feed(data):
                self.publish_feedback(feedback, sequence)

    def publish_feedback(self, feedback, _sequence):
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
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

    def shutdown(self):
        if self.closed:
            return

        self.timer.shutdown()
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
        if self.reader.is_alive():
            self.reader.join(timeout=0.2)
        self.serial.close()


def main():
    rospy.init_node('dm_motor_usb')
    try:
        DmMotorUsbNode()
        rospy.spin()
    except (serial.SerialException, rospy.ROSInitException) as error:
        rospy.logfatal('dm_motor_usb startup failed: %s', error)


if __name__ == '__main__':
    main()
