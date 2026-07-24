#!/usr/bin/env python3

import struct
import threading

import rospy
import serial
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray, UInt8MultiArray
from std_srvs.srv import Trigger, TriggerResponse


HEADER = b"\xAA\x55"
TYPE_COMMAND = 0x01
TYPE_FEEDBACK = 0x81
MOTOR_COUNT = 4
COMMAND_PAYLOAD_SIZE = 21 * MOTOR_COUNT
FEEDBACK_PAYLOAD_SIZE = 15 * MOTOR_COUNT

FLAG_ENABLE = 0x01
FLAG_CLEAR_ERROR = 0x02
FLAG_SET_ZERO = 0x04


def crc16_ccitt(data):
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


class D4340pUsbNode:
    def __init__(self):
        port = rospy.get_param("~port", "/dev/ttyACM0")
        rate = rospy.get_param("~rate", 250.0)

        self.serial = serial.Serial(port=port, baudrate=115200, timeout=0.01)
        self.lock = threading.Lock()
        self.rx_buffer = bytearray()
        self.sequence = 0
        self.enabled = False
        self.one_shot_flags = 0

        self.position = [0.0] * MOTOR_COUNT
        self.velocity = [0.0] * MOTOR_COUNT
        self.torque = [0.0] * MOTOR_COUNT
        self.kp = self._read_gain("~kp", 0.0)
        self.kd = self._read_gain("~kd", 0.0)

        self.feedback_pub = rospy.Publisher(
            "~feedback", JointState, queue_size=10)
        self.status_pub = rospy.Publisher(
            "~status", UInt8MultiArray, queue_size=10)
        self.temperature_pub = rospy.Publisher(
            "~temperature", Float32MultiArray, queue_size=10)

        rospy.Subscriber("~command", JointState, self.command_callback,
                         queue_size=1)
        rospy.Subscriber("~enable", Bool, self.enable_callback, queue_size=1)
        rospy.Service("~clear_error", Trigger, self.clear_error_callback)
        rospy.Service("~set_zero", Trigger, self.set_zero_callback)

        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()
        self.timer = rospy.Timer(rospy.Duration(1.0 / rate),
                                 self.send_command)

    @staticmethod
    def _read_gain(name, default):
        value = rospy.get_param(name, [default] * MOTOR_COUNT)
        if not isinstance(value, list):
            value = [float(value)] * MOTOR_COUNT
        if len(value) != MOTOR_COUNT:
            raise rospy.ROSInitException(
                "{} must contain {} values".format(name, MOTOR_COUNT))
        return [float(item) for item in value]

    def command_callback(self, message):
        with self.lock:
            if len(message.position) >= MOTOR_COUNT:
                self.position = list(message.position[:MOTOR_COUNT])
            if len(message.velocity) >= MOTOR_COUNT:
                self.velocity = list(message.velocity[:MOTOR_COUNT])
            if len(message.effort) >= MOTOR_COUNT:
                self.torque = list(message.effort[:MOTOR_COUNT])

    def enable_callback(self, message):
        with self.lock:
            self.enabled = message.data

    def clear_error_callback(self, _request):
        with self.lock:
            self.one_shot_flags |= FLAG_CLEAR_ERROR
        return TriggerResponse(success=True, message="clear-error queued")

    def set_zero_callback(self, _request):
        with self.lock:
            self.one_shot_flags |= FLAG_SET_ZERO
        return TriggerResponse(success=True, message="set-zero queued")

    def send_command(self, _event):
        payload = bytearray()

        with self.lock:
            flags = self.one_shot_flags
            if self.enabled:
                flags |= FLAG_ENABLE

            for index in range(MOTOR_COUNT):
                payload.extend(struct.pack(
                    "<Bfffff",
                    flags,
                    self.position[index],
                    self.velocity[index],
                    self.kp[index],
                    self.kd[index],
                    self.torque[index]))

            sequence = self.sequence
            self.sequence = (self.sequence + 1) & 0xFFFF
            self.one_shot_flags = 0

        frame = bytearray(HEADER)
        frame.extend(struct.pack("<BHH",
                                 TYPE_COMMAND,
                                 len(payload),
                                 sequence))
        frame.extend(payload)
        frame.extend(struct.pack("<H", crc16_ccitt(frame[2:])))

        try:
            self.serial.write(frame)
        except serial.SerialException as error:
            rospy.logerr_throttle(1.0, "USB write failed: %s", error)

    def read_loop(self):
        while not rospy.is_shutdown():
            try:
                data = self.serial.read(256)
            except serial.SerialException as error:
                rospy.logerr_throttle(1.0, "USB read failed: %s", error)
                continue

            if data:
                self.rx_buffer.extend(data)
                self.parse_received_data()

    def parse_received_data(self):
        while True:
            header_index = self.rx_buffer.find(HEADER)
            if header_index < 0:
                self.rx_buffer.clear()
                return

            if header_index > 0:
                del self.rx_buffer[:header_index]

            if len(self.rx_buffer) < 7:
                return

            frame_type, payload_length, sequence = struct.unpack_from(
                "<BHH", self.rx_buffer, 2)
            frame_length = payload_length + 9
            if len(self.rx_buffer) < frame_length:
                return

            frame = bytes(self.rx_buffer[:frame_length])
            del self.rx_buffer[:frame_length]

            received_crc = struct.unpack_from("<H", frame, frame_length - 2)[0]
            if received_crc != crc16_ccitt(frame[2:-2]):
                rospy.logwarn_throttle(1.0, "USB CRC error")
                continue

            if (frame_type == TYPE_FEEDBACK and
                    payload_length == FEEDBACK_PAYLOAD_SIZE):
                self.publish_feedback(frame[7:-2], sequence)

    def publish_feedback(self, payload, _sequence):
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = [
            "d4340p_1", "d4340p_2", "d4340p_3", "d4340p_4"
        ]

        status = UInt8MultiArray()
        temperature = Float32MultiArray()

        for index in range(MOTOR_COUNT):
            offset = index * 15
            state, position, velocity, torque, mos_temp, rotor_temp = \
                struct.unpack_from("<BfffBB", payload, offset)
            status.data.append(state)
            joint_state.position.append(position)
            joint_state.velocity.append(velocity)
            joint_state.effort.append(torque)
            temperature.data.extend([float(mos_temp), float(rotor_temp)])

        self.feedback_pub.publish(joint_state)
        self.status_pub.publish(status)
        self.temperature_pub.publish(temperature)


if __name__ == "__main__":
    rospy.init_node("d4340p_usb")
    D4340pUsbNode()
    rospy.spin()
