import struct
from dataclasses import dataclass


HEADER = b'\xAA\x55'
TYPE_COMMAND = 0x01
TYPE_FEEDBACK = 0x81

MOTOR_COUNT = 7
COMMAND_RECORD_SIZE = 21
FEEDBACK_RECORD_SIZE = 15
COMMAND_PAYLOAD_SIZE = COMMAND_RECORD_SIZE * MOTOR_COUNT
FEEDBACK_PAYLOAD_SIZE = FEEDBACK_RECORD_SIZE * MOTOR_COUNT
COMMAND_FRAME_SIZE = COMMAND_PAYLOAD_SIZE + 9
FEEDBACK_FRAME_SIZE = FEEDBACK_PAYLOAD_SIZE + 9

FLAG_ENABLE = 0x01
FLAG_CLEAR_ERROR = 0x02
FLAG_SET_ZERO = 0x04

JOINT_NAMES = [
    'd4340p_1',
    'd4340p_2',
    'd4340p_3',
    'd4340p_4',
    'd4310p_5',
    'd4310p_6',
    'd4310p_7',
]


@dataclass(frozen=True)
class MotorFeedback:
    status: int
    position: float
    velocity: float
    torque: float
    mos_temperature: int
    rotor_temperature: int


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


def _check_motor_array(name, values):
    if len(values) != MOTOR_COUNT:
        raise ValueError(
            f'{name} must contain exactly {MOTOR_COUNT} values')


def build_command_frame(sequence, flags, position, velocity, kp, kd, torque):
    _check_motor_array('position', position)
    _check_motor_array('velocity', velocity)
    _check_motor_array('kp', kp)
    _check_motor_array('kd', kd)
    _check_motor_array('torque', torque)

    payload = bytearray()
    for index in range(MOTOR_COUNT):
        payload.extend(struct.pack(
            '<Bfffff',
            flags,
            float(position[index]),
            float(velocity[index]),
            float(kp[index]),
            float(kd[index]),
            float(torque[index])))

    frame = bytearray(HEADER)
    frame.extend(struct.pack(
        '<BHH', TYPE_COMMAND, len(payload), sequence & 0xFFFF))
    frame.extend(payload)
    frame.extend(struct.pack('<H', crc16_ccitt(frame[2:])))
    return bytes(frame)


def decode_feedback_payload(payload):
    if len(payload) != FEEDBACK_PAYLOAD_SIZE:
        raise ValueError(
            f'feedback payload must be {FEEDBACK_PAYLOAD_SIZE} bytes')

    feedback = []
    for index in range(MOTOR_COUNT):
        offset = index * FEEDBACK_RECORD_SIZE
        values = struct.unpack_from('<BfffBB', payload, offset)
        feedback.append(MotorFeedback(*values))
    return feedback


class FeedbackStreamParser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        frames = []

        while True:
            header_index = self.buffer.find(HEADER)
            if header_index < 0:
                if self.buffer[-1:] == HEADER[:1]:
                    self.buffer[:] = HEADER[:1]
                else:
                    self.buffer.clear()
                return frames

            if header_index > 0:
                del self.buffer[:header_index]

            if len(self.buffer) < 7:
                return frames

            frame_type, payload_length, sequence = struct.unpack_from(
                '<BHH', self.buffer, 2)

            if payload_length > FEEDBACK_PAYLOAD_SIZE:
                del self.buffer[0]
                continue

            frame_length = payload_length + 9
            if len(self.buffer) < frame_length:
                return frames

            frame = bytes(self.buffer[:frame_length])
            del self.buffer[:frame_length]

            received_crc = struct.unpack_from(
                '<H', frame, frame_length - 2)[0]
            if received_crc != crc16_ccitt(frame[2:-2]):
                continue

            if (frame_type == TYPE_FEEDBACK and
                    payload_length == FEEDBACK_PAYLOAD_SIZE):
                frames.append((sequence, decode_feedback_payload(frame[7:-2])))

        return frames
