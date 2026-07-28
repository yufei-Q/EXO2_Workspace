#!/usr/bin/env python3
"""Windows USB CDC console for the STM32G474 seven-motor controller."""

import argparse
import shlex
import struct
import threading
import time
from dataclasses import dataclass

import serial
from serial.tools import list_ports

HEADER = b"\xAA\x55"
TYPE_COMMAND = 0x01
TYPE_FEEDBACK = 0x81
MOTOR_COUNT = 7
COMMAND_RECORD_SIZE = 21
FEEDBACK_RECORD_SIZE = 15
COMMAND_PAYLOAD_SIZE = COMMAND_RECORD_SIZE * MOTOR_COUNT
FEEDBACK_PAYLOAD_SIZE = FEEDBACK_RECORD_SIZE * MOTOR_COUNT
FLAG_ENABLE = 0x01
FLAG_CLEAR_ERROR = 0x02
FLAG_SET_ZERO = 0x04
ONE_SHOT_FLAGS = FLAG_CLEAR_ERROR | FLAG_SET_ZERO


@dataclass
class MotorCommand:
    flags: int = 0
    position: float = 0.0
    velocity: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    torque: float = 0.0


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
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_command_frame(sequence, commands):
    payload = bytearray()
    for command in commands:
        payload.extend(struct.pack(
            '<Bfffff',
            command.flags,
            command.position,
            command.velocity,
            command.kp,
            command.kd,
            command.torque,
        ))

    frame = bytearray(HEADER)
    frame.extend(struct.pack('<BHH', TYPE_COMMAND, len(payload), sequence & 0xFFFF))
    frame.extend(payload)
    frame.extend(struct.pack('<H', crc16_ccitt(frame[2:])))
    return bytes(frame)


def decode_feedback(payload):
    result = []
    for index in range(MOTOR_COUNT):
        offset = index * FEEDBACK_RECORD_SIZE
        result.append(MotorFeedback(*struct.unpack_from('<BfffBB', payload, offset)))
    return result


class FeedbackParser:
    def __init__(self):
        self.buffer = bytearray()
        self.crc_error_count = 0
        self.format_error_count = 0

    def feed(self, data):
        self.buffer.extend(data)
        decoded = []

        while True:
            header_index = self.buffer.find(HEADER)
            if header_index < 0:
                self.buffer[:] = HEADER[:1] if self.buffer[-1:] == HEADER[:1] else b''
                return decoded
            if header_index:
                del self.buffer[:header_index]
            if len(self.buffer) < 7:
                return decoded

            frame_type, payload_length, sequence = struct.unpack_from('<BHH', self.buffer, 2)
            if payload_length > FEEDBACK_PAYLOAD_SIZE:
                self.format_error_count += 1
                del self.buffer[0]
                continue

            frame_length = payload_length + 9
            if len(self.buffer) < frame_length:
                return decoded

            frame = bytes(self.buffer[:frame_length])
            del self.buffer[:frame_length]
            received_crc = struct.unpack_from('<H', frame, frame_length - 2)[0]
            if received_crc != crc16_ccitt(frame[2:-2]):
                self.crc_error_count += 1
                continue
            if frame_type != TYPE_FEEDBACK or payload_length != FEEDBACK_PAYLOAD_SIZE:
                self.format_error_count += 1
                continue
            decoded.append((sequence, decode_feedback(frame[7:-2])))


class MotorUsbLink:
    def __init__(self, port, rate):
        self.serial = serial.Serial(port, 115200, timeout=0.01, write_timeout=0.05)
        self.period = 1.0 / rate
        self.commands = [MotorCommand() for _ in range(MOTOR_COUNT)]
        self.feedback = None
        self.feedback_sequence = None
        self.feedback_time = 0.0
        self.tx_count = 0
        self.rx_count = 0
        self.tx_error_count = 0
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.parser = FeedbackParser()
        self.tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)

    def start(self):
        self.serial.reset_input_buffer()
        self.tx_thread.start()
        self.rx_thread.start()

    def _snapshot_commands(self):
        with self.lock:
            snapshot = [MotorCommand(**vars(item)) for item in self.commands]
            for item in self.commands:
                item.flags &= ~ONE_SHOT_FLAGS
            return snapshot

    def _restore_one_shots(self, snapshot):
        with self.lock:
            for current, sent in zip(self.commands, snapshot):
                current.flags |= sent.flags & ONE_SHOT_FLAGS

    def _tx_loop(self):
        sequence = 0
        deadline = time.monotonic()
        while not self.stop_event.is_set():
            snapshot = self._snapshot_commands()
            try:
                self.serial.write(build_command_frame(sequence, snapshot))
                self.tx_count += 1
                sequence = (sequence + 1) & 0xFFFF
            except (serial.SerialException, serial.SerialTimeoutException):
                self.tx_error_count += 1
                self._restore_one_shots(snapshot)
                self.stop_event.set()
                return

            deadline += self.period
            delay = deadline - time.monotonic()
            if delay > 0.0:
                self.stop_event.wait(delay)
            else:
                deadline = time.monotonic()

    def _rx_loop(self):
        while not self.stop_event.is_set():
            try:
                data = self.serial.read(256)
            except serial.SerialException:
                self.stop_event.set()
                return
            if not data:
                continue
            for sequence, feedback in self.parser.feed(data):
                with self.lock:
                    self.feedback_sequence = sequence
                    self.feedback = feedback
                    self.feedback_time = time.monotonic()
                    self.rx_count += 1

    def select(self, token):
        if token.lower() == 'all':
            return range(MOTOR_COUNT)
        motor_id = int(token)
        if motor_id < 1 or motor_id > MOTOR_COUNT:
            raise ValueError('motor ID must be 1..7 or all')
        return [motor_id - 1]

    def enable(self, token):
        with self.lock:
            for index in self.select(token):
                self.commands[index].flags |= FLAG_ENABLE

    def disable(self, token):
        with self.lock:
            for index in self.select(token):
                self.commands[index] = MotorCommand()

    def action(self, token, flag):
        with self.lock:
            for index in self.select(token):
                self.commands[index].flags |= flag

    def set_target(self, motor_id, values):
        index = int(motor_id) - 1
        if index < 0 or index >= MOTOR_COUNT:
            raise ValueError('motor ID must be 1..7')
        position, velocity, kp, kd, torque = map(float, values)
        with self.lock:
            flags = self.commands[index].flags
            self.commands[index] = MotorCommand(flags, position, velocity, kp, kd, torque)

    def print_feedback(self):
        with self.lock:
            feedback = self.feedback
            sequence = self.feedback_sequence
            age = time.monotonic() - self.feedback_time if feedback else None

        if feedback is None:
            print('No valid feedback frame received.')
            return
        print(f'feedback sequence={sequence}, age={age * 1000.0:.1f} ms')
        print(' ID status    position     velocity       torque  MOS rotor')
        for index, item in enumerate(feedback, 1):
            print(f'{index:3d} {item.status:6d} {item.position:11.5f} '
                  f'{item.velocity:12.5f} {item.torque:12.5f} '
                  f'{item.mos_temperature:4d} {item.rotor_temperature:5d}')

    def print_stats(self):
        print(f'TX={self.tx_count}, RX={self.rx_count}, TX errors={self.tx_error_count}, '
              f'CRC errors={self.parser.crc_error_count}, '
              f'format errors={self.parser.format_error_count}')

    def close(self):
        if self.stop_event.is_set() and not self.serial.is_open:
            return
        self.stop_event.set()
        self.tx_thread.join(timeout=0.3)
        self.rx_thread.join(timeout=0.3)
        with self.lock:
            self.commands = [MotorCommand() for _ in range(MOTOR_COUNT)]
            disabled_commands = [MotorCommand() for _ in range(MOTOR_COUNT)]
        for sequence in range(5):
            try:
                self.serial.write(build_command_frame(sequence, disabled_commands))
                time.sleep(0.01)
            except serial.SerialException:
                break
        if self.serial.is_open:
            self.serial.close()


def show_ports():
    ports = list(list_ports.comports())
    if not ports:
        print('No serial ports found.')
    for item in ports:
        print(f'{item.device:8s}  {item.description}  [{item.hwid}]')


def print_help():
    print('''Commands:
  feedback                         show latest seven-motor feedback
  stats                            show USB frame counters
  enable <1..7|all>                enable selected motor(s)
  disable <1..7|all>               disable and clear selected target(s)
  set <ID> <p> <v> <kp> <kd> <t>  set one MIT target
  clear <1..7|all>                 send clear-error once
  zero <1..7|all>                  set current position as zero once
  help                             show this help
  quit                             disable all motors and exit''')


def main():
    parser = argparse.ArgumentParser(description='STM32G474 USB motor control console')
    parser.add_argument('--port', help='CDC serial port, for example COM3')
    parser.add_argument('--rate', type=float, default=500.0, help='command rate in Hz (default: 500)')
    parser.add_argument('--list', action='store_true', help='list serial ports and exit')
    args = parser.parse_args()

    if args.list:
        show_ports()
        return
    if not args.port:
        show_ports()
        parser.error('--port is required')
    if args.rate <= 0.0:
        parser.error('--rate must be greater than zero')

    link = MotorUsbLink(args.port, args.rate)
    link.start()
    print(f'Opened {args.port}; sending safe disabled commands at {args.rate:.1f} Hz.')
    print_help()

    try:
        while not link.stop_event.is_set():
            try:
                parts = shlex.split(input('motor> '))
            except EOFError:
                break
            if not parts:
                continue
            command = parts[0].lower()
            try:
                if command == 'feedback' and len(parts) == 1:
                    link.print_feedback()
                elif command == 'stats' and len(parts) == 1:
                    link.print_stats()
                elif command == 'enable' and len(parts) == 2:
                    link.enable(parts[1])
                elif command == 'disable' and len(parts) == 2:
                    link.disable(parts[1])
                elif command == 'set' and len(parts) == 7:
                    link.set_target(parts[1], parts[2:])
                elif command == 'clear' and len(parts) == 2:
                    link.action(parts[1], FLAG_CLEAR_ERROR)
                elif command == 'zero' and len(parts) == 2:
                    link.action(parts[1], FLAG_SET_ZERO)
                elif command == 'help':
                    print_help()
                elif command in ('quit', 'exit'):
                    break
                else:
                    print('Invalid command. Enter help for usage.')
            except (ValueError, serial.SerialException) as error:
                print(f'Error: {error}')
    except KeyboardInterrupt:
        print('\nStopping...')
    finally:
        link.close()
        link.print_stats()
        print('All motors disabled; serial port closed.')


if __name__ == '__main__':
    main()