# STM32–ROS USB CDC protocol

All multibyte values use little-endian byte order. Floating-point values are
IEEE-754 single precision. Each frame has this layout:

| Offset | Size | Field |
|---:|---:|---|
| 0 | 2 | Header `AA 55` |
| 2 | 1 | Frame type |
| 3 | 2 | Payload length |
| 5 | 2 | Sequence |
| 7 | N | Payload |
| 7+N | 2 | CRC16-CCITT |

CRC uses initial value `0xFFFF` and polynomial `0x1021`. It covers bytes from
frame type through the end of payload; the header and CRC itself are excluded.

## PC to STM32: type `0x01`

The payload contains four 21-byte motor records, ordered by CAN ID 1–4:

| Offset in record | Size | Field |
|---:|---:|---|
| 0 | 1 | Flags |
| 1 | 4 | Desired position, rad |
| 5 | 4 | Desired velocity, rad/s |
| 9 | 4 | MIT Kp |
| 13 | 4 | MIT Kd |
| 17 | 4 | Feedforward torque, N·m |

Flags are: bit 0 enable, bit 1 clear error, and bit 2 save current position as
zero. Clear-error and set-zero are one-shot operations.

## STM32 to PC: type `0x81`

The payload contains four 15-byte feedback records, ordered by CAN ID 1–4:

| Offset in record | Size | Field |
|---:|---:|---|
| 0 | 1 | Motor status |
| 1 | 4 | Position, rad |
| 5 | 4 | Velocity, rad/s |
| 9 | 4 | Torque, N·m |
| 13 | 1 | MOS temperature, °C |
| 14 | 1 | Rotor temperature, °C |

## ROS Noetic node

Install the serial dependency:

```bash
sudo apt install python3-serial
```

Copy or link `ros_noetic/d4340p_usb_bridge` into the `src` directory of a
catkin workspace, then build and run:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
chmod +x src/d4340p_usb_bridge/scripts/d4340p_usb_node.py
rosrun d4340p_usb_bridge d4340p_usb_node.py _port:=/dev/ttyACM0
```

If no valid command frame is received for 100 ms, the STM32 disables every
enabled D4340P motor.

Topics and services are private to the node namespace:

- `command` (`sensor_msgs/JointState`): four positions, velocities and efforts.
- `enable` (`std_msgs/Bool`): enables or disables all four motors.
- `feedback` (`sensor_msgs/JointState`): measured motor state.
- `status` (`std_msgs/UInt8MultiArray`): four motor status codes.
- `temperature` (`std_msgs/Float32MultiArray`): MOS/rotor pairs.
- `clear_error` (`std_srvs/Trigger`).
- `set_zero` (`std_srvs/Trigger`).

Set MIT gains with `~kp` and `~kd`, each either one scalar or a four-value list.
The default gain is zero, so enabling alone does not create position stiffness.
