# ROS 2 Jazzy 七电机 USB Bridge

该包运行于 Ubuntu 24.04 + ROS 2 Jazzy，通过 USB CDC 与 `EXO2_DM_G474` 固件通信。

- CAN ID 1～4：D4340P。
- CAN ID 5～7：D4310P。
- ROS 2 控制帧默认发送频率：500 Hz。
- STM32 控制与反馈频率：每台电机500 Hz。
- USB 控制帧：156字节。
- USB 反馈帧：114字节。

## 1. 安装和编译

将本包放入 ROS 2 工作空间：

```bash
mkdir -p ~/jazzy_ws/src
cp -r dm_motor_usb_bridge ~/jazzy_ws/src/
sudo apt update
sudo apt install python3-serial
source /opt/ros/jazzy/setup.bash
cd ~/jazzy_ws
colcon build --symlink-install --packages-select dm_motor_usb_bridge
source install/setup.bash
```

将当前用户加入串口设备组：

```bash
sudo usermod -aG dialout $USER
```

执行后注销并重新登录。连接 STM32 后检查：

```bash
ls -l /dev/ttyACM*
```

## 2. 启动

使用 launch 文件：

```bash
source /opt/ros/jazzy/setup.bash
source ~/jazzy_ws/install/setup.bash
ros2 launch dm_motor_usb_bridge dm_motor_usb.launch.py
```

也可以直接运行并覆盖参数：

```bash
ros2 run dm_motor_usb_bridge dm_motor_usb_node --ros-args \
  -p port:=/dev/ttyACM0 \
  -p rate:=500.0 \
  -p kp:='[0.0,0.0,0.0,0.0,0.0,0.0,0.0]' \
  -p kd:='[0.0,0.0,0.0,0.0,0.0,0.0,0.0]'
```

USB CDC 的 `115200` 参数只是满足虚拟串口接口，实际 USB 传输速度不由这个波特率限制。

## 3. ROS 2 接口

| 名称 | 类型 | 功能 |
|---|---|---|
| `/dm_motor_usb/command` | `sensor_msgs/msg/JointState` | 7台电机目标位置、速度和前馈力矩 |
| `/dm_motor_usb/enable` | `std_msgs/msg/Bool` | 同时使能或失能7台电机 |
| `/dm_motor_usb/feedback` | `sensor_msgs/msg/JointState` | 位置、速度和力矩反馈 |
| `/dm_motor_usb/status` | `std_msgs/msg/UInt8MultiArray` | 7台电机状态码 |
| `/dm_motor_usb/temperature` | `std_msgs/msg/Float32MultiArray` | 每台电机的 MOS、转子温度 |
| `/dm_motor_usb/clear_error` | `std_srvs/srv/Trigger` | 同时清除7台电机错误 |
| `/dm_motor_usb/set_zero` | `std_srvs/srv/Trigger` | 同时设置7台电机零点 |

数组映射：

| 下标 | CAN ID | JointState 名称 |
|---:|---:|---|
| 0 | 1 | `d4340p_1` |
| 1 | 2 | `d4340p_2` |
| 2 | 3 | `d4340p_3` |
| 3 | 4 | `d4340p_4` |
| 4 | 5 | `d4310p_5` |
| 5 | 6 | `d4310p_6` |
| 6 | 7 | `d4310p_7` |

## 4. 读取反馈

```bash
ros2 topic echo /dm_motor_usb/feedback
ros2 topic echo /dm_motor_usb/status
ros2 topic echo /dm_motor_usb/temperature
ros2 topic hz /dm_motor_usb/feedback
```

`feedback.position` 单位为 rad，`feedback.velocity` 为 rad/s，`feedback.effort` 为 N·m。

温度数组顺序为：

```text
[电机1 MOS, 电机1转子, 电机2 MOS, 电机2转子, ... , 电机7 MOS, 电机7转子]
```

## 5. 安全通讯测试

电机首次测试必须脱离人体和外骨骼负载，并准备硬件急停。先保持 `kp=0`、`kd=0` 和全部力矩为0。

发布完整的7元素安全目标：

```bash
ros2 topic pub --once /dm_motor_usb/command sensor_msgs/msg/JointState \
  '{position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'
```

清错：

```bash
ros2 service call /dm_motor_usb/clear_error std_srvs/srv/Trigger '{}'
```

使能：

```bash
ros2 topic pub --once /dm_motor_usb/enable std_msgs/msg/Bool '{data: true}'
```

仅向1号电机发送 `0.1 N·m` 前馈力矩：

```bash
ros2 topic pub --once /dm_motor_usb/command sensor_msgs/msg/JointState \
  '{position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], effort: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'
```

停止时先发布全零目标，再失能：

```bash
ros2 topic pub --once /dm_motor_usb/enable std_msgs/msg/Bool '{data: false}'
```

设置零点前必须失能电机、确保电机完全静止并位于机械零点：

```bash
ros2 service call /dm_motor_usb/set_zero std_srvs/srv/Trigger '{}'
```

## 6. 工作机制和限制

- 节点保存最近一次完整目标，并以500 Hz重复发送。
- `JointState` 的 `name` 字段不参与命令映射，映射只由数组下标决定。
- 某个命令数组少于7个元素时，该字段继续使用上一次目标。因此控制程序应始终发送完整7元素数组。
- `kp`、`kd` 在节点启动时读取，必须各有7个元素。
- clear_error 和 set_zero 是一次性标志，对全部7台电机生效。
- USB 读取按字节流缓存，可处理一个114字节反馈帧被拆成多个 USB 包的情况。
- 节点正常退出时会尝试连续发送3帧失能指令；STM32超过100 ms没有收到有效命令也会自动失能，但两者都不能代替硬件急停。
- 实机前必须核对 STM32 `dm_motor.c` 中两种电机的 PMAX、VMAX、TMAX 与电机内部参数一致。
