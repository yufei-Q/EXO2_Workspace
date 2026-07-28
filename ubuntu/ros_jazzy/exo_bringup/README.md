# ROS 2 Jazzy `exo_bringup`

该功能包运行于 Ubuntu 24.04 + ROS 2 Jazzy，通过 USB CDC 与 `EXO2_DM_G474` 通讯，并提供7台达妙电机的控制、反馈、状态和温度接口。

- CAN ID 1～4：D4340P。
- CAN ID 5～7：D4310P。
- USB控制帧：156字节。
- USB反馈帧：114字节。
- 默认控制及反馈频率：500 Hz。

## 文件结构

```text
exo_bringup/
├── CMakeLists.txt
├── package.xml
├── LICENSE
├── README.md
├── launch/
│   └── dm_motor_usb.launch.py
└── scripts/
    ├── __init__.py
    ├── node.py
    └── protocol.py
```

`node.py`负责ROS接口、串口收发和安全退出，`protocol.py`负责二进制帧、CRC16及反馈流解析。

## 安装与编译

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/exo_ws/src
cp -r EXO2_Workspace/ubuntu/ros_jazzy/exo_bringup ~/exo_ws/src/
cd ~/exo_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select exo_bringup
source install/setup.bash
```

串口依赖及权限：

```bash
sudo apt update
sudo apt install python3-serial
sudo usermod -aG dialout $USER
```

加入`dialout`后注销并重新登录。连接STM32后执行：

```bash
ls -l /dev/ttyACM*
```

## 启动

```bash
source /opt/ros/jazzy/setup.bash
source ~/exo_ws/install/setup.bash
ros2 launch exo_bringup dm_motor_usb.launch.py
```

覆盖串口或发送频率：

```bash
ros2 launch exo_bringup dm_motor_usb.launch.py \
  port:=/dev/ttyACM1 rate:=500.0
```

也可直接运行节点：

```bash
ros2 run exo_bringup dm_motor_usb_node --ros-args \
  -p port:=/dev/ttyACM0 \
  -p rate:=500.0 \
  -p kp:='[0.0,0.0,0.0,0.0,0.0,0.0,0.0]' \
  -p kd:='[0.0,0.0,0.0,0.0,0.0,0.0,0.0]'
```

## ROS接口

| 名称 | 类型 | 功能 |
|---|---|---|
| `/dm_motor_usb/command` | `sensor_msgs/msg/JointState` | 7台电机位置、速度和前馈力矩目标 |
| `/dm_motor_usb/enable` | `std_msgs/msg/Bool` | 同时使能或失能7台电机 |
| `/dm_motor_usb/feedback` | `sensor_msgs/msg/JointState` | 位置、速度和力矩反馈 |
| `/dm_motor_usb/status` | `std_msgs/msg/UInt8MultiArray` | 7台电机状态码 |
| `/dm_motor_usb/temperature` | `std_msgs/msg/Float32MultiArray` | MOS和转子温度 |
| `/dm_motor_usb/clear_error` | `std_srvs/srv/Trigger` | 清除全部电机错误 |
| `/dm_motor_usb/set_zero` | `std_srvs/srv/Trigger` | 设置全部电机零点 |

数组下标0～6依次对应CAN ID 1～7。`JointState.name`依次为`d4340p_1`～`d4340p_4`、`d4310p_5`～`d4310p_7`。

## 读取反馈

```bash
ros2 topic echo /dm_motor_usb/feedback
ros2 topic echo /dm_motor_usb/status
ros2 topic echo /dm_motor_usb/temperature
ros2 topic hz /dm_motor_usb/feedback
```

位置单位为rad，速度单位为rad/s，力矩单位为N·m。温度数组顺序为：

```text
[电机1 MOS, 电机1转子, ..., 电机7 MOS, 电机7转子]
```

## 安全控制测试

首次测试必须脱离人体和外骨骼负载，并准备硬件急停。先发布完整7元素全零目标：

```bash
ros2 topic pub --once /dm_motor_usb/command sensor_msgs/msg/JointState \
  '{position: [0.0,0.0,0.0,0.0,0.0,0.0,0.0], velocity: [0.0,0.0,0.0,0.0,0.0,0.0,0.0], effort: [0.0,0.0,0.0,0.0,0.0,0.0,0.0]}'
```

清错、使能和失能：

```bash
ros2 service call /dm_motor_usb/clear_error std_srvs/srv/Trigger '{}'
ros2 topic pub --once /dm_motor_usb/enable std_msgs/msg/Bool '{data: true}'
ros2 topic pub --once /dm_motor_usb/enable std_msgs/msg/Bool '{data: false}'
```

设置零点前必须先失能、确认电机静止并位于机械零点：

```bash
ros2 service call /dm_motor_usb/set_zero std_srvs/srv/Trigger '{}'
```

节点保存最近一次完整目标并以500 Hz重复发送。退出时会发送3帧失能指令；STM32超过100 ms未收到有效命令也会自动失能，但软件保护不能代替硬件急停。