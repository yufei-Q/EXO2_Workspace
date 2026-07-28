# ROS 1 Noetic `exo_bringup`

该功能包运行于 Ubuntu 20.04 + ROS 1 Noetic，通过 USB CDC 与 `EXO2_DM_G474` 通讯。接口、协议和电机数组映射与ROS 2版本一致。

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
│   └── dm_motor_usb.launch
└── scripts/
    ├── __init__.py
    ├── node.py
    └── protocol.py
```

## 安装与编译

```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/catkin_ws/src
cp -r EXO2_Workspace/ubuntu/ros_noetic/exo_bringup ~/catkin_ws/src/
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

安装串口依赖并配置权限：

```bash
sudo apt update
sudo apt install python3-serial
sudo usermod -aG dialout $USER
```

加入`dialout`后注销并重新登录。

## 启动

终端一：

```bash
source /opt/ros/noetic/setup.bash
roscore
```

终端二：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch exo_bringup dm_motor_usb.launch
```

覆盖串口或发送频率：

```bash
roslaunch exo_bringup dm_motor_usb.launch \
  port:=/dev/ttyACM1 rate:=500.0
```

也可直接运行：

```bash
rosrun exo_bringup node.py \
  _port:=/dev/ttyACM0 \
  _rate:=500.0 \
  _kp:='[0.0,0.0,0.0,0.0,0.0,0.0,0.0]' \
  _kd:='[0.0,0.0,0.0,0.0,0.0,0.0,0.0]'
```

## ROS接口

| 名称 | 类型 | 功能 |
|---|---|---|
| `/dm_motor_usb/command` | `sensor_msgs/JointState` | 7台电机位置、速度和前馈力矩目标 |
| `/dm_motor_usb/enable` | `std_msgs/Bool` | 同时使能或失能7台电机 |
| `/dm_motor_usb/feedback` | `sensor_msgs/JointState` | 位置、速度和力矩反馈 |
| `/dm_motor_usb/status` | `std_msgs/UInt8MultiArray` | 7台电机状态码 |
| `/dm_motor_usb/temperature` | `std_msgs/Float32MultiArray` | MOS和转子温度 |
| `/dm_motor_usb/clear_error` | `std_srvs/Trigger` | 清除全部电机错误 |
| `/dm_motor_usb/set_zero` | `std_srvs/Trigger` | 设置全部电机零点 |

数组下标0～6依次对应CAN ID 1～7。

## 读取反馈

```bash
rostopic echo /dm_motor_usb/feedback
rostopic echo /dm_motor_usb/status
rostopic echo /dm_motor_usb/temperature
rostopic hz /dm_motor_usb/feedback
```

## 安全控制测试

首次测试必须脱离人体和外骨骼负载，保持Kp、Kd和力矩为0，并准备硬件急停。

```bash
rostopic pub -1 /dm_motor_usb/command sensor_msgs/JointState \
'{position: [0,0,0,0,0,0,0], velocity: [0,0,0,0,0,0,0], effort: [0,0,0,0,0,0,0]}'

rosservice call /dm_motor_usb/clear_error '{}'
rostopic pub -1 /dm_motor_usb/enable std_msgs/Bool 'data: true'
rostopic pub -1 /dm_motor_usb/enable std_msgs/Bool 'data: false'
```

设置零点前必须失能并保证电机完全静止：

```bash
rosservice call /dm_motor_usb/set_zero '{}'
```

节点保存最近一次完整目标并以500 Hz重复发送。退出时会发送3帧失能指令；STM32超过100 ms未收到有效命令也会自动失能，但软件保护不能代替硬件急停。