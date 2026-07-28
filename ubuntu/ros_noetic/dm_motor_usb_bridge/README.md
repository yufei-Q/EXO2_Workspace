# ROS 1 Noetic 七电机 USB Bridge

本目录只用于 Ubuntu 20.04 + ROS 1 Noetic，对应 `EXO2_DM_G474` STM32G474七电机固件。

- CAN ID 1～4：D4340P。
- CAN ID 5～7：D4310P。
- 控制帧156字节，反馈帧114字节。
- 默认控制发送频率500 Hz。

## 编译

```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/catkin_ws/src
cp -r dm_motor_usb_bridge ~/catkin_ws/src/
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

安装串口依赖和配置权限：

```bash
sudo apt update
sudo apt install python3-serial
sudo usermod -aG dialout $USER
```

加入 `dialout` 后需要注销并重新登录。

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
rosrun dm_motor_usb_bridge dm_motor_usb_node.py \
  _port:=/dev/ttyACM0 \
  _rate:=500.0 \
  _kp:='[0.0,0.0,0.0,0.0,0.0,0.0,0.0]' \
  _kd:='[0.0,0.0,0.0,0.0,0.0,0.0,0.0]'
```

## 接口

| 名称 | 类型 | 功能 |
|---|---|---|
| `/dm_motor_usb/command` | `sensor_msgs/JointState` | 7台电机目标位置、速度和前馈力矩 |
| `/dm_motor_usb/enable` | `std_msgs/Bool` | 同时使能或失能7台电机 |
| `/dm_motor_usb/feedback` | `sensor_msgs/JointState` | 位置、速度和力矩反馈 |
| `/dm_motor_usb/status` | `std_msgs/UInt8MultiArray` | 7台电机状态码 |
| `/dm_motor_usb/temperature` | `std_msgs/Float32MultiArray` | MOS和转子温度 |
| `/dm_motor_usb/clear_error` | `std_srvs/Trigger` | 清除7台电机错误 |
| `/dm_motor_usb/set_zero` | `std_srvs/Trigger` | 设置7台电机零点 |

数组下标0～6分别对应 CAN ID 1～7。命令应始终发送完整的7元素 position、velocity 和 effort 数组。

读取反馈：

```bash
rostopic echo /dm_motor_usb/feedback
rostopic echo /dm_motor_usb/status
rostopic echo /dm_motor_usb/temperature
rostopic hz /dm_motor_usb/feedback
```

发布全零安全目标：

```bash
rostopic pub -1 /dm_motor_usb/command sensor_msgs/JointState \
'{position: [0,0,0,0,0,0,0], velocity: [0,0,0,0,0,0,0], effort: [0,0,0,0,0,0,0]}'
```

使能和失能：

```bash
rostopic pub -1 /dm_motor_usb/enable std_msgs/Bool 'data: true'
rostopic pub -1 /dm_motor_usb/enable std_msgs/Bool 'data: false'
```

清错和设零：

```bash
rosservice call /dm_motor_usb/clear_error '{}'
rosservice call /dm_motor_usb/set_zero '{}'
```

首次测试必须脱离人体和外骨骼负载，保持 kp、kd、力矩为0并准备硬件急停。设置零点前必须失能且保证电机完全静止。节点正常退出时会尝试发送3帧失能指令，STM32在100 ms命令超时后也会失能，但都不能代替硬件急停。
