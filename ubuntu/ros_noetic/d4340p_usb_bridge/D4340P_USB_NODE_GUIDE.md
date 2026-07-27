# D4340P ROS USB 节点使用说明

本文说明如何使用 `d4340p_usb_node.py` 通过 USB CDC 与 STM32 通信，读取4个达妙 D4340P 电机的数据，并使用 MIT 模式控制电机。

## 1. 系统通信关系

```text
ROS Noetic
  │
  │ d4340p_usb_node.py
  │ 93字节控制帧 / 69字节反馈帧
  ▼
USB CDC
  ▼
STM32
  │
  │ CAN 1 Mbps
  ▼
D4340P：CAN ID 1、2、3、4
```

ROS 节点默认以 250 Hz 向 STM32 发送控制帧。STM32 如果超过100 ms没有收到有效控制帧，会自动失能已经使能的电机。

## 2. 使用前检查

### 2.1 STM32正式通信代码

`usbd_cdc_if.c` 中应使用正式协议接收：

```c
USB_MotorComm_Receive(Buf, *Len);
```

临时回环发送应保持注释：

```c
/* (void)CDC_Transmit_FS(Buf, (uint16_t)(*Len)); */
```

`main.c` 中应启用反馈发送：

```c
(void)USB_MotorComm_SendFeedback();
```

修改后需要重新编译并烧录 STM32。

### 2.2 安全要求

首次控制前建议：

- 电机脱离人体和外骨骼负载；
- 固定电机本体，确保输出端不会碰撞；
- 准备可以快速切断48 V电源的急停；
- 从很小的力矩、位置变化和增益开始；
- 第一次只连接一个 CAN ID 为1的电机；
- 重新使能前始终先发送安全的全零命令。

当前ROS节点会同时使能或失能4个电机，不能单独使能某一个电机。

## 3. 准备ROS环境

以下命令假设使用 Ubuntu 20.04 + ROS Noetic，catkin工作空间为 `~/catkin_ws`。

安装依赖：

```bash
sudo apt update
sudo apt install python3-serial
```

编译工作空间：

```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

检查USB设备：

```bash
ls /dev/ttyACM*
```

通常STM32对应：

```text
/dev/ttyACM0
```

如果提示没有权限，可以临时执行：

```bash
sudo chmod 666 /dev/ttyACM0
```

长期使用建议：

```bash
sudo usermod -aG dialout $USER
```

加入 `dialout` 组后需要注销并重新登录。

## 4. 启动节点

终端一启动ROS Master：

```bash
source /opt/ros/noetic/setup.bash
roscore
```

终端二启动D4340P USB节点：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

rosrun d4340p_usb_bridge d4340p_usb_node.py \
  _port:=/dev/ttyACM0 \
  _rate:=250 \
  _kp:=0.0 \
  _kd:=0.0
```

启动参数：

| 参数 | 含义 | 默认值 |
|---|---|---:|
| `_port` | STM32 USB CDC设备 | `/dev/ttyACM0` |
| `_rate` | 控制帧发送频率 | 250 Hz |
| `_kp` | MIT位置增益 | 0 |
| `_kd` | MIT速度增益 | 0 |

确认节点：

```bash
rosnode list
```

应能看到：

```text
/d4340p_usb
```

## 5. ROS话题和服务

所有接口都是节点私有名称。节点名称为 `d4340p_usb` 时，完整名称如下：

| 名称 | 类型 | 方向/用途 |
|---|---|---|
| `/d4340p_usb/command` | `sensor_msgs/JointState` | 向电机发送目标 |
| `/d4340p_usb/enable` | `std_msgs/Bool` | 同时使能或失能4个电机 |
| `/d4340p_usb/feedback` | `sensor_msgs/JointState` | 位置、速度、力矩反馈 |
| `/d4340p_usb/status` | `std_msgs/UInt8MultiArray` | 电机状态码 |
| `/d4340p_usb/temperature` | `std_msgs/Float32MultiArray` | MOS和转子温度 |
| `/d4340p_usb/clear_error` | `std_srvs/Trigger` | 同时清除4个电机错误 |
| `/d4340p_usb/set_zero` | `std_srvs/Trigger` | 同时设置4个电机零点 |

## 6. 读取电机反馈

### 6.1 位置、速度和力矩

```bash
rostopic echo /d4340p_usb/feedback
```

反馈类型为 `sensor_msgs/JointState`：

- `position`：位置，单位 rad；
- `velocity`：速度，单位 rad/s；
- `effort`：反馈力矩，单位 N·m。

数组与CAN ID的对应关系：

| 数组下标 | CAN ID | 名称 |
|---:|---:|---|
| 0 | 1 | `d4340p_1` |
| 1 | 2 | `d4340p_2` |
| 2 | 3 | `d4340p_3` |
| 3 | 4 | `d4340p_4` |

查看反馈频率：

```bash
rostopic hz /d4340p_usb/feedback
```

注意：当前STM32反馈结构没有 `valid` 标志。刚启动且尚未收到CAN反馈时，也可能看到状态和数据全为0。只有当位置、温度或状态随电机实际状态变化时，才能确认STM32已经收到CAN反馈。

### 6.2 状态码

```bash
rostopic echo /d4340p_usb/status
```

数组中的4个值依次对应CAN ID 1～4。

| 状态码 | 含义 |
|---:|---|
| 0 | 失能 |
| 1 | 使能 |
| 3 | 输出端编码器错误 |
| 4 | 传感器错误 |
| 5 | 电机编码器错误 |
| 8 | 过压 |
| 9 | 欠压 |
| 10 | 过流 |
| 11 | MOS过温 |
| 12 | 电机过温 |
| 13 | 通信丢失 |
| 14 | 过载 |

### 6.3 温度

```bash
rostopic echo /d4340p_usb/temperature
```

数组顺序：

```text
data[0]：电机1 MOS温度
data[1]：电机1转子温度
data[2]：电机2 MOS温度
data[3]：电机2转子温度
data[4]：电机3 MOS温度
data[5]：电机3转子温度
data[6]：电机4 MOS温度
data[7]：电机4转子温度
```

温度单位为摄氏度。

## 7. 清除错误和设置零点

清除4个电机错误：

```bash
rosservice call /d4340p_usb/clear_error "{}"
```

设置4个电机当前机械位置为零点：

```bash
rosservice call /d4340p_usb/set_zero "{}"
```

设置零点前必须确保电机失能、完全静止，并位于所需机械零点。当前服务会同时操作4个电机。

## 8. 首次使能测试

节点启动时使用：

```text
kp = 0
kd = 0
torque = 0
```

先发布全零安全命令：

```bash
rostopic pub -1 /d4340p_usb/command sensor_msgs/JointState \
"{position: [0.0, 0.0, 0.0, 0.0],
  velocity: [0.0, 0.0, 0.0, 0.0],
  effort: [0.0, 0.0, 0.0, 0.0]}"
```

使能全部电机：

```bash
rostopic pub -1 /d4340p_usb/enable std_msgs/Bool "data: true"
```

检查状态是否变为1：

```bash
rostopic echo /d4340p_usb/status
```

测试失能：

```bash
rostopic pub -1 /d4340p_usb/enable std_msgs/Bool "data: false"
```

## 9. 小力矩测试

启动节点时保持 `kp=0`、`kd=0`，先发送全零命令并使能。

给CAN ID 1电机发送 `0.1 N·m` 前馈力矩：

```bash
rostopic pub -1 /d4340p_usb/command sensor_msgs/JointState \
"{position: [0.0, 0.0, 0.0, 0.0],
  velocity: [0.0, 0.0, 0.0, 0.0],
  effort: [0.1, 0.0, 0.0, 0.0]}"
```

反方向可使用负值，例如 `-0.1`。

测试完成后先发送全零命令：

```bash
rostopic pub -1 /d4340p_usb/command sensor_msgs/JointState \
"{position: [0.0, 0.0, 0.0, 0.0],
  velocity: [0.0, 0.0, 0.0, 0.0],
  effort: [0.0, 0.0, 0.0, 0.0]}"
```

然后失能：

```bash
rostopic pub -1 /d4340p_usb/enable std_msgs/Bool "data: false"
```

节点会一直保存并重复发送最近一次目标。重新使能前必须先发布安全目标，否则会继续执行上一次命令。

## 10. 位置控制测试

重新启动节点并设置较小增益：

```bash
rosrun d4340p_usb_bridge d4340p_usb_node.py \
  _port:=/dev/ttyACM0 \
  _rate:=250 \
  _kp:='[2.0, 2.0, 2.0, 2.0]' \
  _kd:='[0.1, 0.1, 0.1, 0.1]'
```

增益在节点启动时读取，运行过程中修改ROS参数不会自动更新节点内部增益。

先发布全零目标，再使能。给电机1发送 `0.05 rad` 目标位置：

```bash
rostopic pub -1 /d4340p_usb/command sensor_msgs/JointState \
"{position: [0.05, 0.0, 0.0, 0.0],
  velocity: [0.0, 0.0, 0.0, 0.0],
  effort: [0.0, 0.0, 0.0, 0.0]}"
```

MIT模式可近似理解为：

```text
输出力矩 = Kp × 位置误差 + Kd × 速度误差 + 前馈力矩
```

STM32驱动的参数限制：

| 参数 | 软件限制 |
|---|---:|
| 位置 | -12.5～12.5 rad |
| 速度 | -20～20 rad/s |
| Kp | 0～500 |
| Kd | 0～5 |
| 力矩 | -28～28 N·m |

这些是通信编码范围，不是安全测试范围。外骨骼测试必须根据机械限位、减速器、电机额定能力和人体安全重新确定更小的限制。

## 11. 在Python节点中读取反馈

```python
#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState


def feedback_callback(message):
    for index in range(4):
        rospy.loginfo(
            "motor%d: position=%.3f rad, velocity=%.3f rad/s, torque=%.3f Nm",
            index + 1,
            message.position[index],
            message.velocity[index],
            message.effort[index],
        )


rospy.init_node("motor_feedback_reader")
rospy.Subscriber(
    "/d4340p_usb/feedback",
    JointState,
    feedback_callback,
    queue_size=10,
)
rospy.spin()
```

## 12. 在Python节点中发送命令

```python
#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


rospy.init_node("motor_command_example")

command_pub = rospy.Publisher(
    "/d4340p_usb/command", JointState, queue_size=1
)
enable_pub = rospy.Publisher(
    "/d4340p_usb/enable", Bool, queue_size=1
)

rospy.sleep(1.0)

command = JointState()
command.position = [0.0, 0.0, 0.0, 0.0]
command.velocity = [0.0, 0.0, 0.0, 0.0]
command.effort = [0.0, 0.0, 0.0, 0.0]

command_pub.publish(command)
rospy.sleep(0.1)
enable_pub.publish(Bool(data=True))

try:
    rospy.spin()
finally:
    command.effort = [0.0, 0.0, 0.0, 0.0]
    command_pub.publish(command)
    enable_pub.publish(Bool(data=False))
    rospy.sleep(0.1)
```

实际控制程序应持续发布目标，并设计独立的急停、限位、通信超时和状态检查逻辑。

## 13. 当前实现的限制

当前 `d4340p_usb_node.py` 有以下限制：

- 固定支持4个D4340P电机；
- 一个 `enable` 话题同时使能或失能4个电机；
- `clear_error` 和 `set_zero` 同时作用于4个电机；
- `JointState.name` 在控制回调中不参与电机映射，实际映射只由数组下标决定；
- 命令数组少于4个元素时，对应字段会继续使用以前保存的值；
- 增益只在节点启动时读取；
- STM32反馈中没有CAN反馈有效标志和接收时间戳。

因此发送命令时应始终提供完整的4元素 `position`、`velocity` 和 `effort` 数组。

## 14. 推荐的安全停止顺序

1. 发布全零力矩或安全位置目标；
2. 发布 `/d4340p_usb/enable = false`；
3. 确认状态变为0；
4. 停止ROS节点；
5. 最后切断48 V电机电源。

如果ROS节点意外停止，STM32会在超过100 ms没有收到有效控制帧后自动失能电机，但这不能替代硬件急停。
