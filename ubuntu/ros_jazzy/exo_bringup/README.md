# ROS 2 Jazzy `exo_bringup`

该功能包运行于 Ubuntu 24.04 + ROS 2 Jazzy，通过 USB CDC 与 `EXO2_DM_G474` 通讯，并提供7台达妙电机的控制、反馈、状态和温度接口。

- CAN ID 1～4：D4340P。
- CAN ID 5～7：D4310P。
- USB控制帧：156字节。
- USB反馈帧：114字节。
- 默认控制及反馈频率：500 Hz。
- 7台电机共享同一个全局控制模式：MIT模式或速度模式。

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
    ├── control_gui.py
    ├── node.py
    └── protocol.py
```

`node.py`负责ROS接口、串口收发和安全退出，`protocol.py`负责二进制帧、CRC16及反馈流解析，`control_gui.py`提供可选的图形化调试控制界面。

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
sudo apt install python3-serial python3-tk
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

同时启动可视化调试界面：

```bash
ros2 launch exo_bringup dm_motor_usb.launch.py use_gui:=true
```

`use_gui`默认为`false`，因此不指定时只启动USB桥接节点。GUI需要桌面图形环境和有效的`DISPLAY`变量。

GUI默认将两种电机的速度限制为±1.0 rad/s、前馈力矩限制为±1.0 N·m。位置范围保持为±12.5 rad。D4340P与D4310P的速度和力矩范围可以分别设置：

```bash
ros2 launch exo_bringup dm_motor_usb.launch.py use_gui:=true \
  gui_d4340_velocity_limit:=0.5 gui_d4340_torque_limit:=0.3 \
  gui_d4310_velocity_limit:=0.4 gui_d4310_torque_limit:=0.2
```

D4340P参数的最大允许值为20 rad/s和28 N·m，D4310P参数的最大允许值为30 rad/s和12.5 N·m；所有限制必须大于0。输入框和滑块使用对应型号的相同限幅，超出范围的输入会被截断到边界值。

覆盖串口或发送频率：

```bash
ros2 launch exo_bringup dm_motor_usb.launch.py \
  port:=/dev/ttyACM1 rate:=500.0
```

### 设置MIT模式的kp和kd

`kp`和`kd`都是长度必须为7的浮点数组，下标0～6对应CAN ID 1～7。固件允许的
范围为`0 <= kp <= 500`、`0 <= kd <= 5`。它们只在MIT模式中参与控制，速度
模式会忽略这两个参数。

通过launch在节点启动时设置：

```bash
ros2 launch exo_bringup dm_motor_usb.launch.py \
  kp:='[10.0,10.0,10.0,10.0,5.0,5.0,5.0]' \
  kd:='[0.5,0.5,0.5,0.5,0.2,0.2,0.2]'
```

节点运行期间可以动态修改，无需重启：

```bash
ros2 param set /dm_motor_usb kp \
  '[12.0,12.0,12.0,12.0,6.0,6.0,6.0]'
ros2 param set /dm_motor_usb kd \
  '[0.6,0.6,0.6,0.6,0.3,0.3,0.3]'
```

读取当前值：

```bash
ros2 param get /dm_motor_usb kp
ros2 param get /dm_motor_usb kd
```

数组长度、非有限数或超出固件范围的数值会被节点拒绝。失能时USB帧中的
`kp/kd`仍强制为零；运行期间设置的新值会保存在节点中，并在下次使能后使用。
修改已使能电机的增益会从下一帧立即生效，因此应先失能、修改参数、确认数值，
再使用安全目标重新使能。

也可直接运行节点：

```bash
ros2 run exo_bringup dm_motor_usb_node --ros-args \
  -p port:=/dev/ttyACM0 \
  -p rate:=500.0 \
  -p kp:='[0.0,0.0,0.0,0.0,0.0,0.0,0.0]' \
  -p kd:='[0.0,0.0,0.0,0.0,0.0,0.0,0.0]'
```

桥接节点已经运行时，也可以单独启动GUI：

```bash
ros2 run exo_bringup dm_motor_control_gui
```

## 可视化调试界面

GUI提供以下功能：

- 选择1～7号电机，并分别保存每台电机的位置、速度和前馈力矩目标。
- 每个控制量都支持输入框和滑块，发送时始终发布完整的7元素`JointState`。
- “发送一次”只发送一次当前目标；“持续发送”按照界面设置的0.1～100 Hz频率重复发布目标。
- MIT/速度模式切换。切换模式时自动发送失能并将全部目标清零。
- 显示7台电机的位置、速度、力矩、状态和温度反馈。
- “使能全部电机”需要二次确认；“失能并清零”和关闭窗口都会发送失能与全零目标。

GUI的持续发送频率不需要等于USB桥接节点的500 Hz。桥接节点会保存最近一次ROS目标，并独立以500 Hz向STM32发送。

> **安全提示：** GUI是调试工具，不能代替硬件急停。连续发送目标不会自动使能；使能操作作用于全部7台电机。MIT模式下位置控制还取决于桥接节点启动时配置的`kp`、`kd`。

## ROS接口

| 名称 | 类型 | 功能 |
|---|---|---|
| `/dm_motor_usb/command` | `sensor_msgs/msg/JointState` | 7台电机位置、速度和前馈力矩目标 |
| `/dm_motor_usb/enable` | `std_msgs/msg/Bool` | 同时使能或失能7台电机 |
| `/dm_motor_usb/control_mode` | `std_msgs/msg/UInt8` | 全局控制模式：0为MIT，1为速度模式 |
| `/dm_motor_usb/feedback` | `sensor_msgs/msg/JointState` | 位置、速度和力矩反馈 |
| `/dm_motor_usb/status` | `std_msgs/msg/UInt8MultiArray` | 7台电机状态码 |
| `/dm_motor_usb/temperature` | `std_msgs/msg/Float32MultiArray` | MOS和转子温度 |
| `/dm_motor_usb/clear_error` | `std_srvs/srv/Trigger` | 清除全部电机错误 |
| `/dm_motor_usb/set_zero` | `std_srvs/srv/Trigger` | 设置全部电机零点 |

数组下标0～6依次对应CAN ID 1～7。`JointState.name`依次为`d4340p_1`～`d4340p_4`、`d4310p_5`～`d4310p_7`。

## 控制模式

7台电机始终使用同一种全局控制模式，不能分别选择模式：

- `0`：MIT模式，使用`position`、`velocity`、`effort`以及节点参数`kp`、`kd`。
- `1`：速度模式，只使用`velocity`，STM32发送`0x200 + CAN ID`的速度控制帧。

切换模式会自动将`enable`置为`false`，并清零已保存的位置、速度和力矩目标。模式切换完成后，必须重新发送安全目标，再单独发送使能命令。

切换为速度模式：

```bash
ros2 topic pub --once /dm_motor_usb/control_mode \
  std_msgs/msg/UInt8 '{data: 1}'
```

发送7台电机的目标速度：

```bash
ros2 topic pub --once /dm_motor_usb/command sensor_msgs/msg/JointState \
  '{position: [0,0,0,0,0,0,0], velocity: [0.1,0.1,0.1,0.1,0.1,0.1,0.1], effort: [0,0,0,0,0,0,0]}'
```

切换回MIT模式：

```bash
ros2 topic pub --once /dm_motor_usb/control_mode \
  std_msgs/msg/UInt8 '{data: 0}'
```

切回MIT模式后，如果`kp`不为0，重新使能前应将每台电机的目标位置设置为其当前反馈位置，避免位置突变。

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

首次测试必须脱离人体和外骨骼负载，并准备硬件急停。先选择控制模式并确认电机处于失能状态，再发布完整7元素安全目标：

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

使能时，节点保存最近一次完整目标并以500 Hz重复发送。失能时，节点仍以相同
频率发送有效USB命令帧，但会清除使能标志，并强制将位置、速度、`kp`、`kd`
和力矩全部置零，避免失能帧携带残留目标。退出时会发送3帧这样的失能全零
指令；STM32超过100 ms未收到有效命令也会自动失能，但软件保护不能代替硬件
急停。

注意：ROS节点持续发送失能全零USB帧，并不等于STM32会在CAN侧持续发送控制
帧。若电机只在收到CAN控制帧后返回反馈，失能状态下能否获得新的实时反馈由
STM32固件的失能分支决定；不能通过在ROS端伪造使能标志解决，否则电机实际会
进入使能状态。
