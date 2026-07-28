# STM32 与 ROS Noetic USB CDC 回环测试说明

## 1. 测试目的

本测试用于验证以下 USB CDC 通信链路：

```text
Ubuntu 20.04 / ROS Noetic
        ↓
      USB CDC
        ↓
STM32 接收数据并原样返回
```

ROS 节点每秒发送一条短字符串，读取 STM32 返回的数据并进行比较。本测试只验证 USB 通信，不依赖 CAN 总线和电机。

> 建议关闭电机 48 V 电源，只连接 STM32 的 USB 数据线。

## 2. 分支和文件

| 电脑 | 开发分支 | 内容 |
|---|---|---|
| Windows | `windows-stm32` | STM32 工程和临时 USB 回环代码 |
| Ubuntu | `ubuntu-ros` | ROS Noetic 包和 Python 测试节点 |

ROS 测试节点：

```text
ubuntu/ros_noetic/d4340p_usb_bridge/scripts/usb_loopback_test.py
```

本测试面向 Ubuntu 20.04 + ROS Noetic。Ubuntu 26.04 不作为本次 ROS Noetic 测试环境。

## 3. 测试前同步代码

Windows 电脑保持在 `windows-stm32`：

```bash
git branch --show-current
git push origin windows-stm32
```

Ubuntu 电脑保持在 `ubuntu-ros`：

```bash
git branch --show-current
git pull origin ubuntu-ros
```

不要在 Windows 的日常 STM32 工作目录中切换到 `ubuntu-ros`。

## 4. Windows：编译并烧录 STM32

### 4.1 检查临时回环代码

文件：

```text
EXO2_DM_MOTOR/USB_DEVICE/App/usbd_cdc_if.c
```

`CDC_Receive_FS()` 中应调用：

```c
(void)CDC_Transmit_FS(Buf, (uint16_t)(*Len));
```

正式协议接收代码在测试期间保留为注释：

```c
/* USB_MotorComm_Receive(Buf, *Len); */
```

文件：

```text
EXO2_DM_MOTOR/Core/Src/main.c
```

正式反馈发送在测试期间保留为注释：

```c
/* (void)USB_MotorComm_SendFeedback(); */
```

关闭正式反馈可以避免二进制电机反馈帧和回环字符串混在一起。

### 4.2 烧录步骤

1. 用 Keil 打开 `EXO2_DM_MOTOR.uvprojx`。
2. 重新编译工程。
3. 将程序烧录到 STM32。
4. 烧录完成后重新连接 USB。
5. 本测试不需要连接 CAN 和电机。

## 5. Ubuntu：配置 ROS 测试环境

以下命令假设仓库位于 `~/EXO2_Workspace`。

### 5.1 检查分支和文件

```bash
cd ~/EXO2_Workspace
git branch --show-current
ls ubuntu/ros_noetic/d4340p_usb_bridge/scripts/usb_loopback_test.py
```

当前分支应显示：

```text
ubuntu-ros
```

### 5.2 安装依赖

```bash
sudo apt update
sudo apt install python3-serial
```

### 5.3 将 ROS 包加入 catkin 工作空间

```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/catkin_ws/src

ln -s ~/EXO2_Workspace/ubuntu/ros_noetic/d4340p_usb_bridge \
  ~/catkin_ws/src/d4340p_usb_bridge

cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

如果软链接已经存在，不要重复执行 `ln -s`。

## 6. 检查 USB 设备和权限

连接 STM32 后执行：

```bash
ls /dev/ttyACM*
```

通常会显示：

```text
/dev/ttyACM0
```

长期使用建议加入 `dialout` 组：

```bash
sudo usermod -aG dialout $USER
```

执行后需要注销并重新登录。

仅用于当前测试时，也可以临时执行：

```bash
sudo chmod 666 /dev/ttyACM0
```

## 7. 启动回环测试

### 7.1 终端一：启动 ROS Master

```bash
source /opt/ros/noetic/setup.bash
roscore
```

### 7.2 终端二：启动测试节点

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

rosrun d4340p_usb_bridge usb_loopback_test.py \
  _port:=/dev/ttyACM0
```

正常时会持续显示：

```text
PASS: b'ROS_USB_TEST_000000\n'
PASS: b'ROS_USB_TEST_000001\n'
```

如果实际设备为 `/dev/ttyACM1`，将启动参数改为：

```bash
rosrun d4340p_usb_bridge usb_loopback_test.py \
  _port:=/dev/ttyACM1
```

### 7.3 终端三：查看 ROS 结果话题

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rostopic echo /usb_loopback_test/ok
```

通信正常时会持续输出：

```text
data: True
---
```

## 8. 常见问题

### 找不到 `/dev/ttyACM0`

检查 USB 数据线、STM32 供电和 USB 接口，并在插拔设备前后分别执行：

```bash
ls /dev/ttyACM*
```

### 提示 `Permission denied`

临时执行：

```bash
sudo chmod 666 /dev/ttyACM0
```

或者加入 `dialout` 组后注销并重新登录。

### 提示缺少 `serial` 模块

```bash
sudo apt install python3-serial
```

### ROS 找不到 `d4340p_usb_bridge`

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 一直显示 `FAIL`，返回数据为空

检查：

- STM32 是否已经烧录临时回环程序；
- ROS 使用的串口设备是否正确；
- 串口是否被串口助手等程序占用；
- `CDC_Receive_FS()` 是否调用了 `CDC_Transmit_FS()`。

### 返回二进制杂乱数据

确认 `main.c` 中正式反馈发送已经暂时注释：

```c
/* (void)USB_MotorComm_SendFeedback(); */
```

## 9. 测试结束后恢复正式通信

在 `usbd_cdc_if.c` 中注释临时回环发送：

```c
/* (void)CDC_Transmit_FS(Buf, (uint16_t)(*Len)); */
```

恢复正式协议接收：

```c
USB_MotorComm_Receive(Buf, *Len);
```

在 `main.c` 中恢复正式反馈：

```c
(void)USB_MotorComm_SendFeedback();
```

然后重新编译并烧录 STM32。

如果临时回环仍对应独立提交 `ded3500`，也可以在 `windows-stm32` 执行：

```bash
git revert ded3500
```

`git revert` 会生成恢复提交，不会破坏已有 Git 历史。

## 10. 测试通过标准

同时满足以下条件即可认为 USB CDC 通信正常：

- Ubuntu 能识别对应的 `/dev/ttyACM*` 设备；
- 测试节点持续输出 `PASS`；
- `/usb_loopback_test/ok` 持续发布 `True`；
- 连续运行一段时间没有丢包或串口异常。
