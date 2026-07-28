# Windows STM32 USB 电机测试工具

该工具通过 Windows 的 USB CDC 虚拟串口直接与 `EXO2_DM_G474` 通讯，不依赖 ROS。协议与 STM32 当前的 7 电机 USB 协议一致：控制帧 156 字节，反馈帧 114 字节，默认控制和反馈频率为 500 Hz。

## 安装

安装 Python 3，然后在 PowerShell 中进入本目录：

```powershell
py -m pip install -r requirements.txt
```

列出串口：

```powershell
py .\stm32_motor_console.py --list
```

启动程序（把 `COM3` 换成实际端口）：

```powershell
py .\stm32_motor_console.py --port COM3 --rate 500
```

串口的 `115200` 仅用于满足虚拟串口接口参数；USB CDC 的实际传输不由传统 UART 波特率决定。

## 安全测试顺序

开始前确保 ROS 节点、串口助手等程序已经关闭，同一时刻只能有一个程序打开该 COM 口。首次测试只连接1号电机，并确保可以随时切断48 V电源。

程序启动后默认持续发送“全部失能、全部目标为零”的安全帧。

先检查 STM32 USB feedback：

```text
motor> stats
motor> feedback
```

`RX` 应持续增加。电机未使能时，反馈值可能为零或保持上一次数据。

只使能1号电机，控制量仍保持全零：

```text
motor> enable 1
```

设置1号电机MIT目标，参数依次为位置、速度、Kp、Kd、前馈力矩：

```text
motor> set 1 0 0 0 0 0
```

建议先保持 `Kp=0`、`Kd=0`、`torque=0`，确认反馈正常后再从很小的增益和目标开始。

读取最新反馈：

```text
motor> feedback
```

失能1号电机：

```text
motor> disable 1
```

其他命令：

```text
clear 1       清除1号电机错误
zero 1        将1号电机当前位置设为零位
enable all    使能全部电机（首次测试不要使用）
disable all   失能全部电机
stats         查看USB收发及CRC计数
quit          失能全部电机并退出
```

退出程序时会连续发送5帧全部失能指令，然后关闭串口。

## 判断结果

- `TX` 增加、`RX` 增加：Windows 与 STM32 USB 双向通讯正常。
- `TX` 增加、`RX=0`：STM32 没有向电脑返回数据，或打开了错误的 COM 口。
- `CRC errors` 增加：上下位机协议或数据流存在不一致。
- feedback 持续收到但电机字段不变化：USB 正常，应检查 STM32 与电机的 FDCAN 接收。