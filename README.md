# EXO2 Workspace

本仓库统一管理外骨骼控制系统的 STM32 固件、ROS 1 Noetic 上位机和 ROS 2 Jazzy 上位机代码。所有电脑都使用同一个 `main` 分支并下载完整仓库。

## 目录结构

```text
EXO2_Workspace/
├── stm32/
│   ├── EXO2_DM_G474/       STM32G474 七电机正式工程
│   ├── EXO2_DM_MOTOR/      STM32F407 旧版四电机工程
│   └── motor_test/         STM32F407 电机测试工程
├── ubuntu/
│   ├── ros_noetic/         Ubuntu 20.04 + ROS 1 Noetic
│   └── ros_jazzy/          Ubuntu 24.04 + ROS 2 Jazzy
└── docs/
    └── archive/            历史测试说明
```

当前正式配套代码：

- STM32：[stm32/EXO2_DM_G474](stm32/EXO2_DM_G474)
- ROS 1：[ubuntu/ros_noetic/dm_motor_usb_bridge](ubuntu/ros_noetic/dm_motor_usb_bridge)
- ROS 2：[ubuntu/ros_jazzy/dm_motor_usb_bridge](ubuntu/ros_jazzy/dm_motor_usb_bridge)
- 通信协议：[stm32/EXO2_DM_G474/G474_MOTOR_PROTOCOL.md](stm32/EXO2_DM_G474/G474_MOTOR_PROTOCOL.md)

三套正式代码均对应7台电机：CAN ID 1～4为D4340P，CAN ID 5～7为D4310P；USB控制帧156字节，反馈帧114字节。

## 分支规则

仓库只长期维护 `main`。Windows、Ubuntu 20.04和Ubuntu 24.04电脑都直接在 `main` 上工作。

每次开始修改前：

```bash
git switch main
git pull --rebase origin main
```

修改和测试完成后：

```bash
git status
git diff
git add -A
git status
git commit -m '说明本次修改'
git push origin main
```

提交前必须检查 `git status`。不要提交Keil、catkin、colcon和Python生成文件。

更完整的协作规则见 [docs/REPOSITORY_WORKFLOW.md](docs/REPOSITORY_WORKFLOW.md)。

## STM32工程

正式工程文件：

```text
stm32/EXO2_DM_G474/EXO2_DM_G474.ioc
stm32/EXO2_DM_G474/MDK-ARM/EXO2_DM_G474.uvprojx
```

Keil编译输出不会进入Git。其他电脑第一次编译时，Keil会重新生成输出目录和HEX文件。

## ROS文档

- ROS 1安装和运行：[ubuntu/ros_noetic/dm_motor_usb_bridge/README.md](ubuntu/ros_noetic/dm_motor_usb_bridge/README.md)
- ROS 2安装和运行：[ubuntu/ros_jazzy/dm_motor_usb_bridge/README.md](ubuntu/ros_jazzy/dm_motor_usb_bridge/README.md)

ROS 1和ROS 2是两套独立代码，不要在两个目录之间直接复制构建产物。
