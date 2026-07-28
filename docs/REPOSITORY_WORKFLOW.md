# 仓库使用规则

## 总体原则

- 一个GitHub仓库：`EXO2_Workspace`。
- 一个长期分支：`main`。
- 所有电脑克隆完整仓库。
- Windows、ROS 1和ROS 2代码都提交到同一个 `main`。
- Git只保存源码、工程配置和说明文档，不保存可重新生成的编译产物。

## 各电脑主要修改范围

| 环境 | 主要目录 |
|---|---|
| Windows + STM32CubeMX/Keil | `stm32/` |
| Ubuntu 20.04 + ROS 1 Noetic | `ubuntu/ros_noetic/` |
| Ubuntu 24.04 + ROS 2 Jazzy | `ubuntu/ros_jazzy/` |

不同电脑可以查看和修改全部代码，但一次提交应只包含本次实际需要的变化。

## 开始工作

```bash
git switch main
git pull --rebase origin main
git status
```

只有在工作区干净或确认现有修改属于自己时再开始工作。

## 提交代码

```bash
git status
git diff
git add -A
git status
git commit -m 'feat(stm32): describe the change'
git push origin main
```

推荐提交前缀：

- `feat(stm32):` STM32新功能。
- `fix(stm32):` STM32问题修复。
- `feat(ros1):` ROS 1功能。
- `feat(ros2):` ROS 2功能。
- `fix(protocol):` 三端通信协议修复。
- `docs:` 文档修改。
- `chore:` 仓库结构或工具配置。

## 多台电脑同步

一台电脑推送后，其他电脑继续修改前先执行：

```bash
git pull --rebase origin main
```

如果本地已经有未提交修改，不要直接强制拉取。先提交修改，或者确认内容后暂存到stash，再进行同步。

## 通信协议同步要求

修改USB帧格式、电机数量、量程或标志位时，必须同时检查：

1. `stm32/EXO2_DM_G474/Core/Src/usb_motor_comm.c`
2. `stm32/EXO2_DM_G474/Core/Src/dm_motor.c`
3. `ubuntu/ros_noetic/exo_bringup/scripts/node.py`
4. `ubuntu/ros_jazzy/exo_bringup/scripts/protocol.py`
5. `stm32/EXO2_DM_G474/G474_MOTOR_PROTOCOL.md`

不能只修改其中一端后直接连接电机测试。

## 不应提交的内容

- Keil的AXF、HEX、OBJ、MAP、日志和个人界面配置。
- ROS 1的 `build/`、`devel/`。
- ROS 2的 `build/`、`install/`、`log/`。
- Python的 `__pycache__/` 和 `.pyc`。
- 编辑器、操作系统和临时文件。

这些内容由根目录 `.gitignore` 自动排除。
