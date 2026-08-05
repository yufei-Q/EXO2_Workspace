# exo_identify

二自由度外骨骼的激励轨迹生成、实机数据采集、动力学参数辨识和重力项导出工具。

## 1. 构建

依赖ROS 2、NumPy、SciPy和Pinocchio。构建`exo_bringup`与`exo_identify`：

```bash
cd ~/exo_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select exo_bringup exo_identify
source install/setup.bash
```

每次修改代码后都需要重新执行`colcon build`并重新`source install/setup.bash`。

## 2. 生成激励轨迹

```bash
cd ~/exo_ws/src/exo_identify
ros2 run exo_identify design_excitation.py --output-dir excitation_trajectory
```

主要输出：

- `excitation_trajectory/excitation_id.csv`：辨识激励轨迹；
- `excitation_trajectory/excitation_validation.csv`：独立验证轨迹；
- `excitation_trajectory/base_parameter_set.npz`：基参数列集合；
- `excitation_trajectory/excitation_report.json`：轨迹指标。

当前设计边界为`-180°～180°`，ID轨迹保留5°余量，实际覆盖`-175°～175°`。
实机执行时放慢3倍，激励段运行90秒：

```text
q1最大速度约0.840 rad/s
q2最大速度约0.945 rad/s
```

## 3. 核对实机配置

实机参数位于`config/experiment.yaml`。当前映射为：

```yaml
motor_indices: [0, 6]       # joint1 -> CAN ID 1，joint2 -> CAN ID 7
joint_directions: [1.0, 1.0]
motor_zero_positions: [0.0, 0.0]
motor_position_per_joint_radian: [1.0, 1.0]
joint_torque_per_motor_torque: [1.0, 1.0]
```

运行前必须确认：

1. 电机零点与URDF零位一致；
2. 两关节方向正确；
3. 编码器和力矩均已转换为输出轴SI单位；
4. `±180°`范围内没有机械、人体或线缆干涉，并有可靠硬件限位；
5. 实验中不会跨越编码器`-12.5/12.5 rad`跳变边界。

程序中的坐标转换为：

```text
q_joint   = direction * (q_motor - motor_zero) / position_scale
dq_joint  = direction * dq_motor / position_scale
tau_joint = direction * torque_scale * tau_motor
```

## 4. 启动控制和采集

以下launch会同时启动`exo_bringup`和激励采集节点，不要另外重复启动bringup：

```bash
cd ~/exo_ws/src/exo_identify

ros2 launch exo_identify trajectory_experiment.launch.py \
  trajectory_file:=$PWD/excitation_trajectory/excitation_id.csv \
  kp:='[2.0,0.0,0.0,0.0,0.0,0.0,2.0]' \
  kd:='[0.1,0.0,0.0,0.0,0.0,0.0,0.1]'
```

首次测试应脱离人体、降低增益和轨迹幅值，并准备硬件急停。

## 5. 准备、使能和开始

另开一个已经source工作区的终端：

```bash
source /opt/ros/jazzy/setup.bash
source ~/exo_ws/install/setup.bash
```

检查反馈：

```bash
ros2 topic hz /dm_motor_usb/feedback
ros2 topic echo --once /dm_motor_usb/feedback
```

锁定当前姿态并发布保持命令：

```bash
ros2 service call /exo_identify/prepare std_srvs/srv/Trigger '{}'
ros2 topic echo --once /dm_motor_usb/command
```

确认目标、方向和机械状态安全后，使能并启动：

```bash
ros2 topic pub --once /dm_motor_usb/enable std_msgs/msg/Bool '{data: true}'
ros2 service call /exo_identify/start std_srvs/srv/Trigger '{}'
```

程序会依次执行：当前位置保持、平滑进入轨迹、90秒激励、平滑停止、保存数据和
自动失能。需要提前停止时：

```bash
ros2 service call /exo_identify/stop std_srvs/srv/Trigger '{}'
```

## 6. 检查采集结果

数据保存在启动launch时当前目录下的：

```text
experiment_output/run_YYYYMMDD_HHMMSS/
```

其中：

- `experiment_raw.csv`：完整原始反馈，必须保留；
- `measured_id.csv`：自动重采样和滤波后的辨识数据；
- `metadata.json`：映射、轨迹范围和处理参数。

`measured_id.csv`格式为：

```text
t,q1,q2,dq1,dq2,ddq1,ddq2,tau1,tau2
```

处理流程为500 Hz均匀化、四阶0.5 Hz Butterworth零相位低通、降采样到100 Hz、
删除首尾各2秒。该过程由采集节点自动完成，不需要额外运行滤波脚本。

## 7. 参数辨识

```bash
ros2 run exo_identify identify_parameters.py \
  --id-data experiment_output/run_YYYYMMDD_HHMMSS/measured_id.csv \
  --validation-data experiment_output/run_VALIDATION/measured_id.csv \
  --base-set excitation_trajectory/base_parameter_set.npz \
  --output-dir identify_result \
  --friction on
```

建议再使用`excitation_validation.csv`完成一次独立实机采集，并加入：

```bash
--validation-data experiment_output/run_VALIDATION/measured_id.csv
```

主要辨识结果为：

- `identify_result/identified_parameters.npz`；
- `identify_result/identification_report.json`；
- `identify_result/prediction_id.csv`；
- `identify_result/prediction_validation.csv`（提供验证数据时）。

## 8. 导出重力和摩擦参数

完成辨识并检查独立验证误差后：

```bash
ros2 run exo_identify export_gravity_formula.py \
  --parameters identify_result/identified_parameters.npz \
  --base-set excitation_trajectory/base_parameter_set.npz \
  --output identify_result/gravity_formula.json
```

不要在缺少独立验证、力矩比例未确认或参数明显不合理时直接部署辨识结果。
生成的`gravity_formula.json`包含位置相关的重力公式，以及辨识出的`Fv`、`Fc`和
摩擦过渡速度；补偿节点直接读取该文件。

## 9. 重力和低比例摩擦补偿测试

停止其他bringup、轨迹控制和GUI节点，再启动独立重力补偿launch：

```bash
ros2 launch exo_identify gravity_compensation.launch.py \
  formula_file:=$PWD/identify_result/gravity_formula.json
```

该launch强制七路`kp=0`、`kd=0`。补偿节点发送的七路位置和速度也始终为0，
只在CAN ID 1和7的力矩字段写入命令。默认配置位于
`config/gravity_compensation.yaml`：

```yaml
gravity_compensation_scale: 0.1
friction_compensation_scale: [0.1, 0.1]
max_gravity_torque: [3.0, 2.0]
max_friction_torque: [0.15, 0.05]
ramp_duration: 3.0
```

`gravity_compensation_scale`控制整体重力比例；`friction_compensation_scale`
按`[关节1, 关节2]`分别控制摩擦比例，每个值的范围均为`[0, 1]`。节点随后分别用
`max_gravity_torque`和
`max_friction_torque`限幅，再将两项相加。摩擦模型为
`Fv*dq + Fc*tanh(dq/vs)`；把摩擦比例或摩擦限幅设置为零即可恢复纯重力补偿。
最终单关节力矩的理论最大绝对值是对应的两个限幅之和。

另开终端，先准备零力矩命令：

```bash
ros2 service call /exo_identify/gravity/prepare std_srvs/srv/Trigger '{}'
ros2 param get /dm_motor_usb kp
ros2 param get /dm_motor_usb kd
ros2 topic echo --once /dm_motor_usb/command
```

确认KP、KD、位置、速度和力矩均为0后，使能电机，再让补偿从0渐增到配置比例：

```bash
ros2 topic pub --once /dm_motor_usb/enable std_msgs/msg/Bool '{data: true}'
ros2 service call /exo_identify/gravity/start std_srvs/srv/Trigger '{}'
```

停止并自动失能：

```bash
ros2 service call /exo_identify/gravity/stop std_srvs/srv/Trigger '{}'
```

重力补偿节点不设置角度限幅，机械角度安全完全由操作者和硬件限位保证。首次测试
只能在脱离人体、可靠支撑、硬件急停可用的台架上进行。若关节向重力方向主动
加速，立即急停；不要继续提高补偿比例。只有确认两关节方向、力矩单位和力矩限幅
均正确后，先逐步提高两个补偿比例；只有发生限幅且确认安全时，才考虑提高对应
力矩限幅。修改配置后需要重启节点。
