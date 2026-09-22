# exo_identify

七自由度外骨骼的 MuJoCo 动力学数据采集、Pinocchio 线性参数辨识、独立验证，以及
ROS 2 实机采集/重力补偿工具。默认模型是：

```text
urdf/exo.SLDASM/urdf/装配体.SLDASM.urdf
```

该模型包含 `joint1` 到 `joint7`、完整惯量和 SolidWorks STL 网格。旧的
`urdf/estimator_kinematics.urdf` 仅保留作历史参考，不再是默认辨识模型。

`urdf/exo.SLDASM/` 保留 SolidWorks 导出的完整 URDF 文件夹，包括网格、关节名
配置及原始 ROS 1 导出元数据。当前 ROS 2 程序直接读取其中的
`urdf/装配体.SLDASM.urdf` 和 `meshes/`；其余导出文件作为模型来源和兼容参考保留，
不会被当前 ROS 2 主流程主动调用。

### 单独检查 MuJoCo 模型方向

在确定安装坐标系前，仅使用 `scripts/mujoco_model.py` 检查模型，不运行数据采集
或辨识。该脚本的姿态、关节、重力和自由体选项只作用于本次生成的 MuJoCo 模型，
不会修改 URDF 或辨识配置。

直接运行脚本会读取统一配置、生成预览 MJCF，并打开交互式 MuJoCo Viewer。省略
`--base-rpy` 和 `--base-pos` 时，程序读取 `config/identification.json` 中
`simulation.base_orientation_rpy` 和 `simulation.base_position`：

```bash
cd ~/exo_ws/src/exo_identify
python3 scripts/mujoco_model.py
```

无图形桌面或只想做无窗口检查时使用 `--no-viewer`；需要保存 PNG 时才显式加入
`--preview`，默认写入 `results/seven_dof/model_visualization/exo_orientation.png`：

```bash
python3 scripts/mujoco_model.py \
  --joint-positions 0 0 0 0 0 0 0 \
  --show-frames --fixed-base --no-viewer --preview
```

测试候选方向时，可以直接用弧度覆盖配置，例如绕世界 Y 轴旋转 90 度：

```bash
python3 scripts/mujoco_model.py \
  --base-pos 0 0 0 \
  --base-rpy 0 1.5708 0 \
  --joint-positions 0 0 0 0 0 0 0 \
  --show-frames --fixed-base --no-viewer \
  --preview results/seven_dof/model_visualization/exo_orientation_y90.png
```

有图形桌面时也可以显式写 `--viewer` 查看交互窗口；两者不能同时使用：

```bash
python3 scripts/mujoco_model.py --show-frames --fixed-base --viewer
```

`--base-rpy` 使用弧度，顺序为世界坐标中的 XYZ roll/pitch/yaw；`--gravity`
是 MuJoCo 世界坐标中的重力向量，必须显式给出。坐标轴颜色为 X 红、Y 绿、Z 蓝；
黄色线段指向重力方向。基座默认固定在 MuJoCo 世界坐标中，`--fixed-base` 用于
显式确认该模式；此时只有各关节在重力下运动，外骨骼整体不会下落。若之后需要
整体自由下落，再使用 `--free-base --contacts` 或组合选项 `--drop`。自由基座模式
下，MuJoCo 窗口中可使用其标准鼠标扰动操作拖动自由体。无图形桌面时使用
`--preview` 生成静态 PNG；它不会模拟掉落。若省略
`--output`，预览 MJCF 默认写入 `results/seven_dof/model_visualization/`；脚本不再默认
生成 PNG，也不使用 `/tmp` 保存结果。

## 1. 安装与构建

系统需要 ROS 2 Jazzy、NumPy、SciPy、Pinocchio、Matplotlib 和 Pillow。
MuJoCo 单独安装：

```bash
python3 -m pip install -r ~/exo_ws/src/exo_identify/requirements-mujoco.txt

cd ~/exo_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select exo_bringup exo_identify
source install/setup.bash
```

`~/exo_ws/src/exo_identify` 和 `~/exo_ws/src/exo_bringup` 是指向 Git 仓库的软连接；
因此只需修改一份源码，两端会保持一致。修改后仍需重新构建已安装的 ROS 包。

## 2. MuJoCo 完整动力学辨识与仿真部署

完整仿真流程由五个可独立执行的阶段组成：

```text
design_excitation.py
    → collect_mujoco_dynamics.py
    → identify_parameters.py
    → export_gravity_formula.py
    → validate_mujoco_compensation.py
```

如果希望一次完成全部仿真阶段，可以运行调度脚本：

```bash
python3 scripts/mujoco_dynamics_pipeline.py all
```

调度脚本也支持逐阶段执行。每个阶段都会调用对应的独立脚本：

```bash
python3 scripts/mujoco_dynamics_pipeline.py design
python3 scripts/mujoco_dynamics_pipeline.py collect
python3 scripts/mujoco_dynamics_pipeline.py identify
python3 scripts/mujoco_dynamics_pipeline.py export
python3 scripts/mujoco_dynamics_pipeline.py validate
```

因此 `pipeline` 只负责顺序和默认路径，不再把采集、辨识和验证的实现隐藏在同一个
文件中。

仿真采集会读取：

```text
results/seven_dof/excitation/excitation_id.csv
results/seven_dof/excitation/excitation_validation.csv
```

并生成带 `_sim` 后缀的数据和模型：

```text
results/seven_dof/simulation_model/exo7_sim.xml
results/seven_dof/simulation_data/dynamics_id_sim.csv
results/seven_dof/simulation_data/dynamics_validation_sim.csv
results/seven_dof/dynamics_identification/identified_parameters_sim.npz
results/seven_dof/dynamics_identification/prediction_id_sim.csv
results/seven_dof/dynamics_identification/prediction_validation_sim.csv
results/seven_dof/dynamics_identification/identification_report_sim.json
results/seven_dof/dynamics_identification/gravity_formula_sim.json
results/seven_dof/compensation_validation/compensation_validation_sim.csv
results/seven_dof/compensation_validation/compensation_report_sim.json
```

`dynamics_id_sim.csv` 中的力矩来自 MuJoCo 前向动力学中的执行器反馈：位置/速度
指令先经过虚拟伺服器，MuJoCo 用 `mj_step` 积分，然后读取实际的 `q`、`dq`、
`qacc` 和 `qfrc_actuator`。模型包含配置的关节阻尼、等效转子惯量和关节摩擦损失，
并默认加入小幅控制扰动、外部力矩扰动和测量噪声；辨识程序强制打开摩擦项。
`identified_parameters_sim.npz`
同时包含刚体参数和 `Fv/Fc` 摩擦参数，可作为补偿节点的仿真模型。

当前虚拟伺服器参数集中写在 `config/identification.json`：`Kp=20`、`Kd=1`、
力矩限幅为 `8 N·m`。采集器不使用模型前馈力矩；因此记录的是控制器真正施加后，
MuJoCo 前向积分产生的执行器反馈。需要改变扰动或传感器噪声时，只修改同一文件
`simulation.noise` 下的六组标准差，或用 `--torque-noise-std` 临时覆盖力矩测量噪声。

`validate` 阶段（`all` 会自动执行）使用独立激励验证轨迹，分别比较 MuJoCo 的重力、
被动摩擦和辨识模型输出，并生成 `compensation_validation_sim.csv` 与
`compensation_report_sim.json`。这是部署前的仿真闭环检查，不等同于实机安全测试；
其中摩擦模型采用平滑 `tanh`，在库仑摩擦速度过零处出现的误差会单独记录。

确认正确方向后，把角度写入 `config/identification.json` 的
`simulation.base_orientation_rpy`。当前方向配置为：

```json
"simulation": {
  "base_position": [0.0, 0.0, 0.2],
  "base_orientation_rpy": [1.5708, 0.0, 0.0]
}
```

顶层的 `gravity: [0.0, 0.0, -9.81]` 仍表示 MuJoCo 世界坐标中的重力，不要为了补偿
基座旋转而手工改成其他方向。采集脚本将基座旋转应用到 MuJoCo 模型；Pinocchio
辨识模型会自动把同一个世界重力转换到 URDF 基座坐标系，因此两边使用的是同一个
物理方向。`base_position` 同理用于设置基座在世界坐标中的位置。

修改方向后，应使用 `--regenerate-trajectory` 重新生成激励、采集、辨识和验证结果：

```bash
python3 scripts/mujoco_dynamics_pipeline.py all --regenerate-trajectory
```

之后直接运行 `mujoco_model.py` 会从该配置读取相同姿态，不需要手工编辑 XML。

`simulation_*`、`compensation_validation/` 和所有 `_sim` 文件只代表仿真数据。实物采集数据使用
`hardware_experiments/` 或用户指定的实物结果目录，不应覆盖仿真文件。
模型可视化目录只在显式使用 `mujoco_model.py --preview` 时生成 PNG；直接运行脚本只打开
Viewer 并生成 MJCF，不会自动保存 PNG。

## 3. 阶段一：生成七自由度动态激励轨迹

完整动力学/摩擦辨识继续使用原有线性回归流程，现已按 `model.nv` 支持七关节：

```bash
python3 scripts/design_excitation.py \
  --output-dir results/seven_dof/excitation
```

输出 `excitation_id.csv`、`excitation_validation.csv`、
`base_parameter_set.npz` 和 `excitation_report.json`。默认轨迹范围、速度和加速度在
`config/identification.json` 中统一配置为七个元素。

## 4. 阶段二：采集 MuJoCo 仿真辨识数据

仿真数据采集现在有单独入口，不需要通过总 pipeline 才能执行：
具体的前向动力学、虚拟伺服、扰动注入和传感器读取都实现在
`scripts/collect_mujoco_dynamics.py`；`scripts/mujoco_dynamics_pipeline.py` 只负责
按顺序调用各阶段，不包含另一套采集实现。

```bash
python3 scripts/collect_mujoco_dynamics.py
```

如果需要在数据计算的同时观察 MuJoCo 当前姿态，可以显式打开实时 Viewer：

```bash
python3 scripts/collect_mujoco_dynamics.py --viewer
```

窗口显示的是采集循环正在计算的当前轨迹样本，不是采集结束后再读取 CSV 的回放。
默认按轨迹真实时间运行；例如 `--viewer-speed 2` 可以用两倍速度观察。无图形界面时
不要加 `--viewer`。一键流程也支持相同选项：

```bash
python3 scripts/mujoco_dynamics_pipeline.py all --viewer
```

该脚本只负责将激励轨迹输入 MuJoCo，运行前向动力学并记录执行器反馈 `tau`。
它不进行参数辨识、公式导出或补偿验证。输出为：

```text
results/seven_dof/simulation_model/exo7_sim.xml
results/seven_dof/simulation_data/dynamics_id_sim.csv
results/seven_dof/simulation_data/dynamics_validation_sim.csv
results/seven_dof/simulation_data/simulation_metadata_sim.json
```

## 5. 阶段三和四：参数辨识与重力公式导出

如果已有七关节动态数据（仿真或实物），可运行通用辨识程序。仿真数据建议保留
`_sim` 后缀：

```bash
python3 scripts/identify_parameters.py \
  --id-data results/seven_dof/simulation_data/dynamics_id_sim.csv \
  --validation-data results/seven_dof/simulation_data/dynamics_validation_sim.csv \
  --base-set results/seven_dof/excitation/base_parameter_set.npz \
  --output-dir results/seven_dof/dynamics_identification \
  --friction on \
  --suffix sim
```

若下游系统只能读取 JSON，可把辨识参数精确展开为七维三角公式：

```bash
python3 scripts/export_gravity_formula.py \
  --parameters results/seven_dof/dynamics_identification/identified_parameters_sim.npz \
  --base-set results/seven_dof/excitation/base_parameter_set.npz \
  --output results/seven_dof/dynamics_identification/gravity_formula_sim.json
```

七维公式会比 `.npz` 大；ROS 补偿节点优先直接读取 `.npz`，这样没有公式近似误差。

独立执行仿真补偿验证：

```bash
python3 scripts/validate_mujoco_compensation.py
```

该脚本只比较 MuJoCo 参考重力/摩擦项与辨识模型输出，不会修改辨识参数。

## 6. 实机七关节采集

实机映射统一位于 `config/hardware_mapping.yaml`，默认是七路一一映射：

```yaml
motor_indices: [0, 1, 2, 3, 4, 5, 6]
joint_directions: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
```

`experiment.yaml` 和 `gravity_compensation.yaml` 只包含各自节点的功能参数；
不要在这两个文件中重复维护电机映射、方向、零位或传动比例。两个 launch
文件默认都会读取 `config/hardware_mapping.yaml`，也可以通过
`hardware_mapping_file:=...` 临时指定另一份标定文件。

实物采集必须逐步执行，不放入仿真一键 pipeline。轨迹实验 launch 已内置默认轨迹
`results/seven_dof/excitation/excitation_id.csv`，因此可以直接启动：

```bash
ros2 launch exo_identify trajectory_experiment.launch.py
```

仍可通过 `trajectory_file:=...` 指定其他轨迹。

这些只是仿真一致的占位值。连接实机前必须校准每个电机的零点、方向、减速比、
力矩比例及机械限位，不可直接沿用默认配置。

```bash
ros2 launch exo_identify trajectory_experiment.launch.py \
  trajectory_file:=$PWD/results/seven_dof/excitation/excitation_id.csv

ros2 service call /exo_identify/prepare std_srvs/srv/Trigger '{}'
ros2 topic pub --once /dm_motor_usb/enable std_msgs/msg/Bool '{data: true}'
ros2 service call /exo_identify/start std_srvs/srv/Trigger '{}'
```

采集节点会保存七路原始反馈和处理后的 `measured_id.csv`。正常停止使用
平滑减速；需要软件立即禁用时调用：

```bash
ros2 service call /exo_identify/emergency_stop std_srvs/srv/Trigger '{}'
```

反馈超时、位置/速度越界也会自动立即禁用。该软件服务不能替代独立的硬件急停。
已有的 2 自由度生成结果保留在 `results/two_dof/`，当前七自由度流程默认写入
`results/seven_dof/`，两者不会混用。

## 7. 实机/仿真重力与摩擦补偿部署

补偿节点可直接加载 `.npz` 线性动力学模型。模型中的刚体参数用于重力项，
辨识出的 `Fv/Fc` 参数用于摩擦项：

默认模型是仿真辨识得到的
`results/seven_dof/dynamics_identification/identified_parameters_sim.npz`，因此可以直接启动：

```bash
ros2 launch exo_identify gravity_compensation.launch.py
```

```bash
ros2 launch exo_identify gravity_compensation.launch.py \
  formula_file:=$PWD/results/seven_dof/dynamics_identification/identified_parameters_sim.npz
```

也兼容 `gravity_formula_sim.json`（该 JSON 主要用于重力公式展示）。部署实物时，
应将 `formula_file` 指向经过实物数据重新辨识并验证的模型，不要直接把 `_sim` 参数
当作实物最终标定结果。安全比例、七关节力矩限幅和映射位于
`config/gravity_compensation.yaml`。准备、渐增和停止：

```bash
ros2 service call /exo_identify/gravity/prepare std_srvs/srv/Trigger '{}'
ros2 topic pub --once /dm_motor_usb/enable std_msgs/msg/Bool '{data: true}'
ros2 service call /exo_identify/gravity/start std_srvs/srv/Trigger '{}'
ros2 service call /exo_identify/gravity/stop std_srvs/srv/Trigger '{}'
```

首次实机测试必须脱离人体、可靠支撑、降低补偿比例并准备硬件急停。仿真辨识只能
证明算法与 URDF/MuJoCo 一致，不能替代真实电机方向、传动比例和机械安全校准。
