# 七自由度神经网络动力学辨识

该目录提供纯 NumPy MLP，用七关节实验数据学习关节总力矩模型。输入按 CSV 自动推断
关节数；当前七自由度特征为：

```text
[sin(q1),cos(q1),...,sin(q7),cos(q7),dq1,...,dq7,ddq1,...,ddq7]
                                      -> [tau1,...,tau7]
```

训练和验证数据必须使用主 README 中的标准七关节 CSV 格式，且必须是不同采集记录：

脚本默认使用完整动力学仿真生成的
`results/seven_dof/simulation_data/dynamics_id_sim.csv` 和
`results/seven_dof/simulation_data/dynamics_validation_sim.csv`，因此可以直接运行：

```bash
python3 neural_network/train_neural_dynamics.py
```

```bash
ros2 run exo_identify train_neural_dynamics.py \
  --id-data run_id_1/measured_id.csv run_id_2/measured_id.csv \
  --validation-data run_validation/measured_id.csv \
  --output-dir results/seven_dof/neural_identification
```

离线复核：

```bash
ros2 run exo_identify predict_neural_dynamics.py \
  --model results/seven_dof/neural_identification/neural_dynamics_model.npz \
  --data run_validation/measured_id.csv \
  --output results/seven_dof/neural_identification/prediction_check.csv \
  --report results/seven_dof/neural_identification/prediction_check.json
```

Python 调用：

```python
from neural_dynamics_model import NeuralDynamicsModel

model = NeuralDynamicsModel.load('neural_dynamics_model.npz')
tau = model.predict(q=q7, dq=dq7, ddq=ddq7)
```

`training_config.json` 中两个部署阈值数组必须各包含七个元素。只有独立验证报告中
`deployment_recommended` 为 `true`，并且完成硬件方向、比例、限幅、斜坡、反馈超时
和训练域保护后，才可以考虑实机前馈。神经网络不是本项目默认的重力补偿模型；
默认路径是可解释的 Pinocchio 线性重力回归。
