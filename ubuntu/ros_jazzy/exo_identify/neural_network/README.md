# 神经网络动力学辨识

该目录提供一个纯NumPy的两层MLP，用激励实验数据直接学习完整逆动力学：

```text
[sin(q1), cos(q1), sin(q2), cos(q2), dq1, dq2, ddq1, ddq2]
                                  -> [tau1, tau2]
```

输出是URDF关节坐标系下的总关节力矩，训练目标中同时包含重力、惯性、科氏/离心、
摩擦和未建模效应。角度使用正余弦编码，避免`-pi/pi`附近出现人为不连续。

## 训练

```bash
cd ~/exo_ws/src/exo_identify

ros2 run exo_identify train_neural_dynamics.py \
  --id-data experiment_output/run_ID/measured_id.csv \
  --validation-data experiment_output/run_VALIDATION/measured_id.csv \
  --output-dir neural_identify_result
```

输出：

```text
neural_identify_result/
├── neural_dynamics_model.npz
├── training_report.json
├── training_history.csv
├── prediction_id.csv
└── prediction_validation.csv
```

`training_config.json`可调整隐藏层、学习率、批大小和早停参数。验证集仅用于模型选择
和报告，不参与梯度更新。

训练报告中的`deployment_gate.deployment_recommended`必须为`true`才表示模型通过
基础离线门槛。单条周期ID轨迹在六维状态空间中的覆盖很窄，即使样本数量很多，
黑箱网络也可能记住轨迹而无法泛化；此时应采集多条相位、频率和幅值不同的ID轨迹，
不能用更大的网络掩盖验证误差。

多个ID训练文件可以一次传入，最后一个参数之后再写验证集：

```bash
ros2 run exo_identify train_neural_dynamics.py \
  --id-data run_id_1/measured_id.csv run_id_2/measured_id.csv \
  --validation-data run_validation/measured_id.csv \
  --output-dir neural_identify_result
```

## 离线复核

```bash
ros2 run exo_identify predict_neural_dynamics.py \
  --model neural_identify_result/neural_dynamics_model.npz \
  --data experiment_output/run_VALIDATION/measured_id.csv \
  --output neural_identify_result/prediction_check.csv \
  --report neural_identify_result/prediction_check.json
```

## 后续实时控制接口

```python
from neural_dynamics_model import NeuralDynamicsModel

model = NeuralDynamicsModel.load('neural_dynamics_model.npz')
tau = model.predict(q=[q1, q2], dq=[dq1, dq2], ddq=[ddq1, ddq2])
```

若只做重力和摩擦前馈，可令`ddq=[0,0]`；若做轨迹逆动力学前馈，应输入期望轨迹的
`q,dq,ddq`。实时部署时仍必须保留零增益MIT模式、逐关节力矩比例、力矩限幅、启动
斜坡、反馈超时、速度保护和训练数据范围检查。神经网络不能替代这些保护。
