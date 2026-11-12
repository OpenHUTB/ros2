# 电机温度预测评估

本包从按时间排序的 CSV 序列评估五秒电机温度预测，以温度、电流、车速和环境温度拟合多项式回归模型。预测时域默认 **5.0 秒**，Python API、命令行和 ROS 2 launch 的口径一致；可用 `--horizon` 或 launch `horizon` 参数设置其他正数秒数。

## 环境与运行

运行环境为 Ubuntu 22.04、ROS 2 Humble 和 Python 3.10。NumPy 支持 1.21 及更高版本，使用 Humble 系统自带的 NumPy 1.21.5 完成验证。`package.xml` 声明 `python3-numpy`、`launch` 和 `ament_index_python` 运行时依赖。

从仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
python3 src/ground/mujoco_motor_temperature_predictor/sample_data.py --output /tmp/sample_temperature_series.csv --samples 500
colcon build --merge-install --symlink-install --packages-select mujoco_motor_temperature_predictor
source install/setup.bash
ros2 launch mujoco_motor_temperature_predictor evaluate.launch.py csv:=/tmp/sample_temperature_series.csv horizon:=5.0 model:=/tmp/temperature_model.json results:=/tmp/forecast_results.json
```

示例生成器输出带固定种子噪声的合成时序，不依赖仿真器。输入 CSV 必须包含 `time_s`、`speed_mps`、`left_temperature_c`、`left_current_a` 和 `ambient_c`，时间戳严格递增。目标值取当前时刻加五秒之后的第一条测温记录；训练/测试边界隔离一个预测时域。

## 验证

4 项测试覆盖预测目标对齐、时间留出、乱序拒绝、样例生成与默认五秒命令行契约。构建后从工作区根目录执行：

```bash
colcon test --merge-install --packages-select mujoco_motor_temperature_predictor --python-testing pytest --return-code-on-test-failure
colcon test-result --verbose
```

500 点示例、5.0 秒预测时域的运行输出为：

```text
horizon=5.0 s train=309 test=90 RMSE=1.087 C
```

模型参数写入 `temperature_model.json`，留出段的时间、真实温度、预测温度和 RMSE 写入 `forecast_results.json`。该 RMSE 对应上述合成输入序列及五秒预测时域。
