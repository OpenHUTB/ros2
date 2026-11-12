# 电机温度时序预测器

本包对用户提供的温度时序 CSV 拟合紧凑的多项式回归模型，并在后续时间段评估预测误差。不导入仿真器、传感器记录器或控制器包。

## CSV 格式

必需字段：`time_s`、`speed_mps`、`left_temperature_c`、`left_current_a` 和 `ambient_c`。数据行必须按时间严格递增。预测目标为当前时刻之后达到指定预测时域的第一条左电机测温记录。

## 运行

从包目录 `src/ground/mujoco_motor_temperature_predictor` 执行下列命令。NumPy 兼容 1.21 及更高版本；ROS 2 Humble 可使用系统包 `python3-numpy`。

```bash
python3 sample_data.py --output output/sample_temperature_series.csv --samples 500
python3 main.py --csv output/sample_temperature_series.csv --horizon 5.0 --model output/temperature_model.json --results output/forecast_results.json
```

预测时域统一为 **5.0 秒**，也是 Python API、命令行与 launch 的默认值；`--horizon` 和 launch `horizon` 参数均以秒为单位，可设置其他正数时域。预测目标取当前时刻加上该时域之后的第一条测温记录。

前 80% 的有效时间序列用于拟合；训练/测试边界隔离一个完整预测时域，最后 20% 用于计算 RMSE。示例生成器输出带固定种子噪声的合成温度时序，用于复现运行流程；评估结果对应该输入序列及所选预测时域。

500 点示例、5.0 秒时域运行结果为 `horizon=5.0 s train=309 test=90 RMSE=1.087 C`。模型 JSON 保存时域与回归参数，结果 JSON 保存留出区间逐点预测、目标温度与 RMSE。

## ROS 2 启动

从仓库根目录执行；先生成示例 CSV，再构建与启动：

```bash
source /opt/ros/humble/setup.bash
python3 src/ground/mujoco_motor_temperature_predictor/sample_data.py --output /tmp/sample_temperature_series.csv --samples 500
colcon build --symlink-install --packages-select mujoco_motor_temperature_predictor
source install/setup.bash
ros2 launch mujoco_motor_temperature_predictor evaluate.launch.py csv:=/tmp/sample_temperature_series.csv horizon:=5.0 model:=/tmp/temperature_model.json results:=/tmp/forecast_results.json
```

## 测试

在 ROS 工作区根目录执行：

```bash
colcon test --packages-select mujoco_motor_temperature_predictor --python-testing pytest --return-code-on-test-failure
colcon test-result --verbose
```

若构建使用 `--merge-install`，测试命令也加入该选项。
