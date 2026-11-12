# 电机传感器 CSV 记录器

本包为通用电机状态 CSV 加入可复现的温度和电流测量噪声。包名沿用电机仿真贡献主题，记录器本身可独立运行，不导入也不要求 MuJoCo 或其他仿真器包。

## 输入 CSV

必需字段：`time_s`、`speed_mps`、`left_temperature_c`、`right_temperature_c`、`left_torque_nm`、`right_torque_nm` 和 `ambient_c`。

## 运行

```bash
python3 -m pip install .
motor_sensor_record --input config/motor_state.csv --output output/motor_sensors.csv --seed 7
```

以上命令在本包目录执行；输入路径 `config/motor_state.csv` 相对于当前目录。

PowerShell 用法：

```powershell
Set-Location src/ground/mujoco_motor_sensor_recorder
python -m pip install .
motor_sensor_record --input .\config\motor_state.csv --output .\output\motor_sensors.csv --seed 7
```

也可在安装后使用 Python 模块入口：`python -m mujoco_motor_sensor_recorder.main --input .\config\motor_state.csv --output .\output\motor_sensors.csv --seed 7`。

温度噪声标准差为 0.12 °C，电流噪声标准差为 0.04 A。电流由转矩按 0.18 Nm/A 转矩常数估算，并加入 0.25 A 空载电流。

所有 CSV 数值字段统一保留六位小数，避免不同操作系统数学库带来的浮点末位差异；相同示例输入和 seed 可得到一致的文本输出。

## ROS 2 启动

```bash
# 以下命令均在 ROS 工作区根目录执行
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mujoco_motor_sensor_recorder
source install/setup.bash
ros2 launch mujoco_motor_sensor_recorder record.launch.py seed:=7
```

launch 默认从已安装包的 share 目录读取示例 CSV；可通过 `input`、`output` 和 `seed` 覆盖默认参数。

## 测试

在 ROS 工作区根目录运行：

```bash
colcon test --packages-select mujoco_motor_sensor_recorder --python-testing pytest --return-code-on-test-failure
colcon test-result --verbose
```

若构建时使用了 `--merge-install`，测试命令也需加入该选项。测试依赖通过 `package.xml` 中的 `python3-pytest` 声明。
