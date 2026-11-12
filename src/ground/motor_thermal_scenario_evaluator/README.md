# 电机热场景评价工具

本包自带一阶电机热模型和三个工况，比较固定速度与温度迟滞降额策略，可独立构建和运行。

## Run

```bash
python3 -m pip install .
motor_thermal_evaluate --steps 6000 --output output/thermal_evaluation
python3 -m motor_thermal_scenario_evaluator.plot --csv output/thermal_evaluation/scenario_curves.csv --output docs/img/ground/motor_thermal_scenario_evaluator/scenario_curves.png
```

以上命令在本包目录执行。

## ROS 2 launch

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select motor_thermal_scenario_evaluator
source install/setup.bash
ros2 launch motor_thermal_scenario_evaluator evaluate.launch.py steps:=6000
```

程序生成逐样本曲线 CSV、六行对比表和 JSON 汇总。
