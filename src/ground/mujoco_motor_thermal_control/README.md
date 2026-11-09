# MuJoCo 差速小车电机温升与降额控制

本模块使用 MuJoCo 仿真差速小车的左右轮驱动，并用集总热模型计算绕组温度。后续阶段将在此基础上增加温度传感、神经网络预测、ROS 2 控制和多场景评测。

## 运行环境

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- MuJoCo 3.2 或更高版本、NumPy、pytest

## Docker 构建与测试

从仓库根目录执行：

```bash
docker build -t mujoco-motor-thermal:humble -f src/ground/mujoco_motor_thermal_control/Dockerfile .
docker run --rm mujoco-motor-thermal:humble bash -lc "source /opt/ros/humble/setup.bash && source install/setup.bash && colcon test --packages-select mujoco_motor_thermal_control && python3 src/mujoco_motor_thermal_control/main.py --steps 600"
```

## 本地运行

```bash
python3 -m pip install "numpy<2" "mujoco>=3.2,<4" pytest
python3 src/ground/mujoco_motor_thermal_control/main.py --steps 600 --linear 0.55
```

`--steps` 控制仿真步数，`--linear` 和 `--angular` 分别设置线速度与角速度指令。运行结束后输出位姿、速度及左右电机温度。
