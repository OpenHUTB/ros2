# MuJoCo 差速小车电机温升与降额控制

本模块使用 MuJoCo 仿真差速小车左右轮驱动，以集总热模型估计左右电机的绕组温度，记录车体运动与电机温升。

## 运行环境

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- MuJoCo 3.2 或更高版本、NumPy、pytest

## Docker 构建与测试

从仓库根目录执行：

```bash
docker build -t mujoco-motor-thermal:humble -f src/ground/mujoco_motor_thermal_control/Dockerfile .
docker run --rm mujoco-motor-thermal:humble bash -lc "source /opt/ros/humble/setup.bash && source install/setup.bash && colcon test --merge-install --return-code-on-test-failure --packages-select mujoco_motor_thermal_control && colcon test-result --verbose && python3 src/mujoco_motor_thermal_control/main.py --steps 600"
```

## 本地运行

```bash
python3 -m pip install "numpy<2" "mujoco>=3.2,<4" pytest
python3 src/ground/mujoco_motor_thermal_control/main.py --steps 600 --linear 0.55
```

`--steps` 控制仿真步数，`--linear` 和 `--angular` 分别设置线速度与角速度指令。运行结束后输出位姿、速度及左右电机温度。

## ROS 2 启动

从仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --merge-install --packages-select mujoco_motor_thermal_control
source install/setup.bash
ros2 launch mujoco_motor_thermal_control main.launch.py steps:=600 linear:=0.55
```

launch 从安装目录启动 `main.py` 对应入口，仿真结束时输出小车运动和左右电机温度。

## 仿真画面

![MuJoCo 差速小车电机热模型运行画面](../../../docs/img/ground/mujoco_motor_thermal_model/mujoco_demo.png)
