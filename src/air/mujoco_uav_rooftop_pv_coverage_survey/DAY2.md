# 第二天：训练、控制与场景评价 / Day 2: Training, Control, and Evaluation

## 运行 / Run

```bash
python src/air/mujoco_uav_rooftop_pv_coverage_survey/main.py --mode dataset --output survey_data
python src/air/mujoco_uav_rooftop_pv_coverage_survey/main.py --mode evaluate --output survey_data
python src/air/mujoco_uav_rooftop_pv_coverage_survey/main.py --mode map-nav --output survey_data
python src/air/mujoco_uav_rooftop_pv_coverage_survey/main.py --mode train-segmentation --output survey_data
python src/air/mujoco_uav_rooftop_pv_coverage_survey/main.py --mode train-control --output survey_data
python src/air/mujoco_uav_rooftop_pv_coverage_survey/main.py --mode train-end-to-end --output survey_data
python src/air/mujoco_uav_rooftop_pv_coverage_survey/main.py --mode keyboard-demo --output survey_data
python src/air/mujoco_uav_rooftop_pv_coverage_survey/main.py --mode all --output survey_data
python -m pytest src/air/mujoco_uav_rooftop_pv_coverage_survey/tests -q
```

中文：`all` 入口按“数据采集 → 分割训练 → 端到端图像控制 → 传感器建图与航线导航 → 场景评价 → 键盘回放”的顺序执行，并写入 `day2_summary.json`。键盘控制使用 `W/S` 前后、`A/D` 左右偏航、`Q/E` 小幅偏航、空格停车；回放生成 `keyboard_demo.gif`。

English: The `all` entry runs dataset collection, segmentation training, end-to-end image control, sensor mapping and route navigation, scenario evaluation, and keyboard replay in order, writing `day2_summary.json`. Keyboard control maps `W/S` to forward/backward, `A/D` to yaw, `Q/E` to fine yaw, and Space to stop; the replay produces `keyboard_demo.gif`.

The segmentation model is a compact encoder-decoder. Its objective is binary cross entropy plus soft Dice loss. The controller is a three-feature MLP mapping cross-track error, heading error, and distance to turn to forward speed and yaw rate.

分割模型采用轻量编码器-解码器，损失为二元交叉熵与软 Dice 损失之和；控制模型采用三层特征 MLP，将横向偏差、航向误差和距转弯点距离映射为前进速度与偏航角速度。

`evaluate` 会读取数据集清单，对稀疏、密集和遮挡三种布局分别计算规则基线的 IoU、Dice、像素准确率与面积相对误差，并写入 `evaluation_metrics.json`。

The `evaluate` mode reads the manifest and reports IoU, Dice, pixel accuracy, and relative area error for sparse, dense, and occluded layouts in `evaluation_metrics.json`.

传感器建图将下视 RGB 光伏掩码按相机位姿投影到屋顶局部 ENU 占据栅格，导航进度按完整序列回放帧数计算。端到端网络直接学习 `RGB → (forward_speed, yaw_rate)`，损失为均方误差。

Sensor mapping projects downward RGB PV masks into a rooftop-local ENU occupancy grid using synchronized poses. Navigation progress is measured from complete sequence replay frames. The end-to-end network learns `RGB → (forward_speed, yaw_rate)` with mean squared error.

## 物理仿真与步骤 / Simulation Procedure

1. 中文：运行 `main.py --mode keyboard-demo` 载入 MuJoCo 屋顶场景、设置四旋翼初始位姿，并按键盘序列更新位置和航向；英文：load the MuJoCo rooftop scene, set the quadrotor pose, and replay the key sequence.
2. 中文：运行 `--mode dataset` 由下视 RGB 传感器同步生成光伏掩码与专家航迹标签；英文：generate synchronized RGB masks and expert trajectory labels from the nadir camera.
3. 中文：运行 `--mode map-nav` 将传感器观测融合进占据栅格并计算导航进度；英文：fuse sensor observations into the occupancy grid and compute route progress.
4. 中文：运行 `--mode train-end-to-end` 训练图像到控制神经网络，并在测试序列上报告均方误差；英文：train and test the image-to-control neural network on held-out sequences.

公式 / Formulae:

`L_seg = BCEWithLogits + (1 - Dice)`, `L_ctrl = (1/N) Σ ||u_pred - u_expert||²`, `coverage = A_pv / A_roof × 100%`。

For a roof projection, coverage is computed as `A_pv / A_roof × 100%`, where `A_pv` is the projected pixel area of visible photovoltaic panels and `A_roof` is the valid roof mask area. Scenario evaluation keeps sparse, dense, and occluded layouts as separate sequences.

屋顶投影覆盖率按 `A_pv / A_roof × 100%` 计算，其中 `A_pv` 为可见光伏板投影面积，`A_roof` 为有效屋顶区域面积；稀疏、密集、遮挡布局按完整序列独立评价。

## ROS 2 接口 / ROS 2 Interface

Live mode is started with `ros2 launch` after the package is installed. The node publishes zero-safe `geometry_msgs/Twist` commands on the configured command topic; image, pose, route, and command topic names are parameters in `config/controller.yaml`. Offline replay uses `ros2_uav_controller.py --replay replay.jsonl` and does not require ROS 2.

实时模式在安装功能包后使用 `ros2 launch` 启动，节点向配置的控制话题发布安全初始的 `geometry_msgs/Twist` 指令；图像、位姿、航线和控制话题写入 `config/controller.yaml`。离线回放使用 `ros2_uav_controller.py --replay replay.jsonl`，不依赖 ROS 2。

The module is an `ament_python` package with `package.xml`, `setup.py`, and a launch file, so it can be installed with `colcon build` and started with `ros2 launch mujoco_uav_rooftop_pv_coverage_survey rooftop_pv_controller.launch.py`.

本模块提供 `ament_python` 包结构，包含 `package.xml`、`setup.py` 和 launch 文件，可使用 `colcon build` 安装，并通过 `ros2 launch mujoco_uav_rooftop_pv_coverage_survey rooftop_pv_controller.launch.py` 启动。
