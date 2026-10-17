# 第二天：训练、控制与场景评价 / Day 2: Training, Control, and Evaluation

## 运行 / Run

```bash
python src/air/mujoco_uav_rooftop_pv_coverage_survey/main.py --mode dataset --output survey_data
python src/air/mujoco_uav_rooftop_pv_coverage_survey/main.py --mode evaluate --output survey_data
python src/air/mujoco_uav_rooftop_pv_coverage_survey/main.py --mode train-segmentation --output survey_data
python src/air/mujoco_uav_rooftop_pv_coverage_survey/main.py --mode train-control --output survey_data
python -m pytest src/air/mujoco_uav_rooftop_pv_coverage_survey/tests -q
```

The segmentation model is a compact encoder-decoder. Its objective is binary cross entropy plus soft Dice loss. The controller is a three-feature MLP mapping cross-track error, heading error, and distance to turn to forward speed and yaw rate.

分割模型采用轻量编码器-解码器，损失为二元交叉熵与软 Dice 损失之和；控制模型采用三层特征 MLP，将横向偏差、航向误差和距转弯点距离映射为前进速度与偏航角速度。

`evaluate` 会读取数据集清单，对稀疏、密集和遮挡三种布局分别计算规则基线的 IoU、Dice、像素准确率与面积相对误差，并写入 `evaluation_metrics.json`。

The `evaluate` mode reads the manifest and reports IoU, Dice, pixel accuracy, and relative area error for sparse, dense, and occluded layouts in `evaluation_metrics.json`.

For a roof projection, coverage is computed as `A_pv / A_roof × 100%`, where `A_pv` is the projected pixel area of visible photovoltaic panels and `A_roof` is the valid roof mask area. Scenario evaluation keeps sparse, dense, and occluded layouts as separate sequences.

屋顶投影覆盖率按 `A_pv / A_roof × 100%` 计算，其中 `A_pv` 为可见光伏板投影面积，`A_roof` 为有效屋顶区域面积；稀疏、密集、遮挡布局按完整序列独立评价。

## ROS 2 接口 / ROS 2 Interface

Live mode is started with `ros2 launch` after the package is installed. The node publishes zero-safe `geometry_msgs/Twist` commands on the configured command topic; image, pose, route, and command topic names are parameters in `config/controller.yaml`. Offline replay uses `ros2_uav_controller.py --replay replay.jsonl` and does not require ROS 2.

实时模式在安装功能包后使用 `ros2 launch` 启动，节点向配置的控制话题发布安全初始的 `geometry_msgs/Twist` 指令；图像、位姿、航线和控制话题写入 `config/controller.yaml`。离线回放使用 `ros2_uav_controller.py --replay replay.jsonl`，不依赖 ROS 2。
