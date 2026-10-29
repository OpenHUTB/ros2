# carla_lane_keeping

基于前视相机的车道线检测与车道保持（CARLA + ROS Noetic）。用针孔相机模型把
前视图像映射到鸟瞰视角，分离左右车道线并拟合多项式，输出横向偏差、航向偏差与
车道曲率；控制器据此给出转向指令，可选闭环。

算法只用 numpy 实现（FFT 卷积、连通域标记、Sobel 梯度都是手写的），不依赖 OpenCV
与深度学习框架，因此在没有 GPU、没有 cv_bridge 的环境里也能跑，每一级都可单测。

## 目录

| 文件 | 职责 |
|---|---|
| `main.py` | 包根入口，转调 `scripts/main.py` |
| `scripts/main.py` | 统一入口：解析模式、加载参数、拉起节点 |
| `scripts/lane_detection.py` | 检测算法核心（相机模型、逆透视、拟合、曲率、置信度） |
| `scripts/lane_detector.py` | 感知节点：订阅前视图像，发布偏差/曲率/掩码 |
| `scripts/lane_keeping_controller.py` | 控制节点：PID + 航向项，输出转向 |
| `config/lane_params.yaml` | 全部可调参数 |
| `launch/main.launch` | 一键启动（检测 / 控制 / 闭环） |
| `tools/run_on_carla.py` | 不依赖 ROS 的 CARLA 验证脚本（单帧评估 / 闭环实跑，带地图真值） |
| `tools/plot_trace.py` | 把闭环 CSV 画成收敛曲线（需 Pillow） |
| `tests/` | 单元测试与离线演示 |

## 运行环境

| 项 | 版本 |
|---|---|
| 操作系统 | Ubuntu 20.04（虚拟机） |
| ROS | Noetic（ROS1） |
| Python | 3.8.10 |
| 依赖 | `numpy`；`rospy` / `std_msgs` / `sensor_msgs` / `geometry_msgs` / `nav_msgs` |
| 可选 | `cv_bridge`（缺失时节点用 numpy 直接解析 `sensor_msgs/Image`）、`carla_msgs`（缺失时控制节点退回发布 `Twist`） |
| 仿真器 | CARLA 0.9.13 + `carla_ros_bridge`（发布 `/carla/ego_vehicle/rgb_front/image`） |

## 运行步骤

```bash
# 0. 放进工作空间并编译
cd ~/catkin_ws
cp -r <本模块> src/
catkin_make
source devel/setup.bash

# 1. 看入口参数
rosrun carla_lane_keeping main.py --help
# 等价于（在包目录下）
cd src/ground/carla_lane_keeping && python3 main.py --help

# 2. 只跑检测，在终端看横向偏差
rosrun carla_lane_keeping main.py --mode detect

# 3. 检测 + 车道保持闭环
rosrun carla_lane_keeping main.py --mode both

# 4. launch 一键启动（推荐）
roslaunch carla_lane_keeping main.launch mode:=both
```

前置条件：CARLA 已启动，且 `carla_ros_bridge` 正在发布前视相机图像。本模块只负责
感知与控制，不负责生成车辆。

## 话题

订阅：

- `/carla/ego_vehicle/rgb_front/image`（`sensor_msgs/Image`，可用 `--image-topic` 改）

发布：

| 话题 | 类型 | 含义 |
|---|---|---|
| `lane/lateral_offset` | `Float32` | 车辆中心相对车道中心的横向偏差（m），右正 |
| `lane/heading_error` | `Float32` | 航向偏差（rad），车头偏右为正 |
| `lane/curvature` | `Float32` | 车道曲率半径（m），直道发 0 |
| `lane/confidence` | `Float32` | 检测置信度 0~1 |
| `lane/departure` | `Bool` | 是否压线 |
| `lane/lane_width` | `Float32` | 本帧实测车道宽（m），标定自检用 |
| `lane/mask` | `Image` | 鸟瞰二值掩码（调试用） |

**检测失败时不发布 `lateral_offset`**，避免下游把「没检到」当成「车在正中」。

## 参数

见 `config/lane_params.yaml`，主要项：

| 参数 | 默认 | 说明 |
|---|---|---|
| `use_calibration` | true | 是否用相机模型算米/像素；关掉退回按图像比例圈梯形 |
| `camera_height_m` / `camera_pitch_deg` / `camera_hfov_deg` | 1.70 / 8.0 / 90.0 | 相机安装参数，标定模式下决定全部物理量 |
| `d_near_m` / `d_far_m` | 3.5 / 15.0 | 鸟瞰图覆盖的前向距离区间 |
| `lane_width_m` | 3.5 | 车道宽先验，用于左右边界配对 |
| `white_min` | 160 | 白漆三通道最小值阈值 |
| `sobel_min` / `sobel_max` | 100 / 900 | 梯度筛选区间，`auto_percentile` 开启时自适应 |
| `max_component_fill_ratio` | 0.45 | 连通域填充率上限，用于剔除块状干扰 |
| `kp` / `ki` / `kd` | 0.28 / 0.02 / 0.06 | 转向 PID |
| `k_psi` | 0.9 | 航向误差增益 |
| `filter_alpha` | 0.35 | 偏差一阶低通系数，压检测抖动 |
| `target_speed` / `speed_gain` | 6.0 / 0.35 | 目标车速（m/s）与速度闭环增益 |

相机换了位置不必改 yaml，launch 里可直接覆盖：
`roslaunch carla_lane_keeping main.launch camera_height:=2.10 camera_pitch:=12.0`

## 测试

```bash
cd src/ground/carla_lane_keeping      # 包根目录，不是 scripts/
python3 -m unittest discover -s tests -v
```

110 个用例，不需要 ROS 与 CARLA（`tests/test_ros_nodes.py` 内置了 rospy 与消息类型的桩）。
已在 Python 3.8.10 与 3.12.8 上分别跑通。

离线演示：

```bash
python3 tests/demo_lane_detection.py
```

三个合成场景（居中 / 右偏 / 加噪）的中心误差分别为 −0.5 / +0.7 / −0.5 px。

## 已知限制

- 标定模式的精度取决于 `camera_height_m` 与 `camera_pitch_deg` 填得准不准；
  可用 `lane/lane_width` 自检（应接近真实车道宽 3.5 m），偏大就说明高度填小了。
- 只测过城市直道与缓弯，未覆盖大曲率弯、坡道与车身侧倾。
- 强逆光、车道线磨损、施工改道时置信度会下降；置信度低于阈值时按「未检测到」处理。
- 只处理结构化道路的实线/虚线，不识别路沿与锥桶。
