# hand_eye_calibration

基于 **MuJoCo** 的机械臂手眼标定（eye-in-hand）仿真模块：Panda 机械臂带动腕部相机，
在 16 组不同位姿下观测固定标定板上的 5 个彩色标记点，求解“相机—末端”手眼变换，
并给出标定板重投影/位姿误差。

## 组成

| 文件 | 说明 |
|---|---|
| `main.py` | 仿真入口：IK、位姿采样、相机观测、手眼标定、精度评估、录制 |
| `models/franka_emika_panda/scene_hand_eye.xml` | 手眼标定场景：机械臂 + 标定板 + 标记点 + 双相机 |
| `models/franka_emika_panda/panda_pick.xml` | Panda 模型（腕部相机、TCP；来自 MuJoCo Menagerie） |
| `scripts/setup_model.py` | 兼容入口（4 行转发） |
| `hand_eye_calibration/setup_model.py` | 实际下载 Menagerie Panda 网格资源（模型 XML 随仓库提供） |
| `scripts/make_gif.py` | 帧序列 → GIF / MP4 |
| `scripts/plot_result.py` | 每个位姿的标定误差柱状图 |
| `scripts/record_demo.sh` | 一键录制 + 截图 + GIF + 误差曲线 |
| `hand_eye_calibration_setup` | 安装后的网格资源下载入口（console script） |
| `requirements.txt` | MuJoCo / OpenCV / Pillow / Matplotlib |

## 环境

- Ubuntu 20.04（VMware，4 vCPU / 4 GB RAM，无 GPU 也可）
- Python 3.8、MuJoCo 2.3.7（CPU）、OpenCV 4.8（headless）、Pillow、Matplotlib
- 渲染：MuJoCo GLFW + Mesa 软件渲染

## 安装与运行

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python3 scripts/setup_model.py      # 首次运行：下载 Panda 网格资源（colcon 安装时打包）
# 可选：生成 MP4 需要系统 ffmpeg
sudo apt-get install -y ffmpeg

# ROS 2 工作区（需先完成上面的网格下载）
colcon build --symlink-install --packages-select hand_eye_calibration
source install/setup.bash
ros2 launch hand_eye_calibration main.launch.py

# 仅仿真与标定（打印精度）
python3 main.py

# 录制：主视图 + 腕部相机画中画 + 标记检测 + HUD
bash scripts/record_demo.sh /tmp/he_frames
# 产物：results/01_overview_and_detection.png、02_camera_markers.png、
#       03_calibration_result.png、calibration_error.png、
#       demo_hand_eye_calibration.gif（MP4 需系统 ffmpeg）、calibration_result.json
```

> 渲染需要图形会话：`DISPLAY=:0`、`XAUTHORITY`（按当前 UID 自动查找）、`MUJOCO_GL=glfw`。

## 流程

1. **采样**：16 个标定板观测位姿（x/y 偏移、z 高度 0.32/0.38 m，yaw ±20°、roll/pitch ±6~8°）；
2. **观测**：腕部相机图像 + 标定板 5 个标记点；仿真使用相机投影对应点，并在
   `T_cam_board` 平移上叠加 0.4 mm、旋转向量上叠加 0.08° 噪声（渲染图像用于展示视觉反馈）；
3. **标定**：OpenCV `calibrateHandEye`（TSAI/PARK/HORAUD/ANDREFF/DANIILIDIS），按平移+旋转组合残差选优；仅对排名第一的解做一次 `AX = XB` 非线性精化；
4. **评估**：用标定结果把标定板反算到机械臂基座系，统计每个位姿的平移/旋转误差与
   重投影误差。

## 实测结果（VMware，无 GPU，16 组位姿）

| 指标 | 数值 |
|---|---|
| 标定板平移误差（均值） | **0.77 mm**（最大 1.08 mm） |
| 标定板旋转误差（均值） | **0.143°**（最大 0.284°） |
| 重投影误差（均值） | 0.48 px（最大 0.78 px） |
| 手眼平移误差 | **0.19 mm / 0.19 mm**（mean/max，真值 (0.09, 0.09, 0.02) m） |
| 手眼旋转误差 | **0.094° / 0.094°**（mean/max） |
| 采样位姿 | 16 个（全部 IK 收敛并完成观测） |
| 录制 | 286 帧 / 20 fps / 640×480 + 腕部相机画中画 |
| GIF / MP4 | 3.7 MB / 0.56 MB（GIF 10 fps 抽帧；MP4 20 fps，需 ffmpeg） |

## 已知限制

- 机械臂/相机为 MuJoCo 简化仿真；观测点由投影 + 噪声生成，不等同于真实图像特征提取；
- 标定板为 5 个彩色点，未使用真实 ArUco/棋盘格与镜头畸变模型；
- 未提供 ROS 话题/服务/action；ROS 2 封装仅用于 `colcon build` / `ros2 launch` 启动 MuJoCo 演示进程；
- 网格资源需先运行 `setup_model.py`，否则安装后的 share 目录只有 XML；
- GIF 为 10 fps 抽帧，MP4 为 20 fps（需系统 ffmpeg）。
- 结果依赖仿真观测噪声与位姿覆盖，真实系统需重新标定。

