# 多传感器融合跟踪（Radar + Camera + IMU）

地面载具多传感器真融合：对单个移动目标做融合跟踪，覆盖 **外参标定 → 时间同步 → EKF 融合** 完整链路。

同一套融合核心（`sensor_fusion_core.py`）被两个平台复用：

- **CARLA 渲染版**（`carla_sensor_fusion.py`）：在 ego 车挂载 相机(camera)、激光雷达(lidar)、IMU 三个传感器，采集数据后做标定、时间同步与 EKF 融合，需本机装 CARLA 运行。
- **2D 轻量版**（`sim_2d_fusion.py`）：合成传感器仿真，可直接运行并生成效果动图 `fusion_demo.gif`——即本融合算法的真实运行效果演示。

## 融合链路

1. **外参标定**：最小二乘估计相机↔雷达相对旋转/平移，把各传感器观测统一到同一坐标系。
2. **时间同步**：雷达 ~10Hz、相机 ~30Hz、IMU ~100Hz，按「最近时间戳」对齐到 20Hz 融合网格，超过容差视为失同步。
3. **载体运动补偿**：IMU 估计载体姿态角，把各传感器体坐标观测转到世界系。
4. **EKF 融合**：世界系 4 态 `[px,py,vx,vy]`，匀速预测，分别用雷达（距离+方位）与相机（方位）观测更新。

## 文件结构

```
carla_sensor_fusion/
├── sensor_fusion_core.py  # 共享核心（外参标定 / 时间同步 / EKF 融合）
├── carla_sensor_fusion.py # CARLA 渲染版主脚本
├── sim_2d_fusion.py       # 2D 轻量仿真版主脚本
├── fusion_demo.gif        # 2D 版真实运行效果动图
├── requirements.txt
└── README.md
```

## 运行

### 2D 轻量版（本工程直接可跑，生成动图）

```bash
pip install numpy matplotlib Pillow
python3 sim_2d_fusion.py                 # 生成 fusion_demo.gif
python3 sim_2d_fusion.py --show          # 弹出窗口实时查看
```

### CARLA 渲染版（需本机装 CARLA）

```bash
export PYTHONPATH=/opt/carla/PythonAPI/carla/dist/carla-<ver>-py3.7-linux-x86_64.egg:$PYTHONPATH
pip install numpy
python3 carla_sensor_fusion.py --map Town03 --frames 600
```

## 参数说明

| 参数 | 含义 | 默认 |
|------|------|------|
| `sig_r` | 雷达测距噪声 (m) | 0.5 |
| `sig_th_r` | 雷达方位噪声 (rad) | 0.10 |
| `sig_th_c` | 相机方位噪声 (rad) | 0.04 |
| `sig_imu` | IMU 姿态估计噪声 (rad) | 0.012 |
| 融合频率 | EKF 更新周期 | 20 Hz |

## 运行效果

`fusion_demo.gif` 为 2D 轻量版真实运行效果：载体(ego)在原点缓慢横摆，跟踪沿 8 字轨迹运动的目标。
图中同时显示 **EKF 融合（雷达+相机）** 与 **仅雷达跟踪** 两条估计线，以及雷达原始观测（红 ×）、相机方位射线（绿）、估计协方差椭圆。
实测融合 RMSE ≈ 0.60 m，仅雷达 ≈ 0.97 m，**融合比单传感器提升约 38%**（相机 30Hz 高频方位补偿了雷达方位分辨率与采样率的不足）。

## 合规说明

本项目为课程/学习用途的仿真代码，不含真实道路行驶与真车控制，遵循开源许可。
