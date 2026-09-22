# 交付说明 · 对照老师要求的差距与运行指引

> 本文档说明本 `carla_assignment`（CARLA 0.9.16 无人车方案）对照老师四项任务与设计要求的
> 覆盖情况、神经网络化改造、以及在 **Windows 原生** / Ubuntu 虚拟机 / ROS 上的运行方式。

---

## 1. 任务覆盖对照

| 老师任务 | 本包模块 | 状态 | 说明 |
|---|---|---|---|
| (1) 物理仿真 + 键盘运动控制 | `01_control/` | ✅ | W/S/A/D 键控特斯拉，pygame 前视画面 |
| (2) 传感器感知 + 给定轨迹控制 | `02_perception/` | ✅（NN 化） | RGB/深度/雷达感知 + 纯跟踪轨迹控制，感知/控制均为神经网络 |
| (3) 建图 + 导航 + SLAM 同步 | `03_navigation/` | ✅（NN 化） | LiDAR 占用栅格建图 + 神经网络导航避障 |
| (4) 端到端（图像 → 控制） | `04_end_to_end/` | ✅ | CNN 相机图 → steer |

## 2. 神经网络覆盖面（老师硬性要求）

老师要求 *"选择的感知、规划、控制、端到端算法需要为神经网络"*。为满足该条，本包新增一个
**统一的最小神经网络库 `nn_models.py`**（纯 numpy 实现，可离线训练/推理，无需 GPU）：

- **感知 NN**（`02_perception`）：感知器 `MLPClassifier` 由图像/雷达特征 → 障碍类别 + 横向偏移。
- **控制 NN**（`02_perception`）：控制器 `MLPPolicy`（策略网络）由 (航向差, 距离) → steer。
- **规划 NN**（`03_navigation`）：规划器 `MLPPolicy` 由 (目标方位, 前方障碍分布) → 前进/转向指令。
- **端到端 CNN**（`04_end_to_end`）：已有的 `tf.keras` CNN，图像 → steer。

每个 NN 都提供：
```bash
# 离线训练（不连 CARLA，可本机验证）
python 02_perception/main.py --mode train --out nn_model.json
python 03_navigation/main.py --mode train --out nn_plan.json

# 在线推理（连 CARLA 自动驾驶）
python 02_perception/main.py --mode run --model nn_model.json --waypoints "..."
python 03_navigation/main.py --mode run --model nn_plan.json --goal "20,8"
```

`nn_models.py` 完全离线可用（只依赖 numpy），可用 `python -c "import nn_models"` 验证。

## 3. 运行方式（Windows 原生 / Ubuntu / ROS）

入口约定：每个模块可 `main.*` 直接运行。

- **Windows 原生**
  ```bat
  cd carla_assignment
  main.bat control            :: 作业一 键控
  main.bat perception train   :: 作业二 训练感知/控制 NN
  main.bat navigation run --goal 20,8
  main.bat end_to_end --mode test
  ```
- **Ubuntu (Noetic/Humble)**
  ```bash
  cd carla_assignment
  bash main.sh control
  # 或纯 python
  python3 main.py --task control
  ```
- **ROS2 Humble / ROS1 Noetic**：`launch/*.launch.py` / `launch/*.launch`。
  见 `main.sh ros2` 与各模块文档的 launch 一节。

## 4. 你需要在本机/目标机完成的（本环境无 CARLA 无法代跑）

本仓库开发机只有 Python+numpy，**没有 CARLA 服务端、TensorFlow、pygame、OpenCV**，因此：
- 端到端 CNN 的 `collect/train/test`、四个在线推理、pygame 窗口、**GIF 录屏**需在
  **装有 CARLA 0.9.16 的机器**（Windows 或 Ubuntu 虚拟机）上执行并截图/录制。
- 你已提供 `ros.zip` 里的 ROS 虚拟机（Noetic+Humble），可直接在上面测 launch 与 ROS 封装。

运行顺序建议：
1. 装 `requirements.txt`（`scripts/setup_env.sh`，Windows 用 `pip install -r requirements.txt`）。
2. 启动 `CarlaUE4.sh -carla-rpc-port=2000`。
3. 离线训练三个 NN（本机即可）→ 把 `nn_model.json` 交到 CARLA 机。
4. CARLA 机上在线跑 4 个模块，逐项录 GIF（<10MB）放入 `docs/assets/`。

## 5. 动图与文档

- 文档 `docs/*.md` 中真实动图待录屏后放入：`docs/assets/{01_control,02_perception,03_navigation,04_end_to_end}.gif`。
  仓库已随附占位图 `placeholder_*.png`（可用脚本 `scripts/gen_placeholder_assets.py` 重新生成），保证 mkdocs 可正常构建。
- 用 **ScreenToGif** 录制（动图应 <10MB，尽量 <20MB）。
- 文档为 mkdocs 规范，可用 `mkdocs serve --config-file docs/mkdocs.yml` 本地预览，
  推到 GitHub 后由 workflow 部署到 GitHub Pages；首页 `docs/index.md` 含跳转到各示例的链接。
