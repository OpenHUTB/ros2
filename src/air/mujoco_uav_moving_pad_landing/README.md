# MuJoCo 无人机移动平台着陆

本模块使用 MuJoCo 仿真无人机对移动平台着陆。程序从渲染的 RGB 图像中识别平台和机体，结合仿真高度与平台速度生成控制指令，并输出可复现的轨迹和首次接触指标。

## 环境与运行

支持 Python 3.10–3.12。安装依赖后，可训练 MLP 策略、运行闭环仿真或使用键盘遥控：

```bash
python -m pip install -r requirements.txt
python main.py train --samples 5000 --steps-per-episode 20
python main.py demo --steps 180 --output output
python main.py teleop --output output
python -m pytest -q tests
```

训练特征和标签来自 MuJoCo 渲染闭环采样，训练集与测试集按完整 episode 划分。训练结果同时给出 episode 分组 RMSE 和逐行随机划分 RMSE；后者用于对比相邻样本混入训练集、测试集时的误差变化。使用 1,200 条样本、每个 episode 20 步时，分组 RMSE 为 0.046130 m/s，逐行随机划分 RMSE 为 0.019977 m/s；使用 5,000 条样本时分别为 0.023880 m/s 和 0.013302 m/s。

`teleop` 每行接收一个命令：`W/A/S/D` 控制水平移动，`R/F` 调整高度，空行保持位置，`Q` 退出。垂直位置伺服带有重力前馈，零速度指令可保持高度。

默认训练命令采集 5,000 条渲染样本，每个 episode 包含 20 步。`python -m pytest -q tests` 运行 10 项自动化测试。

复现训练指标：

```bash
python main.py train --samples 1200 --steps-per-episode 20
python main.py train --samples 5000 --steps-per-episode 20
```

English: Python 3.10–3.12 is supported. Features and labels are collected from rendered MuJoCo rollouts, and complete episodes stay together during the train/test split. With 1,200 samples and 20 steps per episode, grouped RMSE is 0.046130 m/s and row-wise RMSE is 0.019977 m/s; with 5,000 samples the values are 0.023880 m/s and 0.013302 m/s. Teleoperation uses `W/A/S/D` for horizontal movement, `R/F` for altitude, an empty line to hold position, and `Q` to exit.

## 平台轨迹参数

平台 x/y 正弦轨迹可分别配置幅度（米）、角频率（弧度/秒）和相位（弧度）。以下为场景默认值：

```bash
python main.py demo --steps 180 --output output \
  --pad-amplitude-x 0.34 --pad-amplitude-y 0.22 \
  --pad-frequency-x 0.42 --pad-frequency-y 0.31 \
  --pad-phase-x 0.0 --pad-phase-y 0.0
```

English: Each horizontal platform axis has configurable amplitude in meters, angular frequency in radians per second, and phase in radians.

## 无头 Docker 渲染

本目录中的 Docker 镜像使用 OSMesa 在无显示环境中渲染 MuJoCo。进入本目录后构建并运行：

```powershell
docker build -t uav-moving-pad-landing .
New-Item -ItemType Directory -Force output | Out-Null
docker run --rm -v "${PWD}/output:/app/output" uav-moving-pad-landing python main.py demo --steps 180 --output /app/output --pad-amplitude-x 0.34 --pad-amplitude-y 0.22 --pad-frequency-x 0.42 --pad-frequency-y 0.31 --pad-phase-x 0 --pad-phase-y 0
```

运行后会在 `output/` 中生成 PNG 渲染帧、GIF 动图、逐步轨迹和指标 JSON 文件。

English: The included Docker image uses OSMesa for display-free rendering. The command writes a PNG frame, animated GIF, per-step trajectory, and metrics JSON file to the mounted `output/` directory.

## 着陆指标与输出

`landing_approach_error_m` 与 `landing_success` 在首次接触平台表面时评估。成功条件包括实际接触、机体投影位于平台范围内、触地高度与模型计算的接触高度相差不超过 0.03 m，以及垂直速度不超过 0.6 m/s。指标文件会记录首次接触状态和平台轨迹配置，不使用末帧或后续平台相位推断着陆结果。

演示文件写入 `output/`：`landing_scene.png`、`landing_run.gif`、`trajectory.json` 和 `metrics.json`。训练模型保存在 `output/landing_controller.joblib`；键盘遥控生成 `teleop_scene.png` 与 `teleop_trace.json`。

下图和动图由上述 MuJoCo 演示实际渲染生成：

![移动平台着陆场景](../../../docs/img/air/mujoco_uav_moving_pad_landing.png)

![无人机移动平台着陆过程](../../../docs/img/air/mujoco_uav_moving_pad_landing.gif)

English: Landing error and success are evaluated at first contact. Success requires contact, the UAV footprint to fit on the pad, altitude within 0.03 m of the model-derived contact height, and vertical speed at or below 0.6 m/s. Demo outputs include a rendered PNG, animated GIF, trajectory, and metrics JSON; teleoperation writes a scene image and command trace.
