# MuJoCo 无人机视觉着陆与栅格导航

本模块使用 MuJoCo 构建四旋翼与移动平台场景，从俯视 RGB 图像提取平台和无人机中心位置，以轻量多层感知机（MLP）输出平面速度与下降速度。

## 环境与运行

需要 Python 3.10 或更新版本。安装 `requirements.txt` 后，在模块目录中训练、运行闭环仿真、键盘遥控或二维建图导航：

```bash
python -m pip install -r requirements.txt
python main.py train --samples 5000
python main.py demo --steps 180 --output output
python main.py teleop --output output
python main.py navigate --output output
python -m pytest -q tests
```

`teleop` 每次在提示符输入一个命令并按回车：`W/A/S/D` 控制水平移动，`R/F` 控制升降，空行悬停，`Q` 退出。位置会在每次输入后更新。`navigate` 使用虚拟 2D 激光扫描更新占据栅格，并在每一步后重新规划到目标点。

运行文件写入 `output/`。着陆演示输出实际渲染的 `landing_scene.png`、`landing_run.gif`、轨迹和指标 JSON；键盘演示输出 `teleop_scene.png` 与指令轨迹；导航输出 `navigation_map.png`、`navigation_run.gif` 和覆盖率指标。模型保存为 `output/landing_controller.joblib`。

## 场景与方法

着陆场景由 MJCF 文件 `assets/moving_pad.xml` 定义。着陆平台沿 x/y 正弦轨迹移动，无人机由 MuJoCo 位置伺服关节驱动。OpenCV HSV 阈值从真实渲染图像中定位绿色平台和红色机体。训练集由固定随机种子采样的视觉观测与专家控制指令组成，并按完整 episode 划分 80% 训练集与 20% 测试集，防止相邻时刻跨集合。`--samples` 精确指定样本数，最少为 200。

着陆流程：渲染 RGB → HSV 区域定位 → 视觉误差与高度/平台速度组合 → MLP 推理 → MuJoCo 控制输出。导航流程：虚拟激光扫描 → 占据栅格更新 → A* 路线规划 → 移动一步 → 更新位姿与地图 → 再次规划。

![MuJoCo 移动平台着陆场景](media/landing_scene.png)

![MuJoCo 移动平台着陆过程](media/landing_run.gif)

## 仿真环境

默认演示在 MuJoCo 仿真环境运行。一次 180 步演示持续 3.6 秒，着陆接近误差为 0.3387 m，最终高度为 0.2348 m，着陆判定成功。一次 1200 样本训练包含 48 个训练 episode 和 12 个测试 episode，测试 RMSE 为 0.036406 m/s。一次二维导航运行到达目标，完成 38 个网格步，地图覆盖率为 99.22%。

English summary: A lightweight MuJoCo UAV project with visual landing, keyboard teleoperation, and 2D lidar mapping with replanned navigation.
