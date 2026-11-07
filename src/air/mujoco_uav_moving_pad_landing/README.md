# 移动平台视觉自主着陆

本模块使用 MuJoCo 构建四旋翼与移动平台场景，从俯视 RGB 图像提取平台和无人机中心位置，以轻量多层感知机（MLP）输出平面速度与下降速度。

## 环境与运行

需要 Python 3.10 或更新版本。安装 `requirements.txt` 后，在模块目录中训练、运行闭环仿真或使用键盘遥控：

```bash
python -m pip install -r requirements.txt
python main.py train --samples 5000
python main.py demo --steps 180 --output output
python main.py teleop --output output
python -m pytest -q tests
```

`teleop` 每次在提示符输入一个命令并按回车：`W/A/S/D` 控制水平移动，`R/F` 控制升降，空行悬停，`Q` 退出。位置会在每次输入后更新。

运行文件写入 `output/`。着陆演示输出实际渲染的 `landing_scene.png`、`landing_run.gif`、轨迹和指标 JSON；键盘演示输出 `teleop_scene.png` 与指令轨迹。模型保存为 `output/landing_controller.joblib`。

## 场景与方法

着陆场景由 MJCF 文件 `assets/moving_pad.xml` 定义。着陆平台沿 x/y 正弦轨迹移动，无人机由 MuJoCo 位置伺服关节驱动。OpenCV HSV 阈值从真实渲染图像中定位绿色平台和红色机体。训练集由固定随机种子采样的视觉观测与专家控制指令组成，并采用 80%/20% 留出集评估测试均方根误差。

流程：渲染 RGB → HSV 区域定位 → 视觉误差与高度/平台速度组合 → MLP 推理 → MuJoCo 控制输出。

![MuJoCo 移动平台着陆场景](media/landing_scene.png)

![MuJoCo 移动平台着陆过程](media/landing_run.gif)

## 仿真环境

默认演示在 MuJoCo 仿真环境运行。一次 180 步演示持续 3.6 秒，落点接近误差约 0.295 m，最终高度约 0.235 m。这些结果对应本项目仿真场景，不代表真实飞行性能。

English summary: A lightweight MuJoCo simulation with visual landing and line-oriented keyboard teleoperation.
