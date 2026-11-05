# 移动平台视觉自主着陆

本模块使用 MuJoCo 构建四旋翼与移动平台场景，从俯视 RGB 图像提取平台和无人机中心位置，以轻量多层感知机（MLP）输出平面速度与下降速度。

## 环境与运行

需要 Python 3.10 或更新版本。安装 `requirements.txt` 后，在模块目录中训练并运行：

```bash
python -m pip install -r requirements.txt
python main.py train
python main.py demo --steps 180 --output output
```

仿真输出写入 `output/`，包括实际渲染的 `landing_scene.png`、`landing_run.gif`、轨迹和指标 JSON。训练控制器模型保存为 `output/landing_controller.joblib`。

## 场景与方法

场景由 MJCF 文件 `assets/moving_pad.xml` 定义。着陆平台沿 x/y 正弦轨迹移动，无人机由 MuJoCo 位置伺服关节驱动。OpenCV HSV 阈值从真实渲染图像中定位绿色平台和红色机体。训练集由固定随机种子采样的归一化观测与专家控制指令组成，并留出 20% 独立样本评估，报告测试均方根误差。

流程：渲染 RGB → HSV 区域定位 → 视觉误差与高度/平台速度组合 → MLP 推理 → 速度限幅 → MuJoCo 或 ROS 2 控制输出。

![MuJoCo 移动平台着陆场景](media/landing_scene.png)

![MuJoCo 移动平台着陆过程](media/landing_run.gif)

## 仿真环境

默认演示在 MuJoCo 仿真环境运行。一次 180 步演示持续 3.6 秒，落点接近误差约 0.295 m，最终高度约 0.235 m。仿真画面和评估文件由运行命令生成。

English summary: A lightweight MuJoCo, OpenCV, and MLP example for visual landing on a moving platform.
