# MuJoCo 无人机移动平台着陆

该示例使用 MuJoCo 场景观察移动平台，并以图像特征、飞行高度和平台速度生成无人机平移指令。训练数据来自仿真 rollout，模型按完整 episode 分组训练和评估。

## 运行

```bash
cd src/air/mujoco_uav_moving_pad_landing
python -m pip install -r requirements.txt
python main.py train --samples 5000 --steps-per-episode 20
python main.py demo --steps 180 --output output
python -m pytest -q tests
```

无显示环境可使用本目录的 OSMesa Docker 镜像。演示会在 `output/` 生成渲染帧、着陆动图、逐步轨迹和指标 JSON。

## 仿真输出

下图和动图来自该 MuJoCo 场景的实际渲染结果。

![移动平台着陆场景](../img/air/mujoco_uav_moving_pad_landing.png)

![移动平台着陆过程](../img/air/mujoco_uav_moving_pad_landing.gif)

着陆成功状态依据首次接触帧判断，记录平台接触、水平偏差、接触高度、垂直速度和接触点数。

English: The example uses rendered MuJoCo rollouts to train and evaluate a moving-platform landing controller. Success metrics are measured at first contact.
