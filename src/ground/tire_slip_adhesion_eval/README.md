# 轮胎滑移率-附着特性与最佳制动滑移率仿真

纯 Python 数值仿真，不依赖 ROS / Gazebo / CARLA / 任何模拟器。采用 Pacejka 魔术公式的简化形式刻画制动滑移率 `s` 与路面附着系数 `μ` 的关系，对比干沥青、湿沥青、积雪、结冰四种路面的峰值附着系数与最佳滑移率区间，并按峰值附着估算不同初速度下的制动距离趋势。

配套教程文档：`docs/ground/tire_slip_adhesion_eval.md`

## 运行环境

- 操作系统：Windows 10 / 11（Linux 亦可，脚本后台出图，无需图形界面）
- Python：3.10 ~ 3.12
- 依赖：见 `requirements.txt`（`numpy`、`matplotlib`、`Pillow`；`mujoco` 可选，仅用于四路面单车场景静态图）

## 运行步骤

1. 创建并激活虚拟环境，安装依赖：

   ```bash
   python -m venv tire_env
   # Windows: tire_env\Scripts\activate
   # Linux:  source tire_env/bin/activate
   pip install -r requirements.txt
   ```

2. 在本目录下运行：

   ```bash
   python main.py
   ```

   脚本会绘制四种路面的滑移率-附着曲线、峰值附着对比与制动距离对比，并合成滑移率扫描演示动图。

## 输出结果

运行结束后在脚本同级的 `results/` 目录下得到：

| 文件 | 说明 |
| --- | --- |
| `slip_curves.png` | 四种路面滑移率-附着系数曲线，标注峰值点与最佳滑移率 |
| `slip_peak_bar.png` | 峰值附着系数与最佳滑移率对比柱状图 |
| `slip_brake_distance.png` | 各路面在 60 / 100 km/h 初速度下的制动距离对比 |
| `slip_demo.gif` | 滑移率扫描时附着曲线演示动图 |
| `slip_scene.png` | MuJoCo 离屏渲染四路面单车场景静态图（网格地面斜俯视，路板配色与附着曲线一致） |
| `metrics.csv` | 各路面峰值附着、最佳滑移率与制动距离数据表 |

> 输出图片命名与教程图片目录 `docs/img/ground/` 中的文件名一致，提交文档时将 `results/` 下的图片直接复制过去即可，无需改名。

## 注意事项

- 附着系数在最佳滑移率附近达到峰值；滑移率继续增大（车轮过度抱死 / 滑拖）后附着系数反而回落，这也是 ABS 把滑移率控制在最佳区间附近的原因。
- 路面越滑，峰值附着越低，最佳滑移率越大；冰面、雪面制动距离相比干路成倍甚至数倍增长，低速与加大跟车距离是主要应对手段。
- 模型为稳态轮胎特性的简化魔术公式，未考虑轮胎温度、磨损、载荷转移、路面排水与胎面花纹影响；实际制动性能需结合轮胎试验与实车道路试验标定。
