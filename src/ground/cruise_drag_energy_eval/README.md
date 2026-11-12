# 巡航风阻与能耗敏感性分析

纯 Python 数值仿真为核心，不依赖 ROS / Gazebo / CARLA；MuJoCo 仅作为可选的离屏渲染后端，用于渲染车型场景静态图，未安装时自动跳过。模块把车辆行驶阻力分解为滚动阻力、坡度阻力与空气阻力，在不同车速、风阻系数（车型）与道路坡度下计算总阻力、驱动功率与百公里能耗，量化车速、空气动力学外形和坡度对巡航能耗的影响。

配套教程文档：`docs/ground/cruise_drag_energy_eval.md`

## 运行环境

- 操作系统：Windows 10 / 11（Linux 亦可，脚本后台出图，无需图形界面）
- Python：3.10 ~ 3.12
- 依赖：见 `requirements.txt`（`numpy`、`matplotlib`、`Pillow`；`mujoco` 为可选，仅用于车型场景静态图离屏渲染）

## 运行步骤

1. 创建并激活虚拟环境，安装依赖：

   ```bash
   python -m venv cruise_env
   # Windows: cruise_env\Scripts\activate
   # Linux:  source cruise_env/bin/activate
   pip install -r requirements.txt
   ```

2. 在本目录下运行：

   ```bash
   python main.py
   ```

   脚本会依次计算各车速、各坡度与三种风阻车型下的阻力分量、驱动功率与百公里能耗，输出阻力分量图、车型功率对比图、坡度能耗对比图，并合成车速扫描演示动图；若已安装 MuJoCo，再离屏渲染车型场景静态图。

## 输出结果

运行结束后在脚本同级的 `results/` 目录下得到：

| 文件 | 说明 |
| --- | --- |
| `cruise_drag_components.png` | 平路各阻力分量（滚动 / 空气 / 总阻力）随车速变化 |
| `cruise_power_model.png` | 三种风阻车型驱动功率随车速对比（平路） |
| `cruise_energy_grade.png` | 不同坡度下百公里能耗随车速对比 |
| `cruise_energy_demo.gif` | 车速扫描时各车型功率曲线演示动图 |
| `cruise_scene.png` | MuJoCo 离屏渲染的三车型场景静态图（可选依赖，未装 MuJoCo 时不生成） |
| `metrics.csv` | 各车速、各坡度下阻力、功率与能耗数据表 |

> 输出图片命名与教程图片目录 `docs/img/ground/` 中的文件名一致，提交文档时将 `results/` 下的图片直接复制过去即可，无需改名。

## 注意事项

- 低速时滚动阻力占主导，车速升高后空气阻力按车速平方增长并反超；这也是高速巡航能耗与风阻系数强相关的原因。
- 下坡且坡度助力超过滚动与空气阻力时，总阻力（及能耗）可能为负，表示车辆处于制动或能量回收状态，负值代表回收的能量，而非消耗。
- 模型为稳态匀速行驶的阻力 / 能耗估算，未考虑加减速惯性、弯道阻力、轮胎温升、空调与附件功耗；实际整车能耗还需结合行驶循环工况与实测标定。
- `cruise_scene.png` 是 MuJoCo 离屏渲染的三车型场景画面，车体体型仅用于可视化风阻外形差异；定量结论以阻力 / 功率 / 能耗曲线与 `metrics.csv` 为准，不把场景画面当作实车或高保真仿真结果。
