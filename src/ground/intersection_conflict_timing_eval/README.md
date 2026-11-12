# 无信号路口冲突时序蒙特卡洛仿真

纯 Python 数值仿真，不依赖 ROS / Gazebo / CARLA / 任何模拟器。模拟无信号十字交叉口两个方向的直行车，各自以带扰动的匀速驶向路口冲突点，按到达时间差判断是否落入安全时距阈值内构成冲突；通过蒙特卡洛仿真统计不同限速、不同让行延迟下的冲突概率，并绘制到达时间差直方图、冲突概率-限速曲线与位置-时间轨迹图。

配套教程文档：`docs/ground/intersection_conflict_timing_eval.md`

## 运行环境

- 操作系统：Windows 10 / 11（Linux 亦可，脚本后台出图，无需图形界面）
- Python：3.10 ~ 3.12
- 依赖：见 `requirements.txt`（`numpy`、`matplotlib`、`Pillow`；`mujoco` 可选，仅用于路口双车场景静态图）

## 运行步骤

1. 创建并激活虚拟环境，安装依赖：

   ```bash
   python -m venv conflict_env
   # Windows: conflict_env\Scripts\activate
   # Linux:  source conflict_env/bin/activate
   pip install -r requirements.txt
   ```

2. 在本目录下运行：

   ```bash
   python main.py
   ```

   脚本会完成蒙特卡洛采样，输出到达时间差直方图、冲突概率-限速曲线、位置-时间轨迹图，并合成限速扫描演示动图。

## 输出结果

运行结束后在脚本同级的 `results/` 目录下得到：

| 文件 | 说明 |
| --- | --- |
| `conflict_histogram.png` | 两种车速情景到达时间差直方图，标注安全时距带 |
| `conflict_probability.png` | 冲突概率随限速变化曲线（无让行 / 让行 2s / 让行 4s） |
| `conflict_spacetime.png` | 两车位置-时间轨迹与冲突点示意 |
| `conflict_demo.gif` | 限速扫描时冲突概率曲线演示动图 |
| `conflict_scene.png` | MuJoCo 离屏渲染的路口双车场景静态图（网格地面斜俯视，配色与时空轨迹图一致） |
| `metrics.csv` | 各情景冲突概率、到达时差均值 / 标准差 / 95 分位数据表 |

> 输出图片命名与教程图片目录 `docs/img/ground/` 中的文件名一致，提交文档时将 `results/` 下的图片直接复制过去即可，无需改名。

## 注意事项

- 距离固定时，车速越快，到达时间 `t = D/v` 对速度扰动越不敏感，到达时差分布越窄，两车同时到达冲突点的概率反而升高；这是高速通过无信号路口风险更高的原因之一。
- 让行延迟相当于给次路车增加起步 / 停车等待时间，把到达时差整体平移出安全时距带，延迟足够大时冲突概率趋近于零。
- 模型为匀速直行车的到达时序统计，未考虑减速让行过程、驾驶员反应延迟、车型尺寸占用时间、视线遮挡与多车连续冲突；实际路口安全评价还需结合视距、转弯车流与实地观测。
