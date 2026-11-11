# 车辆稳态操稳特性对比仿真（不足 / 中性 / 过度转向）

核心分析为纯 Python 数值仿真（线性自行车模型），不依赖 ROS / Gazebo / CARLA；MuJoCo 仅作为可选的离屏渲染后端，用于渲染三车场景静态图与演示动图，未安装时自动跳过。模型在固定质量、轴距与轮胎侧偏刚度下改变质心前后位置（前重、居中、后重），计算稳态转向梯度 K，判定每种布置属于不足转向（understeer）、中性转向（neutral）还是过度转向（oversteer），并绘制横摆角速度增益、侧向加速度增益随车速的变化曲线，以及前后轴侧偏角随侧向加速度的变化曲线。

配套教程文档：`docs/ground/steady_state_handling_eval.md`

## 运行环境

- 操作系统：Windows 10 / 11（Linux 亦可，脚本后台出图，无需图形界面）
- Python：3.10 ~ 3.12
- 依赖：见 `requirements.txt`（`numpy`、`matplotlib`、`Pillow`；`mujoco` 为可选，仅用于场景静态图与演示动图离屏渲染）

## 运行步骤

1. 创建并激活虚拟环境，安装依赖：

   ```bash
   python -m venv handling_env
   # Windows: handling_env\Scripts\activate
   # Linux:  source handling_env/bin/activate
   pip install -r requirements.txt
   ```

2. 在本目录下运行：

   ```bash
   python main.py
   ```

   脚本会依次计算三种质心布置的轴荷与转向梯度，输出转向梯度柱状图、增益-车速曲线、前后侧偏角曲线，合成车速扫描曲线演示动图；若已安装 MuJoCo，再离屏渲染三车场景静态图与演示动图。

## 输出结果

运行结束后在脚本同级的 `results/` 目录下得到：

| 文件 | 说明 |
| --- | --- |
| `handling_gradient.png` | 三种布置的稳态转向梯度 K 柱状图，标注不足 / 中性 / 过度转向判定 |
| `handling_gains.png` | 横摆角速度增益、侧向加速度增益随车速变化曲线（双子图） |
| `handling_slip_angle.png` | 前 / 后轴侧偏角随侧向加速度变化曲线（三种布置对比） |
| `handling_demo.gif` | 车速扫描时横摆增益曲线的演示动图 |
| `handling_scene.png` | MuJoCo 离屏渲染的三车场景静态图（可选依赖，未装 MuJoCo 时不生成） |
| `handling_scene_demo.gif` | MuJoCo 离屏渲染的三车场景演示动图（可选依赖，未装 MuJoCo 时不生成） |
| `metrics.csv` | 各布置轴荷、转向梯度、判定与临界车速等指标数据表 |

> 输出图片命名与教程图片目录 `docs/img/ground/` 中的文件名一致，提交文档时将 `results/` 下的图片直接复制过去即可，无需改名。

## 注意事项

- 本模块分析的是车辆本体的稳态操稳固有特性，不包含转向控制器；与横向控制算法对比类模块（纯跟踪 / Stanley / LQR 等）的关注点不同。
- 转向梯度 K 的符号决定操稳类型：K>0 不足转向，K≈0 中性转向，K<0 过度转向；过度转向在临界车速处分母趋于零、横摆增益发散，车辆易后甩失稳。
- 模型为线性自行车模型，忽略轮胎非线性饱和、侧风、载荷转移对侧偏刚度的耦合影响；实车操稳还需结合轮胎试验数据与实车稳态回转试验标定。
- `handling_scene.png` 与 `handling_scene_demo.gif` 是 MuJoCo 离屏渲染的三车场景画面，横向偏移仅用于可视化三种操稳倾向；定量结论以曲线图与 `metrics.csv` 指标为准，不把场景画面当作实车或高保真仿真结果。
