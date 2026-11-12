# UWB 多基站测距定位布站几何精度对比评测

纯 Python 数值仿真，不依赖 ROS / Gazebo / CARLA / 任何模拟器。在 100m×100m 区域内布置四种典型基站构型（等边三角形、正方形、一字形、L 形），对每种构型在区域网格上计算几何精度因子 GDOP，并通过蒙特卡洛测距仿真（测距叠加高斯噪声、最小二乘三边定位）统计定位 RMSE、95% 误差半径与最大误差，量化布站几何对测距定位精度的影响。

配套教程文档：`docs/ground/uwb_baseline_geometry_eval.md`

## 运行环境

- 操作系统：Windows 10 / 11（Linux 亦可，脚本后台出图，无需图形界面）
- Python：3.10 ~ 3.12
- 依赖：见 `requirements.txt`（`numpy`、`matplotlib`、`Pillow`）

## 运行步骤

1. 创建并激活虚拟环境，安装依赖：

   ```bash
   python -m venv uwb_env
   # Windows: uwb_env\Scripts\activate
   # Linux:  source uwb_env/bin/activate
   pip install -r requirements.txt
   ```

2. 在本目录下运行：

   ```bash
   python main.py
   ```

   脚本会依次计算四种构型的网格 GDOP、完成蒙特卡洛定位统计，输出布局图、GDOP 热力图、定位误差散点图、指标柱状图，并合成演示动图。

## 输出结果

运行结束后在脚本同级的 `results/` 目录下得到：

| 文件 | 说明 |
| --- | --- |
| `uwb_baseline_layout.png` | 四种构型基站布局俯视图 |
| `uwb_baseline_gdop_heatmap.png` | 各构型区域 GDOP 热力图（统一色标） |
| `uwb_baseline_error_scatter.png` | 蒙特卡洛真值与估计位置散点对比 |
| `uwb_baseline_metrics.png` | RMSE、95% 误差半径、平均 GDOP 对比柱状图 |
| `metrics.csv` | 各构型指标数据表 |
| `uwb_baseline_demo.gif` | 目标沿椭圆轨迹移动时正方形与一字形构型定位对比动图 |

> 输出图片的命名与教程图片目录 `docs/img/ground/` 中的文件名一致，提交文档时将 `results/` 下的图片直接复制过去即可，无需再改名。

## CARLA 实景运行（可选）

除纯数值仿真外，模块另提供一个连接 CARLA 服务端的实景出图脚本 `carla_demo.py`：在 Town 内生成一辆真值载具，按正方形构型布置虚拟 UWB 基站，读取载具真实位姿叠加测距噪声并三边定位，由高空俯视相机采集一帧 CARLA 真实渲染画面，与二维定位叠加层合成实景效果图。

### 环境与依赖

- CARLA 服务端 0.9.16（Windows 双击 `CarlaUE4.exe` 启动，监听 2000 端口）
- Python 3.10，额外安装：

  ```bash
  pip install carla==0.9.16 opencv-python
  ```

### 运行步骤

1. 先启动 CARLA 服务端，等待地图加载完成。
2. 在本目录下运行：

   ```bash
   python carla_demo.py
   ```

3. 脚本在 `results/` 目录生成 `carla_uwb_layout.png`（左为 CARLA 实景渲染帧，右为基站 / 真值 / 估计位置叠加层）。

> 实景效果图必须由已运行的 CARLA 服务端真实跑出；未连接服务端时脚本无法出图，也不应用纯仿真图像替代实景帧。

## 注意事项

- 仿真使用固定随机种子（`SEED`），测距噪声序列可复现；修改种子会改变蒙特卡洛的具体数值，但不改变构型之间的相对优劣。
- 一字形布站在几何上退化：基站共线时垂直于站列方向缺少观测约束，`JᵀJ` 接近奇异，定位结果会塌缩到站列直线上，这是该构型被列为反面教材的原因。
- 本实验为二维测距定位的几何精度分析，未考虑非视距（NLOS）遮挡、时钟漂移和基站位置误差；实际 UWB 部署还需结合现场多径环境标定测距噪声。
