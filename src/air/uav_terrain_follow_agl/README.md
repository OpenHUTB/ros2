# uav_terrain_follow_agl — 低空地形跟随（恒定离地高度保持）

要求无人机全程保持固定离地高度（AGL 15 m），但地形在变、雷达高度表又慢又抖。
对比三种策略：**定高（MSL）**、**纯雷达反馈**、**DEM 前馈 + 慢环校正**，在
MuJoCo 四旋翼闭环上按真值口径评估最小离地间隙。

核心结论：

- 定高在所有带起伏的地形上都会侵入地形（30 m 山脊上深 15 m）；
- 纯反馈在恒坡上最好（AGL RMS 0.20 m），但山脊上会跌破安全间隙（4.17 m），
  12 m/s 与混合地形直接侵入；
- DEM 前馈把爬升率预算显式用起来（提前量 ≈ 需爬升高度 ÷ 爬升率上限 × 速度），
  四个算例最小间隙 12.2 / 10.2 / 6.8 / 15.0 m，全部高于 5 m 安全阈值；
- **安全靠前馈，精度靠慢环**：DEM 质量三档下最小间隙几乎不变（12.15~12.29 m），
  AGL RMS 从 3.31 涨到 5.35 m。

## 快速开始

```bash
python main.py                                   # MuJoCo 闭环主表
python main.py --sweep --dem-levels --ablation --json   # 全部实验组
python main.py --no-sim                          # 不装 mujoco 也能跑
python -m pytest test -q                         # 101 项测试（需 mujoco + 同仓库 uav_mujoco_sim）
python tools/make_figures.py                     # 文档配图
```

详细实验报告（含公式、消融与设计规则）见仓库文档
`docs/air/uav_terrain_follow_agl.md`。

## 结构

```text
uav_terrain_follow_agl/
  core/terrain.py      解析地形场（缓坡 + 高斯山脊/沟谷，坡度有闭式）
  core/altimeter.py    雷达高度表：采样保持 + 0.12 s 延迟 + 量化 + 噪声 + 丢帧
  core/dem.py          DEM 及其误差（偏置 / 水平移位 / 网格分辨率）
  core/control.py      三种策略与垂向限幅
  core/flyer.py        闭环仿真（MuJoCo / 一阶滞后运动学两种被控对象）
  core/evaluate.py     真值口径的指标
  core/cases.py        算例定义
  cli.py               main.py 的实现
  ros/terrain_follow_node.py   ROS 2 在线节点（核心状态机可脱离 rclpy 单测）
  tools/make_figures.py        生成 docs/img/air/uav_terrain_follow_agl/ 下的配图
```

## 与仓库已有模块的关系

- `uav_altitude_sensor_fusion`：高度怎么估（IMU+气压融合），不涉及地形；
- `uav_altitude_command_guard`：高度指令限幅防护，不看地形；
- `airsim_height_aware_overfly`：前视深度图决定越障还是绕行，不保持恒定 AGL；
- 本模块：沿航迹连续保持恒定离地高度，核心是**延迟传感 + 爬升率受限下的
  前视参考生成**。

## 已知局限

地形不进入 MuJoCo 碰撞场景（底座模型机身 geom 本就关闭碰撞，见文档第 3 节），
"侵入深度"是几何量；前视剖面默认沿航向；风未单独建模。完整清单见文档第 9 节。

## 大模型使用声明

使用了 AI 大语言模型辅助代码实现、测试编写与文档撰写，作者已逐项复核并对内容负责。

——

蒋宇鑫 2535030012
