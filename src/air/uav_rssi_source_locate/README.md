# uav_rssi_source_locate —— 隐藏无线信标的搜索与 RSSI 三边定位

"某处有一个会发信号的设备，飞过去找到它"这件事，难点不在搜，而在
**一次 RSSI 读数到底值几米**：对数距离路损模型下，0.5 dB 的量化步长
在 40 m 处就是 ±1.3 m 的单点测距误差，而且接收机有底噪——43 m 以外
根本收不到。本模块把这条物理链路完整建模，然后在它上面做三边定位：
扩张方形搜索到首次检测，收缩测量圈凑齐几何，阻尼最小二乘解出信标
位置，最后 MuJoCo 闭环真飞验证。

仓库里 `uav_waypoint_tracking` 是给定航点的跟踪，`uav_poi_orbit` 绕的
是已知点——"目标位置未知、要靠测量反解"这一层还没有模块覆盖，本模块
与它们互补。

## 四个坑（每个都实测踩过）

1. **量化取整方向**：RSSI 芯片按 0.5 dB 量化，向下取整会让读数系统性
   偏低、反演距离整体偏大 ~2%，最小二乘吸收不掉固定偏差——必须就近
   取整（`np.round`）。
2. **样本高度不能压扁**：RSSI 来自信标的真实 3D 距离，把 20 m 高度的
   测量点压到地面再解，反演距离凭空短了 ~6 m，解整体外移 20 m——
   样本必须存 3D，残差里显式带上高度差。
3. **单侧弧的镜像歧义**：首次检测发生在可检测半径边界上，探圈往往只
   圈到一小段信号弧；近共线样本让最小二乘在主轴两侧各有一个解，幽灵
   解残差反而更小（实测偏 57 m、残差 0.05 m vs 真解 0.4 m）。三道闸：
   估计出界即沿样本主轴镜像、逼近段信号越飞越差/长时间无信号即镜像、
   精圈整圈无信号退回逼近。
4. **最小二乘的局部极小**：同一组样本，初值差一点就收敛到残差 6.3 m
   的假点——多起点（质心 + 先验 + 8 方向网格）取残差最小者。

## 快速开始

```bash
pip install numpy mujoco matplotlib pytest

cd src/air/uav_rssi_source_locate
python main.py                    # 四个算例各真飞一次
python main.py --case northwest   # 单个算例（镜像歧义最重的那个）
python main.py --no-sim           # 仅算法层（链路面 + 解算自检 + 朴素基线）
python -m pytest test/ -q         # 19 项测试
python tools/make_gif.py          # 重录五张图

# ROS 2 侧（Humble）
colcon build --packages-select uav_rssi_source_locate
source install/setup.bash
ros2 launch uav_rssi_source_locate main.launch.py
# 节点订阅 /uav/odom 与 /rssi（Float32 dBm），攒够样本后在
# /uav_rssi_source_locate/estimate 发 JSON 定位结果（x/y/residual/n）
```

## 结果（MuJoCo 真飞，Quadrotor + GeometricController，dt = 0.01 s）

| 算例 | 信标位置 | 首次检测 | 定位误差 | 样本数 | 朴素"最强航点" |
| --- | --- | --- | --- | --- | --- |
| `center` | (140, 90) | 23.0 s | 0.15 m | 30 | 20 m |
| `southeast` | (200, 20) | 66.3 s | 0.31 m | 31 | 30 m |
| `northwest` | (30, 140) | 96.1 s | 0.19 m | 27 | 20 m |
| `east_edge` | (215, 150) | 81.9 s | 0.37 m | 31 | 25 m |

- 电波模型：rssi0 = −40 dBm@1 m、路损指数 n = 2.2、底噪 −76 dBm
  （可检测半径 43.3 m）、量化 0.5 dB；无人机在 20 m 高度搜索；
- 定位误差 = 最终估计与真信标的水平距离；四个算例全部通过复核
  （估计点上空的实测 RSSI 与"估计点即信标"自洽）；
- "朴素最强航点"列不做事后解算、只挑信号最强的方形航点，误差被
  30 m 网格步长锁死——这就是三边定位的收益（两个数量级）；
- `results/case_<name>_result.json` 有链路面、解算自检与闭环全程的
  机读记录。

## 模块结构

```
uav_rssi_source_locate/
├── core/rssi.py         RssiCfg：对数距离路损 + 底噪 + 量化 + 距离反演
├── core/locator.py      扩张方形航线、测量圈、LM 阻尼三边定位（多起点）
├── core/flyer.py        MuJoCo 闭环：爬升 → 方形搜索 → 探圈 → 逼近 →
│                        精圈收缩 → 复核；镜像歧义三道闸都在这里
├── core/cases.py        四个信标算例（中心/东南/西北/东北边）
├── ros/scan_node.py     薄壳节点：订阅 /uav/odom + /rssi，定期解算发布
├── config/…yaml         电波模型 / 解算调度 / 话题名
├── test/                16 项纯算法 + 3 项 MuJoCo 冒烟
└── tools/make_gif.py    轨迹 + 误差收敛双栏动图
```

## 已知限制

- 电波模型是理想对数距离路损，未建模多径衰落与遮挡（城区实测会有
  5~8 dB 的阴影衰落，需要把 quant_db 调大或加鲁棒核）；
- 镜像判据依赖"信标必在搜索区内"这一先验，信标贴界时（east_edge）
  出界判据的余量变小，主要靠逼近段的信号趋势翻转；
- 搜索高度固定 20 m：更低的高度差能压小测距误差，但可检测半径同步
  缩小，搜索时间变长，未做联合优化。
