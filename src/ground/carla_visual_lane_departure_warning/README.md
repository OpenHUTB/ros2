# 相机版车道偏离预警（carla_visual_lane_departure_warning）

用一台前视 RGB 相机代替地图 API 取车道线：图像进网络，出左右两条车道边界的位置，
再折算成带符号横向偏移，最后送进与仓库现有 LDW 相同的三级状态机。

地图算出来的偏移不作输入，只作**对照真值**——同一帧里两边同时算，直接比误差。

## 与仓库现有模块的区别

| 对比项 | `carla_lane_departure_warning` | 本模块 |
|---|---|---|
| 偏移来源 | 地图 API 的路点 | 前视相机图像 |
| 是否需要传感器 | 否 | 是（`sensor.camera.rgb`） |
| 神经网络 | 无 | 4096-64-32-2 全连接，26 万参数，手写反向传播 |
| 输出口径 | 带符号横向偏移（左正右负） | 同左，可直接对照 |

两个模块输出同一套符号、同一套阈值，所以能把「视觉算的偏移」和「地图算的偏移」
逐帧相减，误差就是视觉这条链路的成色。

## 运行环境

| 组件 | 版本 / 说明 |
|---|---|
| 仿真器 | CARLA 0.9.16，跑在 Windows 宿主机 |
| 客户端 | Ubuntu 22.04 虚拟机，VMware NAT |
| ROS | ROS 2 Humble |
| Python | 3.10 |
| 依赖 | `carla==0.9.16`、`numpy`、`Pillow`、`matplotlib`（仅画图用） |

宿主机 IP 在 VMware 的 VMnet8 网卡上，虚拟机里用 `ip route | grep default` 看不到，
要在 Windows 上 `ipconfig` 查，通常是 `192.168.148.1`。

## 安装

```bash
pip3 install carla==0.9.16
pip3 install numpy Pillow matplotlib
cd ~/ros2_ws/src/ros2
colcon build --packages-select carla_visual_lane_departure_warning
source install/setup.bash
```

## 运行步骤

先启动 Windows 上的 `CarlaUE4.exe`，等主界面加载完，再在虚拟机里按顺序执行。

### 1. 探路：确认路面有没有车道线

```bash
python3.10 main.py --probe --host 192.168.148.1 --frames 40 --every 10
```

`out/probe/` 下会出两种图：`probe_000.png` 是原图，`probe_000_annot.png` 上
红线是地图给的左边界、蓝线是右边界、绿线是中心线。红线蓝线压在路面白线上，
说明投影对得上，可以继续。

### 2. 采集数据集

```bash
python3.10 main.py --collect --host 192.168.148.1 --frames 4000 \
    --lane-change 80 --max-offset 2.0 --data data/lane_dataset.npz
```

自车挂在自动驾驶上，由交通管理器周期性强制变道，让横向偏移扫过量程；只贴车道中心
跑的话标签全是 0，网络只会学成「永远输出 0」。每帧存一张 32×128 的灰度图和左右边界
真值，共三类数据：
`images`、`targets`、`speeds`。采集时会把不可用的帧丢掉并打印原因：
车道宽异常、自车不在地图给的车道里、边界上没画线、边界出了画面、偏移过大。

4000 帧的采集在 Town03 上大约跑八分钟，能留下 2000 帧左右。

### 3. 训练与评价

```bash
python3.10 main.py --train --data data/lane_dataset.npz \
    --checkpoint models/lane_regressor.npz
```

训练默认 120 轮，并自动做左右镜像扩增（样本翻倍），纯 NumPy 跑十几秒。
训练完打印偏移误差，并把权重和 `models/metrics.json` 落盘。
仓库里已经带了一份训练好的权重，验证集横向偏移平均误差 0.122 m。训练目标是自车
前方 5 m 处的左右边界，在线真值用的是自车位置到中心线的垂距，两者还差一个曲率项；
实测变道演示的平均误差 0.317 m，其中车道内（偏移小于 0.3 m）为 0.19 m，压线时
误差明显变大，原因与改进方向写在 `docs/ground/carla_visual_lane_departure_warning.md`
的实验结果一节。

### 4. 在线演示

```bash
python3.10 main.py --demo --host 192.168.148.1 --frames 300 --lane-change 60 \
    --checkpoint models/lane_regressor.npz \
    --record out/frames --save-metrics out/demo_metrics.json
```

`--record` 会把每帧叠上 HUD 存成 PNG，方便用 ScreenToGif 合成动图。

### 5. 用 launch 启动

```bash
ros2 launch carla_visual_lane_departure_warning main.launch.py \
    host:=192.168.148.1 checkpoint:=models/lane_regressor.npz frames:=0
```

节点发布四个话题：

| 话题 | 类型 | 含义 |
|---|---|---|
| `/visual_lane_departure_warning/offset` | `Float32` | 相机估计的横向偏移（米，左正右负） |
| `/visual_lane_departure_warning/offset_map` | `Float32` | 地图真值 |
| `/visual_lane_departure_warning/state` | `String` | `normal` / `deviation` / `departure` |
| `/visual_lane_departure_warning/event` | `String` | 状态跃迁事件 |

## 离线自检

不连 CARLA 也能验证整条链路：

```bash
python3.10 main.py --self-check
python3.10 test/test_geometry.py
python3.10 test/test_model.py
python3.10 test/test_metrics.py
python3.10 test/test_lanes.py
```

`--self-check` 用合成图像训练一个网络，偏移误差应在 0.1 m 以内；
四个测试文件共 32 项，全部通过才算环境没装错。

## 目录说明

```text
carla_visual_lane_departure_warning/
├── main.py                 入口，五个模式
├── main.sh / main.bat      统一入口脚本
├── launch/main.launch.py   ROS 2 启动
├── config/params.yaml      参数
├── carla_visual_lane_departure_warning/
│   ├── geometry.py         内参、投影、横向偏移（纯函数）
│   ├── dataset.py          采集、存取、划分
│   ├── model.py            网络与训练
│   ├── metrics.py          误差指标
│   ├── lanes.py            三级滞回状态机与事件对照
│   ├── selfcheck.py        合成数据自检
│   ├── carla_common.py     CARLA 连接与相机挂载
│   ├── probe.py            探路
│   ├── demo.py             在线演示
│   └── node.py             ROS 2 节点
├── figures.py              画文档配图
└── test/                   离线测试
```

## 已知限制

- 只在有清晰车道线的路段有效，停车场、小路没有标线，网络给不出可信结果。
- 网络输入是 32×128 灰度图，弯道半径小的时候远端边界会被压扁。
- 训练数据与测试数据来自同一条路线，跨地图的泛化能力没有验证。