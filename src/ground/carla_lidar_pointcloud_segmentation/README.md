# carla_lidar_pointcloud_segmentation

在 CARLA 里做**激光雷达点云语义分割**：给自车装一台普通激光雷达
（`sensor.lidar.ray_cast`）和一台同机位的语义激光雷达（`sensor.lidar.ray_cast_semantic`），
用后者的逐点标签当监督信号，训练一个把普通点云逐点分成
**可行驶面 / 车辆 / 行人 / 障碍物** 四类的神经网络。

分割结果再交给轨迹跟踪：可行驶区域之外的激光点在地图上形成"禁行带"，
自车据此在给定参考轨迹上做减速与横向避让。

和仓库里其它激光雷达模块的区别：AEB、BSD、CSL 用的是**几何规则**（聚类、扇区、
高度阈值），本模块是**学习**出来的，且不需要下载任何外部数据集——
标签由仿真器自己给。

## 1. 运行环境

| 项目 | 版本 |
|---|---|
| 操作系统 | Ubuntu 22.04（虚拟机）/ Windows 11 |
| ROS | ROS 2 Humble（Ubuntu 22.04 官方源即可） |
| 仿真器 | CARLA 0.9.16 |
| Python | 3.10（Ubuntu 22.04 自带） |

CARLA 装在 Windows 宿主机、ROS 装在 VMware 虚拟机时，虚拟机的网卡是 NAT 模式，
宿主机在虚拟网段上的地址一般为 `192.168.148.1`，用 `--host` 指过去即可。

## 2. 目录结构

    carla_lidar_pointcloud_segmentation/
    ├── main.py                 统一入口（--self-check / --probe / --collect / --train / --demo）
    ├── main.sh / main.bat      一键入口
    ├── config/params.yaml      ROS 2 节点参数
    ├── launch/main.launch.py   启动文件
    ├── carla_lidar_pointcloud_segmentation/
    │   ├── labels.py           语义标签 -> 四类（含实测标签表）
    │   ├── features.py         逐点特征、体素下采样、标准化
    │   ├── metrics.py          混淆矩阵、IoU/F1、分距离评价
    │   ├── model.py            纯 NumPy 多层感知机
    │   ├── pipeline.py         特征 -> 训练 -> 评价 的流水线
    │   ├── control.py          走廊筛选、纯跟踪、速度系数
    │   ├── figures.py          混淆矩阵与分距离曲线出图
    │   ├── carla_common.py     连接、同步模式、生成车辆/行人、双雷达配对
    │   ├── probe.py            实测语义标签编号
    │   ├── app.py              采集与闭环演示
    │   └── node.py             ROS 2 节点
    ├── test/                   离线单元测试（不需要 CARLA）
    ├── models/                 训练好的权重（.npz，入库便于免训复现）
    └── data/                   采集的数据集（.npz，不入库，可再生）

## 3. 安装

    cd src/ground/carla_lidar_pointcloud_segmentation
    python3 -m pip install --user -r requirements.txt

ROS 2 工作空间里还需要：

    colcon build --packages-select carla_lidar_pointcloud_segmentation
    source install/setup.bash

## 4. 运行步骤

### 4.1 离线自检（不需要 CARLA）

    python3.10 main.py --self-check

用合成点云跑通"特征 -> 训练 -> 评价"，确认整条链路可用，末行打印宏平均 IoU。

自检的产物单独叫 `models/_selfcheck.*`，不会覆盖入库的 `models/seg_mlp.npz`——
这两个文件长得一模一样，混在一起会让 `--demo` 悄悄用上合成数据训出来的模型。

### 4.2 实测语义标签（换地图或换 CARLA 版本后必做）

```bash
python3.10 main.py --probe --host 192.168.148.1
```

CARLA 的 `object_tag` 编号在不同打包版本里不一致，官方文档给的是 Roads=7、
Vehicles=11、Pedestrians=4，而本项目实测的 0.9.16 是 Roads=1、Vehicles=14、
Pedestrians=12。这个模式会在自车前方放一辆已知车辆、右侧放一个已知行人，
打印各自附近的标签，用来核对 `labels.py` 里的实测表。

### 4.3 采集数据

先启动 CARLA 服务端（Windows 上双击 `CarlaUE4.exe`），然后在虚拟机里：

    python3.10 main.py --collect --host 192.168.148.1 --town Town03 --frames 300

生成 `data/dataset.npz`，里面是 `points / tags / frames` 三个数组。

### 4.4 训练与评价

    python3.10 main.py --train --data data/dataset.npz --epochs 30

148 万点、30 轮约 1 分 40 秒。终端打印逐类 IoU/精确率/召回率、混淆矩阵、
分距离档准确率，并写出四个文件：

    models/seg_mlp.npz          权重
    models/seg_mlp.stats.npz    特征标准化统计量
    models/metrics.json         全部指标（文档里的数字直接取自这里）
    figures/lidar_seg_confusion.png、figures/lidar_seg_range_iou.png

出图需要 matplotlib，没装会提示并跳过，不影响训练：

    python3.10 -m pip install --user matplotlib

### 4.5 闭环演示

    python3.10 main.py --demo --checkpoint models/seg_mlp.npz --host 192.168.148.1 --town Town03 --frames 150

自车由分割结果驱动：走廊内最近阻挡物距离经 `speed_factor` 换成目标车速，
再折算成油门；参考轨迹取当前车道中心线往前 30 m，用纯跟踪算转角。
每帧打印点数、可行驶比例、前方余量、速度系数、车速与三个控制量，
结束时汇总前方余量、车速、横向偏差和与其它车辆的真实最小中心距。

要录演示动图就加 `--record`，每帧存一张 PNG（用无界面后端，不需要桌面）：

    python3.10 main.py --demo --record frames --checkpoint models/seg_mlp.npz --host 192.168.148.1 --frames 120

之后用 Pillow 或 ffmpeg 合成 gif 即可，例如：

    python3.10 -c "from PIL import Image; import glob; \
    ps=sorted(glob.glob('frames/frame_*.png')); \
    ims=[Image.open(p).convert('RGB').resize((480,360)) for p in ps]; \
    ims[0].save('demo.gif', save_all=True, append_images=ims[1:], duration=80, loop=0, optimize=True)"

调参：`--comfort-distance`（默认 12 m，越大越早减速）、`--lookahead`
（默认 6 m，越大越平顺但过弯会切内道）、`--max-speed`（默认 8 m/s）。

### 4.6 ROS 2 节点

    ros2 run carla_lidar_pointcloud_segmentation lidar_segmentation_node --ros-args \
        --params-file config/params.yaml

或：

    ros2 launch carla_lidar_pointcloud_segmentation main.launch.py

## 5. 离线单元测试

    python3.10 test/test_features.py
    python3.10 test/test_labels.py
    python3.10 test/test_segmentation.py
    python3.10 test/test_control.py
    python3.10 test/test_buffers.py
    python3.10 test/test_demo_offline.py

每个文件末行输出 `[结果] N 项全部通过`，六个文件共 170 项。

其中 `test_demo_offline.py` 用假的 `carla` 模块把 `--demo` 的整条循环跑一遍，
控制量、统计指标、清理流程都能在没有 CARLA 的机器上验证。

## 6. 指标

### 6.1 分割精度（300 帧采集、30 轮训练）

| 类别 | IoU | 精确率 | 召回率 | F1 |
|---|---|---|---|---|
| 可行驶面 | 0.996 | 0.998 | 0.999 | 0.998 |
| 车辆 | 0.955 | 0.958 | 0.996 | 0.977 |
| 行人 | 0.993 | 0.995 | 0.998 | 0.996 |
| 障碍物 | 0.993 | 0.998 | 0.994 | 0.996 |

宏平均 IoU 0.984，宏平均 F1 0.992，总体准确率 0.997。

分距离：0-20 m IoU 0.977、20-40 m 0.970、40-60 m 0.886。

### 6.2 下游控制（120 帧闭环）

| 指标 | 数值 |
|---|---|
| 前方余量 | 最小 8.2 m，平均 12.2 m |
| 车速 | 平均 4.0 m/s，最高 5.2 m/s |
| 横向偏差 | 平均 0.25 m，最大 0.91 m |
| 与其它车辆的真实最小中心距 | 9.0 m |

跟车时速度系数从 1.00 降到 0.52，车速随之从 5.2 m/s 降到 3.3 m/s，
说明减速确实由分割结果驱动。

完整分析与图片见 [docs/ground/carla_lidar_pointcloud_segmentation.md](../../../docs/ground/carla_lidar_pointcloud_segmentation.md)。

## 7. 已知限制

* 只处理单帧点云，没有时序融合，被遮挡后重新出现的物体要等下一帧；
* `object_tag` 编号依赖 CARLA 打包版本，换版本必须重跑 `--probe` 核对映射表；
* 行人和杆子在 40 m 外只有几个点，逐点分类必然吃亏，靠分距离曲线如实呈现；
* 行人是静止的：移动行人需要额外的控制器生命周期管理，当前只保证数据里有这一类；
* 控制是纯油门调节，余量变小时松油门减速，没有主动制动；
* 网络只有 1508 个参数，属于轻量基线，不是精度上限；
* 合成数据的自检结果只证明链路跑得通，不代表真实精度。

## 8. 大模型声明

本模块的代码与文档在编写过程中使用了大语言模型辅助，作者对提交内容的正确性负责。