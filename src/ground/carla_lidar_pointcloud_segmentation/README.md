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
    ├── main.py                 统一入口（--collect / --train / --self-check）
    ├── main.sh / main.bat      一键入口
    ├── carla_lidar_pointcloud_segmentation/
    │   ├── labels.py           语义标签 -> 四类
    │   ├── features.py         逐点特征、体素下采样、标准化
    │   ├── metrics.py          混淆矩阵、IoU/F1、分距离评价
    │   ├── model.py            纯 NumPy 多层感知机
    │   └── pipeline.py         特征 -> 训练 -> 评价 的流水线
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

    python3 main.py --self-check

用合成点云跑通"特征 -> 训练 -> 评价"，确认整条链路可用，末行打印宏平均 IoU。

### 4.2 采集数据

先启动 CARLA 服务端（Windows 上双击 `CarlaUE4.exe`），然后在虚拟机里：

    python3 main.py --collect --host 192.168.148.1 --town Town03 --frames 300

生成 `data/dataset.npz`，里面是 `points / tags / frames` 三个数组。

### 4.3 训练与评价

    python3 main.py --train --data data/dataset.npz --epochs 30

终端打印逐类 IoU/精确率/召回率、分距离档准确率和混淆矩阵，
权重写到 `models/seg_mlp.npz`。

### 4.4 ROS 2 节点

    ros2 run carla_lidar_pointcloud_segmentation lidar_segmentation_node --ros-args \
        --params-file config/params.yaml

或：

    ros2 launch carla_lidar_pointcloud_segmentation main.launch.py

## 5. 离线单元测试

    python3 test/test_features.py
    python3 test/test_labels.py
    python3 test/test_segmentation.py

每个文件末行输出 `[结果] N 项全部通过`。

## 6. 指标

| 指标 | 说明 |
|---|---|
| 逐类 IoU / F1 | 可行驶面、车辆、行人、障碍物 |
| 分距离准确率 | 0~20 m、20~40 m、40~60 m 三档 |
| 混淆矩阵 | 重点看"可行驶面被判成障碍物"这类致命错误 |
| 下游：避障成功率 | 参考轨迹上放障碍物，能否绕开 |
| 下游：横向偏差 RMS | 避障后回到轨迹的精度 |

## 7. 已知限制

* 只处理单帧点云，没有时序融合，被遮挡后重新出现的物体要等下一帧；
* 语义标签依赖 CARLA 的 `CityObjectLabel` 编号，换 CARLA 版本需重新核对映射表；
* 行人和杆子在 40 m 外只有几个点，逐点分类必然吃亏，靠分距离曲线如实呈现；
* 合成数据的自检结果只证明链路跑得通，不代表真实精度。

## 8. 大模型声明

本模块的代码与文档在编写过程中使用了大语言模型辅助，作者对提交内容的正确性负责。