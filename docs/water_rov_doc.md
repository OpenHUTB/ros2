# water 功能包：水下 ROV 声呐图像 CNN 目标识别

## 概述

`water` 是 ROS2 Python 功能包，用 Gazebo 搭建水下场景，通过相机采集图像，用 3 层卷积网络对图像做 3 类目标分类（rock / pipe / sand），并把带标注的结果图发布到 RViz 或 `rqt_image_view`。

> 说明：当前 Gazebo 世界中使用的是普通 RGB 相机（`libgazebo_ros_camera.so`）模拟声呐成像，不是真实侧扫声呐传感器。若用于论文实验，应改为真实声呐数据集离线推理，或在文中如实说明"以相机图像代理声呐图像验证流程"。

## 目录结构

```
src/water
├── water/
│   ├── __init__.py
│   ├── sonar_model.py        # 网络定义（训练/推理共用）
│   ├── sonar_infer_node.py   # 推理节点
│   └── train_sonar.py        # 训练脚本（ImageFolder 数据集）
├── launch/
│   └── rov.launch.py
├── worlds/
│   └── underwater.world
├── weights/                  # 训练输出目录（*.pth，可选安装）
├── resource/
│   └── water
├── package.xml
├── setup.cfg
└── setup.py
```

## 环境依赖

- ROS2（Humble 及以上）+ Gazebo classic
- `sudo apt install ros-$ROS_DISTRO-gazebo-ros-pkgs ros-$ROS_DISTRO-cv-bridge python3-opencv`（注意：是 Ubuntu 侧的 ROS2 环境，不是 Windows PowerShell）
- pip 依赖：

```bash
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
pip install opencv-python -i https://pypi.tuna.tsinghua.edu.cn/simple
```

## 编译

```bash
colcon build --packages-select water
source install/setup.bash
```

## 训练（必须先有真实数据集）

数据集按类别子文件夹组织：

```
dataset/
├── rock/*.jpg
├── pipe/*.jpg
└── sand/*.jpg
```

```bash
cd src/water
ros2 launch water rov.launch.py
```

确认数据链路：

```bash
ros2 topic hz /sonar/camera/image_raw     # 应为 15 Hz 左右
ros2 topic echo /sonar/result --once      # 带标注的结果图
rqt_image_view                            # 选 /sonar/result 查看
```

常用参数覆盖：

```bash
ros2 run water sonar_infer_node --ros-args \
  -p weight_file:=/absolute/path/sonar_cnn_weights.pth \
  -p image_topic:=/sonar/camera/image_raw \
  -p infer_period:=0.5 \
  -p device:=cuda \
  -p classes:="[rock,pipe,sand]"
```

## 效果图

![Gazebo 水下仿真场景](img/sonar_gazebo_simulation.jpg)

![声呐识别节点运行输出](img/sonar_infer_output.jpg)

## 常见问题

| 现象 | 原因 | 处理 |
|---|---|---|
| Gazebo 打开后世界为空 / 报 world 解析错误 | world 文件未安装或 SDF 语法错误 | 确认 `install/water/share/water/worlds/underwater.world` 存在；`gz sdf -k src/water/worlds/underwater.world` 自检 |
| 节点一直无输出、`ros2 topic hz` 无数据 | 订阅话题与实际话题不一致 | 用 `ros2 topic list | grep sonar` 确认真实话题名，再改 `-p image_topic:=` |
| 启动即报"未找到权重文件" | 未训练或未重新 colcon build | 先训练到 `weights/`，再 `colcon build` 安装 |
| 图像全黑 | 相机贴住海床或场景过暗 | 已通过 `<static>true</static>` 固定 ROV；可调高 `rov` 的 pose z 值 |
