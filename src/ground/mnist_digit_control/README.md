# mnist_digit_control：MNIST 手写数字视觉感知与机器人运动控制

对应课程任务 **(2) 传感器感知及运动控制**：使用摄像头（虚拟相机视频 / 真实 USB 摄像头）采集画面，
通过 **卷积神经网络（CNN）** 识别画面中的手写数字（MNIST 0-9），数字作为运动指令，
经连续多帧防抖确认后控制 **turtlesim** 机器人完成平移、旋转、停止等动作。

整个模块 **不依赖任何物理仿真器**（turtlesim 为 ROS2 自带组件），
在 Ubuntu 20.04 + ROS2 Humble 上开箱即用。

## 运行环境（实测）

| 项目 | 版本 |
| --- | --- |
| 系统 | Ubuntu 20.04.6 LTS（VMware 虚拟机，6 核 / 7.7 GB） |
| ROS2 | Humble（同时兼容 Noetic 的 ROS1 思路，但本模块为 ROS2 实现） |
| Python | 3.8.10 |
| PyTorch | 2.4.1+cpu（仅训练与推理） |
| OpenCV / numpy | 4.5.2 / 1.21.0（系统 ROS 自带） |

## 目录结构

```
src/ground/mnist_digit_control/
├── main.py                          # 一键入口：run/build/train/scenes/eval
├── package.xml  setup.py  setup.cfg # ROS2 ament_python 包
├── launch/mnist_digit_control.launch.py
├── mnist_digit_control/
│   ├── model.py                     # DigitCNN 网络定义
│   ├── preprocess.py                # Otsu 二值化 + 最大轮廓 + MNIST 归一化
│   ├── detector_node.py             # 相机帧 -> CNN -> 数字指令话题
│   └── command_node.py              # 防抖 + 数字-动作映射 + 位姿闭环控制
├── scripts/
│   ├── train_mnist.py               # 训练（自动下载 MNIST）
│   ├── make_scene_videos.py         # 生成 4 个多场景虚拟相机视频
│   ├── evaluate_scenes.py           # 多场景离线性能评价
│   └── evaluate_control.py          # 控制精度评价（旋转/平移）
├── models/mnist_cnn.pth             # 已训练权重（约 0.8 MB，随仓库提交）
├── videos/scene_*.mp4               # 4 个场景演示视频（main.py scenes 生成，不入库）
└── artifacts/                       # 评价指标 json（曲线/混淆矩阵等 png 由脚本生成，不入库，
                                     #   文档中的图表见 docs/mnist_digit_control*.png）
```

## 快速开始

```bash
cd ~/ros2/src/ground/mnist_digit_control

# 1) 安装依赖（torch CPU 版；OpenCV 等使用系统自带）
python3 -m pip install -r requirements.txt --index-url https://download.pytorch.org/whl/cpu

# 2) 生成 4 个演示场景视频（视频类二进制不入库，需本地生成一次）
python3 main.py scenes

# 3) 一键启动（首次自动 colcon build）：turtlesim + 检测节点 + 控制节点
python3 main.py

# 指定其他场景 / 真实摄像头：
python3 main.py run source:=video source_path:=/abs/path/scene_rotated.mp4
python3 main.py run source:=webcam display:=true
```

已训练权重 `models/mnist_cnn.pth` 随仓库提交，无需重新训练；如需复现训练执行
`python3 main.py train`。运行后向摄像头展示手写数字即可：数字需稳定出现约 0.6 秒
（6 帧 × 10 Hz）且置信度 ≥ 0.7，对应动作执行一次；数字消失或改变后才能再次触发。

## 数字-动作映射（小键盘布局）

| 数字 | 动作 | 数字 | 动作 |
| --- | --- | --- | --- |
| 2 | 前进 | 8 | 后退 |
| 4 | 左移 | 6 | 右移 |
| 7 | 左转 90° | 9 | 右转 90° |
| 1 | 左转 45° | 3 | 右转 45° |
| 0 | 原地旋转 360° | 5 | 停止 |

平移动作为定时长开环速度脉冲；旋转动作为基于 `/turtle1/pose` 的 **P 控制器闭环**（角度误差按
最短角差展开），转角精度见文档性能评价。

## 训练与评价

```bash
python3 main.py train    # 训练 CNN，输出 models/mnist_cnn.pth 与 artifacts 曲线
python3 main.py scenes   # 生成 4 个场景虚拟相机视频（用 MNIST 测试集数字合成）
python3 main.py eval           # 多场景准确率 / 检测率 / 置信度 / 推理耗时
python3 scripts/evaluate_control.py   # 控制精度：旋转误差 / 平移位移（需已构建并在图形环境运行）
```

## 话题与参数

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/digit_detector/digit` | std_msgs/Int32 | 当前帧识别数字（-1 无目标） |
| `/digit_detector/confidence` | std_msgs/Float32 | softmax 置信度 |
| `/digit_detector/debug_image` | sensor_msgs/Image | 检测框可视化 |
| `/mnist_digit_control/action` | std_msgs/String | 当前执行动作名 |
| `/turtle1/cmd_vel` | geometry_msgs/Twist | 速度指令 |
| `/turtle1/pose` | turtlesim_msgs/Pose | 旋转闭环反馈 |

关键参数（均可通过 launch/节点参数覆盖）：`stable_frames=6`、`conf_threshold=0.7`、
`angular_kp=3.0`、`angular_max=2.0 rad/s`、`angular_tol=0.03 rad`。

完整教程（含公式推导、算法流程、源码解析与性能评价）见仓库文档 `docs/mnist_digit_control.md`。

## 大模型使用声明

本模块代码与文档在 OpenHUTB 课程任务要求下，使用大模型辅助编写，作者已逐行审查、
在虚拟机中实际运行验证，并对提交内容负全部责任。
