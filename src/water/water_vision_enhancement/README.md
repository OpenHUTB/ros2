# water_vision_enhancement

基于 OpenCV 的浑浊水体视觉增强与目标追踪 ROS 2 模块。

## 功能概述

本模块面向水下机器人（ROV/AUV）在浑浊水体中的视觉感知需求，提供两级处理：

1. **浑浊水体图像增强**：针对水下成像的色彩衰减（红光被吸收）、对比度下降（悬浮颗粒散射）和雾化效应，依次执行白平衡、通道增益校正、CLAHE 局部对比度增强和暗通道去散射。
2. **目标追踪**：在增强后的图像上运行 OpenCV 追踪器（KCF / CSRT / MOSSE），输出目标边界框与归一化中心点坐标。

模块附带一个合成视频演示节点，可在无外部相机/仿真环境时独立验证整条链路。

## 运行环境

- 操作系统：Ubuntu 22.04
- ROS 2 版本：Humble
- 依赖：rclcpp、sensor_msgs、geometry_msgs、std_msgs、cv_bridge、image_transport、tf2_ros、OpenCV 4.5.4（含 opencv_contrib 追踪模块）；scripts/render_demo.py 另需 python3-pil（rosdep 可自动安装）

## 核心文件结构

```
src/water/water_vision_enhancement/
├── include/water_vision_enhancement/
│   ├── image_enhancer.hpp        # 图像增强器声明
│   └── object_tracker.hpp        # 目标追踪器声明
├── src/
│   ├── image_enhancer.cpp        # 色彩校正+CLAHE+去散射实现
│   ├── object_tracker.cpp        # KCF/CSRT/MOSSE 追踪实现
│   ├── vision_node.cpp           # 主节点：订阅→增强→追踪→发布
│   └── video_demo_node.cpp       # 演示节点：合成浑浊水体视频
├── launch/
│   └── main.launch.py            # 一键启动
├── config/
│   └── params.yaml               # 参数配置
├── rviz/
│   └── vision_enhancement.rviz   # RViz2 可视化配置
├── scripts/
│   └── render_demo.py            # 离线渲染演示 GIF（抓取节点实际输出合成三联对比动图）
├── resource/
│   └── water_vision_enhancement  # 资源标记
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 快速运行

1. 创建工作空间并拷贝模块：

   ```
   mkdir -p ~/ros2_ws/src
   cp -r ~/ros2/src/water/water_vision_enhancement ~/ros2_ws/src/
   cd ~/ros2_ws
   ```

2. 安装依赖并编译：

   ```
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --packages-select water_vision_enhancement
   source install/setup.bash
   ```

   注：追踪器依赖 opencv_contrib 的 tracking 模块。若系统中存在多套 OpenCV，编译时需显式指定带 contrib 模块的那一套，例如追加 --cmake-args -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4。

3. 启动演示（合成浑浊水体视频 + 增强 + 追踪）：

   ```
   ros2 launch water_vision_enhancement main.launch.py
   ```

4. 另开终端查看话题：

   ```
   source install/setup.bash
   ros2 topic echo /enhancement_metrics
   ros2 topic echo /tracked_object
   ```

5. 可选：RViz2 查看增强前后对比：

   ```
   ros2 run rviz2 rviz2 -d install/water_vision_enhancement/share/water_vision_enhancement/rviz/vision_enhancement.rviz
   ```

## 话题说明

| 话题 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/camera/image_raw` | `sensor_msgs/Image` | 输入 | 原始水下图像（演示节点发布） |
| `/enhanced_image` | `sensor_msgs/Image` | 输出 | 增强后图像 |
| `/tracking_result` | `sensor_msgs/Image` | 输出 | 带追踪边界框的可视化图像 |
| `/tracked_object` | `geometry_msgs/PointStamped` | 输出 | 目标归一化中心点（x,y 为 0～1，z 为置信度） |
| `/enhancement_metrics` | `std_msgs/Float32MultiArray` | 输出 | [对比度, 色彩度, PSNR(dB), 追踪置信度] |

## 参数说明

| 参数 | 默认值 | 所属节点 | 说明 |
| --- | --- | --- | --- |
| `red_gain` | 1.35 | water_vision_node | 红光通道增益（补偿水体吸收） |
| `green_gain` | 1.0 | water_vision_node | 绿光通道增益 |
| `blue_gain` | 0.85 | water_vision_node | 蓝光通道增益（抑制偏蓝） |
| `use_clahe` | true | water_vision_node | 是否启用 CLAHE |
| `clahe_clip_limit` | 2.0 | water_vision_node | CLAHE 裁剪阈值 |
| `clahe_tile_grid` | 8 | water_vision_node | CLAHE 网格大小 |
| `use_dehaze` | true | water_vision_node | 是否启用去散射 |
| `dehaze_strength` | 0.6 | water_vision_node | 去散射强度（0～1） |
| `atmosphere_light` | 0.95 | water_vision_node | 大气光估计值（0～1） |
| `tracker_type` | KCF | water_vision_node | 追踪器类型（KCF/CSRT/MOSSE） |
| `min_confidence` | 0.3 | water_vision_node | 追踪置信度下限 |
| `auto_reinit` | true | water_vision_node | 丢失后是否自动重置追踪器 |
| `detect_interval` | 30 | water_vision_node | 自动重置前的连续丢失帧数 |
| `init_bbox_x/y/w/h` | 290 / 220 / 60 / 40 | water_vision_node | 初始追踪框，对准演示目标初始位置 |
| `turbidity` | 0.5 | water_video_demo_node | 合成水体浑浊度（0～1） |
| `target_speed` | 80.0 | water_video_demo_node | 目标移动速度（px/s） |
| `width / height / fps` | 640 / 480 / 20 | water_video_demo_node | 输出图像尺寸与帧率 |
