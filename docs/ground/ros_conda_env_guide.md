# ROS 与 Conda 环境共存排错指南

在 OpenHUTB 项目中开发感知、控制等算法模块时（如 #125、#129），深度学习依赖（PyTorch、YOLO）通常安装在 Conda 环境中，而 ROS Noetic 默认依赖系统 Python 3.8。两者混合使用极易导致各类报错，本文总结了 5 个核心排错经验。

## 1. import rospy 报错 ModuleNotFoundError: No module named yaml / rospkg

**原因**：Conda 环境相对独立，没有继承系统 ROS 的基础包。
**解决**：在 Conda 环境中补齐 ROS 的 Python 依赖（必须固定 empy==3.3.4，否则会导致编译报错）：

    pip install pyyaml rospkg catkin_pkg "empy==3.3.4" -i https://mirrors.aliyun.com/pypi/simple/

## 2. cv_bridge 与 Conda 环境中的 OpenCV 冲突

**原因**：ROS 自带的 cv_bridge 是基于系统 OpenCV（如 4.2）编译的。如果在 Conda 环境里装了更高版本的 OpenCV（如 4.9 或 5.0），调用 cv_bridge 时会报 KeyError 或 Segmentation fault。
**解决**：有两种方案。

- **方案 A（推荐）**：在 ROS 节点代码中绕过 cv_bridge，手动构造和解析 sensor_msgs/Image 消息。
- **方案 B**：强制降级 Conda 环境的 OpenCV 版本，使其与系统版本一致。

## 3. roslaunch 找不到节点或报 RLException

**原因**：新开终端没有 source 工作空间的 devel/setup.bash，或者 python 解释器找不到。
**解决**：
在使用 roslaunch 或 rosrun 之前，确保：

    source /opt/ros/noetic/setup.bash
    source ~/carla_xxx_ws/devel/setup.bash

## 4. import torch 或 import ultralytics 失败

**原因**：直接 roslaunch 时调用的是系统 Python 3.8，而依赖装在 Conda 中。
**解决**：在 launch 文件启动前，通过脚本先激活 Conda 环境，或者显式指定使用 Conda 的 Python 路径。推荐使用 conda run 或 launch-prefix，**不要直接修改全局 PYTHONPATH**（会导致 cv_bridge 冲突）。

    # 推荐方式一：使用 conda run 启动单节点
    conda run -n carla38 python my_ros_node.py

    # 推荐方式二：在 roslaunch 中使用 launch-prefix
    <node pkg="my_pkg" type="my_node.py" name="my_node" launch-prefix="conda run -n carla38" />

    # 如果需要编写 main.sh 非交互脚本，必须先 source 环境：
    source $(conda info --base)/etc/profile.d/conda.sh
    conda activate carla38

## 5. Linux 下 Carla 客户端报 undefined symbol

**原因**：Conda 环境中的 libc++ 与 Carla whl 不兼容。
**解决**：在运行节点前设置 LD_PRELOAD。**注意：必须先激活 Conda 环境**，否则 $CONDA_PREFIX 为空，会导致节点直接崩溃。

    if [ -z "$CONDA_PREFIX" ]; then
        echo "错误: 请先执行 conda activate carla38"
        exit 1
    fi
    export LD_PRELOAD=$CONDA_PREFIX/lib/libc++.so.1:$CONDA_PREFIX/lib/libc++abi.so.1
