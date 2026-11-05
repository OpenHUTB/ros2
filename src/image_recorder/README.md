# 地面载具 - 图像录制 ROS 2 封装模块

本模块属于 ROS 2 与仿真环境之间的桥接功能。基于 ROS 2 实现了一个图像录制节点，用于订阅 Carla 仿真器发布的摄像头话题，按下回车键即可将当前图像保存至本地。

## 核心代码内容

```text
src/
└── image_recorder/               # 包名根目录
    ├── image_recorder/           # 核心代码同名子文件夹
    │   ├── __init__.py           # 空文件（标识 Python 包）
    │   └── main.py               # 主程序
    ├── resource/                 # 资源目录
    │   └── image_recorder        # 无后缀空文件（供 ament 索引识别）
    ├── launch/                   # 启动文件目录
    │   └── main.launch.py        # 一键启动节点
    ├── package.xml               # 依赖声明
    ├── setup.py                  # 构建配置
    ├── setup.cfg                 # 新增文件
    └── README.md             
```

## 运行步骤

1. 新建标准工作空间并进入 src 目录：
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```
2.从 ros2 仓库拷贝模块代码到工作空间：
```
cp -r ~/OpenHUTB/ros2/src/image_recorder ~/ros2_ws/src/
cd ~/ros2_ws
```
3.编译工作空间并部署环境：
```
colcon build --packages-select image_recorder
source install/setup.bash
```
4.运行图像录制节点：
```
ros2 launch image_recorder main.launch.py
```

> 📖 **详细文档与运行截图请参阅：** [查看主文档](../../docs/ground/image_recorder.md)
