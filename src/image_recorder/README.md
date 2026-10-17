# 地面载具 - 图像录制 ROS 2 封装模块

本模块属于 ROS 2 与仿真环境之间的桥接功能。基于 ROS 2 实现了一个图像录制节点，用于订阅 Carla 仿真器发布的摄像头话题，按下回车键即可将当前图像保存至本地。

## 核心代码内容

```text
src/
└── image_recorder/
    ├── main.py                 # 图像订阅与录制主程序
    ├── setup.py                # ROS 2 功能包配置文件
    ├── package.xml             # ROS 2 依赖声明文件
    ├── README.md               # 模块说明文档 (本文件)
    └── launch/
        └── main.launch.py      # 一键启动文件
```

运行步骤

```bash
colcon build --packages-select image_recorder
source install/setup.bash
ros2 launch image_recorder main.launch.py
```

> 📖 **详细文档与运行截图请参阅：** [查看主文档](../../docs/ground/image_recorder.md)
