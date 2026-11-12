# bug2_wall_following

包定位说明：本模块是基于 pygame 的独立 2D 仿真演示，不创建 rclpy 节点，也不发布 ROS 话题。

Bug2 反应式避障 2D 仿真。机器人从起点直线朝目标走，碰到圆形障碍时切换到沿墙跟随模式，绕障碍一周后再次穿过起点到目标连线 Lm、且该点比碰墙点更靠近目标时，切回直线朝目标，直到到达终点。

对应文档：docs/ground/bug2_wall_following.md

## 目录结构

```
bug2_wall_following/
├── main.py                     # 入口，转发到 bug2_wall_following.main
├── main.bat                    # Windows 启动脚本
├── main.sh                     # Linux 和 macOS 启动脚本
├── package.xml                 # 包声明，ament_python
├── requirements.txt            # 依赖 pygame 和 pytest
├── setup.py                    # 打包与安装路径
├── setup.cfg
├── README.md
├── resource/bug2_wall_following
├── launch/main.launch.py       # ROS 2 launch 占位入口
├── bug2_wall_following/
│   ├── __init__.py
│   ├── config.py               # 世界、起终点、障碍与运动参数
│   ├── geometry.py             # 点到线段距离、线段相交、障碍表面最近点
│   ├── bug2.py                 # 三态状态机与 hit_point 记录
│   └── main.py                 # pygame 渲染与主循环
└── test/test_bug2_logic.py
```

## 环境

- Windows 11，Python 3.13.16，pygame 2.6.1
- 安装依赖：

```bash
pip install -r requirements.txt
```

## 运行

```bash
cd src/ground/bug2_wall_following
python main.py
```

Windows 用 main.bat，Linux 和 macOS 用 main.sh。

## 测试

```bash
cd src/ground/bug2_wall_following
python -m pytest test/test_bug2_logic.py
```

覆盖：初始状态、无遮挡时朝目标移动、被挡进入沿墙、最终到达目标、几何工具。

## 实测结果

本地复跑：393 步抵达目标，终点距目标 12.18 px，途中触发沿墙跟随绕开障碍。

## 限制

- 纯 2D 几何演示，障碍统一为圆形。
- 沿墙间距按固定修正系数保持，不做实时避障规划。
