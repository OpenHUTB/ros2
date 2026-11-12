# potential_field_navigation

包定位说明：本模块是基于 pygame 的独立 2D 仿真演示，不创建 rclpy 节点，也不发布 ROS 话题。

人工势场法二维导航仿真。目标点对小车产生引力，圆形障碍产生随距离衰减的斥力，两者合成决定期望航向，小车按最大转向角速率跟踪该方向；遇到局部极小点时叠加切向偏置绕出。

对应文档：docs/ground/potential_field_navigation.md

## 目录结构

```
potential_field_navigation/
├── main.py                     # 入口，转发到 potential_field_navigation.main
├── main.bat                    # Windows 启动脚本
├── main.sh                     # Linux 和 macOS 启动脚本
├── package.xml                 # 包声明，ament_python
├── requirements.txt            # 依赖 pygame 和 pytest
├── setup.py                    # 打包与安装路径
├── README.md
├── resource/potential_field_navigation
├── launch/main.launch.py       # ROS 2 launch 占位入口
├── potential_field_navigation/
│   ├── __init__.py
│   ├── config.py               # 画布、障碍、目标点与势场参数
│   ├── field.py                # 引力、斥力合成与期望航向
│   ├── robot.py                # 小车运动学、间隙统计与逃逸状态机
│   └── main.py                 # pygame 渲染与主循环
└── test/test_potential_field.py
```

## 环境

- Windows 11，Python 3.13.16，pygame 2.6.1
- 安装依赖：

```bash
pip install -r requirements.txt
```

## 运行

```bash
cd src/ground/potential_field_navigation
python main.py
```

Windows 用 main.bat，Linux 和 macOS 用 main.sh。

## 测试

```bash
cd src/ground/potential_field_navigation
python -m pytest test/test_potential_field.py
```

覆盖：引力方向、斥力在影响半径外为零、斥力随距离变近而增大、斥力背离障碍指向、绕障抵达目标、全程保持正间隙，以及障碍对称挡路时的局部极小点逃逸。

## 实测结果

本地复跑：小车 401 步抵达目标，终点距目标 5.85 px（到达容差 8.0 px），全程最小障碍间隙 33.18 px。本次场景未触发逃逸，局部极小点逃逸由对称障碍单测覆盖。

## 限制

- 纯二维运动学演示，没有真实激光雷达与代价地图，障碍是已知圆形。
- 斥力只在影响半径 55 像素内生效，参数偏保守，绕障时与障碍表面保留约 33 像素间隙，路径不是最优。
- 局部极小点靠固定时长的切向偏置逃出，遇到复杂凹形区域不保证成功。
