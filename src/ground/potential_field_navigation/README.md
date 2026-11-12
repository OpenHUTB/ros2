# potential_field_navigation

人工势场法二维导航仿真。目标点对小车产生引力，圆形障碍产生随距离衰减的斥力，两者合成决定期望航向，小车按最大转向角速率跟踪该方向；遇到局部极小点时叠加切向偏置绕出。

包内提供两个入口：pygame 离线演示（`main.py`，可开窗口或离屏渲染），以及发布位姿与轨迹的 ROS 2 节点（`node.py`，不做渲染，无显示器也能运行）。

对应文档：docs/ground/potential_field_navigation.md

## 目录结构

```
potential_field_navigation/
├── main.py                     # 入口，转发到 potential_field_navigation.main
├── main.bat                    # Windows 启动脚本
├── main.sh                     # Linux 和 macOS 启动脚本
├── package.xml                 # 包声明，ament_python，声明 pygame/rclpy/launch_ros 等依赖
├── requirements.txt            # 依赖 pygame 和 pytest
├── setup.py                    # 打包元数据、launch 安装路径与两个 console_scripts
├── setup.cfg                   # 可执行脚本安装到 lib/potential_field_navigation
├── README.md
├── resource/potential_field_navigation
├── launch/main.launch.py       # 启动 true 节点 potential_field_node，透传 sim_rate / max_steps
├── potential_field_navigation/
│   ├── __init__.py
│   ├── config.py               # 画布、障碍、目标点、势场参数与运行上限
│   ├── field.py                # 引力、斥力合成与期望航向
│   ├── robot.py                # 小车运动学、间隙统计与逃逸状态机
│   ├── node.py                 # ROS 2 节点：发布位姿与轨迹
│   └── main.py                 # pygame 渲染与主循环
└── test/test_potential_field.py
```

## 环境

- 复跑环境：Windows 11，Python 3.13.16，pygame 2.6.1，pytest 9.2.0
- 安装依赖：

```bash
pip install -r requirements.txt
```

## 运行

pygame 演示（需要显示器）：

```bash
cd src/ground/potential_field_navigation
python main.py
```

Windows 用 main.bat，Linux 和 macOS 用 main.sh。

离屏运行（无显示器，例如 CI、SSH、容器），只推进固定帧数并打印指标：

```bash
python main.py --headless --frames 400
```

重新生成文档截图（写入仓库 `docs/img/`）：

```bash
python main.py --shots
```

ROS 2 节点（发布 `/potential_field/pose` 与 `/potential_field/trail`，不加载 pygame、不建窗口）：

```bash
colcon build --packages-select potential_field_navigation
source install/setup.bash
ros2 launch potential_field_navigation main.launch.py sim_rate:=60.0
```

中文字体：HUD 按候选列表取字体（SimHei → Noto Sans CJK SC → WenQuanYi Micro Hei → PingFang SC → Microsoft YaHei → 默认），避免只在 Windows 装了 SimHei 时 Linux/macOS 渲染成缺字方块。

## 测试

```bash
cd src/ground/potential_field_navigation
python -m pytest test/test_potential_field.py -q
```

10 项：引力方向、斥力在影响半径外为零、斥力随距离变近而增大、斥力背离障碍指向、绕障抵达目标、全程保持正间隙、障碍对称挡路时的局部极小点逃逸、`k_att` 覆盖不触发 TypeError、轨迹与历史长度有上限、超过最大步数标记 timeout。

## 实测结果

本地复跑：小车推进 400 步，第 401 次 `step()` 判定到达目标，终点距目标 5.85 px（到达容差 8.0 px），全程最小障碍间隙 33.18 px。本次场景未触发逃逸，局部极小点逃逸由对称障碍单测覆盖。

尚未验证：本机无 ROS 2 环境，未执行 `colcon build --packages-select potential_field_navigation`，需要评审在 Ubuntu 22.04 + Humble 上确认安装路径与节点启动。

## 限制

- 纯二维运动学演示，没有真实激光雷达与代价地图，障碍是已知圆形。
- 斥力只在影响半径 55 像素内生效，参数偏保守，绕障时与障碍表面保留约 33 像素间隙，路径不是最优。
- 局部极小点靠固定时长的切向偏置逃出，遇到复杂凹形区域不保证成功；失败时由 `MAX_STEPS` 兜底并标记 `timeout`。
- 节点只发布位姿与轨迹，未发布 `tf`、不含控制器。

## 声明

过程中借助大模型工具辅助代码编写、整理包目录结构与本描述排版，核心逻辑与操作、场景参数设定与实际运行调试为本人完成，作者对提交内容负责。
