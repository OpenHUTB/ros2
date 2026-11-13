# bug2_wall_following

包定位说明：本模块是基于 pygame 的独立 2D 仿真演示，不创建 rclpy 节点，也不发布 ROS 话题。

Bug2 反应式避障 2D 仿真。机器人从起点直线朝目标走，碰到圆形障碍时切换到沿墙跟随模式，绕障碍一周后再次穿过起点到目标连线 Lm、且该点比碰墙点更靠近目标时，切回直线朝目标，直到到达终点。

对应文档：docs/ground/bug2_wall_following.md

## 与 reactive_navigation/bug2_navigation 的区别

仓库里另有一个已合并的 Bug2 模块：`src/reactive_navigation/bug2_navigation`
（文档 `docs/reactive_navigation/bug2_navigation.md`，合并记录 #1400）。两者都实现
“直线朝目标—碰障沿墙—再次跨过 Lm 且更近则脱离”这三条规则，但不是同一实现，也
不是替代关系：本模块保留为**圆形障碍的 pygame 2D 演示**，`bug2_navigation` 保留
为其矩形障碍的 ROS 2 闭环版本，文档站两个入口分别讲“状态机可视化”与“完备性判据”。

| 对比项 | 本模块 `ground/bug2_wall_following` | 已合并的 `reactive_navigation/bug2_navigation`（#1400） |
|---|---|---|
| 定位与栏目 | 地面载具栏目的可视化演示，重点是状态机与轨迹 | “反应式导航与覆盖”主线，重点是“不建图也能给结论”的完备性与不可达判据 |
| 障碍模型 | **单个圆形障碍** `(400, 300, 70)`（像素坐标）；沿圆切线步进、沿半径方向修正保持 20 px 间距，圆上没有角点，沿墙一步到位 | 多块**轴对齐矩形**障碍（先按安全间隙膨胀）；边界**弧长参数化**沿墙，要处理角点与墙体接缝 |
| 场景坐标 | 800×600 屏幕像素，起点 (80,500)、目标 (720,120) | 50 m×50 m 米制地图，起点 (2,2)、目标 (48,48)，另有门宽 2.0 m 的窄门腔场景 |
| 状态机 | `GOAL_SEEKING / WALL_FOLLOWING / REACHED` 三态加 `hit_point`；回 Lm 用“点到线段距离 ≤ 容差”判定 | `MOVE / FOLLOW` 两态加 `hit / leave / blocked` 事件序列；另有“绕满一整圈回到 hit 点”与“沿墙撞到相邻墙体”两条终止判据 |
| 不可达处理 | 不做判定：只演示能绕开的圆形障碍，目标被围死时靠步数上限退出 | 有确定性判据：窄门腔在 25.04 m 处返回 `blocked`，是只对封闭腔成立的保守启发式（适用范围见其文档第 1 节） |
| 运行形态 | 纯 pygame 窗口实时渲染，不创建 rclpy 节点、不发布话题；`launch/main.launch.py` 仅为占位入口 | ROS 2 闭环：`sim_node` + `plan_control_node`，`/odom` 进、`/cmd_vel` 与 `/planning/path` 出，也能纯离线出图 |
| 构建方式 | `ament_python`（`setup.py`） | `ament_cmake`（`CMakeLists.txt`），与 APF、牛耕式覆盖共用 `../common` |
| 验证规模 | 6 项 pytest 离线逻辑：初始态、无遮挡直行、受阻转沿墙、4000 步内到达、几何 2 项 | 16 项：12 项算法加 4 项 rclpy 接口桩，并接入 `.github/workflows/reactive_navigation_tests.yml` |
| 输出证据 | 实时动画窗口：绿色起点、红色目标、灰色障碍、青色轨迹、橙色沿墙段 | 离线 PNG 配图、RViz 中的 Path、节点逐条 hit/leave/blocked 日志 |

选用建议：要验证“沿圆形障碍的几何沿墙与状态切换”并直接看到轨迹，用本模块；
要验证完备性结论、确定性不可达，或要接 ROS 2 话题与 RViz，用
`reactive_navigation/bug2_navigation`。两者的三条行为规则同源，障碍模型、
终止判据与验证目标不同。

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
