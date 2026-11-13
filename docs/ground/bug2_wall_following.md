# Bug2 反应式避障（圆形障碍 · pygame 2D 演示）

演示经典 Bug2 算法在 2D 平面上绕过圆形障碍到达目标。

## 与 reactive_navigation/bug2_navigation 的区别

仓库里有两个 Bug2 模块，实现的是同一条行为规则（直线朝目标、碰障沿墙、再次跨过
起点-目标连线 Lm 且更近则脱离），但**不是同一实现，也不是替代关系**：本页只讲
“圆形障碍 + pygame 实时轨迹”的演示，完备性与确定性不可达判据见
[`../reactive_navigation/bug2_navigation.md`](../reactive_navigation/bug2_navigation.md)
（已合并模块 `src/reactive_navigation/bug2_navigation`，合并记录 #1400）。

| 对比项 | 本页 `ground/bug2_wall_following` | `reactive_navigation/bug2_navigation`（#1400） |
|---|---|---|
| 障碍模型 | **单个圆形障碍** `(400, 300, 70)`，像素坐标；沿圆切线步进、沿半径方向修正保持 20 px 间距，圆上无角点 | 多块**轴对齐矩形**障碍，按安全间隙膨胀后对边界做**弧长参数化**，要处理角点与墙体接缝 |
| 场景与坐标 | 800×600 屏幕像素，起点 (80,500)，目标 (720,120) | 50 m×50 m 米制地图，起点 (2,2)，目标 (48,48)，另有窄门腔场景 |
| 状态机 | `GOAL_SEEKING / WALL_FOLLOWING / REACHED` 三态加 `hit_point`；回 Lm 用点到线段距离容差判定 | `MOVE / FOLLOW` 两态加 `hit / leave / blocked` 事件序列；另有“绕满一整圈回到 hit 点”与“撞到相邻墙体”两条终止判据 |
| 不可达处理 | 不做判定：目标被围死时靠步数上限退出，不宣告失败 | 有确定性判据：窄门腔 25.04 m 处返回 `blocked`（只对封闭腔成立的保守启发式） |
| 运行形态 | 纯 pygame 窗口实时渲染，不创建 rclpy 节点、不发布话题，launch 仅为占位入口 | ROS 2 闭环：`sim_node` + `plan_control_node`，`/odom` 进、`/cmd_vel` 与 `/planning/path` 出 |
| 构建方式 | `ament_python`（`setup.py`） | `ament_cmake`（`CMakeLists.txt`），与 APF、牛耕式覆盖共用 `../common` |
| 验证规模 | 6 项 pytest 离线逻辑，本机实跑全过 | 16 项（12 项算法 + 4 项 rclpy 接口桩），接入 GitHub Actions |
| 输出证据 | 实时动画：绿色起点、红色目标、灰色障碍、青色轨迹、橙色沿墙段 | 离线 PNG 配图、RViz 中的 Path、节点逐条 hit/leave/blocked 日志 |

选用建议：看“圆形障碍下沿墙几何怎么写、状态怎么切”用本页；要完备性结论、
确定性不可达或 ROS 2 话题闭环，用 `bug2_navigation` 页。

## 环境

- Windows 11，Python 3.13.16，pygame 2.6.1

## 运行

见包内 README（`src/ground/bug2_wall_following/README.md`）。

## 分层

- geometry.py：点到线段距离、线段相交、障碍表面最近点。
- bug2.py：状态机 GOAL_SEEKING / WALL_FOLLOWING / REACHED，记录 hit_point。
- main.py：pygame 可视化，绿色起点、红色目标、灰色障碍、青色轨迹、橙色沿墙段。

## 实测结果

本地复跑：393 步抵达目标，终点距目标 12.18 px，途中触发沿墙跟随。

## 限制

- 纯 2D 几何演示，障碍统一为单个圆形，不含矩形或多障碍场景。
- 无不可达判据：目标被围死或沿墙绕不出来时只靠步数上限退出，不返回失败结论（该能力见 `bug2_navigation`）。
- 沿墙间距按固定修正系数保持，不做实时避障规划。

## 声明

过程中借助大模型工具辅助代码编写、整理包目录结构与本描述排版，核心逻辑与操作、场景参数设定与实际运行调试为本人完成，作者对提交内容负责。
