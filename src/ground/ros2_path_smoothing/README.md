# ROS 2 A* 全局路径平滑（Catmull-Rom）

对 A* 栅格路径做后处理：先用视线法简化，再用 Catmull-Rom 三次样条平滑，
在保持无碰撞和最小间隙的前提下降低最大曲率。独立模块，不依赖规划器。

## 1. 方法

1. **A\***：8 邻域、禁止斜穿障碍角点，得到栅格路径；
2. **视线简化**：从当前点尽量连到最远可见点，去掉栅格锯齿；
3. **Catmull-Rom**：按 0.5 m 重采样控制点后插值，端点保持不变；
4. **安全回退**：如果平滑点进入障碍或最小间隙 < 0.1 m，按 α=1→0.25 向原路径收缩；
   仍不满足则返回简化路径。

## 2. 运行

```bash
cd ~/hutb_ws
colcon build --packages-select ros2_path_smoothing
source install/setup.bash
ros2 launch ros2_path_smoothing main_launch.py
```

节点订阅 `nav_msgs/Path` 的 `/plan`，发布 `/plan_smoothed`；参数 `samples` 控制每段采样数。
节点只做几何平滑，不做地图碰撞检查（离线 `sim.py` 才检查间隙）。

## 3. 离线复现

```bash
cd src/ground/ros2_path_smoothing
python3 -m unittest discover -s test -v   # 5 项
python3 -m ros2_path_smoothing.sim
```

## 4. 结果

| 地图 | 原始长度 | 平滑长度 | 原始最大曲率 | 平滑最大曲率 | 平滑最小间隙 | 安全 |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| corridor | 19.28 m | 18.76 m | 5.685 1/m | **0.419 1/m** | 0.10 m | 是 |
| doorway | 21.19 m | 19.64 m | 5.685 1/m | **0 1/m**（直线通过门洞） | 0.50 m | 是 |
| clutter | 22.01 m | 20.49 m | 5.685 1/m | **0.949 1/m** | 0.10 m | 是 |

最大曲率下降 83%–100%，路径长度基本不变或略短；代价是平滑路径更贴近障碍
（corridor/clutter 最小间隙从 0.14/0.1 m 降到 0.10 m）。

![A* 与平滑路径对比](../../../docs/img/ground/ros2_path_smoothing/path_compare.png)

## 5. 局限

* 只处理几何路径，不重新规划；平滑后仍需下游控制器跟踪；
* 节点不做地图碰撞检查，只做样条；间隙检查在离线 `sim.py` 中；
* Catmull-Rom 在急弯处会切角，密集障碍场景需要更小的步长或额外约束；
* 2D 路径，不处理高度/坡度；
* 只支持静态地图，动态障碍需要重规划。

## 6. 大模型声明

代码与文档在编写过程中使用了 AI 大模型辅助。作者已实际运行 5 项单元测试和
3 张离线地图，并对提交内容的正确性与完整性负全部责任。
