# figure8_sim — 8 字形轨迹可视化（ROS 1）

在 RViz 里让一个 `CUBE` 沿 8 字轨迹运动，用 `Marker` 发布位姿，用来验证 ROS 1 的
发布/订阅与 TF 之外的 Marker 可视化链路（无需 Gazebo 或真实车）。

## 轨迹

```
x(t) = 2·sin(t)
y(t) = 2·sin(t)·cos(t) = sin(2t)
theta(t) = atan2(dy/dt, dx/dt)
```

- `x` 的周期是 `2π`，`y` 的周期是 `π`，整条曲线关于原点对称；
- 航向取轨迹切向，用绕 z 轴的四元数写入 `Marker.pose.orientation`；
- 轨迹参数 `t` **按真实时间推进**（`rospy.Time.now() - start`），
  循环频率变化不再影响小车速度（原来每轮固定 `t += 0.03`）。

## 运行

```bash
# 把 src/figure8_sim 放进 ROS 1 工作空间
cd ~/catkin_ws && catkin_make && source devel/setup.bash
roslaunch figure8_sim figure8_sim.launch
```

RViz 里应看到 `/car_visual` 下的 `CUBE` 沿 8 字循环运动。

## 文件

```
figure8_sim/
├── scripts/
│   ├── figure8.py   # 纯函数：位置 / 速度 / 航向 / 四元数
│   └── main.py      # ROS 节点：按真实时间取轨迹点，发布 Marker
├── launch/figure8_sim.launch
├── rviz/rviz_config.rviz
└── test/            # 12 项单元测试（桩掉 rospy / visualization_msgs）
```

## 测试

`test/` 下是一个**纯标准库**的离线测试脚本，不需要 ROS、不需要 RViz、不需要 pytest
（脚本内部桩掉 `rospy` / `visualization_msgs`，只用于 import）：

```bash
cd src/figure8_sim
python3 test/test_figure8_sim_local.py
```

输出形如：

```text
== 速度 ==
  [PASS] 速度与中心差分一致（y 导数系数回归）
--------------------------------------------------
通过 16 / 失败 0
```

覆盖 16 项：

- 轨迹：t=0 在原点、t=pi/2 在最右点 (2, 0)、|x|<=2 且 |y|<=1、x 周期 2π、y 周期 π；
- 速度：与中心差分一致（曾抓到 y 导数多乘 2 的错误）；
- 航向：等于切向、处处有限且在 [-pi, pi]、四元数为绕 z 的单位四元数；
- Marker：frame_id/type/action、z 高度与尺寸、位置与朝向跟轨迹一致、位姿随时间变化。

脚本返回码 0/1，可直接接进 CI 或 `catkin_make run_tests` 之外的任何流水线。
