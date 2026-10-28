# bipedal_gait_zmp

双足步态生成与 ZMP 稳定性验证。把机器人简化成线性倒立摆（LIPM），由落脚点
计划给出 ZMP 参考，求开环周期解，再说明它为什么不稳定、怎样闭环稳住。

离线分析只用 Python 标准库；另含一个 rospy 节点，把仿真结果发布成话题，
可以在 rqt_plot / RViz 里看。

详细说明见站点文档：`docs/humanoid/bipedal_gait_zmp.md`

## 文件

```
main.py                 包根入口，转调 scripts/main.py
launch/main.launch      一键启动（离线演示与 ROS 节点，可单独关掉其一）
package.xml, CMakeLists.txt  纳入 catkin 工作空间

scripts/lipm.py         LIPM 动力学、ZMP 反算、捕获点、解析解
scripts/footstep.py     落脚点、支撑多边形、ZMP 参考、稳定裕度
scripts/gait.py         对称步态构造、周期解、开环仿真
scripts/dcm.py          发散分量与反馈控制
scripts/simple_plot.py  PNG 折线图（手写，只用 zlib）
scripts/gait_node.py    ROS 节点：发质心、ZMP、稳定裕度、落脚点
scripts/main.py         演示逻辑

tests/test_gait.py      73 个测试
```

## 运行（不需要 ROS）

在**包目录下**直接跑（不需要先 cd 进 scripts）：

```bash
python3 main.py            # 完整演示，图输出到 demo_out/
python3 main.py --quick    # 跳过增益扫描
python3 main.py --gain 1.5 # 指定反馈增益
```

`demo_out/` 在包根（与 launch 的默认输出目录一致），想换位置加 `--outdir DIR`。

## 运行（ROS）

```bash
catkin_make
source devel/setup.bash
roslaunch bipedal_gait_zmp main.launch
```

launch 起来两个节点：`bipedal_gait_demo` 跑离线演示并出图，`bipedal_gait_zmp`
把结果发成话题。只要话题不发图用 `demo:=false`，只要演示不发话题用 `node:=false`。

节点发布的话题：

- `/bipedal_gait/com_path` — `nav_msgs/Path`，整条质心轨迹（latched）
- `/bipedal_gait/zmp_path` — `nav_msgs/Path`，整条 ZMP 轨迹（latched）
- `/bipedal_gait/com_pose` — `geometry_msgs/PoseStamped`，当前质心
- `/bipedal_gait/zmp_pose` — `geometry_msgs/PoseStamped`，当前 ZMP
- `/bipedal_gait/margin` — `std_msgs/Float64`，当前稳定裕度
- `/bipedal_gait/footsteps` — `visualization_msgs/MarkerArray`，12 个落脚点
- `/bipedal_gait/support` — `visualization_msgs/Marker`，当前支撑区间

两个 Pose 与裕度是按 `playback_rate` 倍速回放时逐帧发的，默认 50 Hz。

验收：

```bash
rostopic list | grep bipedal_gait
rostopic hz /bipedal_gait/margin        # 约 50 Hz
rostopic echo -n1 /bipedal_gait/margin  # 稳定裕度一时一值
rqt_plot /bipedal_gait/margin/data      # 曲线无发散
```

节点参数（在 launch 里用 `<param>` 覆盖）：`step_length` 0.35、`step_time` 0.5、
`duty` 0.6、`n_steps` 12、`com_height` 0.8、`gain` 1.0、`playback_rate` 1.0、
`publish_rate` 50、`loop` true、`frame_id` map。

## 测试

```bash
python3 -m unittest discover -s tests
```

跑完应显示 `Ran 73 tests ... OK`。
