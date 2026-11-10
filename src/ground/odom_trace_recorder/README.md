# 里程计轨迹记录器（odom_trace_recorder）

把「车开过的这一段」记录下来：订阅 `/odom` 边收边累积采样点，到点写成 CSV，
再回放成 `nav_msgs/Path`，并与一条参考圆弧逐点比对，输出误差直方图与四条断言。

不依赖 CARLA、不需要显示器、不需要任何 pip 包。

## 目录结构

```
odom_trace_recorder/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/trace_recorder.yaml      # 13 个默认参数
├── launch/main.launch              # 13 个带 doc 的 arg
└── scripts/
    ├── main.py                     # 入口
    ├── trace_core.py               # 纯逻辑（不 import rospy）
    └── trace_node.py               # ROS 收发
```

## 快速运行

```bash
catkin_make
source devel/setup.bash
roslaunch odom_trace_recorder main.launch
```

节点在 `record/seconds` 秒后自动结算并退出，不需要手动 `Ctrl+C`。

## 两个反例

```bash
# 让「误差在限」失败：参考半径与实际行驶半径不一致
roslaunch odom_trace_recorder main.launch reference_radius:=4.0

# 让「采样完整」失败：要求 500 个采样点，而默认只有 161 个
roslaunch odom_trace_recorder main.launch min_samples:=500
```

## 离线测试

```bash
python3 tests/test_odom_trace_recorder_local.py
```

期望输出 `通过 N 项，失败 0 项` 与 `全部通过`。测试同样兼容 Python 2.7（Melodic）
与 Python 3.x（Noetic），并且可以用 `python3 -m pytest tests/test_odom_trace_recorder_local.py -q`
运行——失败时包装函数会抛 `AssertionError`，不会出现「假绿」。

## 实现要点

- **记录**：`_on_odom` 把每帧位姿转成 `TracePoint(t, x, y, yaw)`，时间戳用
  `rospy.get_time()` 相对启动时刻。
- **落盘**：结算时写 `t,x,y,yaw` 四列 CSV（4 位小数），`parse_trace_text` 能原样读回。
- **回放**：把记录点逐个转成 `PoseStamped` 装进 `nav_msgs/Path`，latched 发布。
- **比对**：参考轨迹用解析公式 `x = R sin θ`、`y = R (1 - cos θ)`；内置车辆用定步长
  积分驾驶，两者之差即离散化误差，所以默认误差不是 0 而是 0.0972 m。

## 参数

见 `config/trace_recorder.yaml` 与 `docs/ground/odom_trace_recorder.md` 的参数表。
