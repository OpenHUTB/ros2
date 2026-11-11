# 生成对象

启动方式：
```shell
roslaunch carla_spawn_objects carla_spawn_objects.launch spawn_sensors_only:=True host:=172.21.108.47 timeout:=60000 town:='Town03' spawn_point:=-25,-134,0.5,0,0,-90
```


## 参考

* [Carla 生成对象](https://openhutb.github.io/doc/carla_spawn_objects/)

## 自主实现：不依赖 CARLA 的场景物体布置校验器

### 为什么另做一个

原做法是「在 CARLA 里用 Python API 一个个 spawn_actor」，坐标靠飞镜头手抄。它有三个绕不开的限制：

1. **必须有 CARLA + GPU**：没有仿真器的机器（比如只有 ROS 的虚拟机）根本没法验证
2. **坐标靠手抄**：改成 6 个车位、每个间距 0.6 m 这种整齐的排布，手算容易错
3. **摆错了只有肉眼看画面才能发现**：车位压墙、车挤在一起、少摆一个，画面小的时候看不出来

`scene_object_layouter` 把这两件事拆开：**摆放**由一份 yaml 描述、**校验**由四条断言自动判定。
确认场景描述没问题之后，再交给 CARLA 脚本去真正生成。

### 场景文件长什么样

```yaml
name: demo_parking_row
field:
  width: 24.0        # 场地宽（x 方向，米）
  length: 18.0       # 场地长（y 方向，米）
min_gap: 0.5         # 任意两个物体之间至少留多少米

objects:
  - kind: slot
    width: 2.5
    length: 5.3
    rows: 2
    cols: 6
    gap_x: 0.6
    gap_y: 6.0
    origin_x: 3.0
    origin_y: 3.0
    blueprint: static.prop.parkingline
  - kind: pillar
    width: 1.0
    length: 1.0
    rows: 1
    cols: 3
    gap_x: 4.0
    origin_x: 6.0
    origin_y: 8.65
    blueprint: static.prop.pillar
```

摆法是严格栅格：第 `(row, col)` 个物体的中心在

```
x = origin_x + col * (width + gap_x)
y = origin_y + row * (length + gap_y)
```

### 四条断言

| 断言 | 判定方式 | 失败意味着 |
| --- | --- | --- |
| 边界内 | 每个矩形的四个角都在 `[0, width] × [0, length]` 内 | 物体出界，在 CARLA 里会生成到墙里或地下 |
| 不重叠 | 两两做分离轴定理（SAT）判交 | 物体穿模 |
| 最小间距 | 两两求多边形最短距离取全局最小值 | 车挤太紧，开不出来 |
| 物体数量 | 实际摆放数 == 各组 `rows × cols` 之和 | 场景文件写错，漏摆一组 |

### 快速运行

```bash
cd ~/catkin_ws6
source /opt/ros/melodic/setup.bash
source devel/setup.bash

# 正例：2 排 × 6 个车位 + 3 个立柱
roslaunch scene_object_layouter main.launch \
  scene_file:=$(rospack find scene_object_layouter)/config/scenes/demo_parking_row.yaml

# 反例：故意摆出界、间距不足
roslaunch scene_object_layouter main.launch \
  scene_file:=$(rospack find scene_object_layouter)/config/scenes/dense_overlap.yaml
```

正例的终端输出（节选）：

```
场地：24.00 m × 18.00 m，物体组 2 个，物体总数 15 个
包围盒：x ∈ [1.75, 19.75]，y ∈ [0.35, 16.95]
最小间距：0.600 m
断言：4 通过 / 0 失败（共 4 条）
  [PASS] 边界内    全部 15 个物体都在 24.0 x 18.0 的场地内
  [PASS] 不重叠    15 个物体两两不重叠
  [PASS] 最小间距   最小间距 0.600 m，要求 >= 0.500 m
  [PASS] 物体数量   摆放 15 个，期望 15 个
结论：PASSED
```

报告末尾还会打印一张字符俯视图，`#` 是车位、`@` 是立柱，能直接看出排布是否符合预期。

反例会变成 `2 通过 / 2 失败`：`边界内` 报 `6/9 个物体超出场地`，`最小间距` 报
`最小间距 0.050 m，要求 >= 0.500 m`，`结论：FAILED`。

### 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `scene/file` | 空 | 场景 yaml 路径，留空则用内置演示场景 |
| `field/width` / `field/length` | 24.0 / 18.0 | 场地尺寸，优先级高于场景文件 |
| `check/min_gap` | 0.5 | 最小间距阈值（米） |
| `render/width` / `render/height` | 60 / 24 | 字符俯视图分辨率 |
| `io/csv_path` | `~/scene_objects.csv` | 物体清单输出路径 |
| `io/json_path` | `~/scene_objects.json` | 汇总输出路径 |
| `io/write_files` | true | 设 false 则只输出到终端与话题 |
| `rate/publish_hz` | 1.0 | 汇总话题重发频率 |
| `topic/objects` | `/scene/objects` | 物体清单（`std_msgs/String`，CSV 内容） |
| `topic/summary` | `/scene/summary` | 汇总（`std_msgs/String`，JSON 内容） |
| `topic/trigger` | `/scene/layout` | 收到 `std_msgs/Empty` 就重算 |

命令行覆盖示例（改场地与阈值）：

```bash
roslaunch scene_object_layouter main.launch field_width:=12.0 field_length:=10.0 min_gap:=1.5
```

改完场景文件不必重启，发一条空消息就会重算：

```bash
rostopic pub -1 /scene/layout std_msgs/Empty
```

### 和 CARLA 生成脚本怎么衔接

本包导出的 `scene_objects.csv` 每行一个物体：

```
index,group,kind,x,y,yaw,width,length,blueprint
0,slot,slot,3.000,3.000,0.000,2.500,5.300,static.prop.parkingline
```

CARLA 脚本读这份清单逐个 `spawn_actor` 即可，不用再手抄坐标。
校验通过（`PASSED`）是「可以进仿真生成」的前置条件。

### 离线测试

不需要 ROS、不需要 CARLA、不需要显示器：

```bash
cd ~/catkin_ws6
python3 tests/test_scene_object_layouter_local.py
```

覆盖：场景解析与报错、栅格坐标、矩形角点与旋转、SAT 判交与间距、四条断言的正反例、
字符俯视图、CSV/JSON 内容、参数读取、工程文件结构约定。

### 实现要点

- **纯标准库**：不 import `rospy`（核心逻辑）、不 import `numpy`、不 import `yaml` —— 自带一个
  只支持缩进映射/列表/行内注释的 YAML 子集解析器，避免 Melodic 上装依赖
- **单文件核心**：`layouter_core.py` 里所有函数都是纯函数，测试与节点共用同一套计算
- **Py2/Py3 双兼容**：`from __future__ import division, print_function, unicode_literals`，
  字符串格式化用 `%`，文件读取用 `io.open(..., encoding="utf-8")`
- **Melodic 日志安全**：节点里所有日志经 `_log()` 转成 byte str 再交给 rospy，
  绕开 rosgraph 对非 ASCII unicode 调 `str()` 的 `UnicodeEncodeError`

### 相关文件

- 源码：`src/ground/scene_object_layouter/`
- 说明：`src/ground/scene_object_layouter/README.md`
- 测试：`tests/test_scene_object_layouter_local.py`
- 原 CARLA 参考：[Carla 生成对象](https://openhutb.github.io/doc/carla_spawn_objects/)

所使用具体的大模型声明：使用大模型辅助代码编写、测试编写与文档整理，提交内容由本人负责。
