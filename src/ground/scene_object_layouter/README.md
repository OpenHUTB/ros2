# scene_object_layouter

把场景里的物体（车位、立柱、隔离墩……）按栅格摆到一块矩形场地上，并当场校验四条：
**边界内 / 不重叠 / 间距够 / 数量对**。输出是终端报告 + 字符俯视图 + CSV + JSON。

不依赖 CARLA、不依赖 numpy、不依赖 PyYAML —— 只订阅/发布标准消息，纯标准库计算。

## 这个包解决什么问题

原页面的做法是「在 CARLA 里用 Python API 一个个 `spawn_actor`」，坐标靠飞镜头手抄。
这在没有 GPU / 没装 CARLA 的机器上完全跑不起来，而且摆错了（车位压到墙上、两辆车穿模）
只有肉眼看画面才能发现。

这个包把「摆哪些、摆哪儿」抽成一份 yaml，把「摆得对不对」变成四条能自动判定的断言。
放在 CI 或本机跑一遍，就知道场景描述有没有写错；确认无误后再交给 CARLA 脚本去真正生成。

## 目录结构

```text
src/ground/scene_object_layouter/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── layouter.yaml                 # 节点默认参数
│   └── scenes/
│       ├── demo_parking_row.yaml     # 正例：2 排 × 6 个车位 + 3 个立柱
│       └── dense_overlap.yaml        # 反例：故意摆出界、间距不足
├── launch/
│   └── main.launch                   # 13 个带 doc 的 arg
└── scripts/
    ├── main.py                       # 入口（仓库约定 main. 开头）
    ├── layouter_core.py              # 纯逻辑：几何 + 校验 + 文本输出（不 import rospy）
    └── layouter_node.py              # ROS 收发放这里
```

## 场景文件格式

```yaml
name: demo_parking_row
field:
  width: 24.0        # 场地宽（x 方向，米）
  length: 18.0       # 场地长（y 方向，米）
min_gap: 0.5         # 任意两个物体之间至少留多少米

objects:
  - kind: slot       # 组名，也是物体类型
    width: 2.5       # 单个物体宽
    length: 5.3      # 单个物体长
    rows: 2          # x 方向摆几排
    cols: 6          # y 方向摆几列
    gap_x: 0.6       # 列间距
    gap_y: 6.0       # 行间距
    origin_x: 3.0    # 第一个物体的中心 x
    origin_y: 3.0    # 第一个物体的中心 y
    yaw: 0.0         # 朝向（弧度）
    blueprint: static.prop.parkingline
```

摆法是严格的栅格：第 `(row, col)` 个物体中心在

```
x = origin_x + col * (width + gap_x)
y = origin_y + row * (length + gap_y)
```

## 四条断言

| 断言 | 判定方式 | 失败意味着 |
|---|---|---|
| 边界内 | 每个矩形的四个角都在 `[0, width] × [0, length]` 内 | 物体出界，在 CARLA 里会生成到墙里/地下 |
| 不重叠 | 两两做分离轴定理（SAT）判交 | 物体穿模 |
| 最小间距 | 两两求多边形最短距离，取全局最小值 | 车挤太紧，开不出来 |
| 物体数量 | 实际摆放数 == `rows × cols` 之和 | 场景文件写错，漏了组 |

## 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `scene/file` | 空 | 场景 yaml 路径；留空用内置演示场景 |
| `field/width` / `field/length` | 24.0 / 18.0 | 场地尺寸，优先级高于场景文件里的值 |
| `check/min_gap` | 0.5 | 最小间距阈值（米） |
| `render/width` / `render/height` | 60 / 24 | 字符俯视图分辨率 |
| `io/csv_path` | `~/scene_objects.csv` | 物体清单输出路径 |
| `io/json_path` | `~/scene_objects.json` | 汇总输出路径 |
| `io/write_files` | true | 设 false 则只在终端和话题里输出 |
| `rate/publish_hz` | 1.0 | 汇总话题重发频率 |
| `topic/objects` | `/scene/objects` | 物体清单（`std_msgs/String`，CSV 内容） |
| `topic/summary` | `/scene/summary` | 汇总（`std_msgs/String`，JSON 内容） |
| `topic/trigger` | `/scene/layout` | 收到 `std_msgs/Empty` 就重新计算 |

## 运行

```bash
cd ~/catkin_ws6
source /opt/ros/melodic/setup.bash
source devel/setup.bash

# 1. 用内置演示场景（不做任何配置就能跑）
roslaunch scene_object_layouter main.launch

# 2. 用配置文件里的正例
roslaunch scene_object_layouter main.launch \
  scene_file:=$(rospack find scene_object_layouter)/config/scenes/demo_parking_row.yaml

# 3. 用反例，看断言怎么抓错误
roslaunch scene_object_layouter main.launch \
  scene_file:=$(rospack find scene_object_layouter)/config/scenes/dense_overlap.yaml

# 4. 改场地尺寸与阈值（验证参数真的生效）
roslaunch scene_object_layouter main.launch field_width:=12.0 field_length:=10.0 min_gap:=1.5

# 5. 改完场景文件不重启也能重算
rostopic pub -1 /scene/layout std_msgs/Empty
```

预期输出（正例）：

```
断言：4 通过 / 0 失败（共 4 条）
  [PASS] 边界内    全部 15 个物体都在 24.0 x 18.0 的场地内
  [PASS] 不重叠    15 个物体两两不重叠
  [PASS] 最小间距   最小间距 0.600 m，要求 >= 0.500 m
  [PASS] 物体数量   摆放 15 个，期望 15 个
结论：PASSED
```

反例则是 `2 通过 / 2 失败`，`边界内` 与 `最小间距` 两条标 `[FAIL]`，`结论：FAILED`。

## 离线测试

不需要 ROS、不需要 CARLA、不需要显示器：

```bash
cd ~/catkin_ws6
python3 tests/test_scene_object_layouter_local.py
```

期望输出 `通过 N 项，失败 0 项` 与 `全部通过`。测试覆盖：场景解析与报错、栅格摆放坐标、
矩形角点与旋转、SAT 判交与多边形间距、四条断言的正反例、字符俯视图尺寸、CSV/JSON 内容、
参数读取、以及 `package.xml` / `CMakeLists.txt` / `main.launch` 的结构约定。

## 与 CARLA 的分工

| 环节 | 谁负责 | 说明 |
| --- | --- | --- |
| 摆哪些物体、摆哪儿 | **本包** | yaml → 坐标清单，可离线校验 |
| 摆得对不对（出界/穿模/太挤） | **本包** | 四条断言，失败直接 `FAILED` |
| 真正在仿真里生成出来 | CARLA 脚本 | 读本包输出的 CSV/JSON，逐个 `spawn_actor` |

## 常见问题

**Q：为什么不用 PyYAML？**
Melodic 上装 PyYAML 要走 pip，而仓库里绝大多数纯 Python 模块都没这个依赖。
本包只解析自己用到的那一小撮语法（缩进映射、`- ` 列表、行内 `#` 注释），
100 行左右就够了，也避免「装的版本不同导致行为不同」。

**Q：为什么不用 numpy 算几何？**
一个场景最多几十个矩形，两两比较是 O(n²)，纯 Python 毫秒级算完。省掉一个依赖更划算。

**Q：`origin_x/origin_y` 是物体中心还是角点？**
是**中心**。这样 `gap_x` 就是两个物体边缘之间的净空隙，和直觉一致。

**Q：Melodic 上日志会不会报 `UnicodeEncodeError`？**
不会。节点里所有日志都经过 `_log()`，先在 Python 2 下把 unicode 编成 utf-8 bytes
再交给 rospy —— 直接传 unicode 或 `loginfo(fmt, *args)` 会被 rosgraph 的格式化炸掉。
