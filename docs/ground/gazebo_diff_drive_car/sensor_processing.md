# 传感器数据处理

本章详细说明两类传感器数据的处理流程，重点在于**如何从原始数据中
提取可靠的感知信息**。

---

## 7.1 激光雷达数据处理

### 7.1.1 原始数据结构

`sensor_msgs/LaserScan` 消息：

```
header:
  seq: 406
  stamp: {secs: 41, nsecs: 19000000}
  frame_id: "laser_link"
angle_min:       -3.141590118408203      ← 扫描起始角 (rad)
angle_max:        3.141590118408203      ← 扫描结束角 (rad)
angle_increment:  0.017501894384622574   ← 角分辨率 (rad)
time_increment:   0.0
scan_time:        0.0
range_min:        0.11999999731779099    ← 最小量程 (m)
range_max:        5.599999904632568      ← 最大量程 (m)
ranges:          "<array: float32, length 360>"   ← 距离数组
intensities:     "<array: float32, length 360>"
```

**角度计算**：第 $i$ 束激光的角度为

$$
\theta_i = \theta_{\min} + i \cdot \Delta\theta
$$

其中 $\Delta\theta = 0.0175$ rad ≈ 1°。

**角度规范化**：`angle_min = -π`、`angle_max = +π`，
但数据可能因数值精度略微超出，需规范化到 $[-\pi, \pi)$：

```python
while theta >= math.pi:
    theta -= 2.0 * math.pi
while theta < -math.pi:
    theta += 2.0 * math.pi
```

### 7.1.2 无效值类型

实测激光数据中存在三类无效值：

| 值 | 出现场景 | 占比（实测） |
|---|---|---|
| `inf` | 该方向超出量程（> 5.6 m） | 约 30% |
| `nan` | 传感器异常 | 罕见 |
| $< 0.12$ m | 小于最小量程，噪声 | 罕见 |

**实测数据示例**（前方空旷时的一帧）：

```
ranges: [0.358, 0.364, 0.365, 0.384, 0.371, ..., 
         4.248, 4.054, 3.868, 3.859, 3.925, 
         inf, inf, inf, inf, inf, inf, inf, inf, ...
         2.886, 2.815, 2.773, ...
         0.120, 0.120, 0.128, ...]
```

可以看到：

- 数值 `inf` 连续出现（看向场地外）
- 数值 `0.120` 恰好等于 `range_min`（贴脸障碍或噪声）

### 7.1.3 处理流程

```
  原始 ranges[360]
        │
        ▼
  ┌─────────────────────┐
  │ ① 类型过滤           │  剔除 nan / inf / < range_min
  └──────────┬──────────┘
             ▼
  ┌─────────────────────┐
  │ ② 角度计算与规范化    │  θ_i = θ_min + i·Δθ → [-π, π)
  └──────────┬──────────┘
             ▼
  ┌─────────────────────┐
  │ ③ 自遮挡过滤 ★       │  邻域连续性校验
  └──────────┬──────────┘
             ▼
  ┌─────────────────────┐
  │ ④ 扇区划分           │  5 个扇区各取最小距离
  └──────────┬──────────┘
             ▼
     {front, left, right, left_bk, right_bk}
```

### 7.1.4 自遮挡过滤详解

**问题现象**：

车辆处于**空旷位置**（正前方 10 m 内无障碍）时，
激光数据仍显示前方有 0.12 ~ 0.40 m 的回波：

```
[DEBUG] 前方扇区最小距离 = 0.358 m    ← 虚假回波！
```

导致算法误判"前方被堵死"，车辆原地打转。

**根本原因**：

激光雷达虽安装在车体前方 0.28 m 处（略超出车体边界 0.22 m），
但激光束以 360° 放射，**部分光束仍会扫到车体自身的边缘、
天线或传感器安装支架**。

```
                    俯视图

              ╱  ╱  ╱  │  ╲  ╲  ╲
            ╱  ╱  ╱    │    ╲  ╲  ╲      ← 激光束 360° 放射
          ╱  ╱  ╱      │      ╲  ╲  ╲
        ╱  ╱  ╱        │        ╲  ╲  ╲
      ╱  ╱  ╱          │          ╲  ╲  ╲
    ╱  ╱  ╱            │            ╲  ╲  ╲
   ─────────────────── ● ─────────────────────
                     雷达
                    ┌──┴──┐
                    │     │
                    │ 车体 │  ← 向后发射的激光扫到车体！
                    │     │
                    └─────┘
```

**判别依据**：

| 特征 | 真实障碍 | 自遮挡 |
|---|---|---|
| 遮挡光束数 | 连续多束（≥ 5） | 孤立少数束（1~2） |
| 稳定性 | 随车辆移动变化 | 固定在特定角度 |
| 距离值 | 有梯度变化 | 恒定 |

**实现代码**：

```python
# 统计邻域内同样近的回波数量
hits = 0
for off in range(-MIN_CLOSE_HITS, MIN_CLOSE_HITS + 1):
    j = idx + off
    if 0 <= j < n and valid_beams[j][1] < SELF_OCCLUSION_RANGE:
        hits += 1

if hits >= MIN_CLOSE_HITS:
    filtered.append((theta, r))    # 连续多束都近 → 真实障碍
# 否则丢弃（自遮挡噪声）
```

**参数**：

| 参数 | 值 | 说明 |
|---|---|---|
| `SELF_OCCLUSION_RANGE` | 0.30 m | 判定"过近"的距离阈值 |
| `MIN_CLOSE_HITS` | 5 | 判定为真实障碍的最少连续光束数 |

**验证效果**：

| 场景 | 过滤前 | 过滤后 |
|---|---|---|
| 空旷区正前方 | 0.358 m（假） | 4.25 m（真） |
| 距障碍物 0.5 m | 0.500 m | 0.500 m（保留 ✓） |

### 7.1.5 扇区划分实现

```python
sectors = {
    "front":    (-math.radians(30.0),  math.radians(30.0)),
    "left":     (math.radians(30.0),   math.radians(90.0)),
    "right":    (-math.radians(90.0),  -math.radians(30.0)),
    "left_bk":  (math.radians(90.0),   math.pi),
    "right_bk": (-math.pi,             -math.radians(90.0)),
}

result = {k: range_max for k in sectors}

for theta, r in filtered:
    for name, (lo, hi) in sectors.items():
        if lo <= theta < hi:
            if r < result[name]:
                result[name] = r        # 取最小值
            break
```

**默认值为 `range_max`**：若某扇区全无有效回波，则该方向视为
"极远/完全通畅"，而非"未知"。这个设计选择是安全的 ——
空扇区通常意味着看向场地外。

---

## 7.2 深度相机数据处理

### 7.2.1 原始数据结构

`sensor_msgs/Image` 消息（深度图）：

```
header:
  seq: 120
  stamp: {secs: 45, nsecs: 320000000}
  frame_id: "camera_optical_link"
height: 480
width: 640
encoding: "32FC1"              ← 32 位浮点，单通道
is_bigendian: 0
step: 2560                     ← 640 × 4 字节
data: "<array: uint8, length 1228800>"
```

**编码类型处理**：

| 编码 | 数据类型 | 单位 | 转换 |
|---|---|---|---|
| `32FC1` | float32 | 米 | 直接使用 |
| `16UC1` | uint16 | 毫米 | × 0.001 |

```python
if msg.encoding == "32FC1":
    dtype = np.dtype(np.float32)
    scale = 1.0
elif msg.encoding == "16UC1":
    dtype = np.dtype(np.uint16)
    scale = 0.001

img = np.frombuffer(msg.data, dtype=dtype)
img = img.reshape((msg.height, msg.width)).astype(np.float32)
if scale != 1.0:
    img = img * scale
```

### 7.2.2 深度地板值问题（核心难点）

**问题现象**：

深度图中出现一个**固定常数区域**，实测为 **1.005 m**：

```
中区 ROI 深度统计：
  min    = 1.000 m
  p15    = 1.000 m
  p40    = 1.000 m
  median = 1.000 m      ← 几乎所有统计量都被锁定
  max    = 8.762 m      ← 但存在真实测距！
```

**根因分析**：

Gazebo 深度相机对**无回波区域**（看向场景外、超出量程、
或激光/结构光无法返回的部分）并非输出 `0` 或 `inf`，
而是输出一个固定的哨兵值。

**为什么这个值恰好是 1.0？**

这与 Gazebo 深度相机的实现有关：无回波像素的深度被设为
`<clip><near>` 附近的一个默认值。由于我们在 URDF 中设置
`<near>0.05</near>`，理论上应该是 0.05，但实测为 1.005。

> 无论该值的来源如何，**关键是识别并剔除它**，
> 而不是依赖某个固定数值做硬编码。

**影响程度**：

| 区域 | 地板值占比 | 后果 |
|---|---|---|
| 左区 | 7%（未检出） | 正常 |
| **中区** | **66%** | 统计量被完全锁定 |
| **右区** | **63%** | 统计量被完全锁定 |

**中区 66% 的像素是假的！** 这意味着：

- 无论取 p15、p25 还是 p40，结果都是 1.000
- 算法恒判定"前方 1 m 有障碍"（$< d_{\text{safe}} = 1.2$ m）
- 车辆进入避障状态，原地打转，**永远无法前进**

实测表现：30 秒仅移动 0.08 m。

### 7.2.3 地板值检测算法

**核心思想**：地板值区域的特征是**像素数占比极高**且**值高度集中**。
用直方图统计即可捕获：

```
      像素数
        │
        │  █                          ← 地板值峰值（占比 66%）
        │  █
        │  █
        │  █        ▄
        │  █    ▄   █   ▄
        │  █  ▄ █ ▄ █ ▄ █  ▄
        └──█──█─█─█─█─█─█──█───────▶ 深度 (m)
          1.0                     8.8
          ↑
       检测到峰值占比 > 30% → 判定为地板值
```

**实现**：

```python
@staticmethod
def _detect_floor_value(values):
    """检测深度图中的固定地板值（无效标记）。"""
    if values.size < 100:
        return None                       # 样本太少，不判断

    vmin, vmax = float(values.min()), float(values.max())
    if vmax - vmin < FLOOR_BIN_WIDTH:
        return None                       # 值域过窄，无区分度

    n_bins = int((vmax - vmin) / FLOOR_BIN_WIDTH) + 1
    n_bins = min(n_bins, 4096)            # 限制分箱，控制内存

    hist, edges = np.histogram(values, bins=n_bins)

    peak_idx = int(np.argmax(hist))
    peak_ratio = float(hist[peak_idx]) / float(values.size)

    if peak_ratio >= FLOOR_VALUE_RATIO:   # 0.30
        return float((edges[peak_idx] + edges[peak_idx + 1]) / 2.0)

    return None
```

**参数选择依据**：

| 参数 | 值 | 依据 |
|---|---|---|
| `FLOOR_BIN_WIDTH` | 0.01 m | 与深度分辨率同量级，避免过粗/过细 |
| `FLOOR_VALUE_RATIO` | 0.30 | 真实场景中同一深度的像素占比通常 < 20%；地板值 63~66%，阈值取 0.30 有明显区分度 |

**阈值 0.30 的合理性**：

- **不能太低**（如 0.10）：墙壁等大面积平面也可能占 10% 以上，
  会误剔除真实障碍
- **不能太高**（如 0.50）：某些视角下地板值占比可能降至 40%，
  会漏检

设 0.30 在两者之间，且实测地板值占比 63~66%，
与真实平面（< 20%）有 3 倍以上的间隔，判别安全。

### 7.2.4 剔除与保护

```python
floor_value = self._detect_floor_value(valid)
if floor_value is not None:
    # 保留与地板值差异超过半个分箱宽度的像素
    keep = np.abs(valid - floor_value) > (FLOOR_BIN_WIDTH * 0.5)
    filtered = valid[keep]

    if filtered.size >= 30:
        # 剔除后仍有足够像素，采用剔除结果
        valid = filtered
    # 否则说明画面几乎全是无效区域，交由上层按开阔处理
```

**保护机制的必要性**：

若画面中**全部**是地板值（例如相机被完全遮挡或朝向纯天空），
强行剔除会导致 `valid.size == 0`，统计量返回 `inf`。

返回 `inf` 的语义是"前方无有效信息"，上层按开阔处理 ——
**这是刻意的设计选择**：宁可认为前方通畅（配合速度衰减与紧急制动
仍可保证安全），也不能因传感器异常而永久停车。

### 7.2.5 修复效果验证

**修复前**：

```
中区 p25 = 1.000 m  (假值)  → 判定有障碍  → 30 秒移动 0.08 m
```

**修复后**：

```
left   floor 未检出 (7%)     → p25 = 0.855 m
mid    floor = 1.005 (66%)   → 剔除后 p25 = 3.509 m   ← 真实！前方 3.5 m 通畅
right  floor = 1.005 (63%)   → 剔除后 p25 = 2.640 m
```

→ 车辆正常行驶，**30 秒移动 7.40 m**

### 7.2.6 与其他传感器的对比

| 传感器 | 无效值表示 | 处理方式 |
|---|---|---|
| 激光雷达 | `inf` / `nan` | 直接剔除（标准做法） |
| 深度相机（Gazebo） | **固定常数**（1.005 m） | **直方图峰值检测 + 剔除** |
| 深度相机（真实设备） | `0` / `nan` / `inf` | 直接剔除 |

**Gazebo 深度相机的这个行为是一个陷阱** —— 它输出的不是标准的
无效值标记，而是看起来完全合理的距离数（1.005 m 是一个"正常"的深度）。
如果开发者不了解这一点，会误认为传感器工作正常，
而算法却在用错误数据做决策。

这也是本项目最有价值的工程发现之一。

---

## 7.3 数据同步

### 7.3.1 仿真时间

Gazebo 仿真使用 `/clock` 话题发布**仿真时间**。所有 ROS 节点
必须设置 `use_sim_time = true`，否则时间戳不一致：

```xml
<param name="use_sim_time" value="true"/>
```

**实测时钟频率**：

```
$ rostopic hz /clock
average rate: 997.989
	min: 0.001s max: 0.005s std dev: 0.00008s
```

约 998 Hz，对应物理步长 1 ms。

### 7.3.2 时间基准的陷阱

`rospy.Time.now()` 在 `use_sim_time = true` 时返回**仿真时间**，
而非墙钟时间。当 `/clock` 尚未发布时，它返回 0。

这会导致 `rospy.Rate.sleep()` **长时间阻塞** ——
详见 [系统架构 § 4.4](architecture.md)。

**本项目解决方案**：控制循环使用墙钟时间（`time.time()`），
仅算法内部的状态计时使用仿真时间（`rospy.Time.now()`）。

### 7.3.3 传感器时间戳

| 传感器 | 频率 | 周期 |
|---|---|---|
| `/scan` | 10 Hz | 100 ms |
| `/camera/depth/image_raw` | 13~15 Hz | 67~77 ms |
| `/odom` | ~48 Hz | 21 ms |
| `/clock` | 998 Hz | 1 ms |

控制回路频率 20 Hz（50 ms）高于所有传感器频率，
保证每帧传感器数据都能被及时处理，且控制指令的下发
比传感器更新更频繁（有利于平滑控制）。

---

## 7.4 数据可视化

### 7.4.1 RViz 配置

```bash
roslaunch openhutb_car main.launch mode:=laser rviz:=true
```

**建议显示的图层**：

| 图层 | 话题 | 作用 |
|---|---|---|
| LaserScan | `/scan` | 查看激光扫描点 |
| PointCloud2 | `/camera/depth/points` | 查看深度点云 |
| Image | `/camera/rgb/image_raw` | 查看 RGB 图像 |
| TF | — | 查看坐标变换关系 |
| Odometry | `/odom` | 查看位姿 |
| Marker | `/odom_recorder/trail` | 查看行驶轨迹 |

### 7.4.2 命令行数据检查

```bash
# 查看激光数据的最小值分布
rostopic echo /scan | grep -A1 "ranges" | head -20

# 统计深度图非零像素比例
python3 -c "
import rospy, numpy as np
from sensor_msgs.msg import Image
rospy.init_node('probe')
msg = rospy.wait_for_message('/camera/depth/image_raw', Image, timeout=10)
img = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
total = img.size
valid = np.sum(np.isfinite(img) & (img > 0.01))
print('valid ratio: %.1f%%' % (100.0 * valid / total))
"
```

### 7.4.3 话题频率监控

```bash
# 同时监控多个话题
rostopic hz /scan /cmd_vel /odom /camera/depth/image_raw

# 输出示例
subscribed to [/scan]
subscribed to [/cmd_vel]
subscribed to [/odom]
average rate: 9.978      ← /scan
average rate: 20.021     ← /cmd_vel
average rate: 48.312     ← /odom
```

---

## 7.5 处理流程总结

```
┌──────────────────────────────────────────────────────────────────┐
│                      激光雷达处理链                                │
│                                                                  │
│  /scan ──▶ 类型过滤 ──▶ 角度规范化 ──▶ 自遮挡过滤 ──▶ 扇区划分     │
│  (360点)   inf/nan     θ→[-π,π)      邻域连续性      5扇区最小值   │
│            /<0.12                     ≥5束判定                     │
│                                                                  │
│  输出: {front, left, right, left_bk, right_bk}                    │
└──────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────┐
│                     深度相机处理链                                 │
│                                                                  │
│  Image ──▶ 解析 ──▶ ROI裁剪 ──▶ 有效值过滤 ──▶ 地板值剔除 ──▶ 百分位 │
│  (640×    bytes→   三区+      isfinite      直方图峰值      P25    │
│   480)    numpy    行范围     >0.01         >30%剔除               │
│                                                                  │
│  输出: {d_left, d_mid, d_right}                                   │
└──────────────────────────────────────────────────────────────────┘

                          两路数据
                             │
                             ▼
                 ┌───────────────────────┐
                 │   FSM 决策 + 势场控制   │
                 └───────────┬───────────┘
                             ▼
                    Twist → /cmd_vel
```
