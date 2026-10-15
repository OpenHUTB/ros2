# 任务二：深度相机感知与视觉避障

> **任务书原文**
>
> (2) 实现传感器感知及运动控制，要求使用传感器获取的数据进行感知
> （如：雷达、摄像头、深度相机等）对机器人进行运动控制
> （5×2：感知、给定轨迹导航与运动控制）。

---

## 3.1 实现目标

使用与激光雷达**原理完全不同**的传感器 —— RGB-D 深度相机，
独立实现第二条感知 → 控制闭环，满足任务书对不同传感器类型的要求。

```
  ┌─────────────┐  depth/image_raw  ┌─────────────┐   Twist   ┌─────────────┐
  │  深度相机    │ ────────────────▶ │  视觉避障    │ ────────▶ │  底盘控制器  │
  │  Gazebo     │   Image (32FC1)   │  ROI+FSM     │  /cmd_vel │ diff_drive  │
  └─────────────┘                   └─────────────┘           └─────────────┘
```

---

## 3.2 传感器配置

### 3.2.1 深度相机规格

| 参数 | 值 | 说明 |
|---|---|---|
| 类型 | `depth` | Gazebo 深度相机 |
| 分辨率 | 640 × 480 | 30.7 万像素 |
| 水平视场角 | 80° (1.396 rad) | 前向视野 |
| 裁剪范围 | 0.05 ~ 30.0 m | near / far |
| 输出频率 | 15 Hz | 实测约 13 Hz |
| 编码格式 | `32FC1` | 32 位浮点，单位米 |
| 噪声 | 高斯，$\sigma = 0.007$ m | 模拟真实深度噪声 |
| 安装位置 | 车头前方 0.24 m，高度 0.29 m | — |
| ROS 话题 | `/camera/depth/image_raw` | `sensor_msgs/Image` |

### 3.2.2 插件选择的踩坑记录

Gazebo 提供两个可用于深度感知的 ROS 插件，本项目**两者都试过**：

| 插件 | 优点 | 缺点 | 结论 |
|---|---|---|---|
| `libgazebo_ros_openni_kinect.so` | 同时输出 RGB、深度、点云 | 深度图经 `pointCloudCutoff` 截断，读数失真（实测所有像素退化为 **1.000 m**） | ❌ 弃用 |
| `libgazebo_ros_depth_camera.so` | 直接发布原始深度图，无截断，测距准确 | 需额外配置 RGB 相机 | ✅ 采用 |

**实测对比**：

```
使用 openni_kinect 插件：
  left   p15=1.00  p40=1.00  median=1.45
  mid    p15=1.00  p40=1.00  median=1.00    ← 全部被压成常数
  right  p15=1.00  p40=1.00  median=1.00

使用 depth_camera 插件：
  left   min=1.000  median=1.314  max=2.460
  mid    min=1.000  median=1.000  max=8.762   ← 出现真实测距
  right  min=1.000  median=1.000  max=9.232
```

### 3.2.3 光学坐标系

深度相机遵循 **REP-103 光学坐标系约定**，与机器人本体坐标系不同：

| 坐标系 | X | Y | Z |
|---|---|---|---|
| 机器人（`camera_link`） | 前 | 左 | 上 |
| 光学（`camera_optical_link`） | 右 | 下 | 前 |

二者通过固定的旋转关系关联：

$$
R = R_z(-90°) \cdot R_y(0°) \cdot R_x(-90°)
$$

在 URDF 中表达为：

```xml
<joint name="camera_optical_joint" type="fixed">
  <parent link="camera_link"/>
  <child link="camera_optical_link"/>
  <origin xyz="0 0 0" rpy="${-PI/2} 0 ${-PI/2}"/>
</joint>
```

深度图像中的像素坐标 $(u, v)$ 与光学坐标系对应，
使得画面中央对应车辆正前方。

---

## 3.3 算法总体流程

```
                ┌────────────────────────────┐
                │  /camera/depth/image_raw   │
                │      (640×480, 32FC1)      │
                └─────────────┬──────────────┘
                              ▼
                ┌────────────────────────────┐
                │  ① 图像解析                 │
                │    bytes → numpy (H, W)    │
                └─────────────┬──────────────┘
                              ▼
                ┌────────────────────────────┐
                │  ② 三区 ROI 划分            │
                │    左区 / 中区 / 右区       │
                └─────────────┬──────────────┘
                              ▼
                ┌────────────────────────────┐
                │  ③ 深度地板值剔除 ★         │
                │    直方图峰值检测           │
                └─────────────┬──────────────┘
                              ▼
                ┌────────────────────────────┐
                │  ④ 低百分位数统计           │
                │    提取近处威胁             │
                └─────────────┬──────────────┘
                              ▼
                ┌────────────────────────────┐
                │  ⑤ 有限状态机决策           │
                │    CRUISE/AVOID/EMERGENCY   │
                └─────────────┬──────────────┘
                              ▼
                ┌────────────────────────────┐
                │  ⑥ 势场法生成转向           │
                └─────────────┬──────────────┘
                              ▼
                     ┌──────────────────┐
                     │  Twist → /cmd_vel│
                     └──────────────────┘
```

---

## 3.4 感知层

### 3.4.1 图像解析

`sensor_msgs/Image` 的 `data` 字段是一维字节数组，需转换为二维数组：

```python
if msg.encoding == "32FC1":
    dtype = np.dtype(np.float32)     # 浮点深度，单位米
    scale = 1.0
elif msg.encoding == "16UC1":
    dtype = np.dtype(np.uint16)      # 整数深度，单位毫米
    scale = 0.001

img = np.frombuffer(msg.data, dtype=dtype)
img = img.reshape((msg.height, msg.width)).astype(np.float32)
```

### 3.4.2 三区 ROI 划分

模仿人眼视野划分，将深度图像在水平方向分为三个竖直条带：

```
   ┌──────────┬──────────┬──────────┐
   │          │          │          │
   │   左区    │   中区    │   右区    │   ← 竖直取样 25%~75%
   │  0%~25%  │ 37%~62%  │ 75%~100% │      （避开地面与天花板）
   │          │          │          │
   └──────────┴──────────┴──────────┘
        ↑          ↑           ↑
      d_left     d_mid      d_right
                 （正前方）
```

| 区域 | 列范围比例 | 对应物理含义 |
|---|---|---|
| 左区 | 0% ~ 25% | 左前方 |
| 中区 | 37% ~ 62% | 正前方（决策主依据） |
| 右区 | 75% ~ 100% | 右前方 |

**竖直方向只取 25% ~ 75% 行**：顶部是天空/远处（无有效回波），
底部是地面（距离恒近），二者都会干扰统计。

### 3.4.3 深度地板值剔除（关键工程处理）

**问题**：Gazebo 深度相机对**无回波区域**（看向场景外或超出量程的部分）
并非输出 `0` 或 `inf`，而是输出一个**固定常数**。

实测该常数为 **1.005 m**，且在某些视角下占比极高：

```
left   floor 未检出 (7%)     → p25 = 0.855 m
mid    floor = 1.005 (66%)   → 剔除后 p25 = 3.509 m
right  floor = 1.005 (63%)   → 剔除后 p25 = 2.640 m
```

**中区 66% 的像素都是这个假的 1.005 m**！

若不剔除，任何百分位统计（无论 p15、p25 还是 p40）都会被锁定为
常数 1.0，导致算法**误判前方恒有 1 m 障碍**，车辆原地打转无法前进。

**解决方案：直方图峰值检测**

对有效像素做直方图统计（分箱宽度 0.01 m），若某一深度值的像素
占比超过阈值（`FLOOR_VALUE_RATIO = 0.30`），则判定为地板值并整体剔除：

```python
hist, edges = np.histogram(values, bins=n_bins)
peak_idx = int(np.argmax(hist))
peak_ratio = float(hist[peak_idx]) / float(values.size)

if peak_ratio >= FLOOR_VALUE_RATIO:
    # 该峰值即为地板值
    floor_value = (edges[peak_idx] + edges[peak_idx + 1]) / 2.0
```

剔除时保留与地板值差异超过半个分箱宽度的像素：

```python
keep = np.abs(valid - floor_value) > (FLOOR_BIN_WIDTH * 0.5)
filtered = valid[keep]
```

**保护机制**：若剔除后所剩像素不足 30 个，说明画面几乎全是无效区域，
此时视为"前方无有效信息"，交由上层按开阔处理，避免误判。

### 3.4.4 为什么用低百分位数而非最小值或均值

| 统计量 | 优点 | 缺点 | 评价 |
|---|---|---|---|
| 最小值 | 对最近障碍最敏感 | 单个噪声像素即可触发误判 | ❌ 鲁棒性差 |
| 均值 | 抗噪声 | 远处空旷区域会稀释近处障碍 | ❌ 会"看不见"障碍 |
| **低百分位数** | 既反映最近威胁，又容忍少量噪声 | 需调参 | ✅ 采用 |

本项目取 `DEPTH_PERCENTILE = 25.0`（第 25 百分位）：

$$
d_{\text{ROI}} = P_{25}\{Z(u,v) \mid (u,v) \in \text{ROI},\; Z \text{ 有效}\}
$$

含义：该区域中 25% 的像素比它更近。选择依据：
既能捕获占据画面一定比例的真实障碍，又不会被少量孤立噪声像素拉偏。

---

## 3.5 决策层与控制层

决策层与控制层的结构与激光雷达版本**完全一致**，便于对比：

### 3.5.1 状态与转移

| 当前状态 | 转移条件 | 下一状态 |
|---|---|---|
| CRUISE | $d_{\text{mid}} < d_{\text{safe}}$ | AVOID |
| AVOID | $d_{\text{mid}} > d_{\text{clear}}$ | CRUISE |
| AVOID | $d_{\text{mid}} < d_{\text{danger}}$ | EMERGENCY |
| EMERGENCY | 超时且前方清空 | CRUISE |
| EMERGENCY | 超时且前方仍近 | AVOID |

### 3.5.2 控制律

与激光版本同构，仅参数不同：

$$
\omega = k_\omega \cdot \text{clamp}\!\left(\frac{d_{\text{left}} - d_{\text{right}}}{2.0},\; -1,\; +1\right)
$$

$$
v = v_{\text{avoid}} \cdot \text{clamp}\!\left(
    \frac{d_{\text{mid}} - d_{\text{danger}}}{d_{\text{safe}} - d_{\text{danger}}},\;
    0,\; 1\right)
$$

### 3.5.3 原地打转的超时保护

**新增的安全机制**：当左右两侧都被判定为阻塞时会原地转向寻找出路。
若因传感器异常导致该状态持续过久，应主动退出，避免车辆无限打转：

```python
if max(dl, dr) < D_SAFE:
    omega = W_MAX if dl >= dr else -W_MAX
    cmd.linear.x = 0.0

    # 超时保护
    now = rospy.Time.now()
    if self._spin_since is None:
        self._spin_since = now
    elif (now - self._spin_since).to_sec() > MAX_SPIN_DURATION:
        rospy.logwarn("原地转向已持续 %.1fs 仍未脱困，恢复正常行驶",
                      MAX_SPIN_DURATION)
        self._spin_since = None
        self._state = CarState.CRUISE
        cmd.linear.x = V_CRUISE
        cmd.angular.z = 0.0
        return cmd
```

`MAX_SPIN_DURATION = 1.5` s。

这条保护是在调试深度相机地板值问题时的**直接产物** ——
当时车辆因传感器读数异常而无限原地打转，暴露出缺少状态超时机制的缺陷。

### 3.5.4 参数汇总

| 参数 | 值 | 激光版本对应值 |
|---|---|---|
| $d_{\text{safe}}$ | 1.2 m | 1.0 m |
| $d_{\text{clear}}$ | 1.6 m | 1.4 m |
| $d_{\text{danger}}$ | 0.5 m | 0.45 m |
| $v_{\text{cruise}}$ | 0.30 m/s | 0.35 m/s |
| $v_{\text{avoid}}$ | 0.15 m/s | 0.18 m/s |
| $v_{\text{back}}$ | −0.10 m/s | −0.12 m/s |
| $\omega_{\min}$ | 0.40 rad/s | 0.45 rad/s |
| $\omega_{\max}$ | 1.20 rad/s | 1.30 rad/s |
| $k_\omega$ | 1.6 | 1.8 |

**参数差异原因**：深度相机视场角仅 80°（激光雷达 360°），
侧向感知能力弱，因此安全距离设得更大（1.2 m vs 1.0 m）、
速度设得更保守（0.15 m/s vs 0.18 m/s），为转向留出更多余量。

---

## 3.6 两种传感器的对比

> 任务书要求"支持多种路径规划与控制算法的对比"，
> 本节对两种感知方案进行系统对比。

| 维度 | 激光雷达 | 深度相机 |
|---|---|---|
| **数据形式** | 1D 极坐标距离序列（360 点） | 2D 稠密深度图（30.7 万像素） |
| **视角** | 360° 全向 | 80° 前向 |
| **盲区** | 车体自身遮挡（可过滤） | 视场外全部不可见 |
| **数据量** | 约 1.4 KB / 帧 | 约 1.2 MB / 帧 |
| **计算量** | 低（360 次比较） | 中（ROI 统计，numpy 向量化） |
| **测距精度** | 高（$\sigma = 0.01$ m） | 中（$\sigma = 0.007$ m，存在地板值） |
| **优势** | 全向感知、精确测距、计算量小 | 可识别障碍形状与纹理、信息丰富 |
| **劣势** | 无法感知低于扫描面的障碍（如台阶） | 视角窄、存在无效值、对反光敏感 |
| **实测平均速度** | 0.319 m/s | 0.219 m/s |
| **实测路径效率** | 0.498 | 0.758 |

### 3.6.1 为什么激光版本的路径效率更低？

路径效率 = 净位移 / 实际路程。激光版本 0.498 < 相机版本 0.758。

**这并非激光版本"更差"，而是它"更激进"**：

- 激光雷达具备 **360° 全向感知**，能提前发现侧后方障碍，
  因此敢于做更大幅度的绕行（路程 18.22 m）；
- 深度相机只有 **80° 前向视野**，看不到两侧情况，
  只能做较保守的小幅转向（路程 12.82 m）。

在避障任务中，**全向感知带来的安全余量**比"路径更直"更有价值。
若以"到达目标"为评价标准，激光方案的可靠性显著更高。

### 3.6.2 适用场景建议

| 场景 | 推荐方案 | 理由 |
|---|---|---|
| 室内自主导航 | 激光雷达 | 全向感知，无盲区 |
| 需要识别障碍类型 | 深度相机 + 视觉算法 | 可获取形状、纹理信息 |
| 算力受限平台 | 激光雷达 | 数据量与计算量低 2~3 个数量级 |
| 狭窄通道通行 | 两者融合 | 激光保安全，视觉辨细节 |
| 台阶/悬空障碍 | 深度相机 | 激光扫描面在障碍上方会漏检 |

---

## 3.7 源码解析

**文件**：`scripts/depth_camera_avoid.py`

### 3.7.1 类结构

```
DepthCameraAvoidNode
├── __init__(cmd_pub, watchdog)       订阅 /camera/depth/image_raw
├── start() / stop()                  生命周期 + 统计输出
├── _depth_callback(msg)              缓存最新深度帧
├── _image_to_array(msg)              ★ 图像解析：bytes → numpy
├── _roi_stats(img, c0, c1)           ★ ROI 统计（含地板值剔除）
├── _detect_floor_value(values)       ★ 直方图峰值检测地板值
├── _roi_percentile(...)              兼容接口
├── _decide(d_mid)                    ★ 有限状态机
├── _compute_command(...)             ★ 控制律 + 超时保护
└── update()                          ★ 主循环
```

### 3.7.2 ROI 统计的完整实现

```python
def _roi_stats(self, img, col_start, col_end):
    h, w = img.shape
    c0, c1 = int(col_start * w), int(col_end * w)
    r0, r1 = int(ROI_ROW_START * h), int(ROI_ROW_END * h)

    patch = img[r0:r1, c0:c1]

    # 步骤 2：剔除 0 / inf / nan
    valid = patch[np.isfinite(patch) & (patch > 0.01)]
    if valid.size == 0:
        return float("inf"), 0

    # 步骤 3：剔除深度地板值
    floor_value = self._detect_floor_value(valid)
    if floor_value is not None:
        keep = np.abs(valid - floor_value) > (FLOOR_BIN_WIDTH * 0.5)
        filtered = valid[keep]
        if filtered.size >= 30:
            valid = filtered

    if valid.size == 0:
        return float("inf"), 0

    # 步骤 4：低百分位数
    return float(np.percentile(valid, DEPTH_PERCENTILE)), int(valid.size)
```

### 3.7.3 地板值检测实现

```python
@staticmethod
def _detect_floor_value(values):
    if values.size < 100:
        return None

    vmin, vmax = float(values.min()), float(values.max())
    if vmax - vmin < FLOOR_BIN_WIDTH:
        return None

    n_bins = int((vmax - vmin) / FLOOR_BIN_WIDTH) + 1
    n_bins = min(n_bins, 4096)          # 限制分箱数量

    hist, edges = np.histogram(values, bins=n_bins)

    peak_idx = int(np.argmax(hist))
    peak_ratio = float(hist[peak_idx]) / float(values.size)

    if peak_ratio >= FLOOR_VALUE_RATIO:
        return float((edges[peak_idx] + edges[peak_idx + 1]) / 2.0)

    return None
```

---

## 3.8 运行验证

### 3.8.1 启动

```bash
roslaunch openhutb_car main.launch mode:=camera
```

### 3.8.2 预期输出

```
[INFO] [openhutb_car_main]: 模式：camera —— 深度相机避障模式（感知→运动控制闭环）
[INFO] [depth_camera]: 已订阅 /camera/depth/image_raw，等待深度图像……
[INFO] [depth_camera]: 视觉避障节点启动 (D_SAFE=1.20m, 百分位=25%)
[INFO] [openhutb_car_main]: 算法节点已启动（控制频率 20 Hz）
```

### 3.8.3 验证深度数据

```bash
rostopic hz /camera/depth/image_raw     # 期望 ~13-15 Hz

python3 -c "
import rospy, numpy as np
from sensor_msgs.msg import Image
rospy.init_node('probe')
msg = rospy.wait_for_message('/camera/depth/image_raw', Image, timeout=10)
print('encoding:', msg.encoding, 'size:', msg.width, 'x', msg.height)
img = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
valid = img[np.isfinite(img) & (img > 0.01)]
print('valid px:', valid.size, 'min:', valid.min(), 'median:', np.median(valid))
"
```

实测输出：

```
encoding: 32FC1 size: 640 x 480
valid px: 307200 min: 0.05 median: 0.944
```

**307200 = 640 × 480，全部像素有效。**

### 3.8.4 实测运动结果

实验条件：车辆自 `(-5.0, -4.0)` 出发，运行 58.5 秒。

| 指标 | 实测值 |
|---|---|
| 起点 | (1.56, -3.96) |
| 终点 | (5.36, 5.00) |
| 行驶路程 | 12.82 m |
| 净位移 | 9.73 m |
| **路径效率** | **0.758** |
| 平均速度 | 0.219 m/s |
| X 范围 | 1.56 ~ 5.58 m |
| Y 范围 | −3.96 ~ 5.00 m |

### 3.8.5 地板值修复前后的对比

| 阶段 | 中区 p25 深度 | 30 秒位移 | 现象 |
|---|---|---|---|
| 修复前 | 1.00 m（假值） | **0.08 m** | 误判前方有障碍，原地打转 |
| 修复后 | 3.51 m（真值） | **7.40 m** | 正常避障行驶 |

### 3.8.6 仿真截图

![深度相机避障运行截图](../images/I_camera_a.png)

*图 3-1：深度相机避障模式运行场景。车辆在画面左下角，
可见其正驶离起始区域。*

---

## 3.9 小结

本任务完成了任务书第 (2) 项中"深度相机感知 → 运动控制闭环"的要求：

| 环节 | 实现 |
|---|---|
| **感知** | 640×480 深度图 → 三区 ROI → 地板值剔除 → 低百分位统计 |
| **决策** | 三态有限状态机 + 原地点转超时保护 |
| **控制** | 势场法转向 + 速度衰减 + 最小转向力度 |

实测自主避障行驶 12.82 m，路径效率 0.758，全程无碰撞。

本任务的关键技术贡献是**识别并解决了 Gazebo 深度相机的
"深度地板值"缺陷** —— 这是仿真环境中不易察觉但影响致命的问题，
详见 [工程问题排查记录](troubleshooting.md)。
