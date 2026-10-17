# 基于前视视觉的车道线检测（carla_lane_keeping）

本模块在 CARLA 仿真环境中，用**前视相机图像**检测车道线，输出车辆相对车道中心的
**横向偏差**、**航向偏差**与**车道曲率**。全部算法用纯 numpy 实现，**不依赖 OpenCV、
不依赖深度学习**，可在教学虚拟机的 ROS Noetic 环境中直接运行。

配套的车道保持控制见[车道保持控制](./lane_keeping_control.md)。

## 1. 为什么做这个模块

仓库中地面载具部分已有手动控制、阿克曼控制、路径点发布器、自动驾驶代理等模块，
但它们都停留在**控制与接口**层面，没有任何一个模块做**视觉感知**。

车道保持是自动驾驶最基础、也最能体现"感知 → 决策 → 控制"闭环的能力：
它必须先从图像中恢复出车道几何，再据此算出车辆偏离了多少，才能谈控制。
本模块负责前者，并输出下游可直接消费的物理量。

## 2. 处理流程

```text
彩色图 (BGR)
  │
  ├─ 灰度化
  │
  ├─ 方向梯度阈值 ──┐
  │                 ├─► 二值车道像素掩码
  └─ 颜色阈值 ──────┘
  │
  ├─ 透视变换（梯形 → 矩形，得到鸟瞰图）
  │
  ├─ 底部直方图定位车道线起点
  │
  ├─ 连通域分组 + 多项式拟合
  │
  └─ 输出：左右车道线系数
             ├─► 横向偏差（米）
             ├─► 航向偏差（弧度）
             └─► 车道曲率半径（米）
```

### 2.1 方向梯度阈值

车道线在图像中近似**竖直**。对图像求 Sobel 梯度后，定义梯度方向角

```text
angle = atan2(|gy|, |gx|)
```

这里有一个**极易搞反**的点：

- **竖直边缘**（车道线）的梯度几乎全在水平方向，`|gy| ≈ 0`，因此角度 **接近 0°**
- **水平边缘**（路面接缝、阴影边界）角度**接近 90°**

所以判据是保留 `angle <= angle_max_deg`，而不是相反。若按 90° 写判据，
整幅图会被过滤干净、一个车道像素都检不到。

梯度幅值阈值采用**自适应**策略：

```text
实际阈值 = max(sobel_min, 该图像梯度分布的 sobel_auto_percentile 分位)
```

原因很实际：实测 CARLA 前视相机的路面纹理梯度中位数约 **45**、90 分位约 **260**，
而合成图像背景梯度几乎为 **0**。用同一个固定阈值处理两类图像必然顾此失彼——
取 25 会在真实图上把 40% 的路面判成车道线（整块路面连成一个巨大连通域），
取 160 又会让低噪声图像几乎没有输出。按分位数自适应可以兼顾。

在真实 CARLA 图像上的实测效果：

| 阈值策略 | 二值掩码占比 | 最大连通域 |
|---|---|---|
| 固定 25（未标定） | **42%** | 271205 像素（整块路面） |
| 自适应（本模块） | **1.6% ~ 4%** | 细长结构，宽高比 < 0.1 |

### 2.2 透视变换

用**直接线性变换（DLT）**自行求解单应矩阵，不依赖 OpenCV：

```text
源梯形（图像中感兴趣的地面区域） → 目标矩形（鸟瞰图）
```

两个容易踩的坑，本模块都已处理：

1. **源梯形底边必须贴到图像最底行**（`bottom_ratio = 1.0`）。
   横向偏差是相对"离车最近处"定义的；若底边抬高到 0.95h，算出来的其实是
   约 3 m 前方的偏差，会引入与距离成正比的**系统性误差**（实测偏右 100 px
   的场景误差达 35 px）。

2. **目标矩形上下必须留边距**。源梯形底边在图像底部，透视外推后该行以下的
   近处像素会映射到目标图之外；若目标矩形下边直接顶到 `height-1`，
   这些像素全部落在图像外被丢弃，导致近处车道（尤其偏在一侧的）整条消失。

### 2.3 车道线拟合：为什么用连通域而不是滑动窗口

绝大多数车道线检测教程用**滑动窗口**。本模块最初也用，但在测试中发现了
它的一个固有缺陷：

> 滑动窗口要求「窗口宽度 > 单条车道线宽度」且「窗口宽度 < 两条车道线间距」。

这两个条件很容易同时不成立。实测：窗口取 ±100 px 时宽达 200 px，
而两条车道线在鸟瞰图中的间距可能只有约 91 px——**每个窗口都同时框住两条线**，
取均值后两条线都被拟合到画面正中，输出完全错误却**不报任何错**。

因此本模块改用**连通域分组**：鸟瞰图中每条车道线是一个细长连通域，
先做连通域标记，再按基点列把左右两侧分别取出、各自拟合多项式。
这样不存在尺度矛盾，且天然把两条线分开。

为防误检，还加了**几何约束**。这里有一个关键的设计选择：

判据用的是**填充率**（连通域像素数 / 其包围盒面积），而**不是包围盒宽度**。
原因是车道线在鸟瞰图中常常是**倾斜**的（车辆偏离车道中心、或相机存在偏航时），
此时它的包围盒会很宽——实测一条完全正常的倾斜车道线，包围盒宽度达 114 px，
但填充率只有 0.27。若按「包围盒宽度 < 90 px」判断，这条真实车道线会被整条
拒绝，导致车辆偏右场景下的横向偏差误差达 **25 px**。

填充率则对竖直与倾斜车道线都成立：车道线很细，填充率必然低；而被误阈值化的
路面是面状结构，包围盒内几乎被填满，填充率很高。此外还限制包围盒宽度、
总面积与纵向长度，共同排除零碎小块与整块路面。

若连通域因虚线车道线或局部遮挡而断裂，会自动退回滑动窗口作为兜底。

### 2.4 几何量解算

设左右车道线在求值行上的列坐标为 `u_l`、`u_r`，则

```text
车道中心      u_center = (u_l + u_r) / 2
横向偏差      lateral = (u_vehicle - u_center) * meters_per_pixel_x
```

其中 `u_vehicle` 取鸟瞰图水平中心（相机装在车辆中心、光轴朝前时，
图像横向中心对应车辆前进方向）。**正值表示车辆偏在车道中心右侧。**

航向偏差由车道中心线在求值行的斜率换算；曲率半径由一阶、二阶导数给出：

```text
R = (1 + (dx/dy)^2)^(3/2) / |d²x/dy²|        （直道为无穷大）
```

> **求值行必须选在有车道像素的行上。**
> 目标矩形底边略高于图像底行，若固定用 `height-1` 求值，会在没有任何数据的
> 行上**外推多项式**，横向偏差出现十几到几十像素的系统误差。本模块从掩码
> 统计出实际有像素的最底行再求值。

## 3. 环境准备

### 3.1 依赖

| 依赖 | 版本要求 | 说明 |
|---|---|---|
| ROS Noetic | — | 虚拟机镜像内已提供 |
| Python | 3.8（ROS Noetic 自带） | **算法本身兼容 3.8 ~ 3.12** |
| numpy | ≥ 1.18 | 唯一的算法依赖 |
| CARLA | 0.9.13（与 `carla_ros_bridge` 一致） | 仅"实车运行"需要 |
| OpenCV | — | **不需要**（算法为纯 numpy 实现） |
| 深度学习框架 | — | **不需要** |

### 3.2 已验证环境

本模块在以下环境完成过实际运行与测试。**请按行对应**——不同项目的验证环境不同，
不要跨行推断：

| 验证项目 | 操作系统 | Python | 依赖 |
|---|---|---|---|
| 84 个单元测试 | Ubuntu 20.04（ROS Noetic 教学镜像） | **3.8.10** | numpy |
| 84 个单元测试 | Windows 11 | **3.12.14** | numpy |
| `demo_lane_detection.py` 演示 | Ubuntu 20.04 | **3.8.10** | numpy |
| `demo_lane_detection.py` 演示 | Windows 11 | **3.12.14** | numpy |
| `mkdocs serve --livereload` 网页验证 | Ubuntu 20.04 | **3.8.10** | 见 3.5 节 |
| 真实 CARLA 图像验证 | Windows 11 | **3.12.8** | numpy + carla 0.9.16 |

需要说明的两点：

1. **单元测试与演示脚本在 Python 3.8 与 3.12 上都跑通了**，两次结果一致
   （均为 `Ran 84 tests ... OK`）。算法本身只依赖 numpy，两个版本都能运行。
2. **真实 CARLA 图像验证只在 Windows 上做过**（Python 3.12.8 + carla 0.9.16）。
   教学虚拟机侧没有做这项验证，原因是 CARLA 版本与桥接要求不一致、
   且 RPC 只监听本机（详见 3.6 节）。

> **重要提示**：算法与单元测试**不依赖 CARLA、不依赖 ROS、不依赖 OpenCV**，
> 因此可以在任何有 Python + numpy 的机器上直接复现（见 3.4 节）。

### 3.3 编译

```shell
# 为避免与教材章节中的同名包冲突，单独建立工作空间
mkdir -p ~/lane_ws/src
ln -s ~/ros2/src/ground/carla_lane_keeping ~/lane_ws/src/
cd ~/lane_ws
catkin_make
source devel/setup.bash
```

### 3.4 无需 CARLA / ROS 的验证方式（推荐给评审者）

这是**最快确认本模块可用的方法**，三分钟内可完成，不需要启动任何仿真器：

```shell
cd ~/ros2/src/ground/carla_lane_keeping

# 1) 单元测试：84 个测试，覆盖梯度、透视变换、拟合、几何量、ROS 节点、PID
python3 -m unittest discover -s tests -v
#    预期结尾: Ran 84 tests ... OK

# 2) 端到端演示：生成合成图像 -> 检测 -> 输出量化指标并保存结果图
python3 tests/demo_lane_detection.py
#    预期: 3/3 个场景检出两侧车道线，最大测量误差 0.70 px（0.035 m）
#    结果图片保存在 demo_out/ 目录

# 3) 查看结果图（蓝=左车道线、红=右车道线、绿=中心线、黄十字=检出中心）
xdg-open demo_out/demo_1.png
```

`numpy` 是唯一前置依赖。若提示缺失：

```shell
pip3 install --user -i https://pypi.tuna.tsinghua.edu.cn/simple numpy
```

### 3.5 文档环境（`mkdocs serve`）的两处已知问题

若需要在本地预览文档网页，仓库现有的 `requirements.txt` 在 **Python 3.8**
环境（ROS Noetic 教学镜像的标配）下会遇到两个问题，这里记录解决办法：

**问题一：`mkdocs-bibtex` 在 Python 3.8 上导入崩溃**

现象：

```
TypeError: 'type' object is not subscriptable
  File ".../mkdocs_bibtex/citation.py", line 30, in Citation
    def from_markdown(cls, markdown: str) -> list[Citation]:
```

原因：新版本 `mkdocs-bibtex` 使用了 `list[Citation]` 这类内建泛型注解，
它在 Python 3.9+ 才可运行时求值。而 `pip` 在安装任何包时都会加载插件入口点，
因此这个崩溃会连带影响**所有** `pip install` 操作。

解决办法：给该包的所有源文件加上惰性注解（不改动任何逻辑）：

```shell
python3 - <<'PY'
import glob, os, ast
root = os.path.expanduser('~/.local/lib/python3.8/site-packages/mkdocs_bibtex')
for path in glob.glob(os.path.join(root, '**', '*.py'), recursive=True):
    src = open(path, encoding='utf-8').read()
    if 'from __future__ import annotations' in src:
        continue
    lines = src.splitlines(keepends=True)
    i = 0
    while i < len(lines) and (lines[i].startswith('#!') or 'coding' in lines[i][:40]):
        i += 1
    lines.insert(i, 'from __future__ import annotations\n')
    open(path, 'w', encoding='utf-8').write(''.join(lines))
    print('已修补', os.path.relpath(path, root))
PY
```

**问题二：`mkdocs-mermaid2-plugin` 需要 Node.js，但 `requirements.txt` 未声明**

现象：`Error: cannot find module 'mermaid2'`，装好后变成
`ModuleNotFoundError: No module named 'typing_extensions'`，
再之后是 `Cannot find node executable`。

原因：该插件依赖 `beautifulsoup4`（它又依赖 `typing_extensions`），
并在渲染时调用外部 `node` 进程，而 `requirements.txt` 里没有体现这些依赖。

解决办法：

```shell
sudo apt install -y nodejs npm
pip3 install --user -i https://pypi.tuna.tsinghua.edu.cn/simple \
    beautifulsoup4 typing_extensions soupsieve
pip3 install --user --no-deps -i https://pypi.tuna.tsinghua.edu.cn/simple \
    mkdocs-mermaid2-plugin
```

> 建议向仓库反馈：在 `requirements.txt` 中补充 Node.js 说明，并为
> `mkdocs-bibtex` 固定一个兼容 Python 3.8 的版本（例如 `mkdocs-bibtex<2.13`）。

### 3.6 实车运行的限制说明

若要与 CARLA 实车联调，需要注意：

1. **CARLA 版本必须与桥接一致**：仓库内 `carla_ros_bridge` 声明要求
   **CARLA 0.9.13**（其 `CARLA_VERSION` 文件值为 `0.9.13`，
   README 亦写明 "This version requires CARLA 0.9.13"）。
   使用 0.9.16 会因 API 差异导致连接异常。
2. **CARLA 的 RPC 默认只监听本机**（`127.0.0.1:2000` / `41451`），
   虚拟机不能直接连宿主机的 CARLA。需按
   [设置并连接到 Carla 模拟器](../../set_up_and_connect_to_carla.md)
   的方式通过 `carla_ros_bridge` 转发图像话题。
3. **CARLA 0.9.16 存在字符串导出缺陷**：在中文 Windows 环境下，
   `world.get_map()`、`client.get_trafficmanager()` 与车道侵入传感器
   都会抛 `UnicodeDecodeError`，导致无法获取标准生成点。
   若遇到该问题，请改用 0.9.13（与本仓库桥接一致）。

## 4. 运行

> **只想快速验证能不能跑？** 请直接看 3.4 节——不需要 CARLA、不需要 ROS，
> 三分钟内即可复现全部结果。

### 4.1 前置条件

需要一个正在运行、且有车辆的 CARLA 环境，且能提供前视相机图像。
两种方式任选：

- **方式一（推荐）**：先启动 CARLA 与桥接，图像话题为
  `/carla/ego_vehicle/rgb_front/image`
  参见[设置并连接到 Carla 模拟器](../../set_up_and_connect_to_carla.md)

- **方式二**：用本模块自带的独立采集脚本（不需要 ROS 桥接），
  直接连接 CARLA 的 Python API 抓图并跑检测，适合快速验证算法

  该方式依赖版本与桥接一致的 CARLA（0.9.13），限制见 3.6 节。

### 4.2 启动检测

```shell
# 方式一：一键启动（推荐）
roslaunch carla_lane_keeping main.launch

# 如果图像话题不同，用参数覆盖
roslaunch carla_lane_keeping main.launch image_topic:=/camera/image_raw

# 方式二：直接运行入口脚本
rosrun carla_lane_keeping main.py --mode detect

# 方式三：只启动检测节点
rosrun carla_lane_keeping lane_detector.py
```

### 4.3 查看结果

```shell
# 横向偏差（米，正 = 车辆偏右）
rostopic echo /lane/lateral_offset

# 航向偏差（弧度）
rostopic echo /lane/heading_error

# 是否偏离车道
rostopic echo /lane/departure

# 置信度
rostopic echo /lane/confidence
```

在 RViz 中可视化鸟瞰掩码：

```shell
rosrun rviz rviz
# Add -> Image -> Topic: /lane/mask
```

还可以用 `rqt_image_view` 查看实时画面。

## 5. 话题接口

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/carla/ego_vehicle/rgb_front/image` | `sensor_msgs/Image` | 前视彩色图（bgr8 / rgb8 / bgra8 / rgba8 / mono8 均支持） |
| 发布 | `lane/lateral_offset` | `std_msgs/Float32` | 横向偏差（米），正 = 车辆偏右 |
| 发布 | `lane/heading_error` | `std_msgs/Float32` | 航向偏差（弧度） |
| 发布 | `lane/curvature` | `std_msgs/Float32` | 车道曲率半径（米），直道发 0 |
| 发布 | `lane/confidence` | `std_msgs/Float32` | 置信度 0~1 |
| 发布 | `lane/departure` | `std_msgs/Bool` | 横向偏差是否超过阈值 |
| 发布 | `lane/mask` | `sensor_msgs/Image` | 鸟瞰二值掩码（调试，默认 5 帧发一次） |

> **未检出时不发布横向偏差。**
> 这一点很重要：如果感知失效时发布 0，下游控制器会误以为"车辆正好居中"而
> 继续直行，这在真实车辆上是危险的。本模块在未稳定检出时只更新置信度，
> 由控制器依据置信度与超时来决定停止输出。

## 6. 参数说明

完整参数见 `config/lane_params.yaml`，参数名与代码字段一一对应。
几个关键参数的标定方法：

| 参数 | 含义 | 标定方法 |
|---|---|---|
| `meters_per_pixel_x` | 鸟瞰图一个像素对应的横向实际距离 | 量出鸟瞰图中左右车道线的像素间距，除以真实车道宽（约 3.5 m）。**该值直接决定横向偏差，务必标定** |
| `sobel_auto_percentile` | 自适应阈值分位数 | 越大越严格。误检多则调大，检出率低则调小 |
| `angle_max_deg` | 允许的最大梯度方向角 | 弯道多则调大，直线道路可调小 |
| `bottom_half_width_ratio` | 源梯形下边半宽 | 车辆偏离车道中心时车道线会跑出梯形，偏航大的场景需调大 |
| `min_lane_pixels` | 单侧最少车道像素数 | 图像分辨率或车道线宽度变化时调整 |

`meters_per_pixel_x` 的标定是最容易忽略又最影响结果的一步。若使用俯视相机，
可由相机参数直接推算：

```text
meters_per_pixel = 2 * 相机高度 * tan(水平FOV / 2) / 图像宽度
```

## 7. 实测结果

### 7.1 合成图像（真值已知）

合成图像中车道线位置是解析给定的，因此可以直接算出测量误差：

| 场景 | 车道中心测量误差 |
|---|---|
| 车辆位于车道正中 | **−0.5 px**（−0.025 m） |
| 车辆偏右 100 px（车道线在鸟瞰图中倾斜） | **+0.7 px**（+0.036 m） |
| 叠加高斯噪声 σ=12 | **−0.5 px**（−0.026 m） |

误差均在 **0.1 m 以内**。第二个场景是倾斜车道线，早期版本因几何判据设计缺陷
在此处误差达 25 px，修正后降到 0.7 px。

### 7.2 真实 CARLA 图像

在 CARLA 俯视相机拍摄的真实道路图像上（800×600，0.025 m/像素）：

| 指标 | 数值 | 说明 |
|---|---|---|
| 二值掩码占比 | 1.59% ~ 4.69% | 未标定阈值时该值高达 **42%**（整块路面被误检） |
| 车道线连通域宽高比 | 0.017 ~ 0.066 | 远小于 1，符合车道线的细长形态 |
| 实测车道宽度 | 2.95 m / 3.16 m | 与真实道路标准（约 3.5 m）吻合 |
| 检出率 | 3/3 张检出两侧 | 置信度均为 1.00 |

> **场景说明（如实记录）**：用于验证的是**俯视相机**拍摄的图像，视角为
> 从高处朝下看，因此画面中包含路口横向标线、多车道虚线等复杂元素。
> 其中一张位于路口区域，检出车道宽度为 7.00 m（一个路口跨距），
> 属于异常值。这些图像用于验证**误检抑制能力**（掩码占比从 42% 降到
> 1.6%），而非作为典型车道场景的精度基准；精度基准见 7.1 节。
>
> 之所以采用俯视而非车载前视视角：本机 CARLA 0.9.16 存在字符串导出缺陷
> （见 3.6 节），无法获取标准车道生成点，难以稳定地把车辆放置在车道中央。
> 俯视相机是完全独立的相机 actor，不受该缺陷影响。

### 7.3 正确性校验

本模块的自动化测试覆盖了从像素到物理量的完整链路，共 **84 个测试**：

- 图形变换：Sobel 梯度、方向阈值、颜色阈值
- 单应矩阵：四点映射精度、透视变换往返一致性、越界处理
- 车道像素：直方图定基点、连通域拟合、填充率约束、像素不足时的拒绝
- 几何量：与解析曲率公式对照、横向偏差符号、外推保护
- 端到端：合成图像往返验证、噪声鲁棒性、倾斜车道线不被误杀、
  只检出一侧时的行为
- ROS 节点：图像编解码（含行填充）、参数加载、PID 限幅与符号约定

### 7.4 如何复现以上结果

| 结果 | 复现命令 | 预期输出 |
|---|---|---|
| 7.1 合成图像精度 | `python3 tests/demo_lane_detection.py` | 3/3 场景检出，最大误差 0.70 px |
| 7.3 单元测试 | `python3 -m unittest discover -s tests -v` | `Ran 84 tests ... OK` |
| 结果图片 | 同上（演示脚本会自动生成） | `demo_out/demo_1.png` 等 |

两条命令都**只需要 numpy**，不需要 CARLA、ROS 或 OpenCV，
可在任意 Linux / Windows 机器上复现。详细步骤见 3.4 节。

## 8. 常见问题

**Q: 一直提示"未稳定检出车道线"**

按顺序排查：

1. 确认相机图像正常：`rostopic echo /carla/ego_vehicle/rgb_front/image/height`
2. 用 `rqt_image_view` 看画面，确认**确实拍到车道标线**——
   这是最常见的原因：车辆停在广场/停车场而不是车道上
3. 检查相机俯仰角：太低会只看到天空与远处，太高会看到大量车头
4. 调小 `sobel_auto_percentile` 与 `sobel_min` 提高灵敏度

**Q: 横向偏差的数值明显偏大或偏小**

`meters_per_pixel_x` 没标定。按第 6 节的方法量一次即可。

**Q: 横向偏差符号相反**

本模块约定"正 = 车辆偏右"。若你的相机安装朝向不同，把 `meters_per_pixel_x`
改为负值即可，无需改代码。

**Q: 曲率半径一直是很大的值**

直道时曲率半径理论上是无穷大，本模块发 0 表示直道。若数值在几千到几万米
之间波动，属于正常的数值噪声（多项式二阶导很小）。

## 9. 后续改进方向

- **时序滤波**：当前按帧独立检测，可加入卡尔曼滤波平滑横向偏差
- **车道线跟踪**：用上一帧结果约束下一帧搜索范围，提升遮挡场景的连续性
- **多车道识别**：当前只识别左右两条，可扩展为识别全部车道
- **标定自动化**：用已知车道宽度自动求解 `meters_per_pixel_x`

## 参考文献

- [CARLA 官方文档](https://carla.readthedocs.io/)
- [设置并连接到 Carla 模拟器](../../set_up_and_connect_to_carla.md)
- 车道保持控制见[车道保持控制](./lane_keeping_control.md)
