# Carla 交通标志识别（Windows 端脚本版）

基于 Carla 仿真器 + YOLOv8 神经网络的交通标志识别与车辆控制模块。使用 YOLOv8n 对摄像头画面实时检测交通标志（stop sign），并通过键盘控制车辆。

> 本模块与 `carla_tsr_ros` 互补：`carla_tsr_ros` 是 ROS 节点版（Linux 离线图像驱动），本模块是 Windows 端脚本版（实时连 Carla 相机、键盘控制）。

## 功能

- 在 Carla 仿真环境中生成自车并安装 RGB 摄像头
- 使用 YOLOv8n 神经网络对摄像头画面进行实时目标检测
- 识别交通标志（COCO 类别 11: stop sign）
- 支持键盘控制车辆运动（W/A/S/D/Q）
- OpenCV 实时显示摄像头画面与检测结果

## 运行环境

- **Windows 10/11**（Carla 服务端与客户端均在本机运行）
- Python 3.10.11
- Carla/OpenHUTB 2.9.16
- PyTorch 2.14.0 (CPU)、Torchvision 0.29.0 (CPU)
- Ultralytics 8.4.160
- OpenCV 4.9.0.80、NumPy 1.26.4

## 前置条件

运行前请确认：

1. **Carla 服务端已启动**：Windows 本机运行 `CarlaUE4.exe`，等待地图加载完成（约 30 秒）
2. **Carla 服务端地址**：默认 `127.0.0.1:2000`。如修改端口，运行时加 `--port` 参数
3. **Carla Python API 已安装**：见下方依赖安装
4. **YOLOv8n 权重已就位**：见下方权重说明

## 依赖安装

### 1. 创建虚拟环境

    py -3.10 -m venv D:\WWW\venv_carla310
    D:\WWW\venv_carla310\Scripts\activate

### 2. 安装 Carla Python API

Carla Python API **无法通过 pip 直接下载**，需从本地 Carla 安装目录安装对应版本的 whl：

    pip install D:\WWW\hutb_car_vr_air_mujoco\PythonAPI\carla\dist\hutb-2.9.16-cp310-cp310-win_amd64.whl

### 3. 安装其余依赖

    pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu --no-deps
    pip install ultralytics opencv-python pillow -i https://mirrors.aliyun.com/pypi/simple/

或直接使用 requirements.txt：

    pip install -r requirements.txt -i https://mirrors.aliyun.com/pypi/simple/

## 模型权重

本模块已包含 `yolov8n.pt`（约 6.3MB），无需额外下载。如需重新获取，可从官方发布页下载：

    wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt

下载后放到模块根目录（与 main.py 同级）。程序会先查找模块目录下的权重，找不到时再查找当前工作目录，并给出明确的路径提示。

## 运行步骤

1. **启动 Carla 服务端**（Windows 端）：

       cd D:\WWW\hutb_car_vr_air_mujoco
       .\CarlaUE4.exe

   等待 Carla 窗口出现，地图加载完成。

2. **运行主程序**（Windows 端）：

       python main.py

   或直接双击 `main.bat`。自定义 Carla 地址：

       python main.py --host 192.168.56.1 --port 2000

## 键盘操作

| 按键 | 功能 |
|---|---|
| W | 油门前进 |
| S | 刹车 / 倒车 |
| A | 左转 |
| D | 右转 |
| Q | 退出 |

**重要**：键盘操作需先**点击 OpenCV 窗口**激活焦点，之后按键才会生效。在终端窗口按键无效。

## 神经网络原理

YOLOv8 为单阶段目标检测网络，将目标检测转化为回归问题。损失函数：

$$
L = \lambda_{box} \cdot L_{CIoU} + \lambda_{cls} \cdot L_{BCE} + \lambda_{dfl} \cdot L_{DFL}
$$

- $L_{CIoU}$：边界框回归损失（Complete IoU）
- $L_{BCE}$：分类损失（二元交叉熵）
- $L_{DFL}$：分布焦点损失

## 算法流程

1. Carla 服务端启动，客户端连接 127.0.0.1:2000
2. 生成自车，随机选 spawn 点
3. 挂载 RGB 摄像头（640x360，FOV 90）
4. 摄像头回调：图像转 numpy 数组
5. 每 200ms（按时间限流）执行一次 YOLOv8 推理，仅检测类别 11
6. 检测结果绘制到画面上，OpenCV 窗口显示
7. 键盘输入映射为 carla.VehicleControl
8. 主循环加 20ms 延时控制帧率，按 Q 退出时清理相机和车辆

## 参数配置

所有可调参数集中在 main.py 顶部的 CONFIG 字典：

    CONFIG = {
        'host': '127.0.0.1',
        'port': 2000,
        'timeout': 10.0,
        'camera_width': 640,
        'camera_height': 360,
        'camera_fov': 90,
        'model_filename': 'yolov8n.pt',
        'target_class': 11,
        'infer_interval': 0.2,
        'throttle_step': 0.06,
        'steer_step': 0.10,
        'key_hold_timeout': 0.18,
        'loop_sleep': 0.02,
    }

## 已知限制

- 本模块运行在 Windows 端（Carla 服务端与客户端同机），Ubuntu 端因 Carla 客户端版本不匹配无法直接连接

## 相关模块

- `carla_tsr_ros`：ROS Noetic 节点版，Linux 离线图像驱动

## 性能评价与源码解析

### 安装验证
安装依赖后执行以下命令，确认 OpenCV 可用：

    python -c "import cv2; print(cv2.__version__)"

### 源码解析
- **推理限流**：camera_callback 中使用 time.monotonic() 与 CONFIG['infer_interval']（200ms）比较，保证 YOLO 推理频率稳定在 5Hz。
- **出生点容错**：world.try_spawn_actor() 配合 random.shuffle() 遍历所有出生点，避免被占用点导致崩溃。
- **异常清理**：finally 中每个 destroy() 独立 try/except，确保某一步失败不影响后续资源释放。

### 性能说明
- 本模块为 Windows 端脚本，直接连接 CARLA 实时相机，使用 cv2.imshow 显示画面，并通过 vehicle.apply_control() 直接控制车辆；没有 ROS 话题。
- 由于缺少带标注的测试集，无法提供 mAP 等检测指标。
- CPU 推理延迟参考：YOLOv8n 单帧约 50ms（5Hz 限流下稳定运行）。
