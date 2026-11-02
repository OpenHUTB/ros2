# Town10 幕墙图像病害识别

模块接收 OpenHUTB 巡航相机图像，识别裂缝、污渍、剥落与玻璃构件，输出带框图像及 JSON。
巡航采集模块通过 `/town10/image_raw` 或 PNG/JPEG 文件接入；识别模块可独立运行。
玻璃构件用于定位立面结构，**不计入病害数量**。

## 原生 Town10 巡航图像

OpenHUTB Town10 在线巡航完成 12 个航点，采集的 12 张相机图像全部通过本模块识别。
置信度阈值 0.25 时，模型输出 113 个玻璃构件框和 2 个污渍候选框，未输出裂缝或剥落候选。
这些在线图像未做人工病害标注，候选数量仅描述模型输出，不代表已确认的病害数量或识别精度。

![原生 Town10 巡航图像与模型输出](../../../docs/img/air/town10_facade_defect_detection/native_comparison.png)

![原生 Town10 巡航照片识别序列](../../../docs/img/air/town10_facade_defect_detection/native_detection_sequence.gif)

动图依次展示 12 个航点照片的识别结果；完整结果见
`docs/air/town10_facade_defect_detection/native_detections.json`。

在 `src/air/town10_facade_cruise_capture` 运行已对准墙面悬停的原生采集：

```bash
python main.py --backend airsim --host 127.0.0.1 --camera front_cam --speed 0.5 --duration 150 --gif --output output/native
```

再进入本模块目录处理该采集目录：

```bash
python main.py --input ../town10_facade_cruise_capture/output/native --output output/native_detection
```

`--input` 直接接受巡航保存的 `capture_*.png`，保留文件名以对应采集清单中的位姿。

## 标注验证图像

![原图与检测结果](../../../docs/img/air/town10_facade_defect_detection/comparison.png)

![六张验证图像识别序列](../../../docs/img/air/town10_facade_defect_detection/detection_sequence.gif)

动图逐张展示验证图像的识别结果。

## 独立运行

Python 3.10 及以上，运行依赖为 NumPy 和 OpenCV。CPU 推理直接使用随包 ONNX 模型。

```bash
python -m pip install -r requirements.txt
python main.py --input samples --output output
```

Windows 可使用同样的 Python 命令。无图形界面的环境可安装
`opencv-python-headless>=4.8,<5` 代替 `opencv-python`。

输出：

- `output/annotated/*.png`：原尺寸带框图像。
- `output/detections.json`：文件名、尺寸、病害数量与每个框的类别、置信度、`xyxy` 像素坐标。

```json
{"class_id": 1, "class_name": "stain", "confidence": 0.54,
 "xyxy": [48.0, 120.0, 110.0, 138.0], "is_defect": true}
```

上面的坐标说明输出字段格式，具体结果见 `docs/air/town10_facade_defect_detection/sample_detections.json`。
可通过 `--confidence 0.25` 和 `--nms-iou 0.45` 调整后处理阈值。

## ROS 2 Humble

将本目录放入 ROS 工作区 `src`，安装 `python3-numpy` 及 OpenCV 4.8 以上，再执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select town10_facade_defect_detection
source install/setup.bash
ros2 launch town10_facade_defect_detection detect.launch.py output:=/tmp/town10_detections
```

默认 `mode:=samples` 处理随包六张图片并正常退出。
接入巡航图像：

```bash
ros2 launch town10_facade_defect_detection detect.launch.py mode:=live input_topic:=/town10/image_raw
```

| 方向 | 话题 | 消息类型 |
|---|---|---|
| 输入 | `/town10/image_raw` | `sensor_msgs/Image` |
| 输出 | `/town10/defects/image` | `sensor_msgs/Image`，`bgr8` |
| 输出 | `/town10/defects/detections` | `std_msgs/String`，JSON |

输入支持 `bgr8`、`rgb8`、`bgra8`、`rgba8`、`mono8`，按 `step` 处理行补齐字节。
输出保留输入的时间戳和相机坐标系，图像订阅采用传感器数据 QoS。

Humble 接口验证使用 640×360 的 `rgb8` 图像、1927 字节输入行步幅，
收到 1920 字节行步幅的 `bgr8` 带框图像与 JSON，原时间戳和相机坐标系保持一致。
该样本得到 13 个候选框，其中病害 9 个、玻璃构件 4 个。

## 数据与评价

模型来自补充的幕墙巡检模型包，来源和 SHA-256 见 `models/facade_4class.json`。
模型元数据确认四类顺序为 `crack/stain/peel/glass`，输入 `1×3×640×640`，输出 `1×8×8400`。
完整人工标注包含 132 张图像、1976 个框；训练数据由 106 张原图和 104 张合成增强副本组成，
验证包含 26 张图像、482 个框。随包仅保留六张验证样本及来源索引。

使用完整外部数据目录重新评价：

```bash
python evaluate.py --dataset /path/to/dataset_final --output evaluation
```

评价使用每类独立 NMS、IoU=0.5 匹配、101 点插值 AP；AP50–95 取 IoU 0.50 至 0.95、步长 0.05 的均值。
AP 评价保留置信度 ≥0.001 的预测，最多 300 框/图；固定工作点的 Precision、Recall 使用置信度 0.25。

| 类别 | 验证框 | AP50 | Precision | Recall |
|---|---:|---:|---:|---:|
| 裂缝 crack | 19 | 0.0050 | 0.0000 | 0.0000 |
| 污渍 stain | 147 | 0.4200 | 0.5268 | 0.4014 |
| 剥落 peel | 124 | 0.1662 | 0.6667 | 0.0806 |
| 玻璃 glass | 192 | 0.9460 | 0.8402 | 0.9583 |

四类 mAP50 为 **0.3843**，mAP50–95 为 **0.1728**。
只统计三类病害时，微平均 Precision 为 **0.5433**、Recall 为 **0.2379**。
因此当前模型适合提供可复查的候选区域，裂缝和剥落仍存在明显漏检。
验证按图像划分，相邻采集帧可能同时进入训练与验证，以上结果不代表严格跨建筑泛化性能。
机器可读指标、验证图像/标签摘要见 `docs/air/town10_facade_defect_detection/`。

## 训练

训练入口使用外部数据目录，不修改随包模型：

```bash
python -m pip install -r requirements-training.txt
python train.py --dataset /path/to/dataset_final --output training --epochs 60 --batch 8 --device cpu --export
```

数据目录须包含 `images/train`、`images/val`、`labels/train`、`labels/val`。
程序建立便携数据索引，训练 YOLOv8n 并按 `--export` 导出 ONNX；
`--weights` 可指定已有初始权重。训练与验证应优先按建筑或采集序列隔离，
合成增强仅用于训练，新的评估记录应与相应权重共同保存。

## 许可

代码及随包模型按 AGPL-3.0 分发，完整文本见 `LICENSE`。
模型原始来源与导出版本见 `models/README.md`。
