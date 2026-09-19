# CarlaAir 神经网络感知与轨迹控制

本文基于 CarlaAir v0.1.7，实现无人机 **RGB 视觉目标检测** 与 **MLP 神经网络给定轨迹控制**。实验包括两个部分：

1. 使用无人机 RGB 相机获取图像，并使用 YOLO11n 神经网络进行实时目标检测；
2. 使用 MLP 神经网络根据目标位置误差和无人机运动状态生成速度控制量，实现正方形、圆形和八字形轨迹跟踪。

---

## 1. 实验环境

| 项目 | 配置 |
| --- | --- |
| 操作系统 | Windows 11 |
| 仿真平台 | CarlaAir v0.1.7 |
| 仿真接口 | AirSim Python API |
| Python | Python 3.10 |
| 深度学习框架 | PyTorch |
| 目标检测 | Ultralytics YOLO11n |
| GPU | NVIDIA GeForce RTX 5060 |
| 图像处理 | OpenCV |
| 数值计算 | NumPy |
| 数据分析 | Pandas / Matplotlib |

CarlaAir 在 Windows 原生环境运行，AirSim RPC 默认使用端口：

```text
41451
```

实验代码位于：

```text
src/air/carlaair_yolo_mlp_control/
```

文档中使用的 GIF 和实验结果图位于：

```text
docs/air/images/carlaair_yolo_mlp_control/
```

---

## 2. 系统总体结构

任务 2 分为神经网络感知和神经网络控制两部分。

### 2.1 神经网络感知

```text
CarlaAir
   ↓
无人机 RGB Camera
   ↓
AirSim simGetImages()
   ↓
OpenCV 图像解码
   ↓
YOLO11n 神经网络
   ↓
目标类别 + 检测框 + 置信度
   ↓
实时显示 + 性能统计
```

### 2.2 神经网络轨迹控制

```text
给定轨迹
正方形 / 圆形 / 八字形
        ↓
目标位置
        ↓
位置误差 + 当前速度
        ↓
MLP 神经网络
        ↓
vx_cmd / vy_cmd / vz_cmd
        ↓
AirSim API
        ↓
CarlaAir 无人机
```

---

# 3. RGB 相机与 YOLO 神经网络感知

## 3.1 AirSim RGB 图像获取

CarlaAir 中的无人机通过 AirSim Python API 获取 RGB 图像。

程序首先连接 AirSim：

```python
client = airsim.MultirotorClient(
    ip="127.0.0.1",
    port=41451
)

client.confirmConnection()
```

然后调用 `simGetImages()` 请求无人机 RGB 图像：

```python
responses = client.simGetImages([
    airsim.ImageRequest(
        "0",
        airsim.ImageType.Scene,
        False,
        True
    )
])
```

AirSim 返回压缩图像数据后，通过 NumPy 和 OpenCV 进行解码：

```python
image_data = np.frombuffer(
    response.image_data_uint8,
    dtype=np.uint8
)

frame = cv2.imdecode(
    image_data,
    cv2.IMREAD_COLOR
)
```

---

## 3.2 YOLO11n 神经网络

目标检测采用 Ultralytics YOLO11n。

加载模型：

```python
from ultralytics import YOLO

model = YOLO("yolo11n.pt")
```

当 CUDA 可用时，程序自动使用 NVIDIA GPU：

```python
if torch.cuda.is_available():
    device = 0
else:
    device = "cpu"
```

YOLO 推理：

```python
results = model.predict(
    source=frame,
    conf=0.20,
    imgsz=640,
    device=device,
    verbose=False
)
```

模型会输出目标类别、边界框与置信度。

实验中主要关注的目标包括：

```text
person
car
bus
truck
```

---

## 3.3 实时目标检测程序

首先启动 CarlaAir。

例如在 Windows 命令行中：

```bat
cd /d D:\CarlaAirDownload\CarlaAir-v0.1.7-Windows11-x86_64
conda activate carlaAir
StartCarlaAir.bat Town10HD --no-traffic
```

保持 CarlaAir 窗口运行，再打开一个新的命令行窗口，进入 YOLO 环境：

```bat
conda activate carlaAirYolo
```

进入源码目录：

```bat
cd /d src\air\carlaair_yolo_mlp_control
```

运行：

```bat
python yolo_realtime.py
```

程序启动后会显示实时检测窗口，并显示：

```text
FPS
YOLO inference time
AirSim image capture time
Object count
GPU information
```

无人机可以继续在 CarlaAir 窗口中使用键盘人工控制：

```text
W / A / S / D   前后左右移动
Space           上升
Left Shift      下降
Mouse           调整方向
```

YOLO 程序只负责从 AirSim 获取 RGB 图像并进行神经网络感知，不接管 CarlaAir 的人工飞行控制。

---

## 3.4 实时检测效果

下面为 CarlaAir 无人机飞行过程中进行 YOLO 实时目标检测的演示。

![CarlaAir YOLO realtime detection](./images/carlaair_yolo_mlp_control/yolo_realtime.gif)

单帧测试中，YOLO11n 能够正常使用 CUDA 推理并输出目标检测结果。例如一次测试中检测到 2 个目标，YOLO 推理时间约为 41.36 ms。

实际实时检测帧率还会受到 CarlaAir 场景渲染、AirSim 图像获取与图像传输时间的影响。

---

## 3.5 感知性能评价

实时程序记录以下指标：

```text
FPS
YOLO 推理时间
AirSim 图像获取时间
每帧检测目标数量
```

运行数据默认保存为：

```text
output/results.csv
```

该 CSV 属于程序运行生成结果，因此不提交到代码仓库。

实验中可以通过以下指标评价系统性能：

| 指标 | 含义 |
| --- | --- |
| FPS | 实时检测系统整体帧率 |
| YOLO inference time | 神经网络单帧推理时间 |
| AirSim capture time | AirSim 获取 RGB 图像所需时间 |
| Object count | 每帧检测出的目标数量 |

---

# 4. MLP 神经网络轨迹控制

## 4.1 控制目标

第二部分要求无人机根据给定轨迹自动飞行。

本实验测试三种轨迹：

```text
正方形
圆形
八字形
```

运行阶段由训练好的 MLP 神经网络输出无人机速度控制量。

---

## 4.2 MLP 输入与输出

MLP 输入为 6 维状态：

```text
ex
ey
ez
vx
vy
vz
```

其中：

```text
ex = target_x - current_x
ey = target_y - current_y
ez = target_z - current_z
```

`vx`、`vy`、`vz` 为无人机当前速度。

因此网络输入为：

```text
[ex, ey, ez, vx, vy, vz]
```

MLP 输出 3 个速度控制量：

```text
[vx_cmd, vy_cmd, vz_cmd]
```

控制指令通过 AirSim API 发送给 CarlaAir：

```python
client.moveByVelocityAsync(
    vx_cmd,
    vy_cmd,
    vz_cmd,
    duration
)
```

---

## 4.3 MLP 网络结构

本实验采用如下 MLP：

```text
Input: 6

6
↓
Linear(6, 64)
↓
ReLU
↓
Linear(64, 64)
↓
ReLU
↓
Linear(64, 32)
↓
ReLU
↓
Linear(32, 3)
↓
Tanh

Output: 3
```

该网络规模较小，适合实时输出无人机速度控制量。

---

## 4.4 训练数据生成

为了降低神经网络控制训练难度，本实验首先使用 PD 专家控制器生成训练数据。

PD 控制器根据：

```text
目标位置误差
+
当前运动速度
```

生成：

```text
vx_cmd
vy_cmd
vz_cmd
```

然后将输入状态和专家控制输出保存为监督学习样本。

训练流程：

```text
随机位置误差和速度状态
        ↓
PD 专家控制器
        ↓
生成监督学习数据
        ↓
MLP 神经网络训练
        ↓
mlp_controller.pth
```

生成训练数据：

```bash
python generate_training_data.py
```

程序会生成：

```text
controller_dataset.csv
```

该文件可以通过程序重新生成，因此不提交到代码仓库。

---

## 4.5 MLP 模型训练

运行：

```bash
python train_mlp.py
```

程序使用 PyTorch 训练 MLP。

如果 CUDA 可用，则自动使用 NVIDIA GPU。

训练完成后生成：

```text
mlp_controller.pth
```

模型文件可以通过源码重新训练生成，因此不直接提交到代码仓库。

---

# 5. 给定轨迹控制实验

## 5.1 正方形轨迹

运行：

```bash
python run_mlp_trajectory.py --trajectory square
```

目标轨迹与 MLP 实际轨迹如下：

![Square trajectory](./images/carlaair_yolo_mlp_control/square_xy.png)

轨迹跟踪误差：

![Square tracking error](./images/carlaair_yolo_mlp_control/square_error.png)

正方形轨迹包含多个 90° 急转弯，对控制器的瞬态响应要求较高。

从实验结果可以看出，转角位置的跟踪偏差较明显。

---

## 5.2 圆形轨迹

运行：

```bash
python run_mlp_trajectory.py --trajectory circle
```

目标轨迹与实际轨迹：

![Circle trajectory](./images/carlaair_yolo_mlp_control/circle_xy.png)

轨迹跟踪误差：

![Circle tracking error](./images/carlaair_yolo_mlp_control/circle_error.png)

圆形轨迹连续、平滑，相比正方形轨迹更适合当前 MLP 控制器进行连续跟踪。

---

## 5.3 八字形轨迹

运行：

```bash
python run_mlp_trajectory.py --trajectory eight
```

目标轨迹与实际轨迹：

![Figure-eight trajectory](./images/carlaair_yolo_mlp_control/eight_xy.png)

轨迹跟踪误差：

![Figure-eight tracking error](./images/carlaair_yolo_mlp_control/eight_error.png)

八字形轨迹包含持续的方向变化，可用于验证神经网络控制器对复杂连续轨迹的跟踪能力。

---

# 6. 轨迹控制性能评价

运行：

```bash
python evaluate_trajectories.py
```

程序会计算：

```text
RMSE
平均轨迹误差
最大轨迹误差
最终位置误差
完成时间
```

实验结果如下：

| 轨迹 | RMSE / m | 平均误差 / m | 最大误差 / m | 最终误差 / m | 完成时间 / s |
| --- | ---: | ---: | ---: | ---: | ---: |
| 正方形 | 6.632 | 5.845 | 10.068 | 1.029 | 14.95 |
| 圆形 | 1.660 | 1.377 | 8.000 | 0.906 | 70.30 |
| 八字形 | 1.229 | 1.202 | 1.895 | 0.828 | 82.50 |

从实验结果可以看出，MLP 控制器对连续轨迹的跟踪效果明显好于包含急转弯的正方形轨迹。

八字形轨迹的 RMSE 为 1.229 m，在三种轨迹中最低；圆形轨迹的 RMSE 为 1.660 m，也能保持较稳定的连续跟踪。

正方形轨迹的 RMSE 为 6.632 m，明显高于圆形和八字形轨迹。主要原因是正方形轨迹包含多个 90° 急转弯，目标运动方向在航点附近发生突变，而当前 MLP 仅使用位置误差和当前速度作为输入，因此容易在急转弯位置产生较大的瞬态误差。

后续可以通过以下方法进一步改善：

- 增加航向误差作为网络输入；
- 增加目标轨迹切线方向或下一航点信息；
- 增加正方形转角附近的训练样本；
- 对目标轨迹进行圆角或样条平滑；
- 使用更复杂的时序网络或强化学习控制器。

---

# 7. 源代码结构

本实验代码位于：

```text
src/air/carlaair_yolo_mlp_control/
```

目录结构：

```text
carlaair_yolo_mlp_control/
├── main.py
├── yolo_realtime.py
├── generate_training_data.py
├── train_mlp.py
├── run_mlp_trajectory.py
└── evaluate_trajectories.py
```

各文件作用如下：

| 文件 | 功能 |
| --- | --- |
| `main.py` | 示例统一入口 |
| `yolo_realtime.py` | CarlaAir RGB + YOLO 实时目标检测 |
| `generate_training_data.py` | 生成 MLP 监督训练数据 |
| `train_mlp.py` | 训练 MLP 神经网络控制器 |
| `run_mlp_trajectory.py` | 运行正方形、圆形、八字形轨迹控制 |
| `evaluate_trajectories.py` | 计算轨迹误差并绘制结果 |

模型权重、训练数据和程序自动生成的 CSV 结果未提交到仓库，可以通过源码重新生成。

---

# 8. 完整运行流程

## 8.1 YOLO 实时目标检测

首先启动 CarlaAir。

然后进入源码目录：

```bash
cd src/air/carlaair_yolo_mlp_control
```

运行：

```bash
python yolo_realtime.py
```

---

## 8.2 生成 MLP 训练数据

```bash
python generate_training_data.py
```

---

## 8.3 训练 MLP

```bash
python train_mlp.py
```

---

## 8.4 正方形轨迹

```bash
python run_mlp_trajectory.py --trajectory square
```

---

## 8.5 圆形轨迹

```bash
python run_mlp_trajectory.py --trajectory circle
```

---

## 8.6 八字形轨迹

```bash
python run_mlp_trajectory.py --trajectory eight
```

---

## 8.7 性能评价

```bash
python evaluate_trajectories.py
```

---

# 9. 使用统一入口运行

如果使用 `main.py`，也可以通过统一入口运行不同功能。

YOLO：

```bash
python main.py yolo
```

生成训练数据：

```bash
python main.py generate
```

训练 MLP：

```bash
python main.py train
```

正方形：

```bash
python main.py square
```

圆形：

```bash
python main.py circle
```

八字形：

```bash
python main.py eight
```

性能评价：

```bash
python main.py evaluate
```

---

# 10. 注意事项

1. 运行程序前必须先启动 CarlaAir。
2. AirSim RPC 默认使用 `41451` 端口。
3. YOLO 实时检测时，无人机仍使用 CarlaAir 窗口中的键盘和鼠标进行人工控制。
4. MLP 轨迹控制实验运行时，不要同时启动其他会接管 AirSim 无人机控制权的程序。
5. `yolo11n.pt`、`mlp_controller.pth`、训练数据 CSV 和运行结果 CSV 均属于可下载或可重新生成文件，因此不提交到仓库。
6. 提交文档使用的 GIF 应控制在仓库要求的大小范围内，并使用英文文件名。
7. 图片和 GIF 文件名不要包含空格或中文字符。

---

# 11. 实验总结

本文在 CarlaAir 仿真环境中完成了无人机神经网络感知和神经网络轨迹控制实验。

在视觉感知部分，通过 AirSim 获取无人机 RGB 摄像头图像，并使用 YOLO11n 神经网络完成实时目标检测。实验验证了 YOLO 模型能够在 CarlaAir 场景中进行实时神经网络感知，并能够利用 NVIDIA GPU 进行 CUDA 推理。

在轨迹控制部分，通过 PD 专家控制器生成监督学习数据，训练 MLP 神经网络。在实际轨迹跟踪过程中，由训练完成的 MLP 根据位置误差和无人机当前速度输出三轴速度控制指令。

实验结果表明，MLP 对圆形和八字形等连续轨迹具有较好的跟踪效果；正方形轨迹由于存在急转弯，轨迹跟踪误差较大。后续可以通过增加航向信息、改善训练数据分布和进行轨迹平滑进一步提高控制精度。

---

# 12. 大模型使用声明

本文档撰写、部分程序整理、环境故障排查和代码重构过程中使用了 ChatGPT 辅助。

所有程序均由提交者在 CarlaAir v0.1.7 环境中实际运行、调试和结果验证。

提交者对本文档及代码内容负责。

---

# 13. 参考资料

- [CarlaAir](https://openhutb.github.io/air_doc/)
- [Microsoft AirSim](https://github.com/microsoft/AirSim)
- [Ultralytics YOLO](https://docs.ultralytics.com/)
- [PyTorch](https://pytorch.org/)
- [OpenCV](https://opencv.org/)
