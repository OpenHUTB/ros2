# 车载端到端仿真作业

本项目基于 Python 3.13.15 在 Windows 11 上实现了车辆的端到端感知、决策与控制仿真。

## 运行说明
1. 安装依赖：`pip install pygame numpy`
2. 运行仿真：`python main.py`
3. 演示动图：请用ScreenToGif录制后放入 `demo.gif`

## 算法原理
### 1. 感知算法 (Perception)
输入为 100x100 的模拟相机灰度图像矩阵。通过阈值分割 `mask = image > 200` 提取障碍物像素，计算最近的障碍物像素坐标。利用公式 \(D = (H - y_{max}) / H 	imes 50\) 将像素坐标映射为物理距离。

### 2. 控制算法 (Control)
采用 PID 控制器进行横向控制。公式为：
\(u(t) = K_p e(t) + K_i \int e(t)dt + K_d rac{de(t)}{dt}\)
根据感知到的横向偏差，计算出期望的方向盘转角。

### 3. 车辆动力学模型 (Physics)
采用自行车运动学模型：
\(\dot{x} = v \cos(	heta)\)
\(\dot{y} = v \sin(	heta)\)
\(\dot{	heta} = rac{v}{L} 	an(\delta)\)
