# MuJoCo 失落公园无人机视觉转向

模块在林荫步道场景中提供确定性无人机运动、前视相机和转向控制接口，场景由 MuJoCo 3.12.0 加载。

## 功能范围

本分支提供 MJCF 场景、确定性运动学仿真、配置校验及统一仿真入口。相机已建模，RGB 采集与神经网络训练接口由后续数据阶段接入。

## 坐标、时间与相机

步道沿世界坐标 (+Y) 延伸，横向坐标为 (X)。正归一化转向量增加航向角，使无人机向左转；负值向右转。指令范围为 `[-1, 1]`，对应 ROS 的正 `angular.z` 左转约定。无人机位置更新使用：

$$
\theta_{t+1}=\theta_t+u_t\omega_{\max}\Delta t,\qquad
x_{t+1}=x_t-v_t\sin\theta_{t+1}\Delta t,\qquad
y_{t+1}=y_t+v_t\cos\theta_{t+1}\Delta t.
$$

`timestep_seconds` 写入 MuJoCo 模型的物理步长，控制周期由 `control_hz` 与物理步数共同确定。基础运动过程没有随机扰动；`seed` 字段用于数据采集与划分阶段的可复现性，不改变基础运动方程。

MJCF 中的 `fovy=78` 表示垂直视场角。以默认 320 × 180 宽高比计算，水平视场角约为 110°：

$$
\mathrm{FOV}_h=2\arctan\left(\frac{W}{H}\tan\frac{\mathrm{FOV}_v}{2}\right).
$$

## 运行环境

- Python 3.10 或更新版本
- MuJoCo 3.12.0

```bash
cd src/air/mujoco_abandoned_park_uav_visual_steering
python -m pip install -r requirements.txt
python main.py --steps 120 --steering 0.0
python -m pytest tests -q
```

配置与场景路径默认相对模块目录解析，运行不依赖调用终端所在目录。
