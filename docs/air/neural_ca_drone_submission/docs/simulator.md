# CA-Sim 仿真器

CA-Sim 是一个约 900 行的轻量 3D 仿真器，只用 NumPy 做几何、Pillow 做渲染。
它不依赖 Gazebo/Unity/AirSim，因此在任何装了 Python 的机器上都能跑；
同时它的接口（深度图、里程计、速度指令）与 ROS 侧完全一致，
同一套算法可以无缝迁移到 Gazebo（见 [Gazebo 仿真步骤](gazebo.md)）。

## 1. 世界与障碍物

世界是 \(52\ \text{m}\times44\ \text{m}\times12\ \text{m}\) 的矩形空域，
飞行高度固定 \(z=3\ \text{m}\)。障碍物有两种解析几何体：

- **长方体** `Box(x, y, z0, sx, sy, sz)`：中心 \((x,y)\)，底面高度 \(z_0\)，
  三轴尺寸 \((s_x,s_y,s_z)\)；
- **圆柱** `Cylinder(x, y, z0, radius, height)`：竖直圆柱。

动态障碍由 `Mover(base, axis, amplitude, omega, phase)` 描述，位置随时间正弦运动：

\[
\mathbf{c}(t) = \mathbf{c}_0 + A\sin(\omega t + \varphi)\,\hat{\mathbf{e}}_{\text{axis}}
\]

`World.t` 由仿真循环每一步同步为无人机时间 \(t_k=k\Delta t\)，因此动障碍真正移动。

## 2. 深度相机模型

相机与机体同向，水平视场 \(\mathrm{FOV}=90^\circ\)，分辨率 \(W\times H=64\times48\)。
像素 \((u,v)\) 的归一化方向：

\[
d_x = 1,\qquad
d_y = \left(u+\tfrac12-\tfrac{W}{2}\right)\frac{2\tan(\mathrm{FOV}/2)}{W},\qquad
d_z = -\left(v+\tfrac12-\tfrac{H}{2}\right)\frac{2\tan(\mathrm{FOV}/2)}{W}
\]

再旋转到世界系（\(\psi\) 为偏航）：

\[
\mathbf{d}_w =
\begin{bmatrix}
\cos\psi & -\sin\psi & 0\\
\sin\psi & \cos\psi & 0\\
0 & 0 & 1
\end{bmatrix}
\begin{bmatrix} d_x\\ d_y\\ d_z \end{bmatrix},
\qquad \hat{\mathbf{d}}_w=\frac{\mathbf{d}_w}{\|\mathbf{d}_w\|}
\]

对每条射线求最近交点距离，截断到 \(R_{\max}=20\ \text{m}\)（训练时用 12 m），
得到深度图 \(D\in\mathbb{R}^{H\times W}\) 与物体 id 图（用于生成真值 mask）。

### 2.1 射线—AABB（slab 法）

对轴对齐包围盒 \([b_{\min},b_{\max}]\)，逐轴计算进入/离开参数：

\[
t_1=\frac{b_{\min}-o}{d},\quad t_2=\frac{b_{\max}-o}{d},\qquad
t_{\text{in}}=\max(\min(t_1,t_2)),\quad
t_{\text{out}}=\min(\max(t_1,t_2))
\]

若 \(t_{\text{out}}\ge\max(t_{\text{in}},0)\) 则命中，距离为 \(t_{\text{in}}\)（若 \(t_{\text{in}}<0\) 取 \(t_{\text{out}}\)）。
实现时对三个轴向量化，一次处理全部像素。

### 2.2 射线—圆柱

射线 \(\mathbf{o}+t\mathbf{d}\) 与竖直圆柱 \((x-c_x)^2+(y-c_y)^2=r^2\) 相交：

\[
a=d_x^2+d_y^2,\quad
b=2\left(d_x(o_x-c_x)+d_y(o_y-c_y)\right),\quad
c=(o_x-c_x)^2+(o_y-c_y)^2-r^2
\]

解 \(at^2+bt+c=0\)，取最小正根，再检查交点高度是否落在 \([z_0,z_0+h]\) 内。
地面上方 \(z\le 0\) 的命中记为地面（id=-1）。

## 3. 无人机动力学

状态 \((\mathbf{p},\mathbf{v},\psi)\)，控制周期 \(\Delta t=0.1\ \text{s}\)。
速度指令 \(\mathbf{u}=(v_x,v_y)\) 在机体系，先转到世界系：

\[
\mathbf{v}^{\text{cmd}} =
\begin{bmatrix}\cos\psi & -\sin\psi\\ \sin\psi & \cos\psi\end{bmatrix}\mathbf{u}
\]

一阶惯性跟踪（时间常数 \(\tau=0.28\ \text{s}\)）：

\[
\mathbf{v}_{k+1}=\mathbf{v}_k+\frac{\Delta t}{\tau}\left(\mathbf{v}^{\text{cmd}}_k-\mathbf{v}_k\right),
\qquad
\mathbf{p}_{k+1}=\mathbf{p}_k+\mathbf{v}_{k+1}\Delta t
\]

偏航角以 \(\omega_{\max}=2.4\ \text{rad/s}\) 跟随速度方向：

\[
\psi_{k+1}=\psi_k+\mathrm{clip}\left(\mathrm{wrap}(\psi_v-\psi_k),\,
-\omega_{\max}\Delta t,\,\omega_{\max}\Delta t\right)
\]

其中 \(\psi_v=\mathrm{atan2}(v_y,v_x)\)。这一阶滞后是“切角”问题的来源：
无人机不会瞬间对准新速度方向，转向半径约 \(v/\omega_{\max}\approx0.5\ \text{m}\)。

`Drone` 同时累计：

- `path_length`：相邻位置欧氏距离之和；
- `min_clearance`：每步到最近障碍表面的距离最小值；
- `max_accel`：相邻速度差 / \(\Delta t\)。

碰撞判定：`clearance < radius=0.32 m` 或飞出世界边界。

## 4. 渲染：画家算法

没有 OpenGL，3D 视图用**画家算法**（由远及近绘制）实现：

1. 相机位于无人机后上方 `pos - fwd*8 + (0,0,2.9)`，俯角约 8°；
2. 每个长方体投影 8 个顶点到相机平面，按深度排序后填充 6 个可见面，
   面颜色按法向做简单明暗；
3. 圆柱用上下两个椭圆 + 侧面四边形近似；
4. 地面画网格线，天空用垂直线性渐变；
5. 路径、目标、无人机、HUD 最后叠加。

深度图与彩色图共享同一套几何，因此“看到的”和“深度图里的”严格一致。

## 5. 画面布局

`compose_frame` 输出 960×600 的帧，GIF 会缩放到 640×400：

| 区域 | 尺寸 | 内容 |
| --- | --- | --- |
| 上方 | 960×424 | 追尾 3D 视图：障碍、路径、目标、无人机、事件日志 |
| 左下 | 250×162 | 机载 RGB-D（深度伪彩色） |
| 中下 | 250×162 | BEV 局部地图（20 m×20 m） |
| 右下 | 460×162 | 遥测：速度、间隙、位置、进度、规划/推理耗时、重规划次数 |

事件日志会显示 `replan: obstacle 3.2 m`、`NN occupancy 96x72 -> 30x30`、
`clearance guard active` 等信息，便于答辩时解释系统在做什么。

## 6. 性能

在 4 核笔记本上：

| 操作 | 耗时 |
| --- | ---: |
| 单帧 960×600 渲染 | 45–60 ms |
| 64×48 深度图 | 6–8 ms |
| 一次回合（435 步，含渲染每 4 步） | 4–5 s |
| 纯仿真无渲染（435 步） | 1.5 s |

## 7. 代码位置

| 文件 | 内容 |
| --- | --- |
| `ca_drone/sim/world.py` | `Box`、`Cylinder`、`Mover`、`World`、`raycast`、五个场景 |
| `ca_drone/sim/drone.py` | 动力学、碰撞、间隙统计 |
| `ca_drone/render/scene.py` | 相机、投影、深度图、HUD、`compose_frame` |
