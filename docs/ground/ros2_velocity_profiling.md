# ROS 2 路径速度规划（曲率/加速度约束）

A\* / 样条给出几何路径，差速底盘还需要每点的速度。本模块按**带符号曲率**和
加速度约束生成速度剖面，输出 $(v,w)$：左转 `w>0`、右转 `w<0`，起终点按参数停车。

## 1. 方法

### 1.1 带符号严格曲率

对相邻三点 $(p_{i-1},p_i,p_{i+1})$，令 $a=p_i-p_{i-1}$、$b=p_{i+1}-p_i$：

$$
\kappa=\frac{2\,(a\times b)}{|a|\,|b|\,|a+b|}
$$

* 左转叉积为正，$\kappa>0$；右转 $\kappa<0$；
* $a$ 与 $b$ 反向（180° 折返）时分母趋 0，单独取 $\kappa=\pi/\Delta s$，
  不会像叉积那样变成 0；
* 与“叉积除以平均弧长三次方”的近似式不同，这里在 90° 角不会低估。

### 1.2 曲率限速与加速度扫描

$$
v_\kappa=\sqrt{\frac{0.9\,a_{lat,max}}{|\kappa|}},\qquad
v_i=\min(v_{max},v_\kappa)
$$

10% 余量用于抵消离散/数值差分误差。随后：

$$
v_i\le\sqrt{v_{i-1}^2+2a_{max}\Delta s},\qquad
v_i\le\sqrt{v_{i+1}^2+2a_{max}\Delta s}
$$

### 1.3 起终点与输出

`v[0]=v_start`、`v[-1]=v_end`（默认 0），端点不再被 `min_speed` 抬高；
$w_i=\kappa_i v_i$ 带符号输出。

### 1.4 参数校验

`v_max/a_max/a_lat_max > 0`、`0 <= min_speed <= v_max`、
`0 <= v_start,v_end <= v_max`、路径至少 2 点；不满足抛 `ValueError`，节点捕获后跳过。

## 2. 运行

```bash
cd ~/hutb_ws
colcon build --packages-select ros2_velocity_profiling
source install/setup.bash
ros2 launch ros2_velocity_profiling main_launch.py
```

| 方向 | 话题 | 类型 |
| --- | --- | --- |
| 订阅 | `/plan` | `nav_msgs/Path` |
| 发布 | `/cmd_profile` | `std_msgs/Float64MultiArray`（`v0,w0,v1,w1,...`） |

参数在 `config/profile_params.yaml`：`v_max`、`a_max`、`a_lat_max`、`min_speed`。

## 3. 离线复现

```bash
cd src/ground/ros2_velocity_profiling
python3 -m unittest discover -s test -v   # 9 项
python3 -m ros2_velocity_profiling.sim
```

## 4. 结果

`v_max=1.0 m/s`、`a_max=0.8 m/s²`、`a_lat_max=0.5 m/s²`。横向加速度由
**速度矢量数值差分**独立计算（不是限速用的 κ）：

| 路径 | 长度 | 恒速时间 | 规划时间 | 恒速最大横向加速度 | 规划最大横向加速度 |
| --- | ---: | ---: | ---: | ---: | ---: |
| circle (r=1.2) | 7.54 m | 7.54 s | 11.18 s | 0.833 | **0.450** |
| slalom | 14.34 m | 14.34 s | 18.73 s | 1.893 | **0.457** |
| rounded square | 18.63 m | 18.63 s | 23.44 s | 1.250 | **0.474** |

- 圆上恒速已超限 1.67 倍，slalom 超 3.8 倍；
- 规划后三条都 ≤0.5，时间增加 26%–48%；
- rounded square 用 0.8 m 圆角替代瞬时直角，避免“尖角处无穷曲率”这种
  不可跟踪的伪场景。

![速度剖面](../img/ground/ros2_velocity_profiling/speed_profile.png)

## 5. 与规划器/控制器的关系

* 输入任意 `nav_msgs/Path`，可接 #1077 的 A\* 或 #1288 的平滑路径；
* 输出逐点 `(v,w)`，可作为 Pure Pursuit/Stanley 的前馈；
* `w` 带符号，右转不会再被下发成左转。

## 6. 局限

* 只做速度剖面，不改变路径几何；
* 离散三点曲率在 1–2 点的尖角仍可能偏大；
* 未考虑轮速/力矩饱和，真机需再限幅；
* $w=\kappa v$ 假设纯滚动、无侧滑；
* 输出 `Float64MultiArray` 无 header，需与输入 Path 索引对齐；
* 只支持 2D 路径。

## 7. 大模型声明

代码与文档在编写过程中使用了 AI 大模型辅助。作者已实际运行 9 项单元测试和
3 条离线路径，并在 Ubuntu 20.04 + Foxy 下完成 `colcon build`；对提交内容的
正确性与完整性负全部责任。
