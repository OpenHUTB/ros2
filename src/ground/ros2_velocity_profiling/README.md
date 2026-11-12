# ROS 2 路径速度规划（曲率/加速度约束）

给定 `nav_msgs/Path`，按**带符号曲率**和加速度限制生成速度剖面，输出每个点的
`(v, w)`：左转 `w>0`、右转 `w<0`，起终点按 `v_start`/`v_end` 停车。

## 1. 方法

1. **带符号严格曲率**：三点外接圆公式
   $$\kappa=\frac{2(a\times b)}{|a||b||a+b|}$$
   左转正、右转负；180° 折返单独按大曲率处理，不会当成直线；
2. **曲率限速**：$v_\kappa=\sqrt{0.9a_{lat,max}/|\kappa|}$（留 10% 余量，
   抵消离散化误差）；
3. **前后向扫描**：$v_i\le\sqrt{v_{i\pm1}^2+2a_{max}\Delta s}$；
4. **端点**：`v[0]=v_start`、`v[-1]=v_end`，默认 0，起终点真正停车；
5. **输出**：$w_i=\kappa_i v_i$，带符号。

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
空路径/非法参数会报错并跳过，不再 IndexError。

## 3. 离线复现

```bash
cd src/ground/ros2_velocity_profiling
python3 -m unittest discover -s test -v   # 12 项
python3 -m ros2_velocity_profiling.sim
```

## 4. 结果

`v_max=1.0`、`a_max=0.8`、`a_lat_max=0.5`；横向加速度由**速度矢量数值差分**
独立计算，不复用限速用的 κ：

| 路径 | 长度 | 恒速时间 | 规划时间 | 恒速最大横向加速度 | 规划最大横向加速度 |
| --- | ---: | ---: | ---: | ---: | ---: |
| circle (r=1.2) | 7.54 m | 7.54 s | 11.18 s | 0.833 | **0.450** |
| slalom | 14.34 m | 14.34 s | 18.73 s | 1.893 | **0.457** |
| rounded square | 18.63 m | 18.63 s | 23.44 s | 1.250 | **0.474** |

时间增加 26%–48%，换横向加速度不超 0.5；恒速方案在圆上已超 1.7 倍。

![速度剖面](../../../docs/img/ground/ros2_velocity_profiling/speed_profile.png)

## 5. 局限

* 只做速度剖面，不改变路径几何；离散曲率对 1–2 个点的尖角仍可能偏大；
* 未考虑轮速/力矩饱和，真机需再限幅；
* $w=\kappa v$ 假设纯滚动、无侧滑；
* 输出是逐点数组，`Float64MultiArray` 无 header，需与输入 Path 的索引对齐；
* 只支持 2D 路径。

## 6. 大模型声明

代码与文档在编写过程中使用了 AI 大模型辅助。作者已实际运行 12 项单元测试和
3 条离线路径，并在 Ubuntu 20.04 + Foxy 下完成 `colcon build`；对提交内容的
正确性与完整性负全部责任。

