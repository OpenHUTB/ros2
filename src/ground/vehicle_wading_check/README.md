# vehicle_wading_check

地面车辆涉水深度与浮力裕度核算模块。它把整车质量、车身尺寸、进气口/排气口/
车门下沿高度、排水体积占比、湿态附着系数与车体水阻放在一起核算，输出：
允许涉水深度、安全涉水深度、浮起水深、允许涉水车速、浮力占比与牵引力余量，
并给出 `SAFE` / `CAUTION` / `FLOODED` / `FLOATING` 四态判定，避免在深水区
继续按干地车速行驶。

## 运行环境

- Ubuntu 22.04.5 LTS
- ROS 2 Humble
- Python 3.10.12
- matplotlib（仅离线报告需要）

## 构建与运行

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select vehicle_wading_check
source install/setup.bash
ros2 launch vehicle_wading_check main.launch.py duration:=24
```

演示结束时模拟器自动退出，`ros2 launch` 通过 `OnProcessExit -> Shutdown`
一并关闭（实测 24 s 场景结束后 launch 自行退出，无残留进程）。

## ROS 2 接口

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/vehicle/wading_depth_m` | `std_msgs/Float64` | 输入 | 当前涉水深度，m |
| `/vehicle/speed_mps` | `std_msgs/Float64` | 输入 | 当前车速，m/s |
| `/wading/status` | `std_msgs/String` | 输出 | 状态、浮力占比、允许车速与余量 JSON |

状态 JSON 示例：

```json
{
  "mode": "FLOODED",
  "depth_m": 0.42,
  "speed_mps": 2.5,
  "depth_limit_m": 0.35,
  "safe_depth_m": 0.28,
  "buoyancy_ratio": 0.7938,
  "v_max_mps": 1.659,
  "traction_margin_n": -498.6
}
```

运行 CSV 默认写入 `/tmp/vehicle_wading_check_runtime.csv`（由参数 `csv_path`
控制）。深度或车速反馈超过 1 s 未刷新时本周期不输出状态，避免依据陈旧数据
放行。

## 算法

浮力（车体按长方体近似，只有 `displacement_volume_factor` 部分真正排水）：

\[
F_b=\rho_w g L W \min(h,H)\,\eta
\]

湿态可传递牵引力与滚动阻力：

\[
F_{trac}=\mu_{wet}(mg-F_b),\quad F_{roll}=C_{rr,wet}(mg-F_b)
\]

水阻（含兴波修正）：

\[
F_d=\tfrac{1}{2}\rho_w C_D k_{wave} (W\max(h,0.05)) v^{2}
\]

由 `F_trac - F_roll = F_d(v_max)` 反解允许车速：

\[
v_{max}=\sqrt{\frac{F_{trac}-F_{roll}}{\tfrac{1}{2}\rho_w C_D k_{wave} W \max(h,0.05)}}
\]

判定规则：

- `SAFE`：`h <= 0.8 * h_limit`；
- `CAUTION`：`0.8 * h_limit < h <= h_limit`；
- `FLOODED`：`h > h_limit`，其中 `h_limit = min(进气口, 排气口, 车门下沿)`；
- `FLOATING`：浮力达到自重的 `float_ratio_limit`（默认 90%），此时允许车速置零。

## 测试

```bash
cd src/ground/vehicle_wading_check
PYTHONPATH=. python3 -m unittest discover -s tests -v
```

测试覆盖限深取最小值、浮起点深度、四态边界、浮力随深度增长与到达车高后饱和、
牵引力随水深下降、允许车速随水深下降、水阻二次律、超速时余量为负、湿态制动
衰减比、双输入新鲜度窗口、`reset` 与非法输入拒绝。

## 离线复现

```bash
cd src/ground/vehicle_wading_check
PYTHONPATH=. python3 -m vehicle_wading_check.demo --duration 24 --rate 10 --csv /tmp/wading_demo.csv
PYTHONPATH=. python3 -m vehicle_wading_check.report --out docs/img/ground/vehicle_wading_check/runtime_demo.png
```

`demo` 打印 24 s 场景的统计摘要，`report` 渲染「允许车速-水深」与「浮力占比/
牵引力-水深」两幅曲线。

## 边界说明

- 车体按长方体近似，未建模机舱、轮拱、行李厢等真实几何差异；
- 水阻用 `C_D` 常数与兴波系数近似，未建模流速、波浪与壅水；
- 湿态附着系数取常数，未建模水深对轮胎接地印迹的动态影响；
- 模块只做核算与状态提示，不替代发动机进气保护、涉水后制动恢复或整车涉水
  标定。
