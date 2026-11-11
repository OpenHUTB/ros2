# ROS 2 验证清单（需要另一位使用者在 Ubuntu 22.04 + Humble 上执行）

离线部分（闭式解、19 项测试、`main.py` 出图、GBK 控制台）已经在 Windows 11 + Python 3.12 上跑通。
**ROS 2 节点没有在作者机器上运行过**（本机没有 ROS 环境），仓库要求换一个人、换一台机器复现，
所以把步骤列在这里，按顺序执行即可。

## 1. 构建

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2                                            # 本仓库
colcon build --packages-select uav_hover_wind_limit
source install/setup.bash
ros2 pkg list | grep uav_hover_wind_limit            # 应能列出包名
```

## 2. 启动节点并看话题

```bash
ros2 launch uav_hover_wind_limit main.launch.py
```

另起一个终端：

```bash
ros2 topic info /uav/wind_limit          # 发布者 1，类型 std_msgs/msg/String
ros2 topic echo /uav/wind_limit          # 应为一行 JSON
```

期望：JSON 里 `max_wind_m_s` ≈ 9.9773、`max_wind_beaufort` = 5、
`tilt_budget.limited_by` = `tilt_limit`、`tilt_budget.effective_tilt_deg` = 25.0。

## 3. 带参数复跑（推重比耦合）

```bash
ros2 launch uav_hover_wind_limit main.launch.py available_thrust_to_weight:=1.05
ros2 topic echo /uav/wind_limit
```

期望：`tilt_budget.limited_by` = `thrust_margin`、`achievable_tilt_deg` ≈ 17.753、
`effective_tilt_deg` ≈ 17.753、`max_wind_m_s` ≈ 8.27。

## 4. 参数边界

```bash
ros2 launch uav_hover_wind_limit main.launch.py available_thrust_to_weight:=0.9
```

期望：节点启动即抛 `ValueError: 实际可用推重比必须大于 1（否则任何倾角都无法悬停）`。

```bash
ros2 launch uav_hover_wind_limit main.launch.py altitude_m:=15000
```

期望：`ValueError: ISA 对流层近似只适用于 -500..11000 m`。

## 5. 离线入口（同机顺带验证安装）

```bash
ros2 run uav_hover_wind_limit wind_limit --output /tmp/wind_out
ls /tmp/wind_out      # wind_vs_tilt.png  capability.png  report.json
```

## 6. 回填结果

请把下面几项贴回 <https://github.com/OpenHUTB/ros2/pull/1073>：

- 系统与 ROS 版本（`lsb_release -d`、`echo $ROS_DISTRO`）、Python 版本；
- `colcon build` 是否成功、有没有 warning；
- 第 2、3 步 `ros2 topic echo` 的实际 JSON（至少含 `max_wind_m_s` 与 `tilt_budget`）；
- 第 4 步的实际报错文本。

执行完把 `validation/offline_validation.json` 里的 `ros_validation.checked` 改为 `true`、
`independent_machine_review` 改为 `true`，并补上机器信息即可。
