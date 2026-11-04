# YOLO 目标视觉伺服：居中与距离保持

`openhutb_yolo_mlp_control` 提供独立的 YOLO 目标视觉伺服模式。节点根据检测框偏差控制无人机运动：**水平居中始终启用**；**距离保持**与**垂直居中**为可选能力，默认关闭，关闭时行为与原有水平居中模式完全一致。本模式不做避障，也不控制相机朝向。无人机须先在仿真器中起飞并稳定，本模式不负责解锁、起飞或降落。

目标类别默认为 `car`，也可通过 `target_class:=person` 等参数切换。运行前需先准备 YOLO11n 权重；该文件不包含在仓库中，可从[模型下载地址](https://pan.baidu.com/s/1nHSTqk4aYdC5HOpih6jPrg?pwd=fffa)下载（提取码：`fffa`）。请将 `yolo11n.pt` 保存到本地，并在启动时传入其绝对路径，避免 Ultralytics 在运行时尝试联网下载。

以下命令在完成[环境配置](setup.md)中的依赖安装、功能包构建并加载工作空间后运行。先将 `REPO_DIR` 设为本仓库根目录，再设置模型文件和当前 Windows 主机地址，并确认文件及 RPC 端口可用：

```bash
export REPO_DIR=/绝对路径/到/ros2
export YOLO_MODEL=/绝对路径/到/yolo11n.pt
export AIRSIM_IP=实际Windows地址
test -f "$YOLO_MODEL"
nc -vz "$AIRSIM_IP" 41451
```

`REPO_DIR` 应设置为本仓库根目录，不依赖特定的工作空间名称；`YOLO_MODEL` 则应指向刚才下载的权重文件。

## 预览控制量

预览模式运行检测和控制计算，但不发送飞行控制指令：

```bash
ros2 launch openhutb_yolo_mlp_control visual_centering.launch.py \
  airsim_ip:="$AIRSIM_IP" \
  yolo_model_path:="$YOLO_MODEL" \
  target_class:=car \
  dry_run:=true \
  show_window:=true
```

在另一个已加载工作空间环境的终端查看状态和预览速度：

```bash
ros2 topic echo /openhutb/visual_centering/status
ros2 topic echo /openhutb/visual_centering/preview_cmd_vel
```

## YOLO 窗口置顶

`start_topmost` 默认是 `true`。启动时可设置 `start_topmost:=false` 关闭置顶；运行中先让 YOLO 窗口获得焦点，再按 `T` 切换。置顶需要图形桌面和窗口管理器支持。在 WSLg 下程序优先调用 Windows 宿主窗口功能，不可用时使用 OpenCV 窗口接口。

## 仿真控制

确认目标类别和相机方向正确、无人机已稳定悬停，并停止其他飞行指令发布者后，将 `dry_run` 设为 `false`：

```bash
ros2 launch openhutb_yolo_mlp_control visual_centering.launch.py \
  airsim_ip:="$AIRSIM_IP" \
  yolo_model_path:="$YOLO_MODEL" \
  target_class:=car \
  dry_run:=false \
  show_window:=true
```

也可从任意当前目录调用仓库入口脚本：

```bash
bash "$REPO_DIR/src/air/openhutb_yolo_mlp_control/main.sh" \
  airsim_ip:="$AIRSIM_IP" \
  yolo_model_path:="$YOLO_MODEL" \
  target_class:=car \
  dry_run:=false
```

退出 ROS 节点不会自动降落；由操作者按照仿真器流程接管无人机。目标丢失、检测或里程计过期，以及姿态、水平位移保护触发时，控制器输出零速度；启用垂直居中时，高度保护（`altitude_limit`）也会输出零速度。

## 距离保持与垂直居中

两项能力默认关闭，需要显式设置参数才会启用。

**距离保持**（`target_area` > 0）：把目标框占画面的面积比作为设定值，检测框偏小说明目标太远，控制器输出前向速度；偏大则后退。该量是**单目视觉的表观尺寸**，不是真实测距，因此只在目标类别已知、目标完整可见且不改变姿态的场景下可靠；目标被遮挡或侧向旋转时面积会剧烈变化，此时不建议开启。参数为 `target_area`、`distance_gain`、`distance_deadband`、`max_forward_speed`。

**垂直居中**（`vertical_gain` > 0）：用检测框的纵向偏差控制升降，**启用后取代原有的定高保持**；目标在画面上方则爬升，在下方则下降。参数为 `vertical_gain`、`vertical_deadband`，升降速度仍受 `max_vertical_speed` 限制。

**安全边界**：`max_altitude_change` 限制的是**相对跟踪起点高度的变化量**，并且**只在启用垂直居中（`vertical_gain` > 0）时生效**；超过该值控制器立即输出零速度并报告 `altitude_limit`。未启用垂直居中时该限制不会触发，高度仍按原有定高逻辑保持，正常爬升或高空作业不受影响。水平位移始终由 `max_displacement` 限制。

## 常用参数与状态

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `dry_run` | `true` | 预览模式，不发送飞行控制指令 |
| `target_class` | `car` | 要跟踪的 YOLO 类别 |
| `gain` | `1.8` | 水平误差控制增益 |
| `max_speed` | `0.8` | 横向速度上限，单位 m/s |
| `max_displacement` | `3.0` | 相对初始位置的水平位移上限，单位 m |
| `target_area` | `0.0` | 期望的目标框面积占画面比例；`0` 表示关闭距离保持 |
| `distance_gain` | `1.0` | 相对面积误差到前向速度的增益 |
| `distance_deadband` | `0.15` | 相对面积误差死区，落在区间内不输出前向速度 |
| `max_forward_speed` | `0.5` | 前向速度上限，单位 m/s |
| `vertical_gain` | `0.0` | 纵向偏差到升降速度的增益；`0` 表示保持定高 |
| `vertical_deadband` | `0.08` | 纵向归一化偏差死区 |
| `max_altitude_change` | `3.0` | 启用垂直居中时，相对跟踪起点高度的最大变化，单位 m |
| `start_topmost` | `true` | 启动时置顶 YOLO 窗口 |

状态发布在 `/openhutb/visual_centering/status`，包含 `status`、`error_x`、`distance_error`、`forward_speed`、`velocity_ned`、`dry_run` 字段；`status` 取值包括 `tracking`、`centered`、`target_lost`、`odom_stale`、`displacement_limit`、`heading_changed`、`altitude_limit`。预览速度发布在 `/openhutb/visual_centering/preview_cmd_vel`。`centered` 表示水平偏差在死区内，且启用距离保持时面积误差也在死区内。

本模式仍不提供避障；需要绕障时请改用避障规划类模块。

返回[文档目录](README.md)。

## 说明

本文档整理、代码重构和问题排查过程中使用了大模型辅助；提交者已检查全部内容并对最终结果负责。
