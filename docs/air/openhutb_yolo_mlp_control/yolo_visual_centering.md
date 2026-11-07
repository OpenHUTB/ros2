# YOLO 目标水平视觉居中

`openhutb_yolo_mlp_control` 提供独立的 YOLO 目标水平居中模式。节点根据检测框的水平偏差控制无人机横向运动，使目标接近画面中心；不控制前进距离、避障、垂直居中或相机朝向。无人机须先在仿真器中起飞并稳定，本模式不负责解锁、起飞或降落。

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

退出 ROS 节点不会自动降落；由操作者按照仿真器流程接管无人机。目标丢失、检测或里程计过期，以及姿态或位移保护触发时，控制器输出零速度。

## 常用参数与状态

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `dry_run` | `true` | 预览模式，不发送飞行控制指令 |
| `target_class` | `car` | 要跟踪的 YOLO 类别 |
| `gain` | `1.8` | 水平误差控制增益 |
| `max_speed` | `0.8` | 横向速度上限，单位 m/s |
| `max_displacement` | `3.0` | 相对初始位置的水平位移上限，单位 m |
| `start_topmost` | `true` | 启动时置顶 YOLO 窗口 |

状态发布在 `/openhutb/visual_centering/status`，包括 `tracking`、`centered`、`target_lost`、`odom_stale` 及保护状态；预览速度发布在 `/openhutb/visual_centering/preview_cmd_vel`。该模式仅修正水平图像偏差，不提供距离保持或避障。

返回[文档目录](README.md)。

## 说明

本文档整理、代码重构和问题排查过程中使用了大模型辅助；提交者已检查全部内容并对最终结果负责。
