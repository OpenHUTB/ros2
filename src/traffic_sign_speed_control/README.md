# traffic_sign_speed_control

基于 CNN 限速标志识别的车辆运动控制（ROS 2 Humble，任务二：传感器感知及运动控制）。

自研轻量世界节点（差速运动学 + 虚拟前视相机 + pygame 渲染），
感知流水线：HSV 红圈定位 → 字符切分 → DigitCNN（0-9）→ 位值组合 → 滑窗防抖；
控制：预瞄 P 巡航 + `min(巡航, 限速)` 目标速度 + 加速度限幅。

## 快速开始

    cd ~/ts_ws
    colcon build --packages-select traffic_sign_speed_control
    source install/setup.bash
    # 首次运行先训练权重（CPU 约 3 分钟，仓库已含训练好的 sign_cnn.pth）
    python3 src/traffic_sign_speed_control/scripts/train_signs.py
    ros2 launch traffic_sign_speed_control traffic_sign_speed_control.launch.py

launch 参数：`render_mode`（none/file/window）、`frame_dir`（GIF 帧目录）、
`scene`（default/degraded/dense）。

## 脚本

| 命令 | 作用 |
|---|---|
| `python3 main.py train` | 合成数据 + 训练 DigitCNN，输出权重与指标 |
| `python3 main.py evaluate` | 字符级退化矩阵评测 |
| `python3 main.py scenes` | 端到端多场景评测（启动真实 ROS 系统） |
| `python3 main.py demo` | file 渲染模式跑默认场景并合成 GIF（需 ffmpeg） |

完整教程（原理、流程、代码结构、性能评价、GIF）见仓库 `docs/ground/traffic_sign_speed_control.md`。
