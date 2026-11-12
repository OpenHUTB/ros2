# uav_cg_trim_envelope

多旋翼重心偏移配平与饱和包线核算：四电机悬停拉力分配、配平倾角、饱和包线与配平裕度。
公式与判定见 [文档页](../../../docs/air/uav_cg_trim_envelope.md)。

## 依赖

运行与测试仅需 Python 3.8+ 标准库（测试文件无 pytest 也能直接跑）。

## 运行

```bash
cd src/air/uav_cg_trim_envelope
python3 main.py --output outputs
python3 main.py --mass 1.5 --dx 8 --thrust-max 5 --output outputs
```

输出 `outputs/report.json`（已进 `.gitignore`）。

## 测试

```bash
python3 test/test_uav_cg_trim_envelope.py   # 13 项
python3 -m pytest test -v                    # CI 用
```

## ROS 2 包装

```bash
colcon build --packages-select uav_cg_trim_envelope
source install/setup.bash
ros2 launch uav_cg_trim_envelope main.launch.py dx:=8
```

`launch/main.launch.py` 用 `ExecuteProcess` 调 CLI，不需要 rclpy；尚未在 Ubuntu + Humble 上实跑，
见 `validation/offline_validation.json`。
