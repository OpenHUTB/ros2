# msk_cocontraction_stiffness

拮抗肌共收缩与关节刚度核算：两侧肌力反解、关节刚度、扰动偏转与共收缩上限。
公式与判定见 [文档页](../../../docs/musculoskeletal/msk_cocontraction_stiffness.md)。

## 依赖

运行与测试仅需 Python 3.8+ 标准库（测试文件无 pytest 也能直接跑）。

## 运行

```bash
cd src/musculoskeletal/msk_cocontraction_stiffness
python3 main.py --output outputs
python3 main.py --torque 10 --coactivation 0.5 --output outputs
```

输出 `outputs/report.json`（已进 `.gitignore`）。

## 测试

```bash
python3 test/test_msk_cocontraction_stiffness.py   # 12 项
python3 -m pytest test -v                            # CI 用
```

## ROS 2 包装

```bash
colcon build --packages-select msk_cocontraction_stiffness
source install/setup.bash
ros2 launch msk_cocontraction_stiffness main.launch.py torque:=10.0
```

`launch/main.launch.py` 用 `ExecuteProcess` 调 CLI，不需要 rclpy；尚未在 Ubuntu + Humble 上实跑，
见 `validation/offline_validation.json`。
