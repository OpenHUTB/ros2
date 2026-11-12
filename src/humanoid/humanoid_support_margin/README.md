# humanoid_support_margin

人形机器人支撑多边形与倾倒裕度核算：双足凸包、稳定裕度、临界推力/冲量与足底载荷分配。
公式与判定见 [文档页](../../../docs/humanoid/humanoid_support_margin.md)。

## 依赖

运行与测试仅需 Python 3.8+ 标准库（测试文件无 pytest 也能直接跑）。

## 运行

```bash
cd src/humanoid/humanoid_support_margin
python3 main.py --output outputs
python3 main.py --mass 60 --com-x 0.03 --com-height 0.9 --output outputs
```

输出 `outputs/report.json`（已进 `.gitignore`）。

## 测试

```bash
python3 test/test_humanoid_support_margin.py   # 11 项
python3 -m pytest test -v                       # CI 用
```

## ROS 2 包装

```bash
colcon build --packages-select humanoid_support_margin
source install/setup.bash
ros2 launch humanoid_support_margin main.launch.py com_x:=0.03
```

`launch/main.launch.py` 用 `ExecuteProcess` 调 CLI，不需要 rclpy；尚未在 Ubuntu + Humble 上实跑，
见 `validation/offline_validation.json`。
