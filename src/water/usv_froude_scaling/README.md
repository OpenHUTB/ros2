# usv_froude_scaling

USV 弗劳德缩比换算与拖曳功率估算：船模-实船速度/阻力/功率换算，ITTC 1957 分解法估算
实船总阻力与拖曳功率。公式与判定见 [文档页](../../../docs/water/usv_froude_scaling.md)。

## 依赖

运行与测试仅需 Python 3.8+ 标准库（测试文件无 pytest 也能直接跑）。

## 运行

```bash
cd src/water/usv_froude_scaling
python3 main.py --output outputs
python3 main.py --speed 6 --model-resistance 3.2 --output outputs
```

输出 `outputs/report.json`（已进 `.gitignore`）。

## 测试

```bash
python3 test/test_usv_froude_scaling.py   # 12 项
python3 -m pytest test -v                  # CI 用
```

## ROS 2 包装

```bash
colcon build --packages-select usv_froude_scaling
source install/setup.bash
ros2 launch usv_froude_scaling main.launch.py speed:=6.0
```

`launch/main.launch.py` 用 `ExecuteProcess` 调 CLI，不需要 rclpy；尚未在 Ubuntu + Humble 上实跑，
见 `validation/offline_validation.json`。
