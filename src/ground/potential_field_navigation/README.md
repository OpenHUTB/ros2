# potential_field_navigation

人工势场法（APF）二维导航仿真。目标点对小车产生引力，圆形障碍产生随距离衰减的斥力，两者合成决定期望航向，小车按最大转向角速率跟踪该方向；遇到局部极小点时叠加切向偏置绕出。

## 运行

```bash
cd src/ground/potential_field_navigation
python main.py        # Windows 用 main.bat，Linux/macOS 用 main.sh
```

依赖：`pip install -r requirements.txt`（仅 pygame）。

## 测试

```bash
python -m pytest test/test_potential_field.py
```

覆盖：引力方向、斥力在影响半径外为零、斥力随距离变近而增大、斥力背离障碍指向、绕障抵达目标、全程保持正间隙，以及障碍对称挡路时的局部极小点逃逸。
