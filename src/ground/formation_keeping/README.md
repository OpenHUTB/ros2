# formation_keeping

多机器人编队保持 2D 仿真。领航者沿圆周运动，两个跟随者保持在领航者坐标系下的固定偏移，形成三角队形；跟随者用比例控制追期望位置，单帧位移有上限。

## 运行

```bash
cd src/ground/formation_keeping
python main.py        # Windows 用 main.bat，Linux/macOS 用 main.sh
```

依赖：`pip install -r requirements.txt`（仅 pygame）。

## 测试

```bash
python -m pytest test/test_formation_keeping.py
```

覆盖：偏移旋转、不同朝向下的期望位置、领航者圆周约束、队形误差定义、初始偏离后收敛到阈值以内，以及收敛后长时间保持队形。
