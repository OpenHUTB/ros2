# line_following_pid

循线小车的 PID 横向控制 2D 仿真。小车用自行车运动学前进，读取自身相对正弦赛道中心线的横向误差，经带积分与输出限幅的 PID 算出前轮转角，沿赛道行进。

## 运行

```bash
cd src/ground/line_following_pid
python main.py        # Windows 用 main.bat，Linux/macOS 用 main.sh
```

依赖：`pip install -r requirements.txt`（仅 pygame）。

## 测试

```bash
python -m pytest test/test_line_following_logic.py
```

覆盖：PID 比例项与输出限幅、积分清零、赛道采样、最近点误差为零、跑完整条赛道的横向误差上界。
