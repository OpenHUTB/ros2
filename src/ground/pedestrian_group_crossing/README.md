# pedestrian_group_crossing

群体行人过街决策模块，补充单人 TTC 策略没有覆盖的“等人、队形、跟随”行为。

## 运行

```bash
cd src/ground/pedestrian_group_crossing
python main.py --group-size 3 --arrived 2 --spread 1.0 --gap 4.0
```

输入 `arrived_count, spread_m, gap_seconds, leader_waiting`，输出是否等待及原因。

## 测试

```bash
python -m unittest discover -s test -p "test_*.py" -v
```

ROS 2 话题：输入 `/pedestrian_group_crossing/observation`，输出
`/pedestrian_group_crossing/decision`。
