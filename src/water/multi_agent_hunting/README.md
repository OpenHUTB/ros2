# multi_agent_hunting

多 AUV 协同围捕示例：拍卖算法做任务分配，水声信道带固定延迟与丢包，
三艘 AUV 从不同方向收拢并由捕获半径判定围捕成功。
完整说明见 [docs/water/multi_agent_hunting.md](../../../docs/water/multi_agent_hunting.md)。

## 包结构

```
src/water/multi_agent_hunting/
├── resource/multi_agent_hunting
├── multi_agent_hunting/
│   ├── __init__.py
│   ├── core.py        # 信道、拍卖、控制、捕获判定
│   └── main.py        # 命令行入口
├── launch/
│   └── run_demo.sh
├── test/
│   └── test_core.py
├── scenario.json
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```

## 编译与测试

```bash
cd ~/ros2_ws
colcon build --packages-select multi_agent_hunting
source install/setup.bash
colcon test --packages-select multi_agent_hunting
```

## 运行

```bash
ros2 run multi_agent_hunting multi_agent_hunting
```

指定随机种子和步数（方便复现）：

```bash
ros2 run multi_agent_hunting multi_agent_hunting --seed 7 --max-steps 300
```

没有安装 HoloOcean 时自动进入 dry-run 模式，用模拟数据演示完整流程。

HoloOcean 真实模式尚未实测：代码已为每个 agent 补 `LocationSensor`，并兼容
`LocationSensor` / `Location` 两种观测键；首次接入前请确认 `env.tick()` 返回里包含
`LocationSensor`。

## 参数（scenario.json）

| 参数 | 默认值 | 说明 |
|---|---|---|
| `seed` | 42 | 随机种子，相同种子初始位置与分配结果完全一致 |
| `comm_delay_ms` | 200 | 水声通信延迟，毫秒 |
| `packet_loss` | 0.05 | 丢包率，0 ~ 1 |
| `energy_weight` | 5.0 | 电量奖励权重，单位相当于「满电可抵扣的距离（米）」 |
| `agents[].energy` | 100.0 | 各 AUV 的初始电量，示例配置为 95 / 45 / 70 |
| `closing_ratio` | 0.35 | 包抄点向逃逸目标收缩的起始比例，0 停在包抄点，1 直接追目标 |
| `closing_time` | 20.0 | 收缩到贴身所需时间，秒 |
| `capture_radius` | 2.0 | 捕获判定半径，米 |
| `idle_drain_rate` | 0.02 | 待机耗电率，%/s |
| `move_drain_rate` | 0.15 | 满推力时的额外耗电率，%/s |
