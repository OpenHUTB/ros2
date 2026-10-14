# 无人机键盘遥控器

`drone_teleop.py` 用于通过终端键盘控制 CarlaAir / AirSim 无人机。

## 运行方式

```bash
python3 src/air/air_teleop/drone_teleop.py --ip 192.168.235.1
```

## 按键说明

| 按键 | 功能 |
| --- | --- |
| W / S | 前进 / 后退 |
| A / D | 左移 / 右移 |
| Space | 上升 |
| Ctrl+P | 下降 |
| J / L | 左偏航 / 右偏航 |
| T | 起飞 |
| G | 降落 |
| H | 紧急悬停 |
| Esc | 安全降落并退出 |

## 注意事项

运行脚本前，请确保 AirSim 服务已经启动，并确认客户端能够访问服务端
IP 地址和 41451 端口。起飞前按 T 解锁电机，退出时优先使用 Esc
执行安全降落。
