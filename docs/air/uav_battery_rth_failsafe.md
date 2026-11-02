# 无人机电池电量监控与自动返航 (RTH) 迫降

按电量分级触发预警、返航、迫降, 防止低电量丢机。

![节点运行日志](./uav_battery_rth_failsafe_run.png)

## 状态机

```mermaid
flowchart TD
    A[NORMAL > 25%] -->|电量 <= 25%| B[WARNING]
    B -->|估算返航电量不足| C[RETURN_HOME]
    B -->|电量 <= 15%| C
    C -->|电量 <= 8%| D[EMERGENCY_LAND 就近迫降]
```

## 运行

```bash
cd src/air/uav_battery_rth_failsafe
python3 main.py
```

订阅 `/drone/battery`、`/drone/position`, 发布 `/drone/setpoint_position`、`/drone/failsafe_state`。

详细参数见模块 `README.md`。
