# 无人机集群编队控制 (Leader-Follower + Consensus)

演示 5 架无人机在 Leader-Follower 架构下通过 Consensus 一致性算法
完成编队保持和队形切换。

## 运行

```powershell
cd src/air/drone_swarm
python main.py
```

## 参数 (scenario.json)

| 参数 | 说明 | 默认 |
|------|------|------|
| n_agents | 无人机数量 | 5 |
| formation | 队形类型 | triangle |
| topology | 通信拓扑 | ring |
| k_consensus | Consensus 增益 | 0.5 |
| k_shape | 队形误差反馈增益 | 1.2 |

## 算法核心

1. **Leader**（索引 0）匀速直线飞行，作为编队整体参考
2. **Followers** 根据期望相对偏移计算目标位置
3. **Consensus 项**：邻居间达成位置协议
4. **队形误差项**：反馈领导者位置 + 期望偏移

## 支持的队形

- `triangle`：等边三角形（3+ 架）
- `diamond`：菱形（4 架）
- `line`：一字直线

## 支持的通信拓扑

- `ring`：环形（每架前后邻居）
- `fully_connected`：全连接

## 依赖

- Python 3.10+
- NumPy
