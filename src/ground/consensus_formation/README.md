# 多机器人 Consensus 编队控制

## 模块说明

6 台差动轮移动机器人，通过 **分布式一致性算法（Consensus）** +
形状偏移（Shape Offset）达成全局编队。与 Leader-Follower 固定编队不同：

- **无固定 Leader**：每台机器人地位平等
- **局部通信**：只靠通信半径内的邻居信息（3.0m）
- **自适应拓扑**：编队走起来后邻居关系自动变化

## 运行环境

- Python 3.10+
- NumPy（`pip install numpy`）

## 运行步骤

```powershell
cd src/ground/consensus_formation
python main.py
```

## 核心机制

```
u_i = -Kp * Σ_{j∈N_i} [(x_i - o_i) - (x_j - o_j)]   # Consensus 项
      -Kp * centroid_err * 0.1                       # 质心收敛项
      +Kp * avoid * 2.0                              # 避障项
```

| 形状 | 说明 |
|------|------|
| hexagon | 正六边形（r=1.25m） |
| line | 直线（长 2.0m） |
| diamond | 菱形 |

## 示例输出

```
  📐 形状: hexagon  |  机器人: 6  |  仿真: 40s
    🎯 质心误差: 0.012 m  形状误差: 0.047 m  ✅ 收敛
    🔗 每台邻居数: [2, 2, 2, 2, 2, 2]  (通信半径 3.0m)
```

## 参数调整

编辑 `scenario.json`：

| 参数 | 默认 | 说明 |
|------|------|------|
| Kp_consensus | 1.5 | Consensus 增益（越大收敛越快但易振荡） |
| Kd_heading | 0.3 | 朝向对齐增益 |
| communication_range_m | 3.0 | 每台通信半径（影响拓扑） |
| safety_radius_m | 0.5 | 圆形避障触发半径 |
