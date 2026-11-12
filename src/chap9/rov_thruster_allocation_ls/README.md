# rov_thruster_allocation_ls

ROV **六自由度推力分配**：8 推力器布局 + 阻尼最小二乘 + 整体缩放饱和 + 故障屏蔽。

## 要点

- 每个推力器由位置 `r` 与方向 `d` 描述，TAM 列为 `[d; r × d]`
- 解 `(T^T T + λI) x = T^T w` 得最小范数推力（λ=1e-6，避免 N>6 时奇异）
- 超过 `limit` 时整体缩放，方向不变、幅值受限，输出利用率
- `failed:=0,1` 可把指定推力器列置零后重新分配

## 环境（实测）

- Ubuntu 20.04 + ROS 2 Foxy，Python 3.8.10
- 无 Gazebo 依赖：仿真在包内节点完成

## 构建与运行

```bash
cd ~/ros2_ws
colcon build --packages-select rov_thruster_allocation_ls
source install/setup.bash
ros2 launch rov_thruster_allocation_ls main.launch.py
```

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `limit` | 10.0 | 单推力器上限 (N) |
| `failed` | 空 | 失效推力器下标，逗号分隔 |

## 验证范围

- ✅ 8 项单元测试通过（分配、饱和、故障屏蔽）
- ✅ Foxy 下实跑：纯升沉 / 升沉+偏航 / 纵推+横滚 / 60 N 过载四个阶段，过载时利用率 1.0
- ⚠️ 推力器布局为示意值；未做加权分配与死区补偿
