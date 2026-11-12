# humanoid_ars_phase_walk

人形机器人**神经网络行走控制**的可行性分析：以手工 CPG 为基线，系统对比四条学习路线。
重点不是「训出一个会走的网络」，而是**如实给出每条路线在这个模型与这份算力预算下的真实结果**。

## 1. 结论（打包代码可复现）

| 方法 | 前进 | 存活步数 | 俯仰 RMS |
|---|---|---|---|
| **CPG 基线**（手工） | **+1.25 m** | **200 / 200** | 0.262 |
| ARS 从零（线性，200 轮） | +0.02 m | 7 | 0.777 |
| 行为克隆（MLP-64） | −0.35 m | 56 | 0.518 |
| DAgger 轮 3 | +0.75 m | 57 | 0.415 |

**四条学习路线没有一条稳定超过手工 CPG。** 另有一次临时运行 DAgger 得到 +2.34 m，
但换初始化就跑不出来 —— 这种结果不计入结论（见第 5 节）。

## 2. 效果图

![](../../docs/img/humanoid/humanoid_ars_phase_walk/matrix.png)

## 3. 运行

```bash
cd src/humanoid/humanoid_ars_phase_walk
python3 -m pip install -r requirements.txt
python3 main.py --version
python3 main.py --test        # 6 项自检
python3 main.py --headless    # 完整矩阵 + CSV + 图
```

ROS 2：

```bash
ros2 launch humanoid_ars_phase_walk main.launch.py period:=0.02
ros2 topic echo /ars_walk_node/joint_target --once
```

## 4. 实现要点

- **自有平地模型**：模块自带 MJCF，不依赖仓库其它模块（否则单独拉这个模块会缺依赖）
- **统一 rollout**：CPG 基线与所有学习策略共用同一份 `env.rollout`，保证对比同源 ——
  这一点很关键：早期用过两份实现，导致「残差 0.49 m vs 基线 1.53 m」的对比本身不成立
- **策略**：线性 ARS（无梯度）、单隐层 tanh MLP（行为克隆 / DAgger）
- **评测**：前进距离、存活步数、俯仰 RMS、平均速度、能耗

## 5. 能力边界

1. **单种子的不稳定结果不作为结论**：DAgger 对初始化敏感，曾出现 +2.34 m 也出现过 +0.75 m。
2. **行为克隆出现「倒着走」**（−0.35 m，俯仰 0.518）：开环拟合误差在闭环下累积成反向步态。
3. **CPG 参数是手调的**，不是学出来的；ARS 无法改进它，说明该参数化已接近局部最优。
4. 平面模型、无侧向平衡；回合 4 s，不做长时程测试。
5. 若要认真训练，需要 GPU + PPO/SAC 数小时以上，并加入足端接触观测与力矩控制。

## 6. 测试

测试统一放在包内 `test/`，不再在仓库根 `tests/` 另放一份。

```bash
python3 test/test_standalone.py     # 单元 6 项
python3 test/test_integration.py    # 集成 6 项
```

## 7. 许可

MIT。CPG 属公开方法，本模块独立实现。