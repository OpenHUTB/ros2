# humanoid_stair_zmp_preview

人形机器人**上下台阶**：ZMP **预瞄控制**（Kajita 2003）规划质心轨迹，两连杆解析 IK 解关节角，
在 MuJoCo 参数化台阶场景里执行并量化。与已有的 `humanoid_stair_climb`（启发式落脚点 + 解析 IK，
固定 0.12 m 台阶、只上不下）相比，本模块的控制律是**模型驱动**的，且做了台阶尺寸扫描与上下双向。

## 1. 结论

| 项 | 结果 |
|---|---|
| ZMP 跟踪（常量参考） | 稳态误差 **0.08 mm** |
| ZMP 跟踪（方波参考，±0.1 m） | RMS **24.6 mm**；关掉前视变成 125.3 mm（**前视有效 5.1 倍**）|
| IK 往返（正向运动学反验，200 组） | 中位 **0.00 mm** |
| 脚掌水平度 | 倾斜 **0.00°**（上下台阶都是） |
| 躯干升幅 | 与台阶总高**精确一致**：3 级 × 0.15 = 0.450 m，误差 0 mm |
| 上台阶中位穿透 | **0.0 mm**（6 组参数全部） |
| 下台阶中位穿透 | 0.10 m：0.0 mm；0.15 m：−32/−78 mm；0.20 m：−46/−107 mm |

**上台阶完全达标；下台阶在 0.10 m 时干净，随台阶增高变差**（见第 5 节能力边界）。

## 2. 效果图

| 上台阶：出发 | 中段 | 到顶 |
|---|---|---|
| ![](../../docs/img/humanoid/humanoid_stair_zmp_preview/scene_up_start.png) | ![](../../docs/img/humanoid/humanoid_stair_zmp_preview/scene_up_mid.png) | ![](../../docs/img/humanoid/humanoid_stair_zmp_preview/scene_up_end.png) |

| 台阶尺寸扫描 | 躯干高度时程 |
|---|---|
| ![](../../docs/img/humanoid/humanoid_stair_zmp_preview/sweep.png) | ![](../../docs/img/humanoid/humanoid_stair_zmp_preview/torso_height.png) |

## 3. 运行

```bash
cd src/humanoid/humanoid_stair_zmp_preview
python3 -m pip install -r requirements.txt

python3 main.py --version      # 版本与关键参数
python3 main.py --test         # 5 项自检（含前视有效性对照）
python3 main.py --headless     # 12 组台阶扫描 + CSV + 2 张图
python3 main.py --snapshots    # 6 张场景快照
```

ROS 2：

```bash
colcon build --packages-select humanoid_stair_zmp_preview
source install/setup.bash
ros2 launch humanoid_stair_zmp_preview main.launch.py direction:=up step_height:=0.15
ros2 topic echo /stair_zmp_node/joint_target --once
```

## 4. 实现要点

- **ZMP 预瞄控制**：LIPM + 误差积分构成 4 维增广状态，离散 Riccati（`scipy.linalg.solve_discrete_are`）
  求状态反馈 `K` 与 160 步前视增益 `f`。前视分子必须是 `P·E`（P 为 Riccati 解、E 为参考通道），
  写成 `Q·e4` 会让 `|f|` 比 `|K|` 小四个数量级，预览等于没开（这是本模块最隐蔽的 bug）。
- **步态**：落脚点序列 → ZMP 参考（换脚处线性过渡）→ CoM 轨迹 → 两连杆解析 IK。
- **踝角**：实测脚的世界俯仰 = `hip + knee + ankle`，故 `ankle = -(hip + knee)` 才能让脚底水平。
  写成 `-(hip - knee)` 差 `2*knee`（约 1.0 rad），脚掌常年翘着插进地面。
- **摆动轨迹**：梯形剖面（竖抬 → 水平 → 竖落）。

## 5. 能力边界

1. **运动学驱动**：躯干位姿与关节角按规划直接写入，MuJoCo 只做几何/接触判定。
   不评价动态平衡与扰动恢复（那需要力矩级控制）。
2. **下台阶穿透**：台阶 ≥0.15 m 时中位穿透 32~107 mm。原因是「刚性脚掌 + 离散台阶」的组合 ——
   已排除抬脚高度（0.06→0.14 完全不变）、摆动轨迹形状（梯形 vs 直线无差别）、
   落脚点位置（前缘 vs 中心）三个因素。彻底解决需要脚掌滚动（ankle roll）或落脚点碰撞检查。
3. **台阶尺寸受限**：0.20 m 时腿接近伸展极限（髋到脚 0.73 m vs 腿长 0.80 m）。
4. 单方向、平面（矢状面）模型，无侧向平衡与转向。

## 6. 测试

```bash
python3 test/test_standalone.py                                   # 5 项（ZMP/前视/IK/升幅）
python3 ../../../../tests/test_humanoid_stair_zmp_preview_local.py # 7 项（升幅/脚水平/接触/穿透）
```

## 7. 许可

代码、场景、IK、步态均为本模块原创；ZMP 预瞄控制与 LIPM 属公开理论（Kajita 2003），独立实现。MIT。