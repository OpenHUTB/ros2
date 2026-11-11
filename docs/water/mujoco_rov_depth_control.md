# 水下 ROV 定深控制（MuJoCo）

> 模块路径：`src/water/mujoco_rov_depth_control` ｜ 载体：水下 ROV ｜ 引擎：MuJoCo（pip 包 `mujoco`）

## 实验目的

在水下机器人方向上实现一条完整的控制闭环：给定深度序列，ROV 能快速、无超调地到达并稳住，
并把稳态误差量化到厘米级。

## 方法

1. **被控对象**（参数全部显式写在 `config/params.yaml`）：
   - 机体 0.4×0.3×0.3 m，`hull_mass = 15.0 kg`；
   - 尾鳍尺寸 0.40×0.04×0.20 m、密度 500 kg/m³，贡献 1.6 kg；
   - 合计 **16.6 kg**（运行时打印 `model.body_mass` 实测值，第一次运行即为 16.60 kg）；
   - 净浮力 = 质量 × 9.81 × 1.02（2% 正浮力，模拟真实配平偏差）；
   - 水体作用显式建模为线性阻尼 12 N·s/m + 二次阻尼 25 N·s²/m²。
     注意：MuJoCo 不含浮力与流体阻力，必须在 `xfrc_applied` 里补；
     而重力由引擎施加，**补浮力时不能再减一次重量**（否则推力打满还在下沉）。
2. **控制器（串级，不是单环 PD）**：
   - `v_des = clip(kp_pos · e, ±1.0 m/s)`（kp_pos = 0.8 1/s），e = 目标深度 − 当前深度；
   - `a_des = kv · (v_des − v_z)`（kv = 4.0 1/s）；
   - `u = m · a_des − 净浮力`，限幅 ±80 N；
   - 物理步长 `dt = 0.002 s`，控制分频 `control_every = 25` → **控制频率 20 Hz**。
3. **任务**：`mission.segments` 里每段是 `(目标深度, 保持时长)`，即
   −5 m / −2 m / −0.5 m 各保持 14 s；取每段最后 4 s 计算稳态误差。
4. **验收**：`pass_threshold = 0.05 m`（实测最差 0.013 m，留 5 cm 余量）。

## 结果

| 目标深度 | 稳态平均误差 | 最大误差 |
|---|---|---|
| −5.00 m | **0.013 m** | 0.042 m |
| −2.00 m | 0.002 m | 0.005 m |
| −0.50 m | 0.000 m | 0.001 m |

- 控制点 840 个（42 s × 20 Hz），最差稳态误差 0.013 m ≤ 0.05 m → **PASS**
- 曲线见 `src/water/mujoco_rov_depth_control/docs/result.png`，
  逐点数据见 `docs/depth_log.csv`，指标摘要见 `docs/summary.md`
  （`tools/summarize_csv.py` 可由 CSV 重新生成摘要，保证文字与数据同源）。

## 结论

串级结构把「位置指令」变成了有速度上限的平滑过渡，因此没有出现大深度指令下的冲过头；
三段定深的稳态误差均在 1.3 cm 以内，满足定深巡航、定点悬停这类任务的精度要求。

## 局限与改进

1. 只有垂向单自由度控制，未做姿态（横滚/俯仰）稳定。
2. 阻尼与浮力是参数化近似，未做系统辨识。
3. 未与 HoloOcean/UE4 场景联调（缺世界包），视觉层面没有水下渲染。
4. 本机（2026-11-08 起）应用控制策略会拦截未签名的原生扩展，`import mujoco` 会失败；
   此时可用 `tools/summarize_csv.py` 基于已记录的 CSV 重算指标，但重跑仿真需要在允许该 DLL 的环境进行。

## 运行效果（截图 / 动图）

![ROV 定深控制（深度跟踪 + 垂向推力曲线动画） 动图](docs/img/water/mujoco_rov_depth_control_run.gif)

![ROV 定深控制（深度跟踪 + 垂向推力曲线动画） 截图 kf1](docs/img/water/mujoco_rov_depth_control_kf1.png)

![ROV 定深控制（深度跟踪 + 垂向推力曲线动画） 截图 kf2](docs/img/water/mujoco_rov_depth_control_kf2.png)

> 动图由实际运行的帧合成（GIF < 5 MB）；素材与生成脚本见 `tools/build_media.py`、`tools/build_lidar_media.py`、`tools/build_air_media.py`。
