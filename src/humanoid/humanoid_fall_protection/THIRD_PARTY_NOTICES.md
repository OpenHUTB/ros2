# 第三方来源说明

本模块**没有**复制任何第三方代码。以下是设计与参数上的引用来源。

## OpenHorizon Labs — Asimov 1 safe-fall

- 项目：`asimov-v1-policies`（训练与部署代码）与 `asimov1-getup-safefall`（ONNX 策略）
- 许可：Apache License 2.0
- 地址：<https://github.com/openhorizon-labs/asimov-v1-policies>
  与 <https://huggingface.co/openhorizon-labs/asimov1-getup-safefall>

引用内容：

| 本模块中的位置 | 引用内容 |
|---|---|
| `config.py` 的 `tilt_threshold_deg=30.0`、`tilt_rate_threshold=2.0` | 取自其 `safefall_policy_spec.json` 的 `fall_trigger.deploy_defaults`（`tilt_deg: 30.0`、`ang_vel: 2.0`） |
| `docs/humanoid/humanoid_fall_protection.md` §3 | 其训练范围 `tilt_deg: [20, 45]`、`ang_vel: [2.0, 4.0]`，用于说明推力扫描覆盖的区间 |
| `metrics.py` 的 `reduction_vs_best_passive` | 评估口径「与表现更好的那个被动策略比」，对应其 `impact_reduction_vs_best_passive` |

上述均为公开的配置数值与方法描述，不构成代码复制。本模块的策略是手工写的姿态 PD 伺服，
与该项目基于 PPO 训练的策略网络在实现上没有共同代码。

## MuJoCo

- 项目：MuJoCo 物理引擎（DeepMind / Google）
- 许可：Apache License 2.0
- 地址：<https://github.com/google-deepmind/mujoco>

通过 pip 依赖使用，未修改、未内嵌。
