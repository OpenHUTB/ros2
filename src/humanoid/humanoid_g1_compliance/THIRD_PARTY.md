# 第三方组件与许可

## 引用

复现对象：

```bibtex
@misc{liu2026lac,
  title={LAC: Linear and Angular Compliance for Humanoid Whole-body Control},
  author={Yang Liu and Zhongkai Gu and Wei Zhu and Mitsuhiro Hayashibe},
  year={2026},
  eprint={2608.25405},
  archivePrefix={arXiv},
  primaryClass={cs.RO},
  url={https://arxiv.org/abs/2608.25405}
}
```

本模块的原创部分（ROS 2 封装、纯 MuJoCo 闭环、柔顺实验与指标、测试、文档）为
**BSD-3-Clause**，与本仓库一致。下列内容来自上游 [lac-code](https://github.com/lac-humanoid/lac-code)
（commit `a8e16291a89184a768069b9453bc432bb93ccbc0`），**均为 MIT 许可**，
除相对导入外未做任何修改。

上游 MIT 许可证全文见 `assets/LICENSE`（`sim/LICENSE` 原样保留）。

## 1. 推理代码 `humanoid_g1_compliance/lac/`

| 本地文件 | 上游路径 |
|---|---|
| `constants.py` | `ros2/lac_deploy/lac_deploy/constants.py` |
| `obs_builder.py` | `ros2/lac_deploy/lac_deploy/obs_builder.py` |
| `action_map.py` | `ros2/lac_deploy/lac_deploy/action_map.py` |
| `actor_critic.py` | `ros2/lac_deploy/lac_deploy/actor_critic.py` |
| `actor_critic_base.py` | `ros2/lac_deploy/lac_deploy/actor_critic_base.py` |
| `adapt_module.py` | `ros2/lac_deploy/lac_deploy/adapt_module.py` |
| `normalization.py` | `ros2/lac_deploy/lac_deploy/normalization.py` |
| `policy_loader.py` | `ros2/lac_deploy/lac_deploy/policy_loader.py` |
| `torch_utils.py` | `ros2/lac_deploy/lac_deploy/torch_utils.py` |

改动只有两处，都不涉及逻辑：

1. `from lac_deploy.x import y` 改为 `from .x import y`（包内相对导入），并新增 `__init__.py`；
2. 每个文件首行加 `# flake8: noqa`——上游代码不符合本仓库的行长/空行约定，
   加豁免比改写上游代码更安全。

上游的 `inference.py`（与 `rclpy` / `unitree_hg` / Xbox 手柄耦合）
**未引入**，其功能由本模块的 `policy.py` + `compliance.py` 重新实现。

## 2. 策略权重 `weights/lac_g1_23dof.pt`

- 上游路径：`checkpoints/lac_g1_23dof.pt`
- 大小：51,154,217 B
- SHA-256：`a54a213637ebbc55ea20a11373b00a4ed7dbef06a4b7c20804009dd2819f4a9c`
- 许可：MIT

**为什么随仓库提供（体积换免联网）**：这是本仓库最大的单文件（48.8 MiB，仍在 GitHub
50 MiB 提示线以内），比第二大的 `docs/img/ground/rviz.gif`（16 MB）大两倍。入库的收益是
评测机断网也能直接构建运行；代价是每个 clone 多拉 48.8 MiB，且这份历史永久留在 `.git` 里。
与 #1167（Whisper 权重改为运行时下载）不同：Whisper 有 `whisper.load_model()` 自动下载
机制，这份策略权重只能从上游仓库手工取，没有可依赖的自动下载路径。若要移除，按上面的
SHA-256 从上游取回即可，功能不受影响。

校验命令：

```bash
sha256sum weights/lac_g1_23dof.pt
```

权重未做任何修改、未重新训练、未量化。它包含 `actor_critic` 的完整 state dict
（teacher / student / critic / adapt_module / 3 个 normalizer），部署路径只走
`adapt_module → student_actor`。

## 3. 机器人模型 `assets/g1`

- 上游路径：`sim/unitree_robots/g1`
- 内容：`g1_23dof.xml`、`scene_23dof.xml`、`meshes/`（29 个 STL，共 19 MB）
- 许可：MIT

G1 机器人本体的 CAD 与模型版权归 Unitree Robotics；上游以 MIT 分发这些文件，
本模块沿用同样的分发方式，未做几何修改。

## 4. 动作库 `motions/lac_motion_library.npz`

- 上游路径：`motions/lac_motion_library.npz`
- 大小：7,084 B（5 个数组共 24,700 B，用 `np.savez_compressed` 存储）
- 许可：MIT

与上游的 `npz` 内容逐元素相同（5 个数组的名称、dtype、shape、数值一致），
只是改成了压缩存储。上游用 `np.savez`（不压缩），本地为省体积换成
`np.savez_compressed`，读取方式不变（`np.load` 对两者一视同仁）。

## 5. 配置文件 `config/lac_policy_cfg.yaml`

- 上游路径：`config/policy_cfg.yaml`
- 用途：`load_policy()` 用它确定网络层宽、`latent_dim`、`adapt_seq_len`、
  `policy_frame_dim`，必须与权重匹配
- 许可：MIT

## 6. 未随仓库提供的内容

| 内容 | 原因 |
|---|---|
| 训练数据集与训练脚本 | 体积大，且本模块只做部署复现，不涉及训练 |
| Isaac Gym 训练环境 | 需要 NVIDIA GPU 与特定驱动，评测机不具备 |
| `unitree_sdk2py` / DDS 桥接 | 本模块改为单进程纯 MuJoCo，不需要中间件 |
| `mujoco_viewer` 交互窗口 | 改为离屏渲染，可无头运行 |
