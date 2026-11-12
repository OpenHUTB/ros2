# 策略权重与训练

随包权重为 `mujoco_forklift_pallet_docking/models/pallet_docking_policy.npz`，包含 `w1`、`b1`、`w2`、`b2` 四组 NumPy 数组。该文件在提交 `244747e75b85aec08982ce70d83acf537ea1127d` 中由 ROS 2 Humble 容器训练并加入仓库。

当前文件的 SHA-256：

```text
d38969fcd53617d7012bdb60b2eea5c82985a486222807795bb862e7d828ec5c
```

从仓库根目录训练并输出新的权重：

```bash
python3 src/ground/mujoco_forklift_pallet_docking/main.py --mode train --seed 17 --model-path /tmp/pallet_docking_policy.npz
```

训练配置：MuJoCo 3.12.0、500 个相机样本、种子 17、前 80% 样本训练、后 20% 样本评估，900 轮、学习率 0.18。命令输出 `held_out_mse`，只对本次训练的留出样本计算；加载已有权重的评估不重新计算该值。渲染后端参与图像采样，重新训练的数值结果及文件哈希随运行环境记录。

检查已发布文件：

```bash
sha256sum src/ground/mujoco_forklift_pallet_docking/mujoco_forklift_pallet_docking/models/pallet_docking_policy.npz
```

MuJoCo 的精确版本由 `requirements.txt` 与 `setup.py` 一致固定；`package.xml` 声明 ROS 运行依赖，容器按 Python 依赖清单安装 MuJoCo。
