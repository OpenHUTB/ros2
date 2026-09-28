# carla_end_to_end_nn —— CARLA 端到端神经网络（图像 → 控制）

> 机器人操作系统及应用 · 课程作业四
>
> **端到端模型**：输入前视相机图像、直接输出控制指令，中间不再有
> "感知 → 规划 → 控制"的人工分层。CNN 为**纯 numpy** 实现（含手写反向传播），
> 无需 TensorFlow / PyTorch。

## 1. 功能

$$
\underbrace{\text{相机图像 } 60\times80\times3}_{\text{输入}}
\ \xrightarrow{\ \text{CNN}\ }\
\underbrace{\text{steer}\in[-1,1]}_{\text{输出}}
\ \xrightarrow{\ \text{应用到车辆}\ }\
(\text{油门},\ \text{转向})
$$

网络结构：`conv3×3 → ReLU → maxpool2 → conv3×3 → ReLU → maxpool2 → 全局平均池化 → FC → tanh`。

## 2. 目录结构

```
src/ground/carla_end_to_end_nn/
├── main.py                                   # 主入口（collect / train / test / headless）
├── main.sh / main.bat                        # 课程约定的 main.* 一键运行脚本
├── carla_end_to_end_nn/
│   ├── __init__.py
│   ├── nn_models.py                          # 纯 numpy CNN（手写反向传播）
│   ├── carla_common.py                       # CARLA Python API 封装
│   └── end_to_end_node.py                    # ROS 2 节点
├── launch/
│   ├── main.launch.py                        # ROS 2 Humble
│   └── main.launch                           # ROS 1 Noetic
├── config/sim_params.yaml                    # 仿真参数
├── resource/carla_end_to_end_nn
├── setup.py / setup.cfg / package.xml
├── requirements.txt
└── test/test_end_to_end_logic.py             # 单元测试（11 项，无需 CARLA）
```

## 3. 快速开始

```bash
# ① 离线取证（不需要 CARLA、图形界面、TensorFlow；约 3~4 分钟）
python3 main.py --headless --demo --epochs 60 --samples 150 --save_dir ~/shots
#    导出：cnn_loss.png / steer_pred_vs_true.png / road_samples.png

# ② 行为克隆采集（需要 CARLA）：专家驾驶并记录 (图像, 转向)
python3 main.py --mode collect --host <宿主机IP> --frames 300 --out_dir dataset

# ③ 训练端到端 CNN
python3 main.py --mode train --data_dir dataset --epochs 60 --model_path models/cnn.json

# ④ 端到端自主驾驶（仅凭相机图像）
python3 main.py --mode test --host <宿主机IP> --model_path models/cnn.json --sim_time 20

# ⑤ ROS 2 / ROS 1
ros2 launch carla_end_to_end_nn main.launch.py host:=<宿主机IP>
roslaunch carla_end_to_end_nn main.launch host:=<宿主机IP> mode:=test
```

一键脚本：`bash main.sh --host <宿主机IP>` 或 Windows `main.bat --headless --demo`。

## 4. 测试

```bash
python3 test/test_end_to_end_logic.py     # 11 项全部通过，无需 CARLA
```

其中两项是**回归测试**，卡住两个会让网络完全学不动的缺陷：

- `test_maxpool_backward_only_max_position` —— 最大池化反向必须**只回传给窗口内最大值位置**。
  早期实现把梯度均摊到窗口全部 $p^2$ 个位置，导致网络塌缩成常数输出
  （相关系数 +0.293、方向判对率 50%）。
- `test_predict_output_in_tanh_range` —— 推理输出必须与训练目标同值域 $[-1,1]$。
  早期 `predict()` 遗漏 tanh，预测值可达 $\pm13$，MAE 高达 13.47。

另有 `test_pool_gradient_matches_finite_difference` 用有限差分校验梯度实现。

## 5. 实测指标（离线取证模式，本机实测）

| 指标 | 数值 |
|---|---|
| MAE | 0.09364 |
| RMSE | 0.11124 |
| 转向方向一致率 | 98.0% |
| 零输出基线 MAE | 0.40268（网络优于基线 **4.3 倍**） |
| 预测/标签相关系数 | +0.992 |

## 6. 与已有示例的关系

本模块是 [`ad_agent`（自动驾驶代理）](https://openhutb.github.io/ros2/ground/ad_agent/)
示例的**拓展**：已有示例用规则式局部规划沿路径点行驶，本模块改用**端到端 CNN**
直接把相机图像映射为转向，且网络可训练。
**CARLA 服务端启动、宿主机 IP 与端口查看等通用配置步骤不在此重复**，
详见 [设置并连接到 Carla 模拟器](https://openhutb.github.io/ros2/set_up_and_connect_to_carla/)
与 [模块文档](https://openhutb.github.io/ros2/ground/carla_end_to_end_nn/)。
