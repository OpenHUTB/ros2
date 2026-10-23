# 无人机测距制动实验报告 / UAV ranging and braking experiment report

## 实验目的 / Objective

比较几何真值参照控制器、前向测距控制器和轻量 MLP 控制器在同一固定航向 MuJoCo 场景中的制动表现，并记录按回合划分的数据集与训练误差。/ Compare the geometric reference, forward-ranging and lightweight MLP controllers in the same fixed-heading MuJoCo scene, and record episode-level data splits and training error.

## 方法 / Method

场景保留质量、重力和障碍碰撞；水平运动沿固定航向，竖直高度由独立 PD 回路保持。MLP 使用前向测距与闭合速度作为输入，回归解析测距控制器的水平加速度，隐藏层包含 16 个 ReLU 单元。/ The scene retains mass, gravity and obstacle contact. Horizontal motion has fixed heading and a separate PD loop holds altitude. The MLP regresses the analytical ranging controller's horizontal acceleration from forward range and closing speed, using 16 ReLU hidden units.

数据由固定种子 2026 的 30 个 12 s 回合生成，按回合划分为 18 个训练、6 个验证和 6 个测试回合；样本数分别为 21600、7200 和 7200。/ A fixed seed of 2026 generates thirty 12 s episodes, split by episode into 18 training, 6 validation and 6 test episodes, with 21600, 7200 and 7200 rows respectively.

## 结果 / Results

| 控制器 / Controller | 初速度 / Initial speed | 最小间距 / Minimum clearance | 末速度 / Final speed | 碰撞步数 / Contact steps |
| --- | ---: | ---: | ---: | ---: |
| 几何参照 / Geometric reference | 1.5 m/s | 0.6294 m | 接近 0 / near 0 | 0 |
| 测距控制 / Range control | 1.5 m/s | 0.6683 m | 接近 0 / near 0 | 0 |
| MLP 控制 / MLP control | 1.5 m/s | 0.4596 m | -0.0562 m/s | 0 |

MLP 测试集 MAE 为 0.1454 m/s²，RMSE 为 0.2604 m/s²。三种控制均在该场景完成 1000 个物理步且未产生碰撞；本结果显示 MLP 已能完成基础制动，但其最小间距小于解析测距控制器，仍有改进空间。/ MLP test MAE is 0.1454 m/s² and RMSE is 0.2604 m/s². All three controllers complete 1000 physics steps in this scene without contact. The MLP performs basic braking, while its minimum clearance is smaller than that of the analytical ranging controller, leaving room for improvement.

## 复现 / Reproduction

```bash
python dataset.py --seed 2026 --episodes 30 --output datasets
python train.py --data datasets --output models
python main.py --headless --controller reference --speed 1.5 --output results/reference
python main.py --headless --controller range --speed 1.5 --output results/range
python main.py --headless --controller mlp --model models/mlp_controller.json --speed 1.5 --output results/mlp
```

## 适用范围 / Scope

该模型用于比较固定航向简化场景中的水平制动控制，不包含完整六自由度姿态、旋翼动力学、真实传感器噪声或飞控硬件接口。指标只描述上述 MuJoCo 场景，不外推为真实飞行安全结论。/ This model compares horizontal braking in a simplified fixed-heading scene. It does not include full six-degree-of-freedom attitude, rotor dynamics, real sensor noise or a flight-controller hardware interface. The metrics apply only to the stated MuJoCo scene and are not a real-flight safety claim.
