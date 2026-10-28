# 基于测距感知与定距控制的 MuJoCo 无人机幕墙图像采集

模块围绕建筑立面的规则扫描组织航线、测距和摄影。物理环境使用 MuJoCo 3.12.0，立面由简单几何体组成；无人机采用三个平移自由度、固定姿态的稳定飞行器模型。运动由力驱动积分产生，相机图像来自仿真渲染。神经网络读取九束测距、航点偏差和当前速度，输出三个方向的归一化速度指令。

## 1. 环境与启动

安装 Python 3.10 或 3.11，在 `src/air/mujoco_facade_capture` 目录执行：

```bash
python3 -m pip install -r requirements.txt
python3 main.py --style grid --output output/grid
python3 main.py --controller neural --style plain --offset 0.5 --output output/neural
python3 train.py --output output/training
```

ROS 2 Humble 环境中可通过统一启动文件运行比例控制扫描：

```bash
ros2 launch ./launch/main.launch.py style:=grid output:=/tmp/facade_grid
```

`grid`、`brick`、`plain` 分别选择网格、砖色和素色立面。输出照片、轨迹及指标位于指定目录。更改 `--offset 0.5` 可从较近壁距开始观察控制修正。

## 2. 航线与控制原理

纵向视场角为 $\theta=50^\circ$、目标壁距为 $d$ 时，相机覆盖高度为

$$ H=2d\tan(\theta/2),\qquad W=\frac{4}{3}H. $$

设重叠率为 $r$，相邻拍摄中心的最大间隔分别为 $(1-r)W$、$(1-r)H$。程序将扫描区域均匀采样，逐行交替遍历左右端点形成蛇形航线。航点误差和速度同时低于阈值后拍照，再进入下一个航点。

九束测距射线沿不同方向指向同一平面。斜向距离 $s_i$ 经方向投影并补偿传感器安装偏移，得到垂直壁距估计：

$$ d_i=s_i\cos\alpha_i\cos\beta_i+0.14. $$

输入特征包括九束原始距离、三个方向位置误差和三个方向速度。壁距方向的位置误差由测距中值计算。单隐藏层网络将标准化特征映射到速度：

$$ h=\max(0,\hat{x}W_1+b_1),\quad \hat{v}=\operatorname{clip}(hW_2+b_2,-1,1),\quad L=\frac{1}{N}\sum_i\|\hat{v}_i-v_i\|_2^2. $$

训练标签来自包含速度阻尼的比例控制器。底层速度内环仍使用仿真力推动无人机运动。

## 3. 神经网络数据划分

训练数据由四个完整 MuJoCo 扫描序列生成，包含网格与砖色立面及不同起始壁距。独立测试序列使用训练中未出现的素色立面。按场景划分序列，避免将同一轨迹相邻图像拆进训练和测试集合。`train.py` 会重新生成测距数据、训练权重、计算独立指令误差，并在完整测试扫描中比较神经网络控制与比例控制基线。

## 4. 源码与流程

1. `facade/scene.py` 构建立面、飞行器、相机和测距传感器。
2. `facade/route.py` 计算相机覆盖尺寸及蛇形航点。
3. `facade/dataset.py` 分别运行训练场景和未见测试场景并记录传感输入与控制标签。
4. `facade/learning.py` 实现 NumPy 前馈网络、权重验证和 JSON 序列化。
5. `facade/simulation.py` 执行测距、控制、物理积分和照片保存。
6. `train.py` 训练网络并在独立立面评估；`main.py --controller neural` 加载权重运行扫描。

## 5. 运行结果与验证

在 Windows / Python 3.11 / MuJoCo 3.12.0 的 CPU 环境中，以默认随机种子和 60 个训练轮次运行训练与测试。结果为同一仿真环境下的可复现实测值。

| 指标 | 神经网络控制 | 比例控制基线 |
|---|---:|---:|
| 素色立面航点完成率 | 12 / 12 | 12 / 12 |
| 壁距均方根误差 | 0.0651 m | 0.0748 m |
| 最大速度 | 0.9863 m/s | 0.9847 m/s |
| 网络对独立测试专家速度指令 MAE | 0.0164 | — |

结果目录还会保存 `evaluation.json`、训练和测试场景清单、压缩样本以及权重文件。训练数据由脚本生成，不需要下载外部数据集。

单元测试覆盖航线输入、蛇形顺序、测距几何、力驱动位移、完整扫描、网络确定性和训练/测试场景分离：

```bash
python3 -m pip install pytest
python3 -m pytest test -q
```

## 6. 适用范围

该环境表达规则平面立面和姿态稳定后的平移运动，不等同于 Town10 地图或真实幕墙。立面纹理、光照和无人机气动均有简化，指标用于相同仿真设置下的比较。
