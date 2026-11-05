# 侧扫声呐图像仿真与 CNN 海底目标分类

> 包名：`mjc_rov_sonar_cnn`
> 仿真环境：MuJoCo 3.1.x + ROS 2 Humble（Ubuntu 20.04.6，Python 3.8.10）
> 神经网络：三层卷积 CNN，三分类（礁石 / 管道 / 残骸）

## 1. 概述与目标

本模块在 MuJoCo 水池场景中模拟 ROV 搭载侧扫声呐，**合成**声呐回波灰度图像，并用一个轻量 CNN 完成海底目标物三分类。与已有"声呐 SLAM 建图"不同，这里关注的是**声呐图像理解**：从声呐回波纹理判断下方是礁石、管道还是残骸，为后续水下巡检提供感知层输出。

核心任务：
1. 用 MuJoCo 场景几何生成合成声呐图像（64×64 mono8）；
2. 训练小型 CNN（3 层 Conv + 2 层 FC）做三分类；
3. ROS 节点实时订阅声呐图，发布分类结果。

## 1.1 运行环境

| 项目 | 版本 |
|---|---|
| 操作系统 | Ubuntu 20.04.6 LTS（VMware 虚拟机，无 GPU） |
| ROS 2 | Humble（`/opt/ros/humble`） |
| Python | 3.8.10 |
| MuJoCo | 3.1.x（OSMesa 离屏渲染） |
| PyTorch | 2.4.1+cpu |
| NumPy | 1.24.4 |

依赖安装：

```bash
pip3 install -i https://mirrors.aliyun.com/pypi/simple/ -r requirements.txt
pip3 install torch==2.4.1+cpu torchvision==0.19.1+cpu \
  -i https://mirrors.aliyun.com/pypi/simple/ \
  --extra-index-url https://download.pytorch.org/whl/cpu
sudo apt install libosmesa6
```

## 2. 物理仿真

### 2.1 场景模型（`models/rov_sonar_pool.xml`）

- 水池：10m×10m，带 checkerboard 池底；
- 三类目标物分布在池底：
  - **礁石**：3 个圆柱堆（灰褐色，弥散回波）；
  - **管道**：2 个横放长圆柱（黄褐色，细长强回波）；
  - **残骸**：2 个 box 碎块（深灰色，弱碎回波）；
- ROV：0.25m×0.15m×0.08m 浮筒，自由漂浮在水柱中层。

### 2.2 声呐图像合成

真实侧扫声呐回波强度随目标物材质、距离、掠射角变化。本模块用高斯亮斑近似：

```text
I(x,y) = Σ_i intensity_i · exp(-((x-px_i)² + (y-py_i)²) / (2σ_i²))
+ 高斯噪声 σ=0.05
```

- 礁石：σ=4，中等弥散；
- 管道：σ=2，细长强斑；
- 残骸：σ=3，弱碎斑。

## 3. 神经网络

### 3.1 CNN 结构

```text
Input (1×64×64)
├─ Conv2d(1→16, 3×3) + ReLU + MaxPool(2)   → 16×32×32
├─ Conv2d(16→32, 3×3) + ReLU + MaxPool(2)   → 32×16×16
├─ Conv2d(32→64, 3×3) + ReLU + MaxPool(2)   → 64×8×8
├─ Linear(4096→128) + ReLU
└─ Linear(128→3)
```

参数量约 548k（主要来自 Linear(4096→128) 的 524,416 个参数），CPU 上 30 个 epoch 约 5 秒收敛。

### 3.2 数据集

`SyntheticSonarDataset` 生成 600 张合成声呐图：随机 ROV 位置 + 随机 label，按 label 控制哪类目标物亮斑强度高（主目标 0.6~0.9，背景类 0.12~0.15）。

### 3.3 训练与测试结果

```text
[train] epoch  0/30  loss=0.9125
[train] epoch 10/30  loss=0.0000
[test] 合成测试集准确率: 1.000 (200/200)
```

三类声呐样例图（CNN 推理 probs 均为 1.0）：

![CNN 分类动画](images/pr2_classify.gif)

![礁石类](images/pr2_rock.png)
![管道类](images/pr2_pipe.png)
![残骸类](images/pr2_wreck.png)

## 4. ROS 接口

| 话题 | 类型 | 频率 | 说明 |
|---|---|---|---|
| `/rov_sonar/image_raw` | sensor_msgs/Image (mono8, 64×64) | 5 Hz | 合成声呐图 |
| `/rov_sonar/odom` | nav_msgs/Odometry | 20 Hz | ROV 位姿 |
| `/rov_sonar/classification` | std_msgs/String | 随图像 | CNN 分类结果 |

## 5. 运行步骤

```bash
# 编译
colcon build --symlink-install --packages-select mjc_rov_sonar_cnn
source install/setup.bash

# 离线训练 + 测试
cd src/water/mjc_rov_sonar_cnn
bash main.sh

# ROS 实时仿真
ros2 launch mjc_rov_sonar_cnn main.launch.py
# 另开终端
ros2 run mjc_rov_sonar_cnn classify_node
```

## 6. 文件清单

```text
mjc_rov_sonar_cnn/
├── package.xml / setup.py / setup.cfg
├── main.py / main.sh
├── mjc_rov_sonar_cnn/
│   ├── common.py        # 声呐图像合成
│   ├── cnn_model.py     # CNN 定义 + 训练 + 测试
│   ├── sim_node.py      # MuJoCo + ROS 发布声呐图
│   └── classify_node.py # CNN 推理节点
├── launch/main.launch.py
├── models/rov_sonar_pool.xml
├── config/sonar_params.yaml
└── resource/mjc_rov_sonar_cnn
```

## 7. 常见故障排查
### 7.1 仿真启动类
1. **MuJoCo报错：无法创建OpenGL窗口**
    - 本模块使用OSMesa离屏渲染，无需图形界面；若报错请确认已安装`libosmesa6`依赖，执行`sudo apt install libosmesa6`
    - VMware虚拟机请开启3D加速，或确认环境变量`MUJOCO_GL=osmesa`已生效

2. **colcon编译失败，提示找不到功能包**
    - 确认已将工程放入`src`目录下，在工作空间根目录执行`colcon build`
    - 缺少依赖时执行：`rosdep install --from-paths src --ignore-src -r -y`

### 7.2 数据与训练类
1. **训练报错：PyTorch版本不匹配**
    - 本模块依赖CPU版PyTorch 2.4.1，请勿直接安装最新版；严格按照文档内pip命令指定版本安装
    - 若已安装其他版本，先执行`pip uninstall torch torchvision`卸载后重装

2. **生成数据集特征模糊、训练准确率低**
    - 检查`sonar_params.yaml`中目标强度参数，主目标回波强度需显著高于背景
    - 高斯噪声σ过大易导致特征模糊，默认0.05为最优值，请勿随意调大

### 7.3 ROS运行类
1. **收不到 `/rov_sonar/image_raw` 话题**
    - 确认仿真节点正常启动，终端无崩溃报错
    - 执行`ros2 topic list`查看话题是否存在，检查节点名与话题名拼写
    - 每次新开终端都需执行：`source install/setup.bash`

2. **分类节点启动后无输出**
    - 分类节点需订阅到声呐图像才会输出结果，请先确认仿真节点已正常发布话题
    - 检查模型权重文件路径是否正确，确保训练生成的权重文件已放入对应目录

## 8. 关键参数说明
配置文件路径：`config/sonar_params.yaml`，核心参数含义与调优建议如下：

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `image_size` | 64 | 输出声呐图像分辨率，单位像素 |
| `noise_sigma` | 0.05 | 高斯噪声强度，值越大图像噪点越多 |
| `rock_intensity` | 0.8 | 礁石类目标回波强度 |
| `pipe_intensity` | 0.9 | 管道类目标回波强度 |
| `wreck_intensity` | 0.7 | 残骸类目标回波强度 |
| `publish_rate` | 5 | ROS话题发布频率，单位Hz |

> 调优建议：若分类准确率偏低，可适当增大目标回波强度、降低噪声强度；若需更贴近真实水下环境，可逐步提高噪声系数。
