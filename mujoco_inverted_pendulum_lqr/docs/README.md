# MuJoCo 小车倒立摆 LQR稳定控制
## 一、项目简介
本项目基于 MuJoCo 物理仿真引擎搭建小车倒立摆模型，使用**LQR线性二次调节器**实现倒立摆的直立稳定控制。
小车倒立摆是典型的非线性不稳定系统，在平衡点附近线性化后，通过状态反馈控制，在摆杆受到微小扰动时，控制器驱动小车移动，保持摆杆竖直不倒。
项目包含MuJoCo模型文件、LQR控制Python脚本、项目说明文档，可直接在Ubuntu20.04环境一键运行。

![倒立摆LQR仿真演示](https://raw.githubusercontent.com/yi-lu-ping-an/ros2/update-readme/mujoco_inverted_pendulum_lqr/balance_demo.gif)

## 二、环境依赖
- 操作系统：Ubuntu 20.04
- Python版本：Python3.8（Ubuntu20.04自带）
- 核心库：
    - mujoco 3.x：物理仿真引擎
    - numpy：矩阵运算
    - scipy：求解黎卡提方程，计算LQR反馈矩阵

## 三、环境安装
### 1. 安装Python依赖包
```bash
pip3 install mujoco numpy scipy -i https://pypi.tuna.tsinghua.edu.cn/simple
###2. 验证环境是否安装成功
 python3 -c "import mujoco,numpy,scipy;print('环境安装成功')"
终端输出 环境安装成功 即代表依赖全部就绪。
```
##四、项目文件结构
mujoco_inverted_pendulum_lqr/
├── docs
│   └── README.md
├── src
│   ├── main.py
│   └── balance.xml
└── balance_demo.gif

##五、运行方式
1. 进入项目文件夹
 cd mujoco_inverted_pendulum_lqr
2. 运行入口脚本
 python3 src/main.py
仿真现象
启动仿真窗口：
 
- 初始状态：小车静止，摆杆竖直向上；

- 延迟一段时间后自动施加微小倾斜扰动；

- LQR控制器接收小车位置、小车速度、摆杆角度、摆杆角速度4维状态量；

- 自动输出控制力驱动小车，将倾斜的摆杆拉回竖直位置，并持续维持稳定。

## 六、核心原理简述

小车倒立摆动力学模型在竖直不稳定平衡点做线性化处理，得到状态空间方程：

\(\dot{\boldsymbol x}=A\boldsymbol x+B u\)
状态向量：\(\boldsymbol x=[x,\dot x,\theta,\dot\theta]^T\)

- x：小车位置
- \(\dot x\)：小车速度
- \(\theta\)：摆杆倾角
- \(\dot\theta\)：摆杆角速度

LQR 通过求解代数黎卡提方程，得到最优状态反馈增益矩阵 K，控制律：

\(u=-K\boldsymbol x\)
实时读取仿真状态，计算控制力作用在小车上，实现摆杆稳定。

## 七、代码与模型优化点

1. 修复 MuJoCo 几何体属性冲突，正确设置刚体质量、转动惯量、关节阻尼；
2. 修正摆杆质心距离，LQR 矩阵参数和物理模型严格匹配；
3. 修正仿真时序：**先读取状态，计算控制量，再执行物理步进**，消除控制滞后；
4. 增加延时逻辑：仿真前期保持竖直，延迟施加扰动，方便录制演示 GIF；
5. 添加时间休眠，限制仿真运行速度，防止引擎超速一闪而过，便于观察与录屏。

## 八、常见问题
1. numpy 版本警告
> 不影响运行，如果需要消除警告，可以降低 numpy 版本：
pip3 install numpy==1.21
2. 仿真窗口打不开
确认 MuJoCo 安装完整，虚拟机开启图形渲染。
3. 摆杆扰动后无法稳住
检查 A、B 矩阵参数，确认模型质量、杆长参数和代码内参数保持一致。

## 辅助大模型：豆包，deepseek
