# msk_tendon_basics

肌肉-肌腱串联弹性（muscle-tendon unit）单关节实验。基础的 Hill 模型把肌腱当成
刚性，肌纤维长度直接由关节角决定；这里在肌纤维外面串一根线性弹性肌腱，观察
肌腱怎么分走长度、存住能量，以及松手时怎么释放。

对应文档：[`docs/musculoskeletal/msk_tendon_basics.md`](../../../docs/musculoskeletal/msk_tendon_basics.md)

## 文件

```
main.py                     包根入口，转调 scripts/main.py
requirements.txt            matplotlib（可选，仅画图需要）
package.xml, CMakeLists.txt 纳入 catkin 工作空间
launch/main.launch          与仓库其他模块一致的启动方式

scripts/muscle_tendon.py    Hill 肌纤维 + 线性肌腱 + 串联平衡二分求解
scripts/joint_tendon.py     单自由度关节、一对拮抗肌、准静态动力学
scripts/experiments.py      等长 / 刚度扫描 / 松手释放
scripts/plots.py            matplotlib 出图
scripts/main.py             命令行入口、汇总表、CSV

tests/test_muscle_tendon.py 曲线、肌腱、平衡（18 个）
tests/test_joint_tendon.py  关节、扫描、释放（12 个）
tests/test_cli.py           命令行入口与 roslaunch 私有参数（2 个）
```

## 环境

* Python 3.8+
* matplotlib（可选，仅画图；`pip install -r requirements.txt`）
* 不需要 ROS、MuJoCo、真实机器人

模型与测试本身只用标准库；没有 matplotlib 时 `main.py` 仍会跑模型并输出 CSV，
只是跳过 PNG。

## 运行

在**包目录下**执行（不要先 cd 进 scripts）。命令里的 `python3` 在 Windows 上
可以写成 `python` 或 `py -3`：

```bash
pip install -r requirements.txt  # 可选，仅出图需要
python3 main.py                  # 完整实验，输出 demo_out/
python3 main.py --quick          # 快速跑一遍
python3 main.py --outdir DIR     # 指定输出目录
```

Windows 终端中文乱码时先执行 `chcp 65001`。

输出：

```
demo_out/isometric.png          等长传力、肌纤维长度、肌腱应变/储能
demo_out/tendon_stiffness.png   肌腱刚度-传力-储能权衡
demo_out/release.png            松手后的关节角、肌力、肌腱能量
demo_out/*.csv                  对应数值
```

放进 catkin 工作空间后：

```bash
catkin_make
source devel/setup.bash
roslaunch msk_tendon_basics main.launch
roslaunch msk_tendon_basics main.launch args:="--quick"
```

## 测试

```bash
python3 -m unittest discover -s tests -v
```

应显示 `Ran 32 tests ... OK`。

## 三组实验看什么

1. **等长传力**：固定关节角，激活度从 0 到 1。弹性肌腱被拉长 6.8%，肌纤维从
   0.85 缩到 0.816 `l_opt`，传到关节的力比刚性情形少 5.1%，同时存下 0.172 J。
2. **刚度权衡**：肌腱越硬，传力越接近刚性极限；越软，储能越多。默认参数下
   15000 N/m 存 0.306 J 但只传 95.7 N，300000 N/m 传 106.8 N 但只存 0.019 J。
3. **松手释放**：等长保持 0.4 s 后松开，肌腱从 0.134 J 卸到最低 0.062 J，
   约 54% 释放；刚性肌腱没有储能，关节峰值角略大。

## 参数

默认值取上肢骨骼肌量级：`F_max = 120 N`、`l_opt = 0.10 m`、`l_slack = 0.05 m`、
肌腱最大应变 8%（`k_t = 30000 N/m`）、关节力矩臂 0.03 m。改参数直接构造
`TendonParams(...)` / `JointParams(...)`，详见文档的参数表。

## 局限

线性肌腱、无羽状角、准静态关节、参数未按具体肌肉标定、只做单关节。适合讲基础
概念，不适合直接当生物力学预测工具。
