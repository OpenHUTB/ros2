# muscle_fatigue

在 Hill 肌肉模型基础上给最大力乘一个随时间变化的疲劳因子 `phi(t)`，比较
**持续收缩**与**间歇收缩**下的力衰减。控制器给的是激活度 `a`，疲劳让"同样的 `a`
能出的力"随时间和收缩节奏变化。

只用 Python 标准库，不用装 MuJoCo、ROS 或 matplotlib。

详细的说明在站点文档里：`docs/musculoskeletal/muscle_fatigue.md`

## 文件

```
main.py                        包根入口，转调 scripts/main.py
launch/main.launch             一键启动（roslaunch）
package.xml, CMakeLists.txt    纳入 catkin 工作空间

scripts/muscle_model.py        Hill 肌肉模型 + 疲劳因子（激活、力-长度、力-速度、疲劳动力学）
scripts/fatigue_model.py       疲劳动力学接口（从 muscle_model 暴露）
scripts/simulate.py            两种收缩协议的仿真与绘图
scripts/simple_plot.py         PNG 折线图，手写的，只用 zlib
scripts/main.py                演示逻辑

tests/test_muscle_model.py     Hill 模型 + 疲劳，若干断言
tests/test_fatigue_model.py    疲劳动力学，若干断言
```

## 运行

在**包目录下**直接跑（不需要先 cd 进 scripts）：

```bash
python3 main.py                  # 完整演示，图输出到 scripts/fatigue_curves.png
python3 main.py --quick          # 缩短仿真时长
python3 main.py --outdir DIR     # 指定输出目录
```

放进 catkin 工作空间后也可以用 launch：

```bash
catkin_make && source devel/setup.bash
roslaunch muscle_fatigue main.launch
roslaunch muscle_fatigue main.launch args:="--quick"
```

模块不依赖 rospy，launch 只是提供统一的启动方式与输出目录。

## 测试

```bash
python3 -m unittest discover -s tests
```

全部断言应通过。
