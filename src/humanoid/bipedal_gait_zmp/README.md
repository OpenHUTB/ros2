# bipedal_gait_zmp

双足步态生成与 ZMP 稳定性验证。把机器人简化成线性倒立摆（LIPM），由落脚点
计划给出 ZMP 参考，求开环周期解，再说明它为什么不稳定、怎样闭环稳住。

只用 Python 标准库。

详细说明见站点文档：`docs/humanoid/bipedal_gait_zmp.md`

## 文件

```
main.py                 包根入口，转调 scripts/main.py
launch/main.launch      一键启动（roslaunch）
package.xml, CMakeLists.txt  纳入 catkin 工作空间

scripts/lipm.py         LIPM 动力学、ZMP 反算、捕获点、解析解
scripts/footstep.py     落脚点、支撑多边形、ZMP 参考、稳定裕度
scripts/gait.py         对称步态构造、周期解、开环仿真
scripts/dcm.py          发散分量与反馈控制
scripts/simple_plot.py  PNG 折线图（手写，只用 zlib）
scripts/animate.py      行走动图渲染（需要 Pillow）
scripts/main.py         演示逻辑

tests/test_gait.py      73 个测试
```

## 运行

在**包目录下**直接跑（不需要先 cd 进 scripts）：

```bash
python3 main.py            # 完整演示，图输出到 demo_out/
python3 main.py --quick    # 跳过增益扫描
python3 main.py --gain 1.5 # 指定反馈增益
python3 scripts/animate.py # 生成行走动图（pip install pillow）
```

放进 catkin 工作空间后也可以用 launch：

```bash
catkin_make && source devel/setup.bash
roslaunch bipedal_gait_zmp main.launch
roslaunch bipedal_gait_zmp main.launch args:="--quick"
```

模块不订阅也不发布话题，launch 只是提供统一的启动方式与输出目录。

## 测试

```bash
python3 -m unittest discover -s tests
```

跑完应显示 `Ran 73 tests ... OK`。
