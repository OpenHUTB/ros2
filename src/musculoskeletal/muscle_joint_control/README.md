# muscle_joint_control

肌肉驱动的单自由度关节仿真。控制器给的不是关节力矩，而是屈肌和伸肌的激活度，
力由 Hill 肌肉模型根据当前长度和收缩速度算出来。

只用 Python 标准库，不用装 MuJoCo、ROS 或 matplotlib。

详细的说明在站点文档里：`docs/musculoskeletal/muscle_joint_control.md`

## 文件

```
main.py                    包根入口，转调 scripts/main.py
launch/main.launch         一键启动（roslaunch）
package.xml, CMakeLists.txt  纳入 catkin 工作空间

scripts/muscle_model.py    Hill 肌肉模型（激活、力-长度、力-速度）
scripts/joint_model.py     关节和拮抗肌对，还有 PD 到激活度的分派
scripts/stiffness.py       关节刚度、工作点判定、扰动响应
scripts/simple_plot.py     PNG 折线图，手写的，只用 zlib
scripts/main.py            演示逻辑

tests/test_muscle_model.py   肌肉模型，57 个
tests/test_joint_model.py    关节和响应，37 个
tests/test_stiffness.py      刚度，40 个
```

## 运行

在**包目录下**直接跑（不需要先 cd 进 scripts）：

```bash
python3 main.py                # 完整演示，图输出到 demo_out/
python3 main.py --quick        # 只跑基本阶跃响应
python3 main.py --outdir DIR   # 指定输出目录
```

放进 catkin 工作空间后也可以用 launch：

```bash
catkin_make && source devel/setup.bash
roslaunch muscle_joint_control main.launch
roslaunch muscle_joint_control main.launch args:="--quick"
```

模块不依赖 rospy，launch 只是提供统一的启动方式与输出目录。

## 测试

```bash
python3 -m unittest discover -s tests
```

跑完应该显示 `Ran 134 tests ... OK`。
