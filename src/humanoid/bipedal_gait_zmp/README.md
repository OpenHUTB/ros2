# bipedal_gait_zmp

双足步态生成与 ZMP 稳定性验证。把机器人简化成线性倒立摆（LIPM），由落脚点
计划给出 ZMP 参考，求开环周期解，再说明它为什么不稳定、怎样闭环稳住。

只用 Python 标准库。

详细说明见站点文档：`docs/humanoid/bipedal_gait_zmp.md`

## 文件

```
scripts/lipm.py         LIPM 动力学、ZMP 反算、捕获点、解析解
scripts/footstep.py     落脚点、支撑多边形、ZMP 参考、稳定裕度
scripts/gait.py         对称步态构造、周期解、开环仿真
scripts/dcm.py          发散分量与反馈控制
scripts/simple_plot.py  PNG 折线图（手写，只用 zlib）
scripts/main.py         演示入口

tests/test_gait.py      73 个测试
```

## 运行

```bash
cd scripts
python3 main.py            # 完整演示，图输出到 demo_out/
python3 main.py --quick    # 跳过增益扫描
python3 main.py --gain 1.5 # 指定反馈增益
```

## 测试

```bash
python3 -m unittest discover -s tests
```

跑完应显示 `Ran 73 tests ... OK`。
