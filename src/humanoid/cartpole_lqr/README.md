# cartpole_lqr

倒立摆（Cart-Pole）的 LQR 最优平衡控制教学模块

## 简介

通过拉格朗日建模倒立摆、竖直平衡点线性化、迭代求解离散 Riccati 方程得到 LQR 增益，闭环把倒立摆稳定在竖直位置，并与无控（自由倒下）对照。

## 运行

仅依赖 Python 3 标准库，无需安装第三方包：

```bash
cd src/humanoid/cartpole_lqr
python3 scripts/main.py --outdir scripts
```

运行后会在 `scripts/` 下生成配图 PNG，并打印关键指标。

## 测试

```bash
cd src/humanoid/cartpole_lqr
python3 tests/test_cartpole.py
```

## 依赖

见 `requirements.txt`（为空，表示无第三方依赖）。
