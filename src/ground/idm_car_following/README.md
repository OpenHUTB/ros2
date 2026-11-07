# idm_car_following

智能驾驶员模型（IDM）跟车与车队减速波仿真教学模块

## 简介

实现经典微观跟驰模型 IDM，仿真单跟随车在不同前车速度下的间距/速度收敛，以及车队中减速波向后传播并逐车衰减的弦向稳定性现象。

## 运行

仅依赖 Python 3 标准库，无需安装第三方包：

```bash
cd src/ground/idm_car_following
python3 scripts/main.py --outdir scripts
```

运行后会在 `scripts/` 下生成配图 PNG，并打印关键指标。

## 测试

```bash
cd src/ground/idm_car_following
python3 tests/test_idm.py
```

## 依赖

见 `requirements.txt`（为空，表示无第三方依赖）。
