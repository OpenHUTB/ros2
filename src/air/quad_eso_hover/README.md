# quad_eso_hover

四旋翼高度 PD + 扩张状态观测器（ESO/ADRC）抗扰控制教学模块

## 简介

在高度通道用线性三阶 ESO 在线估计并补偿“重力+未知扰动”总扰动，实现对抗动的零稳态误差，对比普通 PD 在常值扰动下残留的稳态偏差。

## 运行

仅依赖 Python 3 标准库，无需安装第三方包：

```bash
cd src/air/quad_eso_hover
python3 scripts/main.py --outdir scripts
```

运行后会在 `scripts/` 下生成配图 PNG，并打印关键指标。

## 测试

```bash
cd src/air/quad_eso_hover
python3 tests/test_eso.py
```

## 依赖

见 `requirements.txt`（为空，表示无第三方依赖）。
