# usv_los_guidance

水面无人艇（USV）视线（LOS）路径跟踪制导。

## 运行

只依赖 Python 标准库：

```bash
cd src/water/usv_los_guidance
python3 scripts/main.py --outdir scripts
```

生成 `los_path.png`（俯视轨迹：路径与船轨迹）与 `los_crosstrack.png`（横漂误差衰减），
并输出末位置与最终横漂。

## 测试

```bash
cd src/water/usv_los_guidance
python3 -m unittest discover -s tests
```

## 内容

- `scripts/los_model.py`：运动学模型、LOS 制导律、航点跟踪仿真
- `scripts/simulate.py`：组装轨迹与横漂曲线并作图
- `scripts/simple_plot.py`：仅用 zlib 的极简折线图输出
- `tests/test_los.py`：单元断言

## 说明

针对恒速欠驱动无人艇，LOS 制导把位置投影到当前路径段得到横漂误差 y_e，再以
`chi_d = chi_p + atan2(-y_e, Delta)` 给出期望航向，配合一阶航向跟踪使船收敛并贴合航线。
文档见 `docs/water/usv_los_guidance.md`。
