# vru_conflict_risk_map

根据自车与行人/儿童/骑行者轨迹生成冲突风险图：输出每条 VRU 的最小距离、
最小 TTC 和 `safe/caution/critical` 等级。

## 运行

```bash
cd src/ground/vru_conflict_risk_map
python main.py
```

默认读取 `data/ego.csv` 与 `data/vru.csv`，输出 `vru_conflict_report.json`。

随包样例包含行人、儿童和骑行者三条轨迹。三条轨迹分别覆盖 `critical`、
`caution` 和 `safe`，便于确认最小距离、TTC 和风险汇总是否符合预期。

## 测试

```bash
python -m unittest discover -s test -p "test_*.py" -v
```

ROS 2 节点输入 `~/report` 参数中的两个 CSV 路径，发布 JSON `String`。该模块与
#444 的轨迹匹配指标不同，重点是最小冲突距离/TTC 与风险等级。
