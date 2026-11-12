# reverse_parking_planner 离线回归测试接入 pytest

`tests/test_reverse_parking_planner_local.py` 是 `src/ground/reverse_parking_planner` 的离线回归测试，
包含 197 项检查，覆盖默认车位参数、角度归一化、路径计算、包结构与文档一致性。
测试只依赖 Python 标准库，不需要 ROS、CARLA 或显示器。

早期所有检查都写在模块顶层，脚本结束前直接 `sys.exit(0)`。直接用 Python 运行没有问题，但 pytest
在收集阶段导入文件时会被 `SystemExit: 0` 打断：

```text
ERROR tests/test_reverse_parking_planner_local.py - SystemExit: 0
!!!!!!!!!!!!!!!!!!!! Interrupted: 1 error during collection !!!!!!!!!!!!!!!!!!!!
1 error in 0.52s
```

PR #1170 在测试文件里增加了 `test_offline_regression()` 作为 pytest 入口，只断言顶层检查的
`PASSED` / `FAILED` 结果；汇总打印和退出码移入 `if __name__ == "__main__":`。
检查逻辑、模块源码和直接运行方式都没有改变。

## 运行方式

以下命令均在仓库根目录执行（即包含 `tests/` 目录的目录）。

```bash
# pytest
python3 -m pytest tests/test_reverse_parking_planner_local.py -q

# 直接运行脚本
python3 tests/test_reverse_parking_planner_local.py

## 修复前后

| 命令 | 修复前 | 修复后 |
| --- | --- | --- |
| `pytest ... -q` | `ERROR ... SystemExit: 0`，退出码 2 | `1 passed` |
| `python3 tests/...` | 通过 197 项 | 通过 197 项（不变） |

## 断言顺序

`test_offline_regression()` 先检查 `FAILED`，再检查 `PASSED`：

```python
assert not FAILED, "失败 %d 项: %s" % (len(FAILED), "; ".join(FAILED))
assert PASSED, "未执行任何检查"
```

这样全部检查失败时会直接列出失败项，而不是误报“未执行”；一项都没跑时由第二条兜底。
