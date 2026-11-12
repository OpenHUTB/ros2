## 这个 PR 做了什么

<!-- 一句话说明动机与改动范围 -->

## 关联任务

- [ ] 感知 / 规划 / 控制 / 端到端（勾选）
- [ ] 仿真 / ROS / 文档

## 自检清单

- [ ] `python main.py --module full --scene sparse` 可直接跑通
- [ ] `python tools/evaluate.py --scenes all --modes full --seeds 1` 无报错
- [ ] 新增图表已用 `python tools/make_figs.py` 重新生成
- [ ] `mkdocs build` 通过，新增页面已加入 `nav`
- [ ] README 的运行步骤仍然有效
- [ ] 没有提交 `data/*.npz`、`runs/`、`dist/`

## 运行效果

<!-- 粘贴截图或 GIF；说明命令、场景、关键指标 -->

```
$ python main.py --module full --scene sparse
success=True steps=435 ...
```

## 风险与回滚

<!-- 可能影响哪些模块？如何回滚？ -->
