# 群体行人过街决策

该模块在单行人 TTC 判据之上增加群体行为：成员是否到齐、队形是否分散、
带队者是否等待，以及车辆间隙是否满足阈值。

## 使用

```bash
cd src/ground/pedestrian_group_crossing
python main.py --group-size 3 --arrived 2 --spread 1.0 --gap 4.0
```

源码：`src/ground/pedestrian_group_crossing`

## 验证

![群体过街单元测试输出](../img/ground/pedestrian_group_crossing_test.png)

本模块只验证决策规则，不替代 CARLA 场景或社会力模型。

## 运行环境验证

HUTB 2.10.1 已在 D 盘以 `Town01`、`Low` 画质启动，RPC 服务与
`OpenHUTB` 渲染窗口均正常；本模块的规则测试仍使用独立输入，不依赖在线仿真。

![HUTB Town01 运行窗口](../img/ground/hutb_runtime_town01.png)
