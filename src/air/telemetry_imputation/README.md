# 无人机遥测缺失重建

本模块研究**丢包之后还能不能继续用这条数据**：从最近 2 秒的观测（其中含缺段）重建被丢掉的时刻。

- 仿真器是本项目自己写的（纯 numpy 四旋翼刚体模型 + 级联 PD 控制），不依赖 AirSim、OpenHUTB 模拟器或外部资产；
- 数据由 `python main.py collect` 生成，语料随仓库以 `assets/flights.zip` 提供；
- 模型是带掩码的 GRU，**预测相对「零阶保持」的偏移**，而不是绝对位置；
- 对照包括零阶保持、线性插值、匀速外推。

完整说明、公式与实验结论见 [模块文档](../../../docs/air/telemetry_imputation.md)。

```bash
bash main.sh doctor
bash main.sh prepare      # 解压并校验语料
bash main.sh test         # 单元测试
bash main.sh train        # 训练（默认 models/imputer_42）
bash main.sh evaluate     # 测试集指标与配图
bash main.sh build
bash main.sh demo         # ROS2 回放（另一个终端开 dashboard）
```
