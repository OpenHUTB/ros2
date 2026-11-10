# uav_lidar_ground_filter

订阅 `/lidar/points`，RANSAC 分离地面与障碍：

- `/lidar/ground`：地面点；
- `/lidar/obstacles`：障碍点。

```bash
roslaunch uav_lidar_ground_filter main.launch distance_threshold:=0.2
```

节点会拒绝与 +z 夹角超过 `max_tilt_deg`（默认 35°）的“地面”，避免把大面积墙面误判为地面；找不到水平面时跳过该帧并限频告警。

测试：`python3 test/test_ground_filter_local.py`，**11/11 通过**。

## 大模型使用声明

- 工具：DeepSeek Harness（代码助手）。
- 辅助范围：整理 README/文档结构、补充边界测试与评审回复；核心 RANSAC 逻辑、参数和安全行为由作者复核。
- 责任声明：提交者对本模块代码、测试结果和文档内容负责；大模型生成内容已逐项检查，不替代真实 ROS 环境下的构建与运行验证。
