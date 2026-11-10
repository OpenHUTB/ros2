# 无人机气压计高度漂移监测

本项目面向多旋翼在悬停或低空飞行时，下洗气流、地面热气、暖柱风或气压扰动导致机载气压计高度输出短时间漂移的情况。模块不依赖已知气压基准，而是把气压高度与 GPS 高度、雷达测高做交叉对比，检测气压计是否被外部气压场污染。

## 1. 风险量化

气压高度与 GPS 高度差（GPS 高度在 HDOP 良好时可信）：

R_gps = clip((|Δh_baro-gps| - 2) / 15)

气压高度与雷达高度差（仅在雷达高度 < 10 m 时雷达有效）：

R_radar = clip((|Δh_baro-radar| - 1) / 5)

气压短时间变化率（下洗气流会让气压在数秒内明显波动）：

R_rate = clip((|dp/dt| - 0.5) / 3)

垂直速度加权（剧烈机动时气压本身会变化）：

R_vs = clip(|vz| / 8)

综合风险：

R = 0.40·R_gps + 0.30·R_radar + 0.20·R_rate + 0.10·R_vs

硬阈值（任一触发立即 BARO_LOST）：

- |Δh_baro-gps| ≥ 25 m；
- |Δh_baro-radar| ≥ 8 m；
- |dp/dt| ≥ 8 hPa/s。

## 2. ROS 2 接口

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/uav_alt/sample` | `std_msgs/String` | 输入 | baro/gps/radar 高度、垂直速度、气压、温度 |
| `/uav_alt/assessment` | `std_msgs/String` | 输出 | 风险、两路高度差、气压变化率 |

## 3. 构建与运行

```bash
ros2 launch uav_baro_altitude_drift main.launch.py duration:=16
```

## 4. 状态迁移

```text
CALM → DRIFT (7.2s) → BARO_LOST (10.0s) → DRIFT (12.2s) → WATCH (12.9s) → CALM (13.8s)
```

纯 Python 下 6 项 unittest 全部通过。

## 5. 限制

1. 输入由确定性仿真生成，未在真实下洗气流中实测。
2. 雷达高度只在 < 10 m 时可信，高空段不参与交叉校验。
3. GPS 高度本身在多径/峡谷环境下也会漂移，未做质量因子。
4. 模块不直接切高度源，只输出建议。

## 6. 大模型使用声明

使用大模型辅助完成代码、测试和文档初稿；作者已复核状态机与风险公式，对提交内容负责。

作者：2535010018
