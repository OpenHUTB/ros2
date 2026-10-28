# 无人机兴趣点(POI)环绕航拍

给定任意地面 POI, 自动生成圆周/螺旋航线, 偏航始终对准目标。

## 几何

- 圆周: `x = poi_x + r*cos(θ)`, `y = poi_y + r*sin(θ)`
- 偏航: `yaw = atan2(poi_y - y, poi_x - x)`
- 螺旋: 每圈高度递增

## 运行

```bash
cd src/air/uav_poi_orbit
python3 main.py
```

发布 `/drone/setpoint_position` 和 `/drone/target_yaw`。参数详见模块 `README.md`。
