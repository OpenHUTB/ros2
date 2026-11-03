# MuJoCo 红绿灯启停小车

MuJoCo 写的红绿灯小车，红灯停绿灯行，黄灯慢下来。

## 跑起来

```bash
pip install mujoco numpy
cd src/ground/mujoco_traffic_light_stop
python3 mujoco_traffic_light_stop/main.py
```

## 效果

红灯5秒，黄灯2秒，绿灯5秒循环。
小车从左边开过来，到停止线前1米就开始减速，红灯就停住，绿灯就继续走。

## 文件

- `main.py` 控制逻辑
- `scene.xml` 场景模型
