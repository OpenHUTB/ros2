# -*- coding: utf-8 -*-
"""无人机激光雷达避障脚本（OpenHUTB / CARLA 仿真）。

运行::

    python drone_control.py

按 Ctrl+C 退出；退出时停止雷达监听并把无人机速度置零，避免重复启动残留活动回调。
"""

import sys
import time

import carla


def main():
    # 连接 OpenHUTB 仿真服务
    try:
        client = carla.Client("127.0.0.1", 2000)
        client.set_timeout(20.0)
        world = client.get_world()
    except Exception as exc:
        print("❌ 无法连接仿真服务: %s" % exc)
        return 1

    print("✅ 成功接入 OpenHUTB 仿真世界")

    # 遍历场景 Actor，查找无人机
    drone = None
    for actor in world.get_actors():
        if actor.type_id.find("drone") != -1:
            drone = actor
            print("找到无人机，ID:%s, 类型:%s" % (drone.id, actor.type_id))
            break
    if drone is None:
        print("❌ 场景内没有找到无人机 Actor")
        return 1

    # 查找挂载在无人机上的激光雷达
    lidar = None
    for sensor in world.get_actors():
        if ("lidar" in sensor.type_id and sensor.parent is not None
                and sensor.parent.id == drone.id):
            lidar = sensor
            print("✅ 找到机载激光雷达")
            break

    min_dist = 100.0

    # 激光雷达回调函数，读取点云
    def lidar_callback(pc_data):
        nonlocal min_dist
        min_dist = 100.0
        for p in pc_data:
            d = p.distance
            if d < min_dist:
                min_dist = d

    if lidar:
        lidar.listen(lidar_callback)
    else:
        print("⚠️ 未检测到机载激光雷达，仅执行匀速前进")

    try:
        while True:
            # 避障决策逻辑
            if min_dist < 4.0:
                print("前方障碍物 %.2fm，停止飞行" % min_dist)
                drone.set_velocity(carla.Vector3D(0, 0, 0))
            else:
                print("前方安全 %.2fm，向前飞行" % min_dist)
                drone.set_velocity(carla.Vector3D(2, 0, 0))
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n程序终止 Ctrl+C")
    finally:
        # 显式停止传感器监听并停车，避免重复启动残留活动回调
        if lidar:
            lidar.stop()
        try:
            drone.set_velocity(carla.Vector3D(0, 0, 0))
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
