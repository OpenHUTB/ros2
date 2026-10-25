# drone_airsim_bringup

AirSim 鏃犱汉鏈?ROS Noetic 鎺ュ叆鍖咃紙OpenHUTB 浣滀笟锛岄樁娈?1 / PR1锛夈€?
## 鍔熻兘

- 杩炴帴 CarlaAir锛圓irSim 1.8.1锛岄粯璁?`192.168.189.1:41451`锛夛紱
- 鍙戝竷 `/drone/pose`锛圢ED 涓栫晫鍧愭爣锛宍geometry_msgs/PoseStamped`锛夛紱
- 鍙戝竷 `/drone/camera/image_raw`锛坄sensor_msgs/Image`锛宐gr8锛夛紱
- 璁㈤槄 `/drone/cmd_vel`锛圢ED 閫熷害锛宍geometry_msgs/Twist`锛夛紱
- 鎻愪緵 `/drone/takeoff`銆乣/drone/land` 鏈嶅姟锛坄std_srvs/Empty`锛夈€?
## 鐜

- Ubuntu 20.04 + ROS Noetic锛?- Windows 涓婅繍琛?CarlaAir锛圓irSim 绔彛 41451锛夛紱
- VM 鍐呭畨瑁?AirSim 1.8.1 Python 瀹㈡埛绔紙`import airsim` 鍙敤锛夈€?
## 缂栬瘧

```bash
source /opt/ros/noetic/setup.bash
cd /opt/drone_ws
catkin_make
source devel/setup.bash
```

## 杩愯

```bash
source /opt/ros/noetic/setup.bash
source /opt/drone_ws/devel/setup.bash
roslaunch drone_airsim_bringup drone_bringup.launch host:=192.168.189.1 port:=41451
```

## 楠岃瘉

```bash
rostopic hz /drone/camera/image_raw
rostopic echo -n1 /drone/pose
rosservice call /drone/takeoff
rosservice call /drone/land
```

## 璇存槑

- 浣嶅Э鏆傛寜 AirSim 鐨?NED 鍧愭爣绯诲彂甯冿紙`frame_id: world_ned`锛夛紝鍚庣画鍦ㄦ劅鐭?鎺у埗妯″潡閲岃浆 ROS ENU锛?- 鐩告満榛樿浣跨敤 AirSim 璁剧疆閲岀殑 `Cameras["0"]`锛屽垎杈ㄧ巼 1280x960锛?- CarlaAir 涓庢爣鍑?CARLA 閮藉崰鐢?2000 绔彛锛屼笉鑳藉悓鏃惰繍琛屻€?