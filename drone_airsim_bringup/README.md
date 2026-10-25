# drone_airsim_bringup

AirSim 鏃犱汉鏈烘帴鍏ュ寘锛圧OS Noetic锛夈€傚睘浜?**CarlaAir-Drone / 绌哄湴涓€浣撴棤浜烘満鑷富椋炶浠跨湡瀹為獙** 鐨勯樁娈?1锛屽搴?PR1銆?
## 杩欎釜鍖呰В鍐充粈涔堥棶棰?
CarlaAir 鎶?CARLA 鍜?AirSim 鏀惧湪鍚屼竴涓?UE4 杩涚▼閲岋紝浣?AirSim 鍙彁渚?RPC 鎺ュ彛锛屾病鏈夌幇鎴愮殑 ROS 璇濋銆傝繖涓寘鎶婃棤浜烘満鐨勪綅濮裤€佺浉鏈哄拰閫熷害鎺у埗鎺ュ埌 ROS 涓婏紝鍚庨潰鐨勬劅鐭ャ€丼LAM 鍜岀鍒扮妯″潡閮戒粠杩欓噷鍙栨暟鎹€?
## 鍔熻兘

- 杩炴帴 CarlaAir 鐨?AirSim锛堥粯璁?`192.168.189.1:41451`锛夛紱
- 鍙戝竷 `/drone/pose`锛圢ED 涓栫晫鍧愭爣锛宍geometry_msgs/PoseStamped`锛夛紱
- 鍙戝竷 `/drone/camera/image_raw`锛坄sensor_msgs/Image`锛宐gr8锛夛紱
- 璁㈤槄 `/drone/cmd_vel`锛圢ED 閫熷害锛宍geometry_msgs/Twist`锛夛紱
- 鎻愪緵 `/drone/takeoff`銆乣/drone/land` 鏈嶅姟锛坄std_srvs/Empty`锛夈€?
## 鐩綍缁撴瀯

```text
drone_airsim_bringup/
鈹溾攢鈹€ CMakeLists.txt
鈹溾攢鈹€ package.xml
鈹溾攢鈹€ README.md
鈹溾攢鈹€ ROADMAP.md
鈹溾攢鈹€ launch/
鈹?  鈹斺攢鈹€ drone_bringup.launch
鈹斺攢鈹€ scripts/
    鈹斺攢鈹€ main.py
```

## 鐜

- Windows锛欳arlaAir v0.1.7锛孉irSim 1.8.1锛岀鍙?41451锛?- Ubuntu锛?0.04.6 LTS锛孯OS Noetic锛孭ython 3.8锛?- 铏氭嫙鏈猴細`D:\ROS2004\Ubuntu2004.vmx`锛岀敤鎴?`ncut`銆?
## 渚濊禆瀹夎

ROS 渚э細

```bash
sudo apt install ros-noetic-cv-bridge ros-noetic-sensor-msgs ros-noetic-geometry-msgs
```

AirSim Python 瀹㈡埛绔槸杩欎竴姝ユ渶瀹规槗鍗′綇鐨勫湴鏂广€侾yPI 涓?`airsim==1.8.1` 鐨?sdist 鍦?Python 3.8 涓嬩細鎶?`metadata-generation-failed`锛岀洿鎺?`pip install` 瑁呬笉涓娿€傚彲鐢ㄧ殑鍋氭硶鏄粠涓€涓凡缁忚濂?AirSim 1.8.1 鐨勭幆澧冨鍒?`airsim` 鍖咃紝骞惰ˉ榻愬畠渚濊禆鐨勬棫鐗堝簱锛?
```bash
python3 -m pip install --user msgpack-python==0.5.6 tornado==4.5.3
# 鎶?airsim/ 鍜?msgpackrpc/ 澶嶅埗鍒?~/.local/lib/python3.8/site-packages/
python3 -c "import airsim; print(airsim.__file__)"
```

娉ㄦ剰 `msgpack` 蹇呴』鏄?0.5.x锛?.x 浼氭姤 `Packer.__init__() got an unexpected keyword argument 'encoding'`锛沗tornado` 蹇呴』鏄?4.5.x锛?.x 娌℃湁 `tornado.platform.auto`銆?
## 缂栬瘧

```bash
source /opt/ros/noetic/setup.bash
cd /opt/drone_ws
catkin_make
source devel/setup.bash
```

## 杩愯

鍏堢‘璁?Windows 涓?CarlaAir 宸插惎鍔紝2000 鍜?41451 閮藉湪鐩戝惉锛?
```powershell
Get-NetTCPConnection -State Listen -LocalPort 2000,41451
```

VM 閲岋細

```bash
source /opt/ros/noetic/setup.bash
source /opt/drone_ws/devel/setup.bash
roslaunch drone_airsim_bringup drone_bringup.launch host:=192.168.189.1 port:=41451
```

## 楠岃瘉

```bash
rostopic list | grep drone
rostopic hz /drone/camera/image_raw
rostopic echo -n1 /drone/pose
rosservice call /drone/takeoff
rosservice call /drone/land
```

棰勬湡杈撳嚭锛?
- 璇濋锛歚/drone/pose`銆乣/drone/camera/image_raw`銆乣/drone/cmd_vel`锛?- 鍥惧儚锛?280x960锛岀害 2.7 Hz锛?- 浣嶅Э绀轰緥锛?
```yaml
header:
  frame_id: "world_ned"
pose:
  position:
    x: -0.0519
    y: 0.0530
    z: 29.2545
```

- `takeoff` / `land` 杩斿洖绌哄搷搴旓紝鏃犳姤閿欍€?
## 瀹炴祴缁撴灉

- 鍥惧儚 1280x960锛岀害 2.7 Hz锛圓irSim RPC + 铏氭嫙鏈虹幆澧冮檺鍒讹級锛?- `takeoff`銆乣land` 璋冪敤鎴愬姛锛?- 鏈嶅姟鍥炶皟鍜屼富寰幆骞跺彂鏃讹紝AirSim 鐨?msgpackrpc 浼氭姤 `IOLoop is already running`锛屽凡鍦?`main.py` 閲岀敤绾跨▼閿佷覆琛屽寲鎵€鏈?RPC 璋冪敤銆?
## 宸茬煡闂

1. **绔彛鍐茬獊**锛欳arlaAir 鍜屾爣鍑?CARLA 閮藉崰鐢?2000 绔彛锛屼笉鑳藉悓鏃跺惎鍔ㄣ€傚仛鏃犱汉鏈烘椂鍙惎鍔?CarlaAir銆?2. **鍥惧儚棰戠巼**锛?.7 Hz 涓昏鍙?AirSim RPC 鍜?VM 闄愬埗锛涘悗缁劅鐭ユā鍧椾細鎶婂垎杈ㄧ巼闄嶅埌 640x480锛屾垨鍙湪鎺ㄧ悊鏃跺彇鍥俱€?3. **鍧愭爣绯?*锛氬綋鍓嶄綅濮挎寜 AirSim NED 鍙戝竷锛坄frame_id: world_ned`锛夛紝ROS 渚ч粯璁?ENU锛屽悗缁湪鎰熺煡/鎺у埗鑺傜偣閲屽仛杞崲銆?4. **绾跨▼瀹夊叏**锛氭墍鏈?AirSim 璋冪敤蹇呴』璧板悓涓€鎶婇攣锛屼笉瑕佸湪澶氫釜鍥炶皟閲岀洿鎺ュ苟鍙戣皟鐢ㄣ€?
## 鏁翠綋浠诲姟杩涘害

瑙?[ROADMAP.md](ROADMAP.md)銆傚綋鍓嶅浜庨樁娈?1 鐨?PR1锛屽悗缁?PR2锛堟繁搴?IMU锛夈€丳R3锛堥敭鐩橀仴鎺э級浼氬湪杩欎釜鍖呬笂缁х画鍔犮€?