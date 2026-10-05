# AirSim 1.8.1 仿真器安装

## Windows 原生（最简单）

1. 安装 [Unreal Engine](https://www.unrealengine.com/) 4.27（通过 Epic Games Launcher）；
2. 下载 AirSim 发布包：<https://github.com/microsoft/AirSim/releases/tag/v1.8.1>；
3. 解压后双击 `run.bat` 启动 Blocks 地图；
4. 无人机默认在 (0,0,0) 附近起飞，API 端口为 **41451**。

## 从源码编译（Ubuntu）

```bash
git clone https://github.com/microsoft/AirSim.git -b v1.8.1
cd AirSim
./setup.sh
./build.sh
cd Unreal/Environments/Blocks && ./run.sh
```

## 配置多无人机与相机/LiDAR

编辑 `~/Documents/AirSim/settings.json`（Windows 为 `C:\Users\<你>\Documents\AirSim\settings.json`）：

```json
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "Vehicles": {
    "drone0": { "VehicleType": "SimpleFlight", "X": 0, "Y": 0, "Z": 0,
      "Cameras": {
        "0": { "X": 0.6, "Y": 0, "Z": -0.2, "Pitch": 0, "Yaw": 0, "Roll": 0 }
      },
      "Lidar": { "Lidar1": { "Range": 30, "RotationsPerSecond": 10,
        "PointsPerRotation": 360, "X": 0, "Y": 0, "Z": -0.2 } }
    }
  }
}
```

保存后重启仿真器，`client.get_depth("0")`、`client.get_lidar()` 即可返回数据。

> 截图占位：`assets/airsim_blocks.png`（Blocks 地图中带前视相机与 LiDAR 的无人机）。
