# AirSim Unity depth obstacle avoidance

报告、真实动图、环境和限制见 [实验报告](../../../docs/air/airsim_obstacle_avoidance.md)。

此模块直接调用 AirSim Python API，不要求安装 ROS；launch/run_demo.ps1 是 PowerShell 启动器，不是 ROS launch 文件。

## 安装

使用 Python 3.9，在仓库根目录执行：

```powershell
python -m venv .venv-airsim
& .\.venv-airsim\Scripts\python.exe -m pip install -r .\src\air\airsim_obstacle_avoidance\requirements.txt
$env:PYTHONPATH = 'C:\Users\颜帆\AirSim\AirSim-1.8.1\PythonClient'
```

将 PYTHONPATH 替换为本机 AirSim 1.8.1 源码中的 PythonClient 目录，不要指向 Unity 目录。实际运行环境已有编译完成的 Unity AirsimWrapper 插件；本模块不负责构建该插件。旧 RPC 依赖仅用于复现实验，不建议替换成 Python 3.13。

## 场景与运行

Unity 2019.3.12f1 打开 AirSim UnityDemo 的 DroneDemo，载具名 SimpleFlight、前视相机 0。柱子中心 Unity X=-320、Z=229，宽和深均 2 米，保留碰撞体；出生点 X=-320、Z=212。保持起降区空旷，停止上次脚本并重新 Play。

在仓库根目录执行（参数化启动器）：

```powershell
& .\src\air\airsim_obstacle_avoidance\launch\run_demo.ps1 -Python "$PWD\.venv-airsim\Scripts\python.exe" -AirSimClient 'C:\Users\颜帆\AirSim\AirSim-1.8.1\PythonClient'
```

用户成功运行的是模块目录中的 avoidance.py 命令；新 main.py/启动器复用同一函数，已检查帮助入口，未另做一轮真实飞行验证。直接运行命令见实验报告。起飞后再升高 3 米，不等于相对地面的绝对高度 3 米。只有在所述出生点，goal-x 29 才对应柱子中心后方 12 米。

## 离线测试

在模块目录执行：

```powershell
& ..\..\..\.venv-airsim\Scripts\python.exe -m unittest -v test_avoidance
```

21 项合成深度与控制逻辑测试通过；不代表对所有场景的无碰撞保证。输出位于运行目录 artifacts/obstacle_avoidance/flight.csv。
