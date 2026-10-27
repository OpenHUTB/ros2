# AirSim Unity 无人机曲线绕柱：复现说明

[实验报告与真实动图](../../../docs/air/airsim_obstacle_avoidance.md)。本模块使用 AirSim Python RPC，无需 ROS。launch/run_demo.ps1 是 Windows PowerShell 启动器，不是 ROS launch。

## 1. 准备模拟器

实测组合：Windows 11、Python 3.9.13 x64、Unity 2019.3.12f1、AirSim 1.8.1 Unity。不要用 Unreal 版本代替，本代码解码的是 Unity DepthVis。Unity 支持属于 AirSim 实验功能。

安装 Git、Python 3.9 x64、Unity Hub 和 Unity 2019.3.12f1。按官方要求安装 Visual Studio 2019 的“使用 C++ 的桌面开发”、v142 工具集及 Windows 10 SDK 10.0.18362。新版 Visual Studio 只有新版工具集时可能无法构建旧项目。

以下命令在 **x64 Native Tools Command Prompt for VS 2019（CMD）** 执行：

```bat
cd /d %USERPROFILE%
mkdir simulators
cd simulators
git clone --branch v1.8.1 --depth 1 https://github.com/microsoft/AirSim.git
cd AirSim
build.cmd
cd Unity
build.cmd
```

每次等待命令成功退出。第二次构建会把 Unity/AirLibWrapper/x64/Release/AirsimWrapper.dll 复制到 Unity/UnityDemo/Assets/Plugins/。缺少 DLL 时不要继续 Python 飞行。参见固定版本的 [官方 Unity 构建说明](https://github.com/microsoft/AirSim/blob/v1.8.1/docs/Unity.md)。本轮未在全新机器重装验证官方构建流程；下载或 MSBuild 失败必须先修复，不能跳过。

## 2. 克隆仓库并安装 Python 依赖

在 **PowerShell** 执行并保留此窗口；已克隆者可进入自己的仓库根目录并切到包含模块的分支。

```powershell
cd $env:USERPROFILE
git clone --branch codex/airsim_obstacle_avoidance_report https://github.com/yanfan117/ros2.git ros2_avoidance
cd .\ros2_avoidance
$repoRoot = $PWD.Path
$airSimRoot = Join-Path $env:USERPROFILE 'simulators\AirSim'
$python39 = (Get-Command python).Source
& $python39 --version
# 上一行必须是 Python 3.9.x，否则把 $python39 改为 Python 3.9 完整路径
& $python39 -m venv .venv-airsim
$pythonExe = Join-Path $repoRoot '.venv-airsim\Scripts\python.exe'
& $pythonExe -m pip install -r .\src\air\airsim_obstacle_avoidance\requirements.txt
$env:PYTHONPATH = Join-Path $airSimRoot 'PythonClient'
& $pythonExe -c "import airsim, msgpackrpc; print('AirSim client', airsim.__version__)"
```

AirSim 已安装在别处时仅修改 $airSimRoot。PYTHONPATH 必须指向 PythonClient，不是 Unity。代码不再使用作者个人绝对路径。Python 3.13 不适合本实验旧 RPC 依赖。

## 3. 布置 Unity 场景（未播放时）

Unity Hub 添加 $airSimRoot/Unity/UnityDemo 工程，用上述 Editor 打开。等待导入完成并解决编译错误。此实验直接打开 Assets/Scenes/DroneDemo.unity，不要同时加载 CarDemo。不同于官方默认 SimModeSelector 流程，本实验需要直接编辑 DroneDemo 的障碍。

将 Documents/AirSim/settings.json 的 SimMode 设为 Multirotor。已有文件先备份，合并配置，不覆盖其他载具设置。新配置的最小内容：

```json
{"SettingsVersion": 1.2, "SimMode": "Multirotor"}
```

保留 SimpleFlight 无人机和地板 Cube (65) 的碰撞体。布局使用 Unity 世界坐标；对象有父级时，Inspector 是局部坐标，须核对父级变换。

| 对象 | Position（X,Y,Z） | Scale / 要求 |
| --- | --- | --- |
| 无人机出生点 | X=-320、Z=212，保留原 DroneDemo 的 Y 与父级 | 起降区空旷，不嵌入地板，保留原机体缩放 |
| 新建 Cube，命名 avoidance_pillar | (-320,4,229) | (2,361,2)，Rotation=(0,0,0)，保留 Box Collider，Is Trigger 不勾选 |
| 地板 Cube (65) | 保留 DroneDemo 原配置 | 保留 Mesh 与 Collider，不当作障碍柱删除 |

通过 GameObject → 3D Object → Cube 创建柱子，避免重复创建。高柱使用演示截图的尺度，覆盖飞行高度；两侧留至少 5 米空旷空间。保存场景。

前视相机名称 0，方向水平朝机头；近/远裁剪面为 0.3/1000 米。项目 Color Space 为 Linear，对应默认 --depth-color-space srgb。若使用 Gamma，传 --depth-color-space linear；若修改裁剪面，传 --depth-near 和 --depth-far。先用已知距离的物体验证测距，不能把相机成功返回图像等同于距离标定正确。

此 PR 不包含整个 Unity 工程或原生 DLL，首次复现必须完成上述手工配置；只安装 Python 无法直接运行仿真。

## 4. 检查连接，不起飞

Unity 点击 Play，保持运行、不要暂停。编辑器开着不等于 RPC 已启动。在同一 PowerShell，工作目录仍为仓库根目录：

```powershell
Get-NetTCPConnection -State Listen -LocalPort 41451
& $pythonExe .\src\air\airsim_obstacle_avoidance\main.py --check-only
```

预期显示 Connected、载具位置、图像尺寸、FOV 和 Preflight OK；此命令不解锁电机，不发送移动指令。若载具名不同，用 --vehicle 指定。检查只证明接口可用，不证明避障成功。

## 5. 运行绕柱

每次重试：Ctrl+C 停止旧 Python → Unity 停止 Play → 再 Play，从相同出生点开始。只运行一个控制脚本。仓库根目录执行：

```powershell
& $pythonExe .\src\air\airsim_obstacle_avoidance\main.py --route goal --goal-x 29 --goal-y 0 --altitude 3 --speed 1.2 --stop-distance 3 --curve-distance 2 --safety-margin 1.5 --vehicle-radius 0.6 --timeout 180
```

也可使用相同参数的启动器：

```powershell
& .\src\air\airsim_obstacle_avoidance\launch\run_demo.ps1 -Python $pythonExe -AirSimClient (Join-Path $airSimRoot 'PythonClient')
```

目标为起飞后位置北向 +29 米、东向 +0 米。在本布局对应 N/E=(241,-320)，即 Unity Z=241、柱子中心后方 12 米。出生点改变需重新计算，不能直接照抄 29。altitude 3 是起飞后再升高 3 米，不是绝对离地高度。

正常输出：Connected → After takeoff → After climb → advance / curve_around_obstacle → Goal reached → Landing confirmed: True。观察飞机朝柱子飞行、沿一侧绕过、继续飞往终点。碰撞、无安全通道或无水平进展会中止并尝试降落，不要把异常降落当成成功。

## 6. 验收与故障排查

运行目录 artifacts/obstacle_avoidance/flight.csv 记录位置、速度、方向和目标；新版本在到点时记录 goal_reached，终端另确认落地。保存终端、视频和 CSV，核对终点误差小于 0.8 米；safety-margin 是规划参数，不等于实际最小间距的测量结果。

| 症状 | 检查 |
| --- | --- |
| Connection refused | Unity 是否 Play、41451 是否监听、AirsimWrapper.dll 是否加载 |
| import airsim 失败 | 是否用 Python 3.9 虚拟环境，PYTHONPATH 是否为 PythonClient |
| MSB8020 / DLLNotFoundException | 安装 v142/SDK 并完成两级 build.cmd，插件为 x64 |
| 地板碰撞、马上降落 | 保留地板 Collider，检查起点是否嵌入、升高结果和异常；停止后重置 Play |
| 相机报错或距离明显不对 | 相机名称、朝向、DepthVis、裁剪面和色彩空间是否一致 |
| No horizontal progress | 两侧空间、重复柱子、CSV；局部算法不保证所有场景可达 |
| Landing confirmed: False | 未确认落地，查看 Unity 并停止 Play 重置，不并发启动其他脚本 |

新入口、check-only、结果获取和事件记录属于本轮改进，已做离线/帮助检查，尚未重新完整实飞；原演示来自报告中的原运行命令。完整依赖安装与原生插件构建未在全新机器重装验证。

## 7. 离线测试

在仓库根目录执行，无需 Unity：

```powershell
Push-Location .\src\air\airsim_obstacle_avoidance
& $pythonExe -m unittest -v test_avoidance
Pop-Location
```

21 项测试覆盖合成深度与控制逻辑，不代表所有场景的实飞保证。动态障碍、全局规划、复杂死胡同、ROS 封装和真实无人机均未验证。
