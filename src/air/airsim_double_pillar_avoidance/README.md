# AirSim Unity 无人机双柱连续 S 形绕行：复现说明

[实验报告](../../../docs/air/airsim_double_pillar_avoidance.md)。本模块使用 AirSim Python RPC，无需 ROS。launch/run_demo.ps1 是 Windows PowerShell 启动器，不是 ROS launch。

> 实飞状态：2026-11-06 用户在本机 Unity 实飞两次均到达目标（终点误差 0.74 m，绕行了 3 根柱，第二柱反向重选形成 S 段），核心逻辑已验证。遗留缺陷：到点后 `landAsync` 30 s 超时、`landing_unconfirmed`（根因是插件 `landed_state` 空中仍报 Landed）。代码已把起飞/降落改用开环速度通道＋实测高度判定，**该降落修复尚未复飞验证**。详见[实验报告](../../../docs/air/airsim_double_pillar_avoidance.md)。

## 0. 与单柱版本的关系

模块继承 PR #597（`src/air/airsim_obstacle_avoidance`）的深度解码、通道膨胀、方向评分与看门狗。差异集中在**绕行侧状态机 SlalomState**：

- 单柱版本锁定绕行侧后，要连续 6 帧 `advance` 才解锁，且解锁后可能重新选到同一侧。
- 双柱版本在越过一根柱子后主动解锁侧向选择，并加一段短冷却：冷却期内即使仍看到刚越过的柱子残留深度，也不立即重新锁侧，避免在两柱之间的走廊里来回摇头。这样第二根柱子若位于航线另一侧，会选到相反绕行侧，形成 S 形路径。
- 顺带修掉 #597 评审指出的问题：扫描偏航速率不再硬编码 15°/s（会被 ±12 限幅吞掉），与 `smooth_yaw_rate` 共用 `MAX_YAW_RATE` 常量。

## 1. 准备模拟器

实测组合（本机已在此环境实飞）：Windows 11、Python 3.9 x64、Unity 2019.3.12f1、AirSim 1.8.1 Unity。不要用 Unreal 版本代替，本代码解码的是 Unity DepthVis。Unity 支持属于 AirSim 实验功能。

安装 Git、Python 3.9 x64、Unity Hub 和 Unity 2019.3.12f1。按官方要求安装 Visual Studio 2019 的"使用 C++ 的桌面开发"、v142 工具集及 Windows 10 SDK 10.0.18362。

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

第二次构建会把 Unity/AirLibWrapper/x64/Release/AirsimWrapper.dll 复制到 Unity/UnityDemo/Assets/Plugins/。缺少 DLL 时不要继续 Python 飞行。参见固定版本的 [官方 Unity 构建说明](https://github.com/microsoft/AirSim/blob/v1.8.1/docs/Unity.md)。

## 2. 克隆仓库并安装 Python 依赖

在 **PowerShell** 执行并保留此窗口：

```powershell
cd $env:USERPROFILE
git clone https://github.com/OpenHUTB/ros2.git ros2_double_pillar
cd .\ros2_double_pillar
$repoRoot = $PWD.Path
$airSimRoot = Join-Path $env:USERPROFILE 'simulators\AirSim'
$python39 = (Get-Command python).Source
& $python39 --version
# 上一行必须是 Python 3.9.x，否则把 $python39 改为 Python 3.9 完整路径
& $python39 -m venv .venv-airsim
$pythonExe = Join-Path $repoRoot '.venv-airsim\Scripts\python.exe'
& $pythonExe -m pip install -r .\src\air\airsim_double_pillar_avoidance\requirements.txt
$env:PYTHONPATH = (Join-Path $airSimRoot 'PythonClient') + [IO.Path]::PathSeparator + $env:PYTHONPATH
& $pythonExe -c "import airsim, msgpackrpc; print('AirSim client', airsim.__version__)"
```

PYTHONPATH 必须指向 PythonClient，不是 Unity。Python 3.13 不适合本实验旧 RPC 依赖。

## 3. 布置 Unity 双柱场景（未播放时）

Unity Hub 添加 `$airSimRoot/Unity/UnityDemo` 工程，直接打开 `Assets/Scenes/DroneDemo.unity`，不要同时加载 CarDemo。将 `Documents/AirSim/settings.json` 的 SimMode 设为 Multirotor：

```json
{"SettingsVersion": 1.2, "SimMode": "Multirotor"}
```

保留 SimpleFlight 无人机和地板 Cube (65) 的碰撞体。布局使用 Unity 世界坐标；对象有父级时 Inspector 是局部坐标，须核对父级变换。

| 对象 | Position（X,Y,Z） | Scale / 要求 |
| --- | --- | --- |
| 无人机出生点 | X=-320、Z=212，保留原 DroneDemo 的 Y 与父级 | 起降区空旷，不嵌入地板 |
| 第一根柱 pillar_1 | (-322.5, 4, 226) | (2,361,2)，Rotation=(0,0,0)，保留 Box Collider，Is Trigger 不勾选 |
| 第二根柱 pillar_2 | (-317.5, 4, 244) | (2,361,2)，同上；与 pillar_1 在 Unity X 方向相差 5 米、Z 方向再前进 18 米 |
| 地板 Cube (65) | 保留原配置 | 保留 Mesh 与 Collider，不当障碍删除 |

两柱通过 GameObject → 3D Object → Cube 创建，避免重复。

布局几何（按 AirSim NED 与 Unity 的对应关系：北向 = Unity Z 增大，东向 = Unity X 增大；此对应关系沿用 PR #597 中经真实飞行核对过的同一布局，出生点 Z=212 对应北向 212）。航线取 Unity X=-320（即 `--goal-y 0`）。Cube 的 `Scale (2,361,2)` 是整边 2 米、半宽 1 米：

- pillar_1 占 X∈[-323.5,-321.5]，靠航线一侧的表面距航线 1.5 米，位于航线以西；
- pillar_2 占 X∈[-318.5,-316.5]，表面同样距航线 1.5 米，位于航线以东；两柱沿程相隔 18 米、中心横向相隔 5 米；
- 安全膨胀半径 = 机体半径 0.6 + 额外间距 1.5 = 2.1 米 > 1.5 米，所以沿航线直行会被规划器判定不安全，**两根柱子都会触发绕行**，这是能验证解锁逻辑的前提；
- 每根柱仍有两侧可绕，代价不同：走靠近航线的一侧只需把机身偏到 X≥-319.4（pillar_1）或 X≤-320.6（pillar_2），即偏离航线 0.6 米；走远侧则要偏到 5.6 米。规划器按代价评分会选 0.6 米的近侧，因此**正确实现应形成浅 S：先东偏 0.6 米绕 pillar_1，再西偏 0.6 米绕 pillar_2**；
- 失败模式可区分：若解锁未生效、两柱走同一侧，第二柱必须改走 5.6 米的远侧，横向摆幅大得多，日志里容易识别。

361 沿用单柱实验截图中的实际 Y 缩放值（不是通用推荐高度），覆盖飞行高度；两柱外侧各留至少 5 米空旷空间。横向偏移临界值由半宽 1 米与膨胀半径 2.1 米推出：偏移 0（柱子正对航线）会退化成单柱对称场景，选边方向不确定，不能用来验证反向解锁；偏移 ≥ 3.1 米时近侧表面距航线已达 2.1 米以上，直行即可通过、绕行不会触发。因此偏移应落在 0 到 3.1 米之间，本布局取 2.5 米。保存场景。

前视相机名称 0，方向水平朝机头；近/远裁剪面 0.3/1000 米。项目 Color Space 为 Linear，对应默认 `--depth-color-space srgb`。先用已知距离的物体验证测距，不能把相机成功返回图像等同于距离标定正确。

本模块不包含整个 Unity 工程或原生 DLL，首次复现必须完成上述手工配置。

## 4. 检查连接，不起飞

Unity 点击 Play 并保持运行（编辑器开着不等于 RPC 已启动）。在仓库根目录：

```powershell
Get-NetTCPConnection -State Listen -LocalPort 41451
& $pythonExe .\src\air\airsim_double_pillar_avoidance\main.py --check-only
```

预期显示 Connected、载具位置、图像尺寸、FOV 和 Preflight OK；此命令不解锁电机、不发送移动指令。若载具名不同用 `--vehicle` 指定。检查只证明接口可用，不证明避障成功。

## 5. 运行多柱绕行（2026-11-06 已实飞两次到达目标）

每次重试：Ctrl+C 停止旧 Python → Unity 停止 Play → 再 Play，从相同出生点开始。只运行一个控制脚本。仓库根目录执行：

```powershell
& $pythonExe .\src\air\airsim_double_pillar_avoidance\main.py --route goal --goal-x 38 --goal-y 0 --altitude 3 --speed 1.2 --stop-distance 3 --curve-distance 2 --safety-margin 1.5 --vehicle-radius 0.6 --timeout 180
```

也可用相同参数的启动器：

```powershell
& .\src\air\airsim_double_pillar_avoidance\launch\run_demo.ps1 -Python $pythonExe -AirSimClient (Join-Path $airSimRoot 'PythonClient')
```

目标为起飞后位置北向 +38 米，对应 Unity Z=250，落在第二柱（Z=244）之后，保证第二柱必然进入视野。altitude 3 是起飞后再升高 3 米，不是绝对离地高度。

实飞输出顺序（本机三柱场景，2026-11-06）：Connected → After takeoff request → After climb → advance / curve_around_obstacle → `Avoidance side locked: +1`（第一柱）→ `Pillar cleared (1); side unlocked` → `Avoidance side locked: -1`（第二柱反向，S 段成立）→ `Pillar cleared (2)` → `Pillar cleared (3)` → Goal reached。终端 CSV 的 `pillars_cleared` 列随绕行递增，到点事件 `goal_reached` 的 reason 记录清除柱数。相邻两柱的锁侧符号相反即说明解锁生效。碰撞、无安全通道或无水平进展会中止并尝试降落，不要把异常降落当成成功。

**已知偏差**：实飞巡航高度 z≈-5.05 m（约离地 4.3 m），高于 `--altitude 3` 请求值，因 `takeoffAsync` 自身爬升与模块开环爬升叠加；到点后 `landAsync` 会 30 s 超时（插件 `landed_state` 空中仍报 Landed），代码已改用开环下降＋实测高度判定，但该降落修复尚未复飞。

## 6. 验收与故障排查

每次飞行在 `artifacts/double_pillar_avoidance/` 下新建"时间戳_随机后缀"目录，终端打印 Run evidence 路径；`flight.csv` 记录位置、速度、方向、目标、绕行侧与已清除柱数；`settings.json` 保存本次命令参数；`events.jsonl` 保存 `mission_started`、`pillar_cleared`、`goal_reached`、`abort`、`timeout`、`interrupted` 及独立的降落确认事件。到点不等于已落地。

验收判据（本机 2026-11-06 实飞实测，见报告 flight.csv/events.jsonl）：

- 终端出现与柱数一致的 `Pillar cleared`（本机三柱场景为 3 次），相邻两柱 `Avoidance side locked` 符号相反；
- `flight.csv` 的 `y`（东向位移）相对航线符号翻转 ≥2 次；实测最大东偏 +3.28 m、最大西偏 -3.21 m（大于设计预估的 0.6 m 近侧绕行，实际最小间距未独立测量）；
- 无新增碰撞；终点误差小于 0.8 米（实测 0.74 m）；
- **`Landing confirmed: True` 尚未达成**：实飞两次均在到点后 `landAsync` 超时、`landing_unconfirmed`。降落改用开环下降的修复已提交但未复飞，复现者需实跑确认此判据。

| 症状 | 检查 |
| --- | --- |
| Connection refused | Unity 是否 Play、41451 是否监听、AirsimWrapper.dll 是否加载 |
| import airsim 失败 | 是否用 Python 3.9 虚拟环境，PYTHONPATH 是否为 PythonClient |
| MSB8020 / DLLNotFoundException | 安装 v142/SDK 并完成两级 build.cmd，插件为 x64 |
| takeoffAsync 返回 False | 本机插件 landed_state 空中仍报 Landed，导致闭环起飞误判；模块已改用开环速度通道爬升，若仍失败检查出生点是否嵌入地板 |
| 只出现一次 pillar_cleared | 后续柱横向偏移不足 1.5 米、未在视野中部竖直带内，或目标未落在其后 |
| 相邻两柱锁同一侧 | 冷却帧数过小、`preferred_heading` 过强，或下一柱近侧被判定不安全 |
| 巡航高度高于 --altitude | takeoffAsync 与开环爬升叠加，实测约高 1.3 m，未精调 |
| No horizontal progress | 柱间走廊宽度、重复柱子、CSV；局部算法不保证所有场景可达 |
| Landing unconfirmed / landAsync 超时 | 插件 landed_state 不可靠；已改开环下降判定，复现者确认 final_z 是否回到 ground_z 附近 |

## 7. 离线测试

在仓库根目录执行，无需 Unity：

```powershell
Push-Location .\src\air\airsim_double_pillar_avoidance
& $pythonExe -m unittest -v test_avoidance
Pop-Location
```

24 项测试覆盖深度解码往返、通道膨胀、完全堵塞、绕行侧保持、偏航限幅、参数校验、运行目录隔离，以及新增的 SlalomState 解锁/冷却/双柱两次清除时序。这些测试只验证算法逻辑，不能替代仿真飞行。

## 大模型使用声明

本模块代码及文档由千问工作助理（阿里巴巴）辅助完成方案讨论、代码起草、离线测试与文档撰写；继承自 PR #597 的算法部分源自该已合并模块。提交者负责检查验证，并对提交内容的正确性和完整性负责。
