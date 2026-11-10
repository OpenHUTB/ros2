# AirSim Unity 无人机高度自适应越障：复现说明

[实验报告](../../../docs/air/airsim_height_aware_overfly.md)。本模块使用 AirSim Python RPC，无需 ROS。launch/run_demo.ps1 是 Windows PowerShell 启动器，不是 ROS launch。

> **实飞状态：2026-11-09 完成两次矮墙越障实飞，均越障到点并确认降落。** committed 爬升 2.06/2.14 m，实测抬升 1.97/2.05 m，越障后保持高度 5.22/5.25 m，到点误差 0.58/0.67 m；证据见 [报告第 5 节](../../../docs/air/airsim_height_aware_overfly.md) 与 `docs/air/airsim_height_aware_overfly/` 下的 CSV/事件日志。这两次数据是相机偏移修正前采集的（当时实际净空约 1.53/1.61 m）；代码现已把 0.49 m 偏移计入 committed 目标，复飞时约 2.6 m、净空恢复 2.1 m，**尚未复飞**。高柱水平绕行尚未实飞。

## 0. 与单柱、双柱版本的关系

本模块沿用 PR #597（`src/air/airsim_obstacle_avoidance`）的深度解码、通道膨胀、偏航平滑、看门狗和“锁定绕行侧”思路，但增加了一个决策维度：**高度**。

- 单柱与双柱版本只扫描图像中部竖直带，巡航高度全程锁定；任何障碍都只能水平绕行，遇到矮墙也要绕一大圈。
- 本模块扫描整幅图像，逐列恢复障碍顶边相对相机的高度 `top_height`，用 `climb_needed = max(0, top_height + 相机安装偏移 + 机体半径 + 安全间距)` 判断能否从上方通过（顶边低于相机时不需要额外的抬升；相机偏移约 0.49 m，见第 2 节）。
- 四种行为：`advance` 通道畅通；`overfly` 先原地爬升到顶边之上再通过，越过后再保持越障高度飞行 `--overfly-hold-distance`（默认 5 m）才下降；`curve` 障碍太高或越障已解除，水平绕行；`blocked` 无安全方向，原地扫描。

与相邻模块的边界：`uav_lidar_safety_corridor` 用 LiDAR 和 ROS 选安全高度；`uav_altitude_command_guard` 只做高度指令限幅；`octree_uav_3d_pathfinding` 做八叉树全局寻路。本模块只做前视深度图的局部决策，不建图、不做全局规划。

## 1. 模拟器与依赖

实测环境沿用同仓库已实飞验证的组合：Windows 11、Python 3.9 x64、Unity 2019.3.12f1、AirSim 1.8.1 Unity，前视相机 0，近/远裁剪面 0.3/1000 m，项目 Color Space 为 Linear（对应默认 `--depth-color-space srgb`）。完整安装步骤见 [单柱模块 README](../airsim_obstacle_avoidance/README.md) 第 1–2 节；本模块的 `requirements.txt` 与其相同。

注意：AirSim Unity 的浮点深度缓冲区与 Unity 纹理一样**自下而上**存储（本机 2026-11-09 现场帧验证），模块读取后先按行翻转再做高度分析；若你换用其他返回顺序的实现，需要同步调整。

在 **PowerShell** 中，从仓库根目录执行：

```powershell
$repoRoot = $PWD.Path
$airSimRoot = Join-Path $env:USERPROFILE 'simulators\AirSim'
$pythonExe = Join-Path $repoRoot '.venv-airsim\Scripts\python.exe'
& $pythonExe -m pip install -r .\src\air\airsim_height_aware_overfly\requirements.txt
$env:PYTHONPATH = (Join-Path $airSimRoot 'PythonClient') + [IO.Path]::PathSeparator + $env:PYTHONPATH
& $pythonExe -c "import airsim, msgpackrpc; print('AirSim client', airsim.__version__)"
```

PYTHONPATH 指向 `PythonClient`，不是 Unity 工程。Python 3.13 不适合这套旧 RPC 依赖。

## 2. 布置越障场景

在 Unity 中打开 `Assets/Scenes/DroneDemo.unity`，`Documents/AirSim/settings.json` 设 `SimMode` 为 `Multirotor`。沿用同仓库已验证的出生点：Unity `X=-320, Y=3, Z=212`（AirSim 世界坐标 `x=Unity Z`、`y=Unity X`、`z=-Unity Y`），保留 SimpleFlight 无人机和地板 Cube (65) 的碰撞体。方块通过 GameObject → 3D Object → Cube 创建，保留 Box Collider，不要勾选 Is Trigger；有父级时 Inspector 显示局部坐标，须核对父级变换。

如果场景里还留着双柱实验的两根 361 m 高立方柱（`Cube (1)` 在 Z=244、`Cube (2)` 在 Z=226），先取消激活或删除：`Cube (2)` 与本节矮墙位置重叠，且两者都会遮挡深度图。

越障场景只需两个变体，二者放在同一位置、分两次运行：

| 变体 | 对象 | Position（X,Y,Z） | Scale | 说明 |
| --- | --- | --- | --- | --- |
| A 矮墙 | wall_low | (-320, 3, 227) | (8, 6, 0.4) | 墙顶 Y=6，宽 8 m，航线北向 15 m；比巡航相机高约 0.2~0.5 m |
| B 高柱 | pillar_tall | (-320, 4.5, 227) | (1.2, 9, 1.2) | 柱顶 Y=9，正对航线，比巡航相机高约 3.2~3.5 m，验证无法越障时改水平绕行 |

**高度必须按实测巡航高度换算。** `After climb` 打印的是**机体**的 NED 高度 `z`，不是相机高度。前视相机 0 装在机体原点上方约 **0.49 m**（本机用 `simGetCameraInfo` 与 `getMultirotorState` 实测：机体 z=-0.761 时相机 z=-1.253），所以相机世界高度 ≈ `|z| + 0.49`。2026-11-09 两次实飞 `z≈-5.0~-5.3`，相机约 5.5~5.8 m；6 m 矮墙顶只比相机高约 0.2~0.5 m，`flight.csv` 的 `top_height_m` 0.04~0.11 m 与之一致（顶边像素混叠会再低一两行，约 0.09 m/行）。

第 3 节先空场景飞一次读 `z`，再按 `墙顶 Y ≈ |z| + 0.5 ~ 1.0`（相机上方 0~0.5 m）放墙；`柱顶 Y ≈ |z| + 4`（相机上方约 3.51 m，`climb_needed` ≈ 3.51 + 0.49 + 2.1 ≈ 6.1 m，超出 `--max-climb 4.5` 判为太高）。模块连接时用 `simGetCameraInfo` 实测相机偏移（0.49 m），并把它计入 `climb_needed` 和 committed 目标——否则实际净空会比配置的 `机体半径 + 安全间距` 少 0.49 m。按 `--max-climb 4.5`，能越的顶边最多比相机高 4.5 − 0.49 − 2.1 ≈ 1.9 m，即墙顶高于 `|z| + 2.4` 时判太高；顶边低于相机超过 0.49 + 2.1 ≈ 2.6 m 时 `climb_needed` 归零、不再抬升。四周至少留 5 m 空旷，飞行高度内不要放其他物体。

航线取 Unity `X=-320`（即 `--goal-y 0`）。墙在 `Z=227`，目标取北向 +20 m（Unity `Z=232`），保证越过墙体后再到点。

## 3. 检查连接（不起飞）

Unity 点击 Play 并保持运行。仓库根目录执行：

```powershell
Get-NetTCPConnection -State Listen -LocalPort 41451
& $pythonExe .\src\air\airsim_height_aware_overfly\main.py --check-only
```

预期显示 Connected、载具位置、图像尺寸、FOV 和 Preflight OK；此命令不解锁电机、不发送移动指令。确认图像尺寸与 FOV 合理，并记下位置。

保持场景空旷，先飞一次 1 m 的短程任务：

```powershell
& $pythonExe .\src\air\airsim_height_aware_overfly\main.py --goal-x 1 --goal-y 0 --timeout 60
```

只观察终端里的 `After climb: z=...`。这个值是机体的 NED 高度（不是相机高度）；相机高度 ≈ `|z| + 0.49 m`，按第 2 节换算墙高后再放障碍。**此步会起飞**，确保起降区空旷。

## 4. 运行高度自适应越障

每次重试：Ctrl+C 停止旧 Python → Unity 停止 Play → 再 Play，从相同出生点开始。只运行一个控制脚本。仓库根目录执行：

```powershell
& $pythonExe .\src\air\airsim_height_aware_overfly\main.py --route goal --goal-x 20 --goal-y 0 --altitude 3 --max-climb 4.5 --max-altitude 10 --speed 1.2 --stop-distance 3 --curve-distance 2 --safety-margin 1.5 --vehicle-radius 0.6 --timeout 180
```

也可用同参数启动器：

```powershell
& .\src\air\airsim_height_aware_overfly\launch\run_demo.ps1 -Python $pythonExe -AirSimClient (Join-Path $airSimRoot 'PythonClient')
```

`--altitude 3` 是起飞后再升高 3 m，不是绝对离地高度；`--max-climb 4.5` 是单次越障允许的最大抬升，必须大于 `机体半径 + 安全间距 + 相机偏移 + 障碍顶边`（0.6 + 1.5 + 0.49 ≈ 2.6 m，再加顶边高度）。`--max-altitude 10` 是相对起飞点的绝对上限，须大于 `altitude + max_climb`。

变体 A 的预期终端顺序：

```text
advance ... → overfly_obstacle（top≈0~1.0 m，本机 0.04~0.11，overhead>0）
Overfly committed: climb ≈ top + 0.49（相机偏移）+ 2.1 m ≈ 2.6 m
overfly_obstacle: forward=0.00 ... lift 逐帧上升
overfly_obstacle: forward>0 ... 越过墙顶
Overfly completed (1); holding 2.6 m for 5.0 m past the obstacle
advance ...（保持越障高度继续飞行）
Overfly hold complete; descending to cruise altitude
Goal reached
Landing confirmed: True
```

`forward=0.00` 是设计行为：越障时先原地爬升，达到 committed 高度（差 0.3 m 以内）才前进，避免一边爬一边撞墙。`Overfly completed` 之后**不会**马上降回巡航：模块保持越障高度再飞 `--overfly-hold-distance`（默认 5 m），打印 `Overfly hold complete` 后才下降，保证越过障碍远端时仍有完整安全间距；设 `--overfly-hold-distance 0` 可关闭保持。把变体 A 换成变体 B 的高柱后重跑，预期不再出现 `Overfly committed`，而是 `curve_around_obstacle` 和 `Avoidance side locked: ±1`。另：任何场景下连续两次越障失败，模块都会打印 `Overfly disarmed for this mission`，本任务永久退化为水平绕行。

## 5. 验收判据与证据

每次飞行在 `artifacts/height_aware_overfly/` 下新建“时间戳_随机后缀”目录，终端打印 Run evidence 路径：

- `flight.csv`：22 列，含 `z`、`lift_m`（相对巡航高度的实测抬升）、`speed`、`climb`、`clearance_m`、`top_height_m`（相对相机）、`overhead_m`、`mode`、`overfly_active`、`overfly_hold`、`overfly_done`、`overfly_armed`；
- `events.jsonl`：`mission_started`、`overfly_started`、`overfly_completed`、`overfly_hold_done`、`lateral_bypass`、`overfly_failed`、`overfly_disarmed`、`goal_reached`、`abort`/`timeout`/`interrupted` 及降落确认事件；
- `settings.json`：本次命令的全部参数。

变体 A（矮墙）通过标准：

- 出现 `Overfly committed`，且 `top_height_m` 在 0–1.5 m（本机两次实测 0.04/0.11 m，墙顶与相机几乎齐平）；
- `flight.csv` 中 `lift_m` 升到 committed 高度附近（≥0.7 倍），`mode` 有连续的 `overfly` 段；
- 越过墙体后才出现 `Overfly completed`，随后 `overfly_hold=1` 持续约 5 m 路程、出现 `overfly_hold_done` 才下降，再 `Goal reached`；
- 无新增碰撞、无 `overfly_failed`，`events.jsonl` 以 `landing_confirmed` 结束，终点误差小于 0.8 m。

变体 B（高柱）通过标准：

- 全程没有 `Overfly committed`，出现 `lateral_bypass`，`mode` 出现 `curve`；
- 绕过柱子后 `Goal reached`，无碰撞。

到点不等于落地；`Landing confirmed: True` 才算降落成功。一次运行只证明这一个静态布局，不能外推为通用避障。

## 6. 离线测试

无需 Unity，在模块目录执行（`$pythonExe` 沿用第 1 节的定义）：

```powershell
Push-Location .\src\air\airsim_height_aware_overfly
& $pythonExe -m unittest -v test_avoidance
Pop-Location
```

57 项测试覆盖：深度解码（线性/sRGB 往返、非法值、自下而上缓冲区的行翻转）、相机几何、列顶边恢复、地面行排除、死像素与全无效图像、通道膨胀、矮墙越障判定、高墙/预算/天花板/顶边不可见时的水平绕行、侧向障碍否决越障、相机安装偏移计入 climb 预算与 committed 目标、越障状态机的保持/提交/只升不降/完成/清空帧中断/失败解除/越障后保持高度与新爬升取消保持、爬升进度宽限、看门狗、运行目录与事件日志、偏航限幅和参数校验。这些测试只验证算法逻辑，不能替代仿真飞行。

## 7. 已知限制

- 高柱水平绕行、多速度/多高度场景尚未实飞；矮墙越障只有 2026-11-09 两次实飞，3 帧保持、6 帧结束、10 s 爬升宽限、2 次失败解除等阈值仅由这两次数据支撑，未做多场景标定。
- 只处理静态障碍，要求相机水平前视并居中；侧后方、细杆、动态物体、玻璃/反光面和复杂死胡同未验证。
- 顶边只在图像竖直带内可验证：障碍高到填满图像上部时 `overhead=0`，模块一定改水平绕行，不会盲爬。
- 图像底部按 `--ground-rows` 当地面排除；相机俯仰、地面坡度或低空飞行会让地面进入障碍带，需重新标定。
- 越障是“原地爬升再前进”的开环高度控制，依赖推力余量和实测高度；高度上限只是相对起飞点的软件上限，不是避障保证。
- 深度解码依赖本机 Unity 插件与裁剪面、色彩空间，不能原样当作 Unreal AirSim 的米制深度。
- AirSim Unity 的 `Camera.fieldOfView` 是**垂直**视场，模块按图像高度换算焦距；前视相机 0 为 `Front_Center`，垂直 90°（16:9 下水平约 121°）。若换到返回水平 FOV 的平台（如 Unreal），必须改回按宽度换算，否则顶边高度会系统性偏差。
- 局部决策不构成全程连续曲率或固定最小间距的数学保证；仿真成功不能作为真实无人机部署结论。

## 大模型使用声明

本模块代码及文档由 DeepSeek Harness 辅助完成方案讨论、代码起草、离线测试与文档撰写；算法基线继承自已合入的单柱模块（PR #597）。提交者负责检查验证，并对提交内容的正确性和完整性负责。
