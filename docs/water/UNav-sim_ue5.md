# 水下机器人 Unreal Engine 5 仿真

## 1. 安装说明（针对 Windows 操作系统）

UNav-Sim 依赖于 Unreal Engine 5 生成逼真的渲染图。首先安装 UE5，然后继续安装 UNav-Sim。

### 1.1 UE5

确保你已经在 Epic Games 注册了。这是获取虚幻引擎源代码访问所必需的。

在你喜欢的文件夹里克隆 Unreal 5.1 并构建它（这需要大量的时间与内存，需要 163 GB）：

```bash
git clone -b 5.1 git@github.com:EpicGames/UnrealEngine.git
cd UnrealEngine
.\Setup.bat
.\GenerateProjectFiles.bat
```

### 1.2 UNav-Sim

在你最喜欢的文件夹里克隆这个仓库：

```bash
git clone git@github.com:open-airlab/UNav-Sim.git
cd UNav-Sim
build.cmd
```

### 1.3 构建 UE5 环境

在 `cd ~\UNav-Sim\Unreal\Environments\Blocks` 目录下：

右键点击 `Blocks.uproject` 文件，在菜单中选择“Generate Visual Studio project files”（生成 Visual Studio 项目文件）。

## 2. 运行模拟

1. 将 `ros2/src/water/ue5_UNav-Sim/settings.json` 文件添加到 `C:\[用户名]\Documents\AirSim`
2. 用 Visual Studio 打开 `Blocks.sln`
3. 进行调试
4. 进入 Unreal Engine 5 后进行模拟
![模拟图](/img/water/wmx0002.png)