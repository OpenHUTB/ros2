# HoloOcean Windows 本机环境搭建教程

## 一、准备阶段

打开 **PowerShell**（不要用 CMD，CMD 对 `conda activate` 支持不完整）。

## 二、创建并激活专属环境

```powershell
conda create -n holo python=3.8
conda activate holo
```

- `holo` 是环境名，可以自定义。
- 激活后提示符从 `(base)` 变为 `(holo)`。

## 三、安装 HoloOcean Python 客户端

```powershell
cd "你自己的路径\holoocean\client"
pip install .
```

验证：

```powershell
python -c "import holoocean; print(holoocean.__version__)"
```

输出 `2.3.0` 即安装成功。

## 四、下载场景包
网盘地址：https://openhutb.github.io/mujoco_plugin/underwater/usage/installation/


官方下载方式（自动下载并放到默认位置）：

```powershell
python -c "import holoocean; holoocean.install('Ocean')"
```

若需自定义存放位置，场景包结构应为：

```
你的路径\
└── 2.3.0\
    └── worlds\
        └── Ocean\
```

设置环境变量（指向版本号文件夹的上一级）：

```powershell
[System.Environment]::SetEnvironmentVariable("HOLODECKPATH", "你的场景包根目录", "User")
```

例如场景包在 `E:\underwater\holoocean_worlds\2.3.0\worlds\Ocean`，则：

```powershell
[System.Environment]::SetEnvironmentVariable("HOLODECKPATH", "E:\underwater\holoocean_worlds", "User")
```

设置完环境变量后，**必须关闭当前 PowerShell 并重新打开**，变量才会生效。验证：

```powershell
echo $env:HOLODECKPATH
```

应输出你设置的路径。

## 五、跑通第一个仿真

前往已经整理好的正文文档：

[正文文档](https://openhutb.github.io/mujoco_plugin/underwater/getting_started/)

找到可直接复制运行的最简示例代码，新建文件放入官方代码。文件可以放入 `.....\holoocean\client` 中。

使用以下指令运行：

```powershell
python 文件名.py
```

> **注意**：页面链接是 v2.2.0 文档，如果本机装的是 2.3.0，示例代码通常兼容；若遇到 API 差异，把 URL 里的 `v2.2.0` 换成 `v2.3.0` 即可。