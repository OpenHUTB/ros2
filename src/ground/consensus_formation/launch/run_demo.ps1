# 多机器人 Consensus 编队控制 — Windows 启动脚本
$dir = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $dir
Write-Host "===================================="
Write-Host "  Consensus 编队控制  Launcher"
Write-Host "===================================="
Write-Host ""

# 检查 Python
$py = Get-Command python -ErrorAction SilentlyContinue
if (-not $py) {
    Write-Host "[ERR] 未找到 python，请到 https://www.python.org 安装 Python 3.10+"
    exit 1
}

# 检查 numpy
try {
    python -c "import numpy" 2>$null
    if ($LASTEXITCODE -ne 0) {
        Write-Host "[WARN] numpy 未安装，正在 pip install numpy ..."
        python -m pip install numpy
    }
} catch {
    python -m pip install numpy
}

Write-Host "[OK] 环境检查通过，开始运行..."
Write-Host ""
python main.py
Write-Host ""
Write-Host "[OK] 仿真完成，按回车退出"
$null = Read-Host
