# 一键启动多 AUV 协同围捕示例
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$moduleDir = Split-Path $scriptDir

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "  多 AUV 协同围捕与动态任务分配" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host "  模块目录: $moduleDir" -ForegroundColor Gray
Write-Host ""

# 切到模块目录
Set-Location $moduleDir

# 检查依赖
try {
    python -c "import numpy" 2>$null
    if ($LASTEXITCODE -ne 0) {
        Write-Host "[WARN] 未安装 numpy，正在安装..." -ForegroundColor Yellow
        pip install numpy
    }
} catch {
    Write-Host "[WARN] 无法检查 numpy，继续尝试运行..." -ForegroundColor Yellow
}

# 运行
python main.py
