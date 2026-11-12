$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location (Split-Path $scriptDir)
Write-Host "========================================" -ForegroundColor Cyan
Write-Host "  四旋翼 PID vs LQR 对比" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
python main.py
