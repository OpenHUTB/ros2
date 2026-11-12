$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location (Split-Path $scriptDir)
Write-Host "========================================" -ForegroundColor Cyan
Write-Host "  AUV 水质监测自主采样任务" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
python main.py
