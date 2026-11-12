$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location (Split-Path $scriptDir)
Write-Host "========================================" -ForegroundColor Cyan
Write-Host "  仓储 AGV 多车交通管理" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
python main.py
