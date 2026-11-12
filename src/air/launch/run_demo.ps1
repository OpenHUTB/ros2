# 一键启动无人机集群编队演示
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location (Split-Path $scriptDir)
Write-Host "========================================" -ForegroundColor Cyan
Write-Host "  无人机集群编队控制演示" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
python main.py
