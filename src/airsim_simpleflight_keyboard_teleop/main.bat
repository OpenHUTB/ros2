@echo off
setlocal

set "AIR_SIM_EXE=<PATH_TO_AirSimNH.exe>"

if not exist "%AIR_SIM_EXE%" (
    echo Edit AIR_SIM_EXE in main.bat first.
    exit /b 1
)

powershell.exe -NoProfile -ExecutionPolicy Bypass -Command ^
  "$d = Join-Path ([Environment]::GetFolderPath('MyDocuments')) 'AirSim';" ^
  "New-Item -ItemType Directory -Force -Path $d | Out-Null;" ^
  "Copy-Item -LiteralPath '%~dp0airsim_settings.json' -Destination (Join-Path $d 'settings.json') -Force"

if errorlevel 1 exit /b 1

start "" "%AIR_SIM_EXE%" -windowed -ResX=1280 -ResY=720
