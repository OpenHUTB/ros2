@echo off
chcp 65001 >nul
title MuJoCo ROV 3D Simulation Viewer
cd /d "%~dp0"
wsl.exe -d Ubuntu-22.04 bash -lic "./main.sh --gui"
pause

