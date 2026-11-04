@echo off
rem 一键入口（Windows）：main.bat --mode scene --host 192.168.148.1
cd /d %~dp0
python main.py %*
