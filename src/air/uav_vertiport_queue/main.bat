@echo off
rem 一行包装：进入本目录并运行 python main.py %*
cd /d "%~dp0" && python main.py %*
