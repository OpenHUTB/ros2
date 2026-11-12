@echo off
setlocal
cd /d "%~dp0"
python main.py %*
exit /b %errorlevel%
