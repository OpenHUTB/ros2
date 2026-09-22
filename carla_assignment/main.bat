@echo off
rem ============================================================
rem  CARLA 0.9.16 Job 1 - Windows entry (keyboard control)
rem  Usage: main.bat control [--sim_time N] [--follow]
rem  Start CARLA server first (another terminal):
rem    CarlaUE4.exe -carla-rpc-port=2000 -quality-level=Low
rem ============================================================
setlocal
set "ROOT=%~dp0"

if "%~1"=="" (
  echo [Usage] main.bat ^<task^> [args...]
  echo   tasks: control
  exit /b 1
)

set "TASK=%~1"
shift

if "%TASK%"=="control" (
  python "%ROOT%01_control\main.py" %1 %2 %3 %4 %5 %6 %7 %8 %9
) else (
  echo [Error] unknown task: %TASK%
  exit /b 1
)
endlocal
