@echo off
rem ============================================================
rem  CARLA 0.9.16 Unmanned-Vehicle NN Assignments - Windows entry
rem  Usage:
rem    main.bat control                : task1 keyboard control
rem    main.bat perception --mode train : task2 train perception/control NN
rem    main.bat perception --mode run  : task2 online perception+tracking
rem    main.bat navigation --mode train : task3 train planning NN
rem    main.bat navigation --mode run --goal 20,8
rem    main.bat end_to_end --mode test
rem  Start CARLA server first (another terminal):
rem    CarlaUE4.exe -carla-rpc-port=2000 -quality-level=Low
rem ============================================================
setlocal
set "ROOT=%~dp0"

if "%~1"=="" (
  echo [Usage] main.bat ^<task^> [args...]
  echo   tasks: control / perception / navigation / end_to_end
  exit /b 1
)

set "TASK=%~1"
shift

if "%TASK%"=="control" (
  python "%ROOT%01_control\main.py" %1 %2 %3 %4 %5 %6 %7 %8 %9
) else if "%TASK%"=="perception" (
  python "%ROOT%02_perception\main.py" %1 %2 %3 %4 %5 %6 %7 %8 %9
) else if "%TASK%"=="navigation" (
  python "%ROOT%03_navigation\main.py" %1 %2 %3 %4 %5 %6 %7 %8 %9
) else if "%TASK%"=="end_to_end" (
  python "%ROOT%04_end_to_end\main.py" %1 %2 %3 %4 %5 %6 %7 %8 %9
) else (
  echo [Error] unknown task: %TASK%
  exit /b 1
)
endlocal
