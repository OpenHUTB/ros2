@echo off
setlocal
if "%~1"=="" (
    python "%~dp0main.py" --selftest
) else (
    python "%~dp0main.py" %*
)
endlocal
