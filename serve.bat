@echo off
setlocal EnableExtensions

set "serve_script=mkdocs_serve.bat"
set "serve_temporary=%serve_script%.part"
set "serve_url=https://raw.githubusercontent.com/OpenHUTB/.github/refs/heads/master/serve.bat"

if exist "%serve_temporary%" del /q "%serve_temporary%"
curl.exe --fail --location --ssl-no-revoke --retry 3 --retry-delay 1 --output "%serve_temporary%" "%serve_url%"
if errorlevel 1 goto :download_failed

move /Y "%serve_temporary%" "%serve_script%" >nul
if errorlevel 1 goto :download_failed

call "%serve_script%"
exit /b %errorlevel%

:download_failed
if exist "%serve_temporary%" del /q "%serve_temporary%"
echo Failed to download %serve_script% from %serve_url%. 1>&2
exit /b 1
