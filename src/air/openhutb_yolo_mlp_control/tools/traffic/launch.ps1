param(
    [Parameter(Mandatory=$true)][string]$PythonExe,
    [ValidateSet('spawn','record','clean')][string]$Mode='record',
    [string]$CarlaHost='127.0.0.1',
    [int]$Port=2000,
    [int]$Count=8,
    [string]$Output='traffic_run',
    [double]$Duration=90
)
# Foreground launcher: Ctrl+C is delivered to Python for cleanup.
$ErrorActionPreference='Stop'
$entry=Join-Path $PSScriptRoot 'main.py'
if($Mode -eq 'spawn') {
    & $PythonExe $entry --host $CarlaHost --port $Port spawn --count $Count
} elseif($Mode -eq 'clean') {
    & $PythonExe $entry --host $CarlaHost --port $Port clean
} else {
    & $PythonExe $entry --host $CarlaHost --port $Port record --output $Output --duration $Duration
}
if($LASTEXITCODE -ne 0){throw "Traffic tool exited with code $LASTEXITCODE"}
