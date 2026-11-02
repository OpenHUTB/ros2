param([string]$Python = "python", [string]$AirSimClient = "")
$ErrorActionPreference = "Stop"
if ($AirSimClient) {
    if (-not (Test-Path -LiteralPath (Join-Path $AirSimClient 'airsim'))) { throw 'PythonClient/airsim not found' }
    $env:PYTHONPATH = "$AirSimClient;$env:PYTHONPATH"
}
& $Python (Join-Path $PSScriptRoot '..\main.py') --route goal --goal-x 29 --goal-y 0 --altitude 3 --speed 1.2 --stop-distance 3 --curve-distance 2 --safety-margin 1.5 --vehicle-radius 0.6 --timeout 180
exit $LASTEXITCODE
