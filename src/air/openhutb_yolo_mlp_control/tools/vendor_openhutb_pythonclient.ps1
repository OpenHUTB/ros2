param(
    [string]$OpenHutbRoot = "D:\无人机\hutb_windows_v2.10.0"
)

$ErrorActionPreference = "Stop"

$ToolDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$PackageRoot = Split-Path -Parent $ToolDir
$RosPythonPackage = Join-Path $PackageRoot "openhutb_yolo_mlp_control"
$VendorDir = Join-Path $RosPythonPackage "vendor"
$SourceDir = Join-Path $OpenHutbRoot "PythonClient\airsim"
$DestinationDir = Join-Path $VendorDir "airsim"

if (-not (Test-Path $SourceDir)) {
    throw "找不到 OpenHUTB AirSim PythonClient: $SourceDir"
}

Write-Host "OpenHUTB PythonClient 来源: $SourceDir"
Write-Host "复制到 ROS package:       $DestinationDir"

if (Test-Path $DestinationDir) {
    Remove-Item -Recurse -Force $DestinationDir
}
New-Item -ItemType Directory -Force -Path $DestinationDir | Out-Null

Copy-Item -Path (Join-Path $SourceDir "*") -Destination $DestinationDir -Recurse -Force

# OpenHUTB/AirSim PythonClient imports legacy msgpackrpc in client.py/types.py.
# Rewrite those imports to the Python-3.12-compatible local shim shipped in this package.
Get-ChildItem -Path $DestinationDir -Filter "*.py" -Recurse | ForEach-Object {
    $path = $_.FullName
    $content = Get-Content -Raw -Encoding UTF8 $path
    $patched = [regex]::Replace(
        $content,
        '(?m)^\s*import\s+msgpackrpc[^\r\n]*$',
        'from openhutb_yolo_mlp_control.vendor import msgpackrpc_compat as msgpackrpc'
    )
    if ($patched -ne $content) {
        Set-Content -Path $path -Value $patched -Encoding UTF8
        Write-Host "patched msgpackrpc import: $path"
    }
}

$provenance = @"
Vendored from the user's installed OpenHUTB distribution.
Source: $SourceDir
Copied: $(Get-Date -Format "yyyy-MM-dd HH:mm:ss")

The AirSim PythonClient source is retained as shipped by OpenHUTB except that
legacy `import msgpackrpc` statements are redirected to the local
msgpackrpc_compat module so the ROS 2 Jazzy / Python 3.12 environment does not
need msgpack-rpc-python or Tornado 4.x.
"@
Set-Content -Path (Join-Path $VendorDir "OPENHUTB_PYTHONCLIENT_SOURCE.txt") -Value $provenance -Encoding UTF8

if (-not (Test-Path (Join-Path $DestinationDir "__init__.py"))) {
    throw "复制完成，但目标目录没有 __init__.py；请确认 $SourceDir 是 PythonClient\airsim。"
}

Write-Host ""
Write-Host "完成。现在不要 pip install airsim/msgpack-rpc-python。"
Write-Host "下一步在 ROS 2 pixi shell 中执行:"
Write-Host "  pixi add numpy msgpack"
Write-Host "  colcon build --packages-select openhutb_yolo_mlp_control --symlink-install"
