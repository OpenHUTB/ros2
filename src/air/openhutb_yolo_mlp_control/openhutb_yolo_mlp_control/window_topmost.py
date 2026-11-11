"""Set the Windows host window stacking order for a WSLg HighGUI window."""
import base64
import os
from pathlib import Path
import shutil
import subprocess


def is_wsl():
    try:
        return 'microsoft' in Path('/proc/sys/kernel/osrelease').read_text().lower()
    except OSError:
        return False


def set_wsl_topmost(title, enabled):
    """Return (success, detail); window titles are passed as data, not code."""
    executable = shutil.which('powershell.exe')
    if executable is None:
        return False, 'Windows PowerShell interop is unavailable'
    encoded_title = base64.b64encode(title.encode('utf-8')).decode('ascii')
    distro_title = f"{title} ({os.environ.get('WSL_DISTRO_NAME', '')})"
    encoded_distro_title = base64.b64encode(distro_title.encode('utf-8')).decode('ascii')
    script = r'''
$ErrorActionPreference = 'Stop'
Add-Type @'
using System;
using System.Runtime.InteropServices;
public static class YoloWindow {
    [DllImport("user32.dll", CharSet=CharSet.Unicode)]
    public static extern IntPtr FindWindowW(string className, string windowName);
    public static IntPtr Find(string title) { return FindWindowW(null, title); }
    [DllImport("user32.dll", SetLastError=true)]
    [return: MarshalAs(UnmanagedType.Bool)]
    public static extern bool SetWindowPos(IntPtr hwnd, IntPtr after,
        int x, int y, int width, int height, uint flags);
    [DllImport("user32.dll", EntryPoint="GetWindowLongW")]
    public static extern int GetWindowLong(IntPtr hwnd, int index);
}
'@
$title = [Text.Encoding]::UTF8.GetString([Convert]::FromBase64String('__TITLE__'))
$distroTitle = [Text.Encoding]::UTF8.GetString([Convert]::FromBase64String('__DISTRO_TITLE__'))
$handle = [IntPtr]::Zero
for ($attempt = 0; $attempt -lt 5; $attempt++) {
    $handle = [YoloWindow]::Find($title)
    if ($handle -eq [IntPtr]::Zero) {
        $handle = [YoloWindow]::Find($distroTitle)
    }
    if ($handle -ne [IntPtr]::Zero) { break }
    Start-Sleep -Milliseconds 100
}
if ($handle -eq [IntPtr]::Zero) { throw 'WSLg host window was not found by title' }
$after = [IntPtr](__AFTER__)
if (-not [YoloWindow]::SetWindowPos($handle, $after, 0, 0, 0, 0, 0x0013)) {
    throw 'Windows SetWindowPos failed'
}
$actual = (([YoloWindow]::GetWindowLong($handle, -20) -band 8) -ne 0)
if ($actual -ne (__ENABLED__)) { throw 'Window manager did not apply topmost state' }
Write-Output 'OK'
'''
    script = script.replace('__TITLE__', encoded_title).replace(
        '__DISTRO_TITLE__', encoded_distro_title).replace(
        '__AFTER__', '-1' if enabled else '-2').replace(
        '__ENABLED__', '$true' if enabled else '$false')
    encoded_script = base64.b64encode(script.encode('utf-16le')).decode('ascii')
    try:
        result = subprocess.run(
            [executable, '-NoProfile', '-NonInteractive', '-WindowStyle', 'Hidden',
             '-EncodedCommand', encoded_script],
            capture_output=True, timeout=8, check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return False, str(exc)
    if result.returncode == 0 and result.stdout.strip() == b'OK':
        return True, 'Windows host window updated'
    return False, 'Windows host could not apply topmost state (check WSLg/interop)'
