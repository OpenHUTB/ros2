$ErrorActionPreference = 'Continue'
$base = "C:\Users\Repeat\Desktop\deepseek\deepseek\uav_traffic_nav"
$py = "C:\Users\Repeat\Desktop\deepseek\deepseek\.pyenv\py\python.exe"
Set-Location $base
$env:PYTHONIOENCODING = 'utf-8'
$env:OMP_NUM_THREADS = '8'
$env:MKL_NUM_THREADS = '8'

$jobs = @()
foreach ($seed in 42, 43, 44) {
    $jobs += Start-Process -FilePath $py -ArgumentList @("scripts\bc_train.py","--episodes","200","--dagger-rounds","3","--dagger-episodes","40","--seed",$seed,"--out","models/bc_policy_s$seed.npz") -PassThru -RedirectStandardOutput "$env:TEMP\bc_$seed.log" -RedirectStandardError "$env:TEMP\bc_$seed.err" -WindowStyle Hidden -WorkingDirectory $base
}
Write-Host ("started " + $jobs.Count + " DAgger runs: " + (($jobs | ForEach-Object { $_.Id }) -join ', '))
foreach ($job in $jobs) { $job.WaitForExit() }
Write-Host "all DAgger runs finished"
foreach ($seed in 42, 43, 44) {
    Write-Host ("--- seed $seed ---")
    Get-Content "$env:TEMP\bc_$seed.log" -Encoding UTF8 -ErrorAction SilentlyContinue | Select-Object -Last 5
}
