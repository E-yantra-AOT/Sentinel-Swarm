# start_swarm.ps1
# Launches both swarm bots in separate terminal windows.
# Run from anywhere: .\start_swarm.ps1
# Add --no-stream to skip the live feed: .\start_swarm.ps1 --no-stream

param(
    [switch]$NoStream
)

$streamFlag = if ($NoStream) { "" } else { "--stream" }

$cmdA = "ssh -o StrictHostKeyChecking=no pi@10.219.37.74 " +
        "`"PYTHONUNBUFFERED=1 ~/swarm_venv/bin/python ~/Sentinel_Ground.py --id A $streamFlag`""

$cmdB = "ssh -o StrictHostKeyChecking=no pi2@10.219.37.184 " +
        "`"PYTHONUNBUFFERED=1 ~/swarm_venv/bin/python ~/Sentinel_Ground.py --id B $streamFlag`""

Write-Host "Starting Robot A..." -ForegroundColor Cyan
Start-Process powershell -ArgumentList "-NoExit", "-Command", $cmdA

Start-Sleep -Milliseconds 500   # Slight delay so A connects first

Write-Host "Starting Robot B..." -ForegroundColor Green
Start-Process powershell -ArgumentList "-NoExit", "-Command", $cmdB

Write-Host ""
Write-Host "Both robots launched in separate windows!" -ForegroundColor Yellow
if (-not $NoStream) {
    Write-Host "Stream A: http://10.219.37.74:5000" -ForegroundColor Cyan
    Write-Host "Stream B: http://10.219.37.184:5000" -ForegroundColor Green
}
Write-Host "Close each window (or Ctrl+C inside it) to stop that robot."
