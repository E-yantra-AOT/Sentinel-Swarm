# Deploy and launch the full Sentinel Swarm (2 ground bots + 1 drone)
# Usage:
#   .\start_sentinel.ps1             # launch all 3 nodes
#   .\start_sentinel.ps1 --no-drone  # ground bots only
#   .\start_sentinel.ps1 --stream    # enable MJPEG streams on bots

param(
    [switch]$NoDrone,
    [switch]$Stream
)

# ── Network config ────────────────────────────────────────────────────────────
$RobotA_IP   = "10.219.37.74"
$RobotA_User = "pi"
$RobotB_IP   = "10.219.37.184"
$RobotB_User = "pi2"
$Drone_IP    = "10.219.37.35"    # Pi 5 (pi3)
$Drone_User  = "pi3"

$StreamFlag  = if ($Stream) { "--stream" } else { "" }

Write-Host ""
Write-Host "=============================================" -ForegroundColor Cyan
Write-Host "   Sentinel Swarm — Full System Launch" -ForegroundColor Cyan
Write-Host "=============================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "  Ground Bot A : $RobotA_User@$RobotA_IP" -ForegroundColor Green
Write-Host "  Ground Bot B : $RobotB_User@$RobotB_IP" -ForegroundColor Green
if (-not $NoDrone) {
    Write-Host "  Drone Pi 5  : $Drone_User@$Drone_IP" -ForegroundColor Yellow
}
Write-Host ""

# ── Launch Ground Bot A ───────────────────────────────────────────────────────
Write-Host "[1/3] Starting Ground Bot A..." -ForegroundColor Green
Start-Process powershell -ArgumentList @(
    "-NoExit",
    "-Command",
    "ssh ${RobotA_User}@${RobotA_IP} 'source ~/swarm_venv/bin/activate && python3 ~/phase7_swarm_tracker.py --id A $StreamFlag'"
) -WindowStyle Normal

Start-Sleep -Milliseconds 800

# ── Launch Ground Bot B ───────────────────────────────────────────────────────
Write-Host "[2/3] Starting Ground Bot B..." -ForegroundColor Green
Start-Process powershell -ArgumentList @(
    "-NoExit",
    "-Command",
    "ssh ${RobotB_User}@${RobotB_IP} 'source ~/swarm_venv/bin/activate && python3 ~/phase7_swarm_tracker.py --id B $StreamFlag'"
) -WindowStyle Normal

Start-Sleep -Milliseconds 800

# ── Launch Drone Companion ────────────────────────────────────────────────────
if (-not $NoDrone) {
    Write-Host "[3/3] Starting Drone companion (Pi 5)..." -ForegroundColor Yellow
    Start-Process powershell -ArgumentList @(
        "-NoExit",
        "-Command",
        "ssh ${Drone_User}@${Drone_IP} 'source ~/drone_venv/bin/activate && python3 ~/drone_companion.py'"
    ) -WindowStyle Normal
}

Write-Host ""
Write-Host "=============================================" -ForegroundColor Cyan
Write-Host " All nodes launched!" -ForegroundColor Cyan
if ($Stream) {
    Write-Host ""
    Write-Host " Live Streams:" -ForegroundColor White
    Write-Host "   Bot A  : http://${RobotA_IP}:5000" -ForegroundColor Gray
    Write-Host "   Bot B  : http://${RobotB_IP}:5000" -ForegroundColor Gray
}
Write-Host "=============================================" -ForegroundColor Cyan
Write-Host ""
