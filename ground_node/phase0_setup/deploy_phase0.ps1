# =============================================================================
# Phase 0 — Windows Deploy Script
# Copies Phase 0 scripts to both Pis over SSH and runs them automatically.
#
# Prerequisites (on your Windows PC):
#   - OpenSSH must be available (Win11 has it built-in)
#   - Both Pis must be on the same WiFi network
#   - SSH keys or password auth must work:
#       ssh pi@pi.local    → Robot A
#       ssh pi2@pi2.local  → Robot B
#
# Run this script from PowerShell:
#   cd C:\Users\ASUS\Code\Swarm\phase0_setup
#   .\deploy_phase0.ps1
# =============================================================================

param(
    [string]$RobotA = "pi@pi.local",
    [string]$RobotB = "pi2@pi2.local"
)

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$CheckScript   = Join-Path $ScriptDir "check_hardware.sh"
$InstallScript = Join-Path $ScriptDir "install_deps.sh"

function Write-Header($text) {
    Write-Host ""
    Write-Host "======================================================" -ForegroundColor Cyan
    Write-Host "  $text" -ForegroundColor Cyan
    Write-Host "======================================================" -ForegroundColor Cyan
}

function Deploy-Pi {
    param([string]$Target, [string]$Label)

    Write-Header "$Label ($Target)"

    Write-Host "`n[1/3] Copying scripts to $Target..." -ForegroundColor Yellow
    scp "$CheckScript"   "${Target}:~/check_hardware.sh"
    scp "$InstallScript" "${Target}:~/install_deps.sh"
    Write-Host "  Scripts copied." -ForegroundColor Green

    Write-Host "`n[2/3] Running hardware check on $Target..." -ForegroundColor Yellow
    Write-Host "-----------------------------------------------"
    ssh $Target "bash ~/check_hardware.sh"
    Write-Host "-----------------------------------------------"

    $answer = Read-Host "`nDid the hardware check PASS for $Label? (y/n)"
    if ($answer -ne "y") {
        Write-Host "  Stopping. Fix hardware issues on $Label before continuing." -ForegroundColor Red
        return
    }

    Write-Host "`n[3/3] Running dependency installer on $Target..." -ForegroundColor Yellow
    Write-Host "  This will take 5-15 minutes (NCNN model export included)."
    Write-Host "-----------------------------------------------"
    ssh $Target "bash ~/install_deps.sh"
    Write-Host "-----------------------------------------------"
    Write-Host "  $Label setup complete!" -ForegroundColor Green
}

# ---- Main ----
Write-Header "Swarm POC — Phase 0 Deployment"
Write-Host "  Robot A: $RobotA"
Write-Host "  Robot B: $RobotB"
Write-Host ""
Write-Host "  This script will:"
Write-Host "  1. Copy check_hardware.sh and install_deps.sh to both Pis"
Write-Host "  2. Run the hardware check on each Pi"
Write-Host "  3. (On confirmation) Run the full dependency installer on each Pi"
Write-Host ""

# Robot A
Deploy-Pi -Target $RobotA -Label "Robot A"

# Robot B
Deploy-Pi -Target $RobotB -Label "Robot B"

Write-Header "Phase 0 Complete"
Write-Host ""
Write-Host "  Both Pis are set up. Next step: Phase 1 — Arduino motor bridge sketch."
Write-Host "  See: ..\phase1_arduino\sketch_pi_motor_bridge.ino"
Write-Host ""
Write-Host "  To manually SSH in and verify:"
Write-Host "    ssh $RobotA"
Write-Host "    source ~/swarm_venv/bin/activate"
Write-Host "    python3 -c `"import ncnn; print('OK')`""
Write-Host ""
