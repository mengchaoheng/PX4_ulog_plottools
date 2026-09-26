# =============================================================================
#  PX4_ulog_plottools - one-time Python environment setup (Windows)
# =============================================================================
#  Creates the project-local environment <project_root>\.venv with the uv
#  binary that ships with the project (tools\uv\windows\uv.exe).
#
#  Nothing has to be installed beforehand: uv is bundled with the repository,
#  and the Python interpreter it needs is downloaded into the project as well.
#  The system PATH is never modified and no global package is installed, so
#  deleting the project folder also removes the entire Python environment.
#
#  Usage (first run only, from the project root):
#      .\scripts\setup_env.ps1
#
#  If PowerShell refuses to run the script, allow it for this terminal only:
#      Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
# =============================================================================

#Requires -Version 5.1
[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'

# This script lives in <project_root>\scripts\, so the root is one level up.
$scriptDir   = Split-Path -Parent $MyInvocation.MyCommand.Path
$projectRoot = Split-Path -Parent $scriptDir

$uvExe = Join-Path $projectRoot 'tools\uv\windows\uv.exe'

function Write-Step { param($Message) Write-Host "`n==> $Message" -ForegroundColor Cyan }
function Write-Ok   { param($Message) Write-Host "    [OK] $Message" -ForegroundColor Green }
function Write-Note { param($Message) Write-Host "    [i]  $Message" -ForegroundColor Yellow }
function Write-Fail { param($Message) Write-Host "    [X]  $Message" -ForegroundColor Red }

Write-Host 'PX4_ulog_plottools - Python environment setup'
Write-Host "Project root: $projectRoot"

# -----------------------------------------------------------------------------
# 0. The bundled uv is the only tool required - nothing has to be installed
# -----------------------------------------------------------------------------
if (-not (Test-Path -LiteralPath $uvExe)) {
    Write-Fail "Bundled uv not found: $uvExe"
    Write-Host 'The repository looks incomplete (tools\uv\windows\uv.exe is missing).'
    Write-Host 'Re-clone the repository to restore it.'
    exit 1
}

# Windows on ARM runs the bundled x64 build through emulation, which works but
# is worth flagging.
if ($env:PROCESSOR_ARCHITECTURE -eq 'ARM64') {
    Write-Note 'Windows on ARM detected - using the bundled x64 build via emulation.'
}

# Keep the interpreter inside the project rather than in the user profile, so
# that removing the project removes the whole environment.
$env:UV_PYTHON_INSTALL_DIR = Join-Path $projectRoot '.uv-python'
# Never reuse a Python already installed on this machine (uv would otherwise
# pick one up from PATH or from the Windows registry): a managed interpreter is
# downloaded into the project instead, keeping .venv independent of the host.
$env:UV_PYTHON_PREFERENCE = 'only-managed'

Push-Location -LiteralPath $projectRoot
try {
    Write-Step 'Using the bundled uv'
    Write-Ok (& $uvExe --version)

    # -------------------------------------------------------------------------
    # 1. Create .venv from pyproject.toml / uv.lock (installs pyulog)
    # -------------------------------------------------------------------------
    Write-Step 'Creating the project-local environment (uv sync)'
    & $uvExe sync
    if ($LASTEXITCODE -ne 0) {
        Write-Fail 'uv sync failed - see the output above.'
        exit $LASTEXITCODE
    }
    Write-Ok 'pyulog installed into .\.venv (no global packages were touched)'

    # -------------------------------------------------------------------------
    # 2. Verify that the tool MATLAB calls is actually usable
    # -------------------------------------------------------------------------
    Write-Step 'Verifying ulog2csv'
    & $uvExe run ulog2csv --help | Out-Null
    if ($LASTEXITCODE -ne 0) {
        Write-Fail 'ulog2csv could not be executed.'
        exit $LASTEXITCODE
    }

    $ulog2csv = Join-Path $projectRoot '.venv\Scripts\ulog2csv.exe'
    if (-not (Test-Path -LiteralPath $ulog2csv)) {
        Write-Fail "ulog2csv.exe was not found at $ulog2csv"
        exit 1
    }
    Write-Ok "ulog2csv is ready: $ulog2csv"

    Write-Host ''
    Write-Host 'Environment setup completed.' -ForegroundColor Green
    Write-Host ''
    Write-Host "Next: open MATLAB, run 'load_data_main' (or 'plot_setpoint_response')."
}
finally {
    Pop-Location
}
