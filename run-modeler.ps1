# Start the experimental modeling GUI (default: new unsaved part at repo-root part.py).
# From the repo root:
#   .\run-modeler.ps1
#   .\run-modeler.ps1 -Part my_design.py
param(
    [string]$Part = "part.py"
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $root

function Find-PythonLauncher {
    if ($env:PYTHON) { return $env:PYTHON }
    $py = Get-Command py -ErrorAction SilentlyContinue
    if ($py) { return @($py.Source, "-3") }
    $cmd = Get-Command python -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }
    throw "Python 3.10+ not found. Install Python, set PYTHON, or put python/py on PATH."
}

function Invoke-Python {
    param([string[]]$Python, [string[]]$CommandArgs)
    if ($Python.Count -eq 1) {
        & $Python[0] @CommandArgs
    } else {
        & $Python[0] $Python[1] @CommandArgs
    }
}

$venvDir = Join-Path $root ".venv"
$venvPy = Join-Path $venvDir "Scripts\python.exe"

if (-not (Test-Path $venvPy)) {
    Write-Host "Creating $venvDir ..." -ForegroundColor Cyan
    $launcher = Find-PythonLauncher
    if ($launcher -is [array]) {
        Invoke-Python $launcher @("-m", "venv", $venvDir)
    } else {
        Invoke-Python @($launcher) @("-m", "venv", $venvDir)
    }
    if ($LASTEXITCODE -ne 0) { throw "Failed to create virtual environment." }
}

function Test-ModelerStack {
    & $venvPy -c "import camber, pyglet" 2>$null
    return $LASTEXITCODE -eq 0
}

function Find-LocalWheel {
    $dist = Join-Path $root "dist"
    if (-not (Test-Path $dist)) { return $null }
    $pattern = if ($IsWindows -or $env:OS -match "Windows") {
        "win_(amd64|arm64)"
    } else {
        "manylinux|linux"
    }
    $wheels = Get-ChildItem (Join-Path $dist "cambercad-*.whl") -ErrorAction SilentlyContinue |
        Where-Object { $_.Name -match $pattern }
    if (-not $wheels) {
        $wheels = Get-ChildItem (Join-Path $dist "cambercad-*.whl") -ErrorAction SilentlyContinue
    }
    if (-not $wheels) { return $null }
    return ($wheels | Sort-Object LastWriteTime | Select-Object -Last 1).FullName
}

if (-not (Test-ModelerStack)) {
    $wheel = Find-LocalWheel
    if (-not $wheel) {
        throw @"
camber (with viewer deps) is not installed and no wheel was found in dist/.
Build and install a local wheel first:
  cd Geo.Python
  .\publish-wheel.ps1
Then run this script again (it will install dist\cambercad-*.whl[view] into .venv).
"@
    }
    Write-Host "Installing $wheel [view] into .venv ..." -ForegroundColor Cyan
    $uv = Get-Command uv -ErrorAction SilentlyContinue
    $wheelSpec = "{0}[view]" -f $wheel
    if ($uv) {
        & $uv.Source pip install --python $venvPy $wheelSpec
    } else {
        & $venvPy -m pip install --upgrade pip
        if ($LASTEXITCODE -ne 0) { throw "pip upgrade failed." }
        & $venvPy -m pip install $wheelSpec
    }
    if ($LASTEXITCODE -ne 0) { throw "Wheel install failed." }
    if (-not (Test-ModelerStack)) {
        throw "camber + pyglet still not importable after install."
    }
}

if (-not [System.IO.Path]::IsPathRooted($Part)) {
    $Part = Join-Path $root $Part
}

# The wheel supplies _camber_native. Python GUI changes live in the repo and
# must come first, or the modeler keeps running the last installed copy.
$source = Join-Path $root "Geo.Python\python"
if (-not (Test-Path (Join-Path $source "camber\__init__.py"))) {
    throw "Python package not found at $source"
}
$env:PYTHONPATH = if ($env:PYTHONPATH) { "$source;$($env:PYTHONPATH)" } else { $source }

Write-Host "Using Python sources: $source" -ForegroundColor Cyan
Write-Host "Opening modeling workspace: $Part" -ForegroundColor Cyan
$env:CAMBER_MODELER_PART = $Part
& $venvPy -c "import camber, os; camber.modeling.open(os.environ['CAMBER_MODELER_PART'])"
exit $LASTEXITCODE
