# Run all unit tests (C# + Python). From the repo root:
#   .\run-tests.ps1
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $root

function Find-Python {
    if ($env:PYTHON) { return $env:PYTHON }
    foreach ($candidate in @(
            (Join-Path $root ".venv\Scripts\python.exe"),
            (Join-Path $root "Geo.Python\.venv\Scripts\python.exe")
        )) {
        if (Test-Path $candidate) { return $candidate }
    }
    $cmd = Get-Command python -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }
    $py = Get-Command py -ErrorAction SilentlyContinue
    if ($py) { return @($py.Source, "-3") }
    throw "Python 3.10+ not found. Put it on PATH, set PYTHON, or create .venv at the repo root."
}

Write-Host "== C# (dotnet test) ==" -ForegroundColor Cyan
dotnet test (Join-Path $root "camber.sln") --nologo
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

Write-Host ""
Write-Host "== Python (unittest) ==" -ForegroundColor Cyan
$python = Find-Python
$pythonDir = Join-Path $root "Geo.Python\python"
$env:CAMBER_PROGRESS = "0"
$env:PYTHONPATH = $pythonDir
if ($python -is [array]) {
    & $python[0] $python[1] -m unittest discover -s (Join-Path $pythonDir "tests") -t $pythonDir -v
} else {
    & $python -m unittest discover -s (Join-Path $pythonDir "tests") -t $pythonDir -v
}
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

Write-Host ""
Write-Host "All tests passed." -ForegroundColor Green
