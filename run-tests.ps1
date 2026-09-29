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
$testModules = @(& git -C $root ls-files --cached --others --exclude-standard -- "Geo.Python/python/tests/test_*.py" |
    ForEach-Object { $_ -replace '^Geo\.Python/python/', '' -replace '/', '.' -replace '\.py$', '' })
if ($LASTEXITCODE -ne 0) { throw "Could not enumerate Python tests from the Git worktree." }
if ($testModules.Count -eq 0) { throw "No non-ignored Python tests found." }

# unittest discovery also imports locally ignored example tests (such as the
# unfinished bike-model tests). Use Git's tracked/unignored file set so those
# local-only files remain available without breaking the repository test run.
Push-Location $pythonDir
try {
if ($python -is [array]) {
    & $python[0] $python[1] -m unittest @testModules -v
} else {
    & $python -m unittest @testModules -v
}
} finally {
    Pop-Location
}
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

Write-Host ""
Write-Host "All tests passed." -ForegroundColor Green
