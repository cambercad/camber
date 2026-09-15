param(
    [string]$Runtime = "win-x64",
    [string]$Configuration = "Release"
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $root
Set-Location $root

Write-Host "Publishing Native AOT shared library for $Runtime..."
dotnet publish Geo.Python.csproj -c $Configuration -r $Runtime --self-contained

$pkg = Join-Path $root "python_project_root"
if (-not (Test-Path $pkg)) {
    $alt = Get-ChildItem -Path $root -Recurse -Directory -Filter "python_project_root" -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($alt) { $pkg = $alt.FullName }
}
if (-not (Test-Path $pkg)) {
    throw "DotWrap did not create python_project_root. Check the publish log."
}

# Replace the overlay package. Copy-Item into an existing `camber` folder
# nests as camber/camber and ships a stale api.py.
$camberDst = Join-Path $pkg "camber"
if (Test-Path $camberDst) {
    Remove-Item -Recurse -Force $camberDst
}
Copy-Item -Recurse (Join-Path $root "python\camber") $camberDst

$setup = Join-Path $pkg "setup.py"
if (Test-Path $setup) {
    $text = Get-Content $setup -Raw
    $text = $text -replace 'name="_camber_native"', 'name="camber"'
    $text = $text -replace "name='_camber_native'", 'name="camber"'
    $text = $text -replace 'name="geo_csg"', 'name="camber"'
    if ($text -notmatch 'extras_require') {
        $text = $text -replace 'install_requires=\["cffi"\]', 'install_requires=["cffi"], extras_require={"view": ["pyglet", "imgui[pyglet]", "numpy"]}'
        $text = $text -replace "install_requires=\['cffi'\]", 'install_requires=["cffi"], extras_require={"view": ["pyglet", "imgui[pyglet]", "numpy"]}'
    }
    Set-Content -Path $setup -Value $text -NoNewline
}

python -m pip install --upgrade pip setuptools wheel cffi

# Stage pip output, then keep only camber-*.whl in repo-root dist/ (PyPI-ready layout).
$stage = Join-Path $root "_wheel_stage"
$dist = Join-Path $repoRoot "dist"
if (Test-Path $stage) { Remove-Item -Recurse -Force $stage }
New-Item -ItemType Directory -Force -Path $stage | Out-Null
New-Item -ItemType Directory -Force -Path $dist | Out-Null

python -m pip wheel --no-deps $pkg -w $stage

$py = & python -c "import sys; print(f'cp{sys.version_info.major}{sys.version_info.minor}')"
$plat = switch -Wildcard ($Runtime) {
    "win-x64" { "win_amd64" }
    "win-arm64" { "win_arm64" }
    default { ($Runtime -replace "-", "_") }
}

Push-Location $stage
try {
    Get-ChildItem -Filter "camber-*.whl" | ForEach-Object {
        # wheel tags --remove deletes the input file and writes a retagged sibling here.
        & python -m wheel tags --remove --python-tag $py --abi-tag $py --platform-tag $plat $_.Name
        if ($LASTEXITCODE -ne 0) {
            if ($_.Name -match '^camber-([^-]+)-py3-none-any\.whl$') {
                Rename-Item $_.Name "camber-$($Matches[1])-$py-$py-$plat.whl"
            }
        }
    }
    Get-ChildItem -Filter "camber-*.whl" | ForEach-Object {
        $dest = Join-Path $dist $_.Name
        Copy-Item $_.FullName $dest -Force
        Write-Host "Wrote $dest"
    }
}
finally {
    Pop-Location
}
Remove-Item -Recurse -Force $stage

Write-Host "Wheels in $dist"
Get-ChildItem $dist -Filter "camber-*.whl"
Write-Host "Install: uv pip install `"$dist\camber-*.whl[view]`""
Write-Host "Smoke:   python python\smoke.py"
Write-Host "Later PyPI: twine upload dist\camber-*.whl"
