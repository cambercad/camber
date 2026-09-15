#!/usr/bin/env bash
# Build a Linux Python wheel for camber inside WSL/Ubuntu (Native AOT + DotWrap).
#
# Usage:
#   bash publish-wheel.sh                 # build only
#   bash publish-wheel.sh --bootstrap     # apt + .NET SDK, then build
#
# Output (same folder as Windows wheels — ready for twine later):
#   <repo>/dist/cambercad-*.whl

set -euo pipefail

RUNTIME="linux-x64"
CONFIGURATION="Release"
WHEEL_VERSION="0.1.1"
BOOTSTRAP=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bootstrap) BOOTSTRAP=1; shift ;;
    --runtime) RUNTIME="$2"; shift 2 ;;
    --configuration) CONFIGURATION="$2"; shift 2 ;;
    --version) WHEEL_VERSION="$2"; shift 2 ;;
    -h|--help)
      sed -n '2,12p' "$0"
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$ROOT/.." && pwd)"
cd "$ROOT"

if [[ "$(uname -s)" != "Linux" ]]; then
  echo "This script must run on Linux (e.g. Ubuntu 24.04 in WSL)." >&2
  exit 1
fi

# Building Native AOT on /mnt/c (NTFS) fails with file timestamp/ACL errors.
# Auto-relaunch from a Linux-filesystem copy when needed.
if [[ "$ROOT" == /mnt/* ]] && [[ "${CAMBER_LINUX_RELAUNCHED:-0}" != "1" ]]; then
  WIN_REPO="$REPO_ROOT"
  BUILD_ROOT="${CAMBER_LINUX_BUILD:-$HOME/camber-linux-build}"
  echo "==> Source is on /mnt/*; copying to $BUILD_ROOT for a reliable Linux build..."
  rm -rf "$BUILD_ROOT"
  mkdir -p "$BUILD_ROOT"
  rsync -a \
    --exclude 'bin/' \
    --exclude 'obj/' \
    --exclude '.venv/' \
    --exclude 'venv/' \
    --exclude 'dist/' \
    --exclude 'Geo.Python/dist/' \
    --exclude 'Geo.Python/python_project_root/' \
    --exclude 'Geo.Python/_wheel_stage/' \
    --exclude 'Geo.Python/_wheel_venv/' \
    --exclude 'Geo.Python/**/__pycache__/' \
    --exclude '.git/' \
    --exclude '.vs/' \
    "$WIN_REPO/" "$BUILD_ROOT/"
  sed -i 's/\r$//' "$BUILD_ROOT/Geo.Python/publish-wheel.sh" || true
  export CAMBER_LINUX_RELAUNCHED=1
  export CAMBER_WIN_DIST="$WIN_REPO/dist"
  bash "$BUILD_ROOT/Geo.Python/publish-wheel.sh" "$@"
  mkdir -p "$WIN_REPO/dist"
  cp -f "$BUILD_ROOT/dist"/cambercad-*.whl "$WIN_REPO/dist/"
  echo "Copied wheels to $WIN_REPO/dist:"
  ls -la "$WIN_REPO/dist"/cambercad-*.whl
  exit 0
fi

bootstrap_deps() {
  echo "==> Installing build dependencies (sudo)..."
  sudo apt-get update
  sudo apt-get install -y \
    curl ca-certificates wget \
    build-essential clang zlib1g-dev \
    python3 python3-venv python3-pip python3-dev \
    python3-cffi

  if ! command -v dotnet >/dev/null 2>&1 && [[ ! -x "$HOME/.dotnet/dotnet" ]]; then
    echo "==> Installing .NET SDK 10 into ~/.dotnet..."
    wget -q https://dot.net/v1/dotnet-install.sh -O /tmp/dotnet-install.sh
    chmod +x /tmp/dotnet-install.sh
    /tmp/dotnet-install.sh --channel 10.0 --install-dir "$HOME/.dotnet"
  fi

  if [[ -x "$HOME/.dotnet/dotnet" ]] && ! grep -q 'DOTNET_ROOT' "$HOME/.bashrc" 2>/dev/null; then
    {
      echo 'export DOTNET_ROOT=$HOME/.dotnet'
      echo 'export PATH=$HOME/.dotnet:$PATH'
    } >> "$HOME/.bashrc"
  fi
}

have_dotnet() {
  command -v dotnet >/dev/null 2>&1 || [[ -x "$HOME/.dotnet/dotnet" ]]
}

ensure_path() {
  if [[ -x "$HOME/.dotnet/dotnet" ]]; then
    export DOTNET_ROOT="$HOME/.dotnet"
    export PATH="$HOME/.dotnet:$PATH"
  fi
}

if [[ "$BOOTSTRAP" -eq 1 ]]; then
  bootstrap_deps
fi

ensure_path

missing=()
command -v python3 >/dev/null 2>&1 || missing+=("python3")
command -v clang >/dev/null 2>&1 || command -v gcc >/dev/null 2>&1 || missing+=("clang-or-gcc")
have_dotnet || missing+=("dotnet")

if [[ ${#missing[@]} -gt 0 ]]; then
  echo "Missing tools: ${missing[*]}" >&2
  echo "Re-run with:  bash $0 --bootstrap" >&2
  exit 1
fi

ensure_path
echo "==> dotnet: $(dotnet --version)"
echo "==> python: $(python3 --version)"
echo "==> Publishing Native AOT shared library for $RUNTIME..."

dotnet publish Geo.Python.csproj -c "$CONFIGURATION" -r "$RUNTIME" --self-contained

PKG="$ROOT/python_project_root"
if [[ ! -d "$PKG" ]]; then
  ALT="$(find "$ROOT" -type d -name python_project_root 2>/dev/null | head -n 1 || true)"
  if [[ -n "$ALT" ]]; then
    PKG="$ALT"
  fi
fi
if [[ ! -d "$PKG" ]]; then
  echo "DotWrap did not create python_project_root. Check the publish log." >&2
  exit 1
fi

CAMBER_DST="$PKG/camber"
rm -rf "$CAMBER_DST"
cp -a "$ROOT/python/camber" "$CAMBER_DST"

SETUP="$PKG/setup.py"
README="$REPO_ROOT/README.md"
if [[ -f "$SETUP" ]]; then
  python3 - "$SETUP" "$README" "$WHEEL_VERSION" <<'PY'
from pathlib import Path
import sys
import re

setup_path = Path(sys.argv[1])
readme = Path(sys.argv[2]).read_text(encoding="utf-8")
version = sys.argv[3]
text = setup_path.read_text(encoding="utf-8")
text = re.sub(r'version\s*=\s*["\'][^"\']+["\']', f'version="{version}"', text, count=1)
text = text.replace('name="_camber_native"', 'name="cambercad"')
text = text.replace("name='_camber_native'", 'name="cambercad"')
text = text.replace('name="geo_csg"', 'name="cambercad"')
text = text.replace('author="DotWrap"', 'author="cambercad"')
text = text.replace("author='DotWrap'", 'author="cambercad"')
summary = (
    "Scripting- and AI-first CAD: triangle-first kernel with sketch, CSG, "
    "optional NURBS, mesh import/export, and a Python API."
)
text = text.replace(
    'description="Auto-generated Python bindings for DotWrap C# library"',
    f"description={summary!r}",
)
text = text.replace(
    "description='Auto-generated Python bindings for DotWrap C# library'",
    f"description={summary!r}",
)
if "author_email=" not in text:
    text = text.replace(
        'author="cambercad"',
        'author="cambercad",\n    author_email="cambercad@proton.me"',
    )
if "long_description" not in text:
    extra_meta = (
        f"    long_description={readme!r},\n"
        '    long_description_content_type="text/markdown",\n'
    )
    text = text.replace(
        f"description={summary!r},\n",
        f"description={summary!r},\n{extra_meta}",
    )
extra = 'extras_require={"view": ["pyglet", "imgui[pyglet]", "numpy"]}'
if "extras_require" not in text:
    text = text.replace(
        'install_requires=["cffi"]',
        'install_requires=["cffi"], ' + extra,
    )
    text = text.replace(
        "install_requires=['cffi']",
        'install_requires=["cffi"], ' + extra,
    )
setup_path.write_text(text, encoding="utf-8")
print("patched", setup_path)
PY
fi

# Use a throwaway venv — Ubuntu 24.04 blocks system pip (PEP 668).
BUILD_VENV="$ROOT/_wheel_venv"
rm -rf "$BUILD_VENV"
python3 -m venv "$BUILD_VENV"
# shellcheck disable=SC1091
source "$BUILD_VENV/bin/activate"
python -m pip install --upgrade pip setuptools wheel cffi

STAGE="$ROOT/_wheel_stage"
DIST="$REPO_ROOT/dist"
rm -rf "$STAGE"
mkdir -p "$STAGE" "$DIST"
python -m pip wheel --no-deps "$PKG" -w "$STAGE"
ls -la "$STAGE"

shopt -s nullglob
PYTAG="$(python -c 'import sys; print(f"cp{sys.version_info.major}{sys.version_info.minor}")')"
case "$RUNTIME" in
  # PyPI rejects bare linux_x86_64; use a manylinux tag for native wheels.
  linux-x64) PLAT="manylinux_2_17_x86_64" ;;
  linux-arm64) PLAT="manylinux_2_17_aarch64" ;;
  *) PLAT="${RUNTIME//-/_}" ;;
esac

# Retag in-place inside STAGE (wheel tags --remove deletes the source file).
(
  cd "$STAGE"
  for whl in cambercad-*.whl; do
    python -m wheel tags --remove --python-tag "$PYTAG" --abi-tag "$PYTAG" --platform-tag "$PLAT" "$whl" || {
      # Fallback rename only (metadata may still say py3-none-any).
      if [[ "$whl" == *-py3-none-any.whl ]]; then
        ver="${whl#cambercad-}"
        ver="${ver%-py3-none-any.whl}"
        mv -f "$whl" "cambercad-${ver}-${PYTAG}-${PYTAG}-${PLAT}.whl"
      fi
    }
  done
)

for whl in "$STAGE"/cambercad-*.whl; do
  dest_name="$(basename "$whl")"
  cp -f "$whl" "$DIST/$dest_name"
  echo "Wrote $DIST/$dest_name"
done
rm -rf "$STAGE" "$BUILD_VENV"
deactivate 2>/dev/null || true

echo
echo "Wheels in $DIST"
ls -la "$DIST"/cambercad-*.whl 2>/dev/null || true
echo
echo "Install example (repo root):"
echo "  uv venv .venv && uv pip install --python .venv/bin/python \"dist/cambercad-\"*\".whl[view]\""
echo "Smoke:"
echo "  .venv/bin/python Geo.Python/python/smoke.py"
echo "Later PyPI:"
echo "  twine upload dist/cambercad-*.whl"
