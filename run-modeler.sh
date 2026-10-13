#!/usr/bin/env bash
# Start the experimental modeling GUI (default: new unsaved part at repo-root part.py).
# From the repo root:
#   ./run-modeler.sh
#   ./run-modeler.sh my_design.py
set -euo pipefail
root="$(cd "$(dirname "$0")" && pwd)"
cd "$root"

part="${1:-part.py}"
if [[ "$part" != /* ]]; then
  part="$root/$part"
fi

find_python_launcher() {
  if [[ -n "${PYTHON:-}" ]]; then
    printf '%s\n' "$PYTHON"
    return
  fi
  if command -v python3 >/dev/null 2>&1; then
    command -v python3
    return
  fi
  if command -v python >/dev/null 2>&1; then
    command -v python
    return
  fi
  echo "Python 3.10+ not found. Install Python, set PYTHON, or put python3 on PATH." >&2
  exit 1
}

venv_dir="$root/.venv"
venv_py="$venv_dir/bin/python"

if [[ ! -x "$venv_py" ]]; then
  echo "Creating $venv_dir ..."
  "$(find_python_launcher)" -m venv "$venv_dir"
fi

test_modeler_stack() {
  "$venv_py" -c "import camber, pyglet" >/dev/null 2>&1
}

find_local_wheel() {
  local dist="$root/dist"
  local wheel=""
  if [[ ! -d "$dist" ]]; then
    return 1
  fi
  if [[ "$(uname -s)" == "Linux" ]]; then
    wheel="$(ls -t "$dist"/cambercad-*manylinux_*.whl 2>/dev/null | head -n 1 || true)"
    if [[ -z "$wheel" ]]; then
      wheel="$(ls -t "$dist"/cambercad-*linux_*.whl 2>/dev/null | head -n 1 || true)"
    fi
  elif [[ "$(uname -s)" == "Darwin" ]]; then
    wheel="$(ls -t "$dist"/cambercad-*macosx_*.whl 2>/dev/null | head -n 1 || true)"
  fi
  if [[ -z "$wheel" ]]; then
    wheel="$(ls -t "$dist"/cambercad-*.whl 2>/dev/null | head -n 1 || true)"
  fi
  [[ -n "$wheel" ]] || return 1
  printf '%s\n' "$wheel"
}

if ! test_modeler_stack; then
  wheel="$(find_local_wheel)" || wheel=""
  if [[ -z "$wheel" ]]; then
    cat >&2 <<'EOF'
camber (with viewer deps) is not installed and no wheel was found in dist/.
Build and install a local wheel first:
  cd Geo.Python
  bash publish-wheel.sh
Then run this script again (it will install dist/cambercad-*.whl[view] into .venv).
EOF
    exit 1
  fi
  echo "Installing $wheel [view] into .venv ..."
  if command -v uv >/dev/null 2>&1; then
    uv pip install --python "$venv_py" "${wheel}[view]"
  else
    "$venv_py" -m pip install --upgrade pip
    "$venv_py" -m pip install "${wheel}[view]"
  fi
  test_modeler_stack || {
    echo "camber + pyglet still not importable after install." >&2
    exit 1
  }
fi

# The wheel supplies _camber_native. Python GUI changes live in the repo and
# must come first, or the modeler keeps running the last installed copy.
source="$root/Geo.Python/python"
if [[ ! -f "$source/camber/__init__.py" ]]; then
  echo "Python package not found at $source" >&2
  exit 1
fi
if [[ -n "${PYTHONPATH:-}" ]]; then
  export PYTHONPATH="$source:$PYTHONPATH"
else
  export PYTHONPATH="$source"
fi

echo "Using Python sources: $source"
echo "Opening modeling workspace: $part"
export CAMBER_MODELER_PART="$part"
exec "$venv_py" -c "import camber, os; camber.modeling.open(os.environ['CAMBER_MODELER_PART'])"
