#!/usr/bin/env bash
# Run all unit tests (C# + Python). From the repo root:
#   ./run-tests.sh
set -euo pipefail
root="$(cd "$(dirname "$0")" && pwd)"
cd "$root"

find_python() {
  if [[ -n "${PYTHON:-}" ]]; then
    printf '%s\n' "$PYTHON"
    return
  fi
  for candidate in "$root/.venv/bin/python" "$root/Geo.Python/.venv/bin/python"; do
    if [[ -x "$candidate" ]]; then
      printf '%s\n' "$candidate"
      return
    fi
  done
  if command -v python3 >/dev/null 2>&1; then
    command -v python3
    return
  fi
  if command -v python >/dev/null 2>&1; then
    command -v python
    return
  fi
  echo "Python 3.10+ not found. Put it on PATH, set PYTHON, or create .venv at the repo root." >&2
  exit 1
}

echo "== C# (dotnet test) =="
dotnet test "$root/camber.sln" --nologo

echo
echo "== Python (unittest) =="
python_dir="$root/Geo.Python/python"
export CAMBER_PROGRESS=0
export PYTHONPATH="$python_dir"
"$(find_python)" -m unittest discover -s "$python_dir/tests" -t "$python_dir" -v

echo
echo "All tests passed."
