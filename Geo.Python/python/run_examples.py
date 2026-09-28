"""Run every published Python example without opening a viewer or writing exports."""
from __future__ import annotations

from pathlib import Path
import subprocess
import sys


PACKAGE_ROOT = Path(__file__).resolve().parent
ROOT = PACKAGE_ROOT / "examples" / "standalone"
EXCLUDED = {"gears.py"}
RUNNER = r'''
import runpy
import sys
from pathlib import Path

source = Path(sys.argv[1]).resolve()
sys.path.insert(0, str(source.parent))
import camber
from camber import api

def noop(*args, **kwargs):
    return None

camber.show = noop
for kind in (api.Part, api.Sketch, api._MeshBody, api.Assembly):
    kind.show = noop
for name in ("save_stl", "save_obj", "save_step", "save_iges", "save_usda"):
    setattr(api._MeshBody, name, noop)
sys.argv = [str(source)]
runpy.run_path(str(source), run_name="__main__")
'''


def examples():
    return sorted(path for path in ROOT.glob("*.py") if path.name not in EXCLUDED)


def main():
    failures = []
    for path in examples():
        print(f"== {path.name} ==", flush=True)
        try:
            subprocess.run(
                [sys.executable, "-c", RUNNER, str(path)], cwd=PACKAGE_ROOT,
                check=True, timeout=180)
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as error:
            failures.append(f"{path.name}: {error}")
    if failures:
        raise SystemExit("\n".join(failures))


if __name__ == "__main__":
    main()
