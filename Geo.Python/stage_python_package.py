"""Copy wheel sources, using Git's ignore rules for local build artifacts."""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path


def stage(source: Path, destination: Path) -> None:
    source = source.resolve()
    repo = source.parents[2]
    # WSL builds copy sources off /mnt/c without .git; query the original checkout.
    git_repo = Path(os.environ.get("CAMBER_SOURCE_GIT_ROOT", repo))
    relative_source = source.relative_to(repo)
    result = subprocess.run(
        ["git", "-C", str(git_repo), "ls-files", "--cached", "--others", "--exclude-standard", "-z", "--", relative_source.as_posix()],
        check=True,
        capture_output=True,
    )
    files = [repo / Path(path.decode("utf-8")) for path in result.stdout.split(b"\0") if path]
    if not files or source / "__init__.py" not in files:
        raise RuntimeError(f"no Python package sources found in {source}")
    if destination.exists():
        shutil.rmtree(destination)
    for file in files:
        if file.is_file():
            target = destination / file.relative_to(source)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(file, target)


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit("usage: stage_python_package.py <source> <destination>")
    stage(Path(sys.argv[1]), Path(sys.argv[2]))
