#!/usr/bin/env python3
"""Patch DotWrap's generated package so the wheel is Python-version independent.

The Native AOT library (Geo.Python.dll / libGeo.Python.so) does not depend on
CPython. DotWrap's default cffi *API* build compiles a cp312 .pyd/.so, which
is why pip on 3.13 sees no matching wheel. This script:

- writes a cffi *ABI* loader that dlopen()s the AOT library
- skips compiling a CPython extension
- sets cambercad metadata (author, README, version)
"""
from __future__ import annotations

import re
import shutil
import sys
from pathlib import Path

SUMMARY = (
    "Scripting- and AI-first CAD: triangle-first kernel with sketch, CSG, "
    "optional NURBS, mesh import/export, and a Python API."
)

LOADER_TEMPLATE = '''\
"""cffi ABI loader for the Native AOT Geo.Python library.

Not compiled against a specific CPython version, so one wheel works on 3.10+.
"""
from __future__ import annotations

import os
import sys
from cffi import FFI

_DIR = os.path.dirname(os.path.abspath(__file__))
_CDEF = __CDEF_REPR__

ffi = FFI()
ffi.cdef(_CDEF)


def _native_lib_path():
    if sys.platform == "win32":
        names = ("Geo.Python.dll",)
    elif sys.platform == "darwin":
        names = ("libGeo.Python.dylib", "Geo.Python.dylib")
    else:
        names = ("libGeo.Python.so", "Geo.Python.so")
    for name in names:
        path = os.path.join(_DIR, name)
        if os.path.isfile(path):
            return path
    raise FileNotFoundError(
        "camber native library not found next to {0} (looked for {1})".format(
            _DIR, ", ".join(names)
        )
    )


if sys.platform == "win32" and hasattr(os, "add_dll_directory"):
    os.add_dll_directory(_DIR)

lib = ffi.dlopen(_native_lib_path())
'''

SETUP_TEMPLATE = '''\
from setuptools import setup, find_packages

setup(
    name="cambercad",
    version={version!r},
    packages=find_packages(),
    python_requires=">=3.10",
    install_requires=["cffi"],
    extras_require={{"view": ["pyglet", "imgui[pyglet]", "numpy"]}},
    author="cambercad",
    author_email="cambercad@proton.me",
    description={summary!r},
    long_description={readme!r},
    long_description_content_type="text/markdown",
    include_package_data=True,
    zip_safe=False,
    package_data={{
        "_camber_native.__dotwrap_generated": [
            "*.dll",
            "*.so",
            "*.dylib",
        ]
    }},
)
'''


def _extract_cdef(lib_build: Path) -> str:
    text = lib_build.read_text(encoding="utf-8")
    match = re.search(r'ffibuilder\.cdef\(\s*"""(.*?)"""\s*\)', text, re.S)
    if not match:
        raise RuntimeError("could not find ffibuilder.cdef(...) in {0}".format(lib_build))
    return match.group(1)


def _write_abi_loader(gen_dir: Path) -> None:
    cdef = _extract_cdef(gen_dir / "lib_build.py")
    (gen_dir / "__camber_native.py").write_text(
        LOADER_TEMPLATE.replace("__CDEF_REPR__", repr(cdef)),
        encoding="utf-8",
    )


def _strip_unused_numpy(main_py: Path) -> None:
    if not main_py.is_file():
        return
    text = main_py.read_text(encoding="utf-8")
    if "import numpy as np" in text and "np." not in text.replace("import numpy as np", ""):
        text = text.replace("import numpy as np\n", "")
        main_py.write_text(text, encoding="utf-8")


def _remove_cpython_extension(gen_dir: Path) -> None:
    patterns = (
        "__camber_native.c",
        "__camber_native.pyd",
        "__camber_native*.pyd",
        "__camber_native*.so",
        "__camber_native*.dylib",
        "__camber_native*.exp",
        "__camber_native*.lib",
        "__camber_native*.obj",
    )
    for pattern in patterns:
        for path in gen_dir.glob(pattern):
            path.unlink()
    release = gen_dir / "Release"
    if release.is_dir():
        shutil.rmtree(release)


def patch(pkg: Path, readme_path: Path, version: str) -> None:
    gen_dir = pkg / "_camber_native" / "__dotwrap_generated"
    if not (gen_dir / "lib_build.py").is_file():
        raise RuntimeError("missing {0}".format(gen_dir / "lib_build.py"))

    _write_abi_loader(gen_dir)
    _strip_unused_numpy(gen_dir / "main.py")
    _remove_cpython_extension(gen_dir)

    readme = readme_path.read_text(encoding="utf-8")
    (pkg / "setup.py").write_text(
        SETUP_TEMPLATE.format(
            version=version,
            summary=SUMMARY,
            readme=readme,
        ),
        encoding="utf-8",
    )
    print("patched ABI loader + setup.py in", pkg)


def main(argv: list[str]) -> int:
    if len(argv) != 4:
        print("usage: patch_wheel_metadata.py <python_project_root> <README.md> <version>", file=sys.stderr)
        return 2
    patch(Path(argv[1]), Path(argv[2]), argv[3])
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
