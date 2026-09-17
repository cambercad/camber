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

import ast
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

import ctypes
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

# Native AOT runtimes cannot be unloaded while their runtime threads exist.
# CDLL owns a process-lifetime OS handle; CFFI borrows it without automatic
# dlclose/FreeLibrary. Object handles and returned strings are still freed by
# the generated wrappers normally.
# https://learn.microsoft.com/en-us/dotnet/core/deploying/native-aot/libraries
# https://cffi.readthedocs.io/en/stable/cdef.html#ffi-dlopen-loading-libraries-in-abi-mode
_native_library = ctypes.CDLL(_native_lib_path())
lib = ffi.dlopen(ffi.cast("void *", _native_library._handle))
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


def _fix_null_cstrings(main_py: Path) -> None:
    """DotWrap 0.3 encodes empty native strings as NULL; cffi.string rejects it."""
    text = main_py.read_text(encoding="utf-8")
    original = '        return _dotwrap_ffi.string(self._dotwrap_ptr).decode("utf-8")'
    fixed = ('        if self._dotwrap_ptr == _dotwrap_ffi.NULL:\n'
             '            return ""\n' + original)
    if fixed in text:
        return
    if text.count(original) != 1:
        raise RuntimeError("unexpected DotWrap CString implementation; review null-string handling")
    main_py.write_text(text.replace(original, fixed), encoding="utf-8")



def _fix_null_objects(main_py: Path) -> None:
    """Normalize DotWrap 0.3's shared object-pointer conversion contract.

    A zero return pointer represents C# null, not an owned GCHandle. Never
    construct a Python owner for it: its destructor would free an invalid handle.
    """
    text = main_py.read_text(encoding="utf-8")
    factories = [node for node in ast.walk(ast.parse(text))
                 if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
                 and node.name == "_dotwrap_from_ptr"]
    original = ("    def _dotwrap_from_ptr(cls, ptr: int):\n"
                "        instance = object.__new__(cls)\n"
                "        instance._dotwrap_ptr = _dotwrap_ffi.cast(\"void *\", ptr)\n"
                "        return instance")
    fixed = ("    def _dotwrap_from_ptr(cls, ptr: int):\n"
             "        return _dotwrap_object_from_ptr(cls, ptr)")
    helper = ("def _dotwrap_object_from_ptr(cls, ptr):\n"
              "    pointer = _dotwrap_ffi.cast(\"void *\", ptr)\n"
              "    if pointer == _dotwrap_ffi.NULL:\n"
              "        return None\n"
              "    instance = object.__new__(cls)\n"
              "    instance._dotwrap_ptr = pointer\n"
              "    return instance\n\n")
    if not (factories and text.count(fixed) == len(factories) and text.count(helper) == 1):
        if not factories or text.count(original) != len(factories) or text.count("class CString:") != 1:
            raise RuntimeError("unexpected DotWrap object factory implementation; review nullable handle ownership")
        text = text.replace(original, fixed).replace("class CString:", helper + "class CString:", 1)

    release = ("def _dotwrap_release_object(instance, destroy):\n"
               "    pointer = _dotwrap_ffi.cast(\"void *\", getattr(instance, \"_dotwrap_ptr\", _dotwrap_ffi.NULL))\n"
               "    if pointer != _dotwrap_ffi.NULL:\n"
               "        instance._dotwrap_ptr = _dotwrap_ffi.NULL\n"
               "        destroy(pointer)\n\n")
    for cls in (node for node in ast.parse(text).body if isinstance(node, ast.ClassDef)):
        if not any(isinstance(node, ast.FunctionDef) and node.name == "_dotwrap_from_ptr" for node in cls.body):
            continue
        destructor = next((node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == "__del__"), None)
        if destructor is None or len(destructor.body) != 1 or not isinstance(destructor.body[0], ast.Expr):
            raise RuntimeError("unexpected DotWrap destructor implementation; review handle ownership")
        call = destructor.body[0].value
        if isinstance(call, ast.Call) and isinstance(call.func, ast.Name) and call.func.id == "_dotwrap_release_object":
            continue
        if not (isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute)
                and isinstance(call.func.value, ast.Name) and call.func.value.id == "_dotwrap_lib"
                and call.func.attr.endswith("___dotwrapDestroy")):
            raise RuntimeError("unexpected DotWrap destructor implementation; review handle ownership")
        original_destructor = ("    def __del__(self):\n"
                               f"        _dotwrap_lib.{call.func.attr}(self._dotwrap_ptr)")
        fixed_destructor = ("    def __del__(self):\n"
                            f"        _dotwrap_release_object(self, _dotwrap_lib.{call.func.attr})")
        if text.count(original_destructor) != 1:
            raise RuntimeError("unexpected DotWrap destructor implementation; review handle ownership")
        text = text.replace(original_destructor, fixed_destructor, 1)
    if release not in text:
        text = text.replace("class CString:", release + "class CString:", 1)
    main_py.write_text(text, encoding="utf-8")

def _remove_cpython_extension(pkg: Path, gen_dir: Path) -> None:
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
        for path in gen_dir.rglob(pattern):
            if path.is_file():
                path.unlink()
    release = gen_dir / "Release"
    if release.is_dir():
        shutil.rmtree(release)
    # Stale setuptools `build/` keeps a previously compiled .pyd; pip wheel
    # copies it into the package and CPython then prefers it over the ABI loader.
    for leftover in (pkg / "build", pkg / "camber.egg-info", pkg / "cambercad.egg-info"):
        if leftover.exists():
            shutil.rmtree(leftover)


def patch(pkg: Path, readme_path: Path, version: str) -> None:
    gen_dir = pkg / "_camber_native" / "__dotwrap_generated"
    if not (gen_dir / "lib_build.py").is_file():
        raise RuntimeError("missing {0}".format(gen_dir / "lib_build.py"))

    _write_abi_loader(gen_dir)
    _strip_unused_numpy(gen_dir / "main.py")
    _fix_null_cstrings(gen_dir / "main.py")
    _fix_null_objects(gen_dir / "main.py")
    _remove_cpython_extension(pkg, gen_dir)

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
