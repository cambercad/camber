# camber

**Alpha.** AI-ready CAD for people and agents: the same Python API, a small dependency footprint, and an optional OpenGL viewer.

Parts are **triangle meshes** (ideally watertight). Exact-rational CSG, constrained sketches, and optional NURBS all work on that mesh. Curves and surfaces are recovered when you need them for export; they do not own the model. You can also load and edit STL/OBJ from elsewhere.

This is **not** a fork of CadQuery, build123d, FreeCAD, or OpenSCAD. Those use OpenCascade B-rep (or OpenSCAD’s language / a full GUI). Camber is a separate mesh kernel; `camber.cqcompat` is only a familiarity shim, not OCCT.

Status: APIs and file formats will change. Expect breakage. Much of the code is AI-assisted; the goal is useful geometry, not AI slop — but plenty of bugs remain. It is already good enough for real scripts.

## API design

This guideline applies throughout Camber: the kernel, language bindings,
modelling and verification tools, rendering, diagnostics, and examples.

Camber's tools should serve engineers, Python users, and AI agents through the
same API. Prefer familiar CAD operations, constrained sketches, explicit part
interfaces, and inspectable assembly relationships. Keep roles clear: geometry
constructs parts, mates locate occurrences, and inspection reports model state.

Add public methods for recurring modelling or verification tasks, reuse existing
kernel and renderer capabilities, and avoid parallel APIs for humans and agents.
Return structured results that can also be displayed, state units and tolerances,
and identify affected components in errors. Inspection should not silently alter
the model or treat a failed calculation as a successful check.

## Install

PyPI package name is **`cambercad`**. The Python import stays **`camber`**.

Wheels are **per OS** (Windows amd64, Linux x86_64), not per Python version. CPython **3.10+** (including 3.13) can install the same file. There is no macOS wheel yet.

```text
pip install cambercad
```

Optional OpenGL viewer (pyglet + imgui + numpy):

```text
pip install "cambercad[view]"
```

```python
import camber
```

`cffi` is a normal dependency of the native module. pip may print `cffi` under `cambercad`; that is expected.

## Tests

From the repo root, after a local wheel is installed in `.venv` (or `Geo.Python/.venv`):

```powershell
.\run-tests.ps1
```

```bash
./run-tests.sh
```

That runs `dotnet test camber.sln` and Python `unittest` under `Geo.Python/python/tests`. Native Python tests skip if the wheel is missing. C# only: `dotnet test GeoTests/GeoTests.csproj -c Release`. More detail: [Geo.Python/README.md](Geo.Python/README.md), [GeoTests/README.md](GeoTests/README.md).

## What you get

- Sketch → extrude / revolve / loft / boolean
- Load / edit triangle meshes from outside camber
- Named topology for picking and scripting
- Export: STL, OBJ, STEP, IGES, USDA
- Optional viewer: pyglet + imgui

## Python dependencies

| Use case | Packages |
|----------|----------|
| Import `camber`, model, export | **`cffi`** only (ships with the wheel’s native module) |
| `.show()` / interactive sketch | **`pyglet`**, **`imgui[pyglet]`**, **`numpy`** |

Install the viewer stack with the `view` extra:

```text
pip install "cambercad[view]"
```

Note: the current DotWrap-generated native module imports **numpy** on Linux even for headless use. Prefer `cambercad[view]` or `pip install numpy` until that import is optional.

## Wheels output (`dist/`)

Both Windows and Linux publish scripts write **only** `cambercad-*.whl` into the **repo-root** folder:

```text
camber/
  dist/
    cambercad-0.1.2-py3-none-win_amd64.whl
    cambercad-0.1.2-py3-none-manylinux_2_17_x86_64.whl
```

That layout matches PyPI (`twine upload dist/cambercad-*.whl`). Dependency wheels (`cffi`, …) are staged temporarily and not kept in `dist/`. Scripts retag DotWrap’s `py3-none-any` name to a platform tag so Win/Linux wheels can sit side by side. PyPI rejects a bare `linux_x86_64` tag; Linux wheels use `manylinux_2_17_x86_64`.

## Build the wheel (Windows)

Needs: Python 3.10+, .NET SDK (this tree targets `net10.0`), and VS **Desktop development with C++** (for Native AOT publish).

```powershell
cd <this-repo>\Geo.Python
.\publish-wheel.ps1
# → writes <this-repo>\dist\cambercad-*.whl
```

Install + smoke (repo root):

```powershell
uv pip install --python .\.venv\Scripts\python.exe (Get-ChildItem ..\dist\cambercad*.whl | Select-Object -Last 1).FullName
# or with viewer:
# uv pip install --python .\.venv\Scripts\python.exe "$((Get-ChildItem ..\dist\cambercad*.whl | Select-Object -Last 1).FullName)[view]"
python python\smoke.py
```

## Build the wheel (Linux / WSL Ubuntu 24.04)

Native AOT must be compiled **on Linux** (WSL is fine). Cross-compiling a Linux wheel from Windows is not supported here.

Your distro needs the .NET SDK, a C/C++ toolchain (`clang` or `gcc`), and Python headers. First time:

```bash
cd <this-repo>/Geo.Python
bash publish-wheel.sh --bootstrap   # first time: apt + .NET 10 into ~/.dotnet
# later: bash publish-wheel.sh
# → writes <this-repo>/dist/cambercad-*-manylinux_2_17_x86_64.whl
#
# If the repo lives on /mnt/c (Windows drive), the script copies sources to
# ~/camber-linux-build automatically (NTFS breaks Native AOT timestamps).
```

Later builds:

```bash
bash publish-wheel.sh
```

Then at the repo root:

```bash
uv venv .venv
uv pip install --python .venv/bin/python "dist/cambercad-"*".whl[view]"
.venv/bin/python Geo.Python/python/smoke.py
```

The wheel is tagged `py3-none-<platform>`: one file per OS, installable on CPython 3.10+.

## Install with uv (local wheel)

Git clone alone cannot replace a published platform wheel: Native AOT needs the .NET SDK and a C++ toolchain.

```powershell
uv venv
# kernel only:
uv pip install dist\cambercad-*.whl
# or kernel + viewer (pyglet, imgui, numpy):
uv pip install "dist\cambercad-*.whl[view]"
```

Wheels for Win/Linux land in repo-root `dist/`. Publishing to PyPI is a separate local checklist (not kept in this repo).

## C# solution

Open [`camber.sln`](camber.sln). Projects required for the Python wheel:

| Project | Role |
|---------|------|
| `Geo.Python` | Native AOT + DotWrap FFI |
| `GeoTests` | C# unit tests (`dotnet test`) |
| `Geo` | Public CAD API, mesh construction, export |
| `GeoMeta` | Naming / metadata |
| `CSG` | Boolean / mesh CSG |
| `Curves` | 2D curves, NACA, gears, sketcher helpers |
| `GeoCore` | Vectors, meshes, adjacency |
| `GeoSolver` | Sketch / constraint solver |
| `NURBS` | NURBS + Clipper-based tessellation |
| `Remeshing` | Mesh cleanup |
| `Triangulation` | Constrained triangulation |
| `RationalNumbers` | Exact rationals (`BigRational` et al.) |

## Third-party code (GitHub-safe)

Full license texts are under [`third_party/`](third_party/).

| Component | License | Where | Notes |
|-----------|---------|--------|--------|
| Microsoft **BigRational** ([microsoftarchive/bcl](https://github.com/microsoftarchive/bcl)) | MIT | `RationalNumbers/BigRational.cs` | Active default for exact arithmetic |
| **BigRat** by Christian Ohle ([c-ohle/RationalNumerics](https://github.com/c-ohle/RationalNumerics)) | MIT | `RationalNumbers/BigRationalFast.cs` | Vendored; not the default path today (`FAST_BIG_RAT` is off in `BigRationalHybrid.cs`) |
| **Clipper** 6.4.2 C# (Angus Johnson) | Boost Software License 1.0 | `NURBS/Triangulator/Clipper.cs` | Polygon clipping / offsets |
| **DotWrap** 0.3.0 (NuGet, build-time) | MIT | `Geo.Python/Geo.Python.csproj` | Generates the Python native module |

Optional runtime deps (not vendored): **cffi**, and for the viewer **pyglet** / **imgui** / **numpy**.
