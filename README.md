# camber

**Alpha** — scripting- and AI-first CAD. Triangle-first kernel (exact rationals, CSG, optional NURBS, sketch constraints) with a thin Python API and an optional OpenGL viewer. Designed to stay small and avoid dependency bloat.

This is **not** a fork or reimplementation of CadQuery, build123d, FreeCAD, or OpenSCAD. Those use OpenCascade B-rep (or OpenSCAD’s own language / a full GUI). Camber is a separate mesh kernel; `camber.cqcompat` is only a familiarity shim, not OCCT.

Status: APIs and file formats will change. Expect breakage. Much of the code is AI-assisted; the goal is useful geometry code, not AI slop — but plenty of bugs remain. It is already good enough to be useful for real scripts.

## Triangles first

The source of truth is a triangle mesh (ideally watertight). You can **load and modify** meshes from any source (STL, OBJ, …). NURBS is optional: objects built inside camber keep NURBS metadata when available, but you do not need it to work on imported meshes.

For export and analytic surfaces, **NURBS boundaries are derived from the triangles** where needed — triangles stay primary; curves/surfaces are recovered for convenience rather than owning the model.

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

Install the viewer stack with the `view` extra once wheels are published, or manually today:

```text
pyglet  imgui[pyglet]  numpy
```

Note: the current DotWrap-generated native module imports **numpy** on Linux even for headless use. Prefer `camber[view]` or `pip install numpy` until that import is optional.

## Wheels output (`dist/`)

Both Windows and Linux publish scripts write **only** `camber-*.whl` into the **repo-root** folder:

```text
camber/
  dist/
    camber-0.1.0-cp312-cp312-win_amd64.whl
    camber-0.1.0-cp312-cp312-linux_x86_64.whl
```

That layout matches a future PyPI release (`twine upload dist/camber-*.whl`). Dependency wheels (`cffi`, …) are staged temporarily and not kept in `dist/`. Scripts retag DotWrap’s `py3-none-any` name to a platform tag so Win/Linux wheels can sit side by side.

## Build the wheel (Windows)

Needs: Python 3.10+, .NET SDK (this tree targets `net10.0`), and VS **Desktop development with C++** (for Native AOT publish).

```powershell
cd <this-repo>\Geo.Python
.\publish-wheel.ps1
# → writes <this-repo>\dist\camber-*.whl
```

Install + smoke (repo root):

```powershell
uv pip install --python .\.venv\Scripts\python.exe (Get-ChildItem ..\dist\camber*.whl | Select-Object -Last 1).FullName
# or with viewer:
# uv pip install --python .\.venv\Scripts\python.exe "$((Get-ChildItem ..\dist\camber*.whl | Select-Object -Last 1).FullName)[view]"
python python\smoke.py
```

More detail: [Geo.Python/README.md](Geo.Python/README.md).

## Build the wheel (Linux / WSL Ubuntu 24.04)

Native AOT must be compiled **on Linux** (WSL is fine). Cross-compiling a Linux wheel from Windows is not supported here.

Your distro needs the .NET SDK, a C/C++ toolchain (`clang` or `gcc`), and Python headers. First time:

```bash
cd <this-repo>/Geo.Python
bash publish-wheel.sh --bootstrap   # first time: apt + .NET 10 into ~/.dotnet
# later: bash publish-wheel.sh
# → writes <this-repo>/dist/camber-*-linux_x86_64.whl
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
uv pip install --python .venv/bin/python "dist/camber-"*".whl[view]"
.venv/bin/python Geo.Python/python/smoke.py
```

The wheel embeds a CPython extension for the Python that built it (e.g. 3.12). Install into a matching interpreter.

## Install with uv (local wheel)

Git clone alone cannot replace a published platform wheel: Native AOT needs the .NET SDK and a C++ toolchain.

```powershell
uv venv
# kernel only:
uv pip install dist\camber-*.whl
# or kernel + viewer (pyglet, imgui, numpy):
uv pip install "dist\camber-*.whl[view]"
```

Wheels for Win/Linux land in repo-root `dist/`. Publishing to PyPI is a separate local checklist (not kept in this repo).

## C# solution

Open [`camber.sln`](camber.sln). Projects required for the Python wheel:

| Project | Role |
|---------|------|
| `Geo.Python` | Native AOT + DotWrap FFI |
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

Scan notes: other `http://…` links in comments are algorithm references (Bezier, NURBS notes, Morton codes, etc.), not copied libraries. A couple of geogram discussion URLs appear as comments only. Sketch text reads installed TrueType files at runtime; there is no vendored font engine. NACA / involute formulas and `GeoSolver/Sparse` are original / public math, not SuiteSparse or OCCT.

This repository currently has **no** top-level license for original camber code (all rights reserved until one is chosen). Third-party notices above still apply and must be retained.

## Layout

```text
camber.sln
Geo.Python/          # wheel build + python/camber package + examples
Geo/ …               # kernel projects listed above
third_party/         # MIT / Boost license copies
```
