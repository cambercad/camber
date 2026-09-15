# Camber (Python)

`camber` is a pip-installable CAD kernel: sketch, extrude, boolean, STL/STEP. The 3D viewer (pyglet + imgui) is **optional**.

Published package: **`cambercad`**. Import: **`import camber`**.

```text
pip install cambercad
pip install "cambercad[view]"
```

This folder is also the **wheel build** (Native AOT + DotWrap), not the old GeoScriptViewer scripts (`from Geo import …`).

---

## What you need (Windows)

Do this once. Skip a step only if you already have it.

1. **Python 3.10+**  
   In a terminal: `python --version`  
   If that fails, install Python from python.org and tick **Add python.exe to PATH**.

2. **.NET SDK** (this tree targets `net10.0`)  
   `dotnet --version` should print a recent SDK.  
   https://dotnet.microsoft.com/download

3. **C++ build tools** (needed to *publish* the native library, not to run Python later)  
   Visual Studio Installer → **Desktop development with C++**  
   (or Build Tools with the MSVC + Windows SDK workloads).

4. A **PowerShell** window. Run all commands below in the **same** window after you activate the venv.

---

## 1. Open this folder and make a venv

```powershell
cd <camber-repo>\Geo.Python

python -m venv .venv
.\.venv\Scripts\Activate.ps1
```

You should see `(.venv)` in the prompt. If Windows blocks the script:

```powershell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
.\.venv\Scripts\Activate.ps1
```

Always use **this** Python from now on (`where.exe python` should point inside `.venv`).

---

## 2. Build the wheel

Still in `Geo.Python`, venv active:

```powershell
.\publish-wheel.ps1
```

This compiles the Native AOT library and writes a `.whl` under `dist\`. First run can take several minutes.

If it fails with linker / `cl.exe` / C++ errors, you do not have the C++ workload (step 3 above).

---

## 3. Install camber + the viewers

Still in the same venv:

```powershell
python -m pip install --upgrade pip
python -m pip install --force-reinstall (Get-ChildItem dist\camber*.whl | Select-Object -Last 1).FullName
# Kernel needs cffi (usually pulled in by the wheel). Viewer is optional:
python -m pip install pyglet "imgui[pyglet]" numpy
```

Check:

```powershell
python -c "import camber, pyglet, imgui; print('ok', camber.__file__)"
```

If `import camber` fails, the wheel is not installed in **this** venv. Activate `.venv` again and retry step 3.

If `import pyglet` or `import imgui` fails: `python -m pip install pyglet "imgui[pyglet]" numpy`.

---

## 4. Run an example (no window)

From `Geo.Python`:

```powershell
python python\smoke.py
```

You should see `ok … triangles` and an STL path. That does **not** open a viewer.

---

## 5. Run with the OpenGL viewer (3D window)

From `Geo.Python`:

```powershell
python -c @"
from camber import Part, vec3
part = Part(vec3(-10), vec3(10), tolerance=0.05)
sk = part.sketch('xy', name='box')
sk.add_line((0, 0), (2, 0))
sk.add_line((2, 0), (2, 2))
sk.add_line((2, 2), (0, 2))
sk.add_line((0, 2), (0, 0))
cube = part.extrude(sk, 2.0, name='cube')
hole = part.cylinder(origin=(1, 1, -0.5), radius=0.4, height=3.0, name='cyl')
(cube - hole).show()
"@
```

A window should open. Click a face/edge/point to copy its name. **Ctrl-click** = multi-select.

Same example as a file:

```powershell
python python\view_smoke.py
```

Classic CSG example (sphere ∩ cube, minus three cylinders — like the screenshot). **Rebuild the wheel first** (`sphere` and `axis=` cylinders are new):

```powershell
.\publish-wheel.ps1
python -m pip install --force-reinstall (Get-ChildItem dist\camber*.whl | Select-Object -Last 1).FullName
python python\csg_sphere_cube.py
```

Triangulate a sketch into an open sheet (not a volume) and show it with a cube:

```powershell
python python\sketch_triangulate.py
```

Herringbone (double-helical) involute gear — Citroën chevron:

```powershell
python python\v_gear.py
```

3:1 planetary gearbox (ring fixed, carrier output; involute sun / planets / internal ring):

```powershell
python python\planetary_gearbox.py
```

---

## 6. Interactive sketch (optional)

Draw in the viewer; **Finish** copies camber Python to the clipboard.

```powershell
python -c "from camber import Part, vec3; Part(vec3(-50), vec3(50)).sketch_interactive(plane='xy', name='box')"
```

---

## Everyday use after the wheel exists

```powershell
cd <camber-repo>\Geo.Python
.\.venv\Scripts\Activate.ps1
python python\smoke.py
```

Rebuild the wheel only after you change C# (`NativePart.cs`, Geo, …) or the Python package under `python\camber\`:

```powershell
.\publish-wheel.ps1
python -m pip install --force-reinstall (Get-ChildItem dist\camber*.whl | Select-Object -Last 1).FullName
```

---

## Layout

| Path | What it is |
|------|------------|
| `python/camber/` | Public package (`Part`, `show`, sketch UI) |
| `python/smoke.py` | Kernel smoke test (no viewer) |
| `python/view_smoke.py` | Same part, opens Polyscope |
| `python/csg_sphere_cube.py` | Sphere ∩ cube − 3 holes |
| `python/sketch_triangulate.py` | Sketch fill as an open sheet + a volume |
| `python/v_gear.py` | Citroën-style herringbone involute gear |
| `python/gears.py` | Involute spur / internal ring / herringbone helpers |
| `python/planetary_gearbox.py` | 3:1 planetary assembly (sun, planets, ring, carrier, housing) |
| `NativePart.cs` | Thin C# FFI for the wheel |
| `publish-wheel.ps1` | Publish AOT + build `dist\*.whl` |
| `dist/` | Output wheels (generated) |

`PYTHONPATH` to `python\` is **not** enough: the native `_camber_native` library only arrives via the installed wheel.

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `No module named 'camber'` | Activate `.venv`, reinstall the `.whl` from `dist\` |
| `The OpenGL viewer needs pyglet` / import error on `show()` | `python -m pip install pyglet "imgui[pyglet]" numpy` |
| Two Pythons / “works in IDE, not in terminal” | `where.exe python` — must be `Geo.Python\.venv\Scripts\python.exe` |
| Publish fails at link | Install VS **Desktop development with C++** |
| Old geometry after code changes | Rebuild wheel (`publish-wheel.ps1`) **and** `--force-reinstall` the new `.whl` |
