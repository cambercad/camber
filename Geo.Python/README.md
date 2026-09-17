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


## Complete racing bicycle and image feedback

The [racing bicycle example](python/racing_bike.md) builds a full sketch-driven
road bike using nested wheel, drivetrain and component assemblies. From the
repository root with a locally built wheel installed:

```bash
.venv/bin/python Geo.Python/python/racing_bike.py
.venv/bin/python Geo.Python/python/racing_bike.py --render output/racing-bike
```

`camber.render_views(obj, "views.png")` captures six orthographic views and two
isometrics in one image using the existing viewer. It needs no new rendering
packages. Both `render_views` and `camber.show` accept an optional `colors` mapping
from entity-name patterns to RGB triples.

Feature edges can be inspected with `solid.edge_names`. These are the named
edges accepted by `part.fillet` and `part.chamfer`, excluding mesh diagonals.
An edge's two incident face names can be given in either order.


### Assembly solve diagnostics

```python
result = assembly.solve()
print(result)  # concise summary; solve itself does not print
if not result.converged:
    for mate in result.unsatisfied:  # largest normalized residual first
        print(mate.label, mate.entities, mate.max_residual, mate.tolerance)
```

The immutable result includes solver status, equation and parameter counts,
and per-mate residuals at the final pose. Residuals are normalized equation
values, not distances in model units. Each mate reports its applicable solver
tolerance; regularized contact uses a different threshold from equality mates.
`converged` reports the solver outcome, while `unsatisfied` checks each mate
against its own threshold. Counts are not remaining degrees of freedom, and
an unsatisfied mate is not proof that it caused a conflict.

### Assembly interference inspection

```python
hits = assembly.interferences(min_volume=0.1)  # mm³ when modelling in mm
for hit in hits:                              # largest overlap first
    print(hit.first, hit.second, hit.volume)
if hits:
    camber.render_views(hits[0].geometry, "largest_overlap.png")
```

Results contain full occurrence paths (indices distinguish repeated parts),
volume in cubic model units, and overlap geometry. Nested assembly transforms
are included. The check uses current poses without solving, moving parts, or
registering new model objects. `min_volume` defaults to zero and excludes volumes
at or below the supplied threshold. Exact boundary contact is not an overlap;
small positive mesh overlaps remain visible unless explicitly filtered.

The existing rational Boolean kernel checks tessellated solids. Construction
and tessellation tolerances can therefore affect measured fits. This is a
static interference check, not minimum clearance or motion validation. A failed
pair raises an error naming both occurrences; it is never reported as clear.
Inspection is separate from model construction and can take longer for large
assemblies with many intersecting bounding boxes.

### Section views and measurements

```python
cut = camber.section(assembly, "XZ", offset=0)  # or a camber.Frame
near = cut.raycast((0, -100, 0), (0, 1, 0))
far = cut.raycast((0, 100, 0), (0, -1, 0))
if near is not None and far is not None:
    distance = cut.measure(near, far, label="Across section")
    print(distance.start, distance.end, distance.length)
camber.render_views(cut, "section.png")
camber.show(cut)  # M, then two surface picks: add a measurement; Esc cancels
```

A section is an independent capped snapshot of the current model pose. It keeps
plane-local Z ≤ 0 and removes the positive side without changing model features,
assembly poses, or source geometry. XY has +Z normal, XZ has −Y normal, and YZ has
+X normal; `offset` moves along that normal. Existing face and occurrence names
are preserved. Generated cut faces are named `section_cap` and accept the same
color rules as other faces.

Sections render in plane-normal and isometric views by default at 1600×1200 per
view. An orthographic scale bar and stored measurement annotations use **model
units**. `measure` accepts ray hits, world 3-vectors, or section-local `(x, y)`
pairs; it returns an immutable result also available in `cut.measurements`.
These are straight point-to-point distances. Arbitrary two surface picks do not
establish minimum clearance or minimum wall thickness. Ray picks and caps use
the existing tessellated geometry and exact predicate kernel.

### Surface lofts

`part.loft_surface(sections)` creates an uncapped NURBS sheet (`is_volume` is
`False`). Sections are ordinary sketches, including constrained sketches. They
retain their authored order and direction; curve degrees and knots are matched
using exact degree elevation and knot insertion. No section fitting is performed.

```python
sheet = part.loft_surface(sections)  # no guides or end controls required
sheet = part.loft_surface(sections, guides=[hub_guide, tip_guide])
sheet = part.loft_surface(sections,
                          start_tangent=(0, 20, 40),
                          end_tangent=(0, -20, 40))
```

Tangents are optional world-space derivative vectors: their direction and magnitude
both matter. Both point forward through the ordered sections. The loft parameter
runs from zero to one, with equal intervals between consecutive sections. Omitting
a tangent uses the automatic end slope; supplying one does not change the sections.

Currently guides control the two side boundaries of open sections. Use a line or
`Curve.hermite` with one knot at the same endpoint of each section, in section
order. A line must cross those endpoints at the corresponding equally spaced
parameters. Guides are preserved between sections, not just at sampled points.
Duplicate, missed, reversed or conflicting guides raise an error. Interior guides
and arbitrary guide reparameterization are not supported yet. Rational sections
must have equal weights after degree/knot matching. Adjacent-face tangency and
curvature matching are not implied by the vector controls.

Run the constrained-sketch example (opens the existing viewer by default):

```bash
.venv/bin/python Geo.Python/python/surface_loft.py
```

### Matched loft faces and explicit first curves

```python
solid = part.loft(sections, first_curves=["bottom", "bottom", "bottom", "bottom"])
```

`first_curves` has exactly one curve name per section, in the same order as
`sections`. Each name resolves within its own sketch. The selected curve's
start-to-end direction defines section traversal, including clockwise or
counterclockwise orientation; no separate orientation flag is required.
Names, list length and incompatible alignment controls are validated.

Supplying `first_curves` with no options selects matching-vertex correspondence.
This mode supports sections with matching named lines and curves, including
two-arc/two-line rounded slots. Rational section weights must agree after degree
and knot matching. Each authored curve produces a separate selectable face named `<loft>-Side-<edge>`, plus
start/end caps. Side names come from the first section's authored curve names,
remain stable under dimension/tessellation changes, and retain analytic UV domains.
Explicit matched correspondence rejects incompatible sections; it does not silently
switch to perimeter-based correspondence. The existing perimeter-based loft remains
available for other section structures and currently retains one side face.

The inspection example has four constrained rectangular sections, varying dimensions
and twist, four side faces and two caps:

```bash
.venv/bin/python Geo.Python/python/loft_transition.py
.venv/bin/python Geo.Python/python/loft_transition.py --sections
```

### Tessellation density

Sketch B-splines and spatial NURBS curves use control-hull subdivision driven by
geometric tolerance. A straight span does not require uniform subdivisions.
Lofts use profile anchors and adaptive surface refinement by default; explicit
C# `ProfileSamplesU` and `VSubdivisionsPerSpan` requests still add uniform samples.
Their defaults are zero. Shared boundaries and surface curvature can require
additional vertices even where an individual boundary curve is straight.
Twisted patches refine both parameter directions as needed. Rational Bezier
spans use a homogeneous control-hull residual bound, so a long straight
extrusion does not inflate the required resolution around its curved profile.

### Loft section connectors

`part.loft(sections)` preserves each sketch's authored start point as its
section connector. This is also the C# `LoftOptions` default (`AsAuthored`).
Previously the default independently chose the vertex nearest each sketch
origin; nearly symmetric profiles could flip by 180° after an insignificant
coordinate change. The authored default avoids that ambiguity without a
proximity tolerance. Explicit C# `OriginFootRoll` and `MinimumTwist` requests
keep their existing behavior; use the former only when its proximity-based
seam is intentional. Author matching start points when building a sequence
of closed sections. The geometry and default analytic metadata both follow
that authored curve order.


### Ellipse sections

`sketch.add_ellipse(center, radii, rotation=0, name=None)` creates a true
ellipse. `radii` contains its two semiaxis lengths; `rotation` is in radians.
Use the existing point and distance constraints to dimension it:

```python
sk = part.sketch("xy", constrained=True)
sk.solve_after_every_constraint = False
profile = sk.add_ellipse((0, 0), (30, 20), name="section")
sk.fix(profile @ "center")
sk.distance(profile @ "center", profile @ 0, 30)
sk.distance(profile @ "center", profile @ .25, 20)
sk.point_on_line(profile @ 0, sk @ "x")
sk.solve()
```

The center and quarter-parameter points remain named sketch references.
Ellipse dimensions use distances; a circular radius constraint does not apply.
Lofts sample the actual sketch curves by default and derive shading normals
from the loft surface. The explicit C# `TessellatedPolyline` sampling option
remains available when polygonal profiles are intended.


### Patterns and mirrors

Solid patterns make copies that can be used as additive bodies or cutting tools:

```python
holes = part.pattern_linear(hole_tool, count=4, step=(20, 0, 0))
plate = part.cut(plate, part.batch_union(holes))
ring_tools = part.pattern_circular(hole_tool, count=6, axis=camber.Frame())
other_hand = part.mirror(bracket, plane=camber.Frame(), name="left_bracket")
```

Assembly patterns use the same count/step/axis convention:

```python
seed = assembly.add_subassembly(bearing_unit, position=(40, 0, 0))
assembly.fix(seed)
units = assembly.pattern_circular(seed, count=4, axis=camber.Frame())
other_hand = assembly.mirror(seed, plane=camber.Frame(), name="mirrored_unit")
result = assembly.solve()
print(result)
```

A seed can be a direct part or a direct subassembly occurrence. Subassembly copies
retain their hierarchy, local poses and internal mate definitions; their internal
mechanisms can be edited independently. Repeated standard parts share their solid
definition. Mirrors create opposite-handed geometry and preserve outward faces.

`count` includes the original seed, returned as the first list item. Full circles
omit the duplicate endpoint; partial circular sweeps include both endpoints.
Angles are radians. A Frame's Z axis defines the circular axis and its XY plane
defines the mirror plane. Steps and planes use the owning assembly's coordinates.

Assembly copies are connected to the seed by recorded rigid placement mates. The
pattern or mirror determines their initial placement; subsequent movement follows
the seed rigidly. This is not a continuously reflected symmetry constraint across
a stationary world plane. As with existing nested assemblies, the parent treats
each child as rigid while retaining the child's internal mates.

### Independent construction with Python threads

Use the standard-library `ThreadPoolExecutor` for independent component builders.
Each worker owns its `Part`, sketches, solids, and cutting tools. Do not mutate
one CAD object from multiple workers, or render while workers are modifying it.
Use explicit feature names when identities must remain readable and repeatable;
automatically generated names are unique but their ordering follows scheduling.

```python
from concurrent.futures import ThreadPoolExecutor
from camber import Part

low, high = (-20, -20, -20), (20, 20, 20)

def build_component(index):
    owner = Part(low, high, tolerance=.01)
    solid = owner.cuboid((0, 0, 0), (2 + index, 2, 2), name="blank")
    if not solid.is_watertight():
        raise ValueError(f"Component {index} is not watertight")
    return solid  # The Solid retains its owning Part.

with ThreadPoolExecutor(max_workers=4) as workers:
    solids = list(workers.map(build_component, range(4)))

part = Part(low, high, tolerance=.01)
assembly = part.assembly("components")
for index, solid in enumerate(solids):
    local = part.copy_solid(solid, name=f"component_{index}")
    assembly.add_part(local, (0, 0, 3 * index))
```

Assembly mutation and transfers occur sequentially. `copy_solid` preserves exact
coordinates and face provenance; cross-Part copies require matching coordinate
lattice origins and steps. Matching working volumes satisfy that requirement.
Directly adding another Part's solid to an assembly is rejected.

Constructing a `Part` does not reset other sessions. The native instance registry
retains sessions until an explicit global reset or process exit, so dropping a
Python variable does not guarantee that all its geometry is reclaimed. Never
reset global state while workers are active. This workflow is not a blanket
thread-safety guarantee for shared objects or viewer operations.

### Face names and rebuilds

Give features and sketch elements explicit names when their faces will be used
by later features or assembly mates. A name such as `frame_head_tube-Side`
expresses the generating feature and face role. Internal numeric face IDs only
associate geometry and metadata during execution; they are not persistent
references and may change on rebuild.

Supported planar Boolean splits retain their creation and separating-boundary
names. For example, `base-ExtrudeTop{main_slot-Line2}` identifies the top region
beside that slot wall. An unrelated earlier cut or hole does not renumber this
reference. Explicitly name the generating features and sketch elements.

A split ancestor, obsolete ordinal alias, or indistinguishable region raises an
ambiguity error during datum lookup. Curved split regions and repeated copies
with indistinguishable ancestry are not yet covered by persistent naming; their
geometry can still be built and inspected. Diagnostic numeric suffixes are not
stable references and must not be used to migrate an old selector automatically.

For a fillet or chamfer, an authored support pair such as
`[frame_top_tube-Side,frame_head_tube-Side]` may identify a unique connected edge
even when either surface has several regions. Zero or multiple matching edges
raise an error. An explicit edge ordinal cannot expand split ancestors. Face
datums still require one unambiguous region.
