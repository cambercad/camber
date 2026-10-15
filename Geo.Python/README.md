# Camber (Python)

`camber` is a pip-installable CAD kernel: sketch, extrude, boolean, STL/STEP. The 3D viewer (pyglet) is **optional**.

Published package: **`cambercad`**. Import: **`import camber`**.

```text
pip install cambercad
pip install "cambercad[view]"
```

The full GUI modeling mode is **experimental and incomplete**. The Operations
tab provides searchable forms for geometry operations, derived from the Python
API signatures. Complex inputs and sketches using unsupported Python constructs still need
direct Python editing. With a wheel built from this checkout and the viewer dependencies
installed, start it with:

```text
python -c "import camber; camber.modeling.open('my_part.py')"
```

From the repository root on Windows, use `.\.venv\Scripts\python.exe` in place of
`python` if the project virtual environment is not activated.

The same entry point works from a Python script:

```python
import camber
camber.modeling.open("my_part.py")
```

If the file does not exist, the workspace starts with an unsaved Part. Add sketches and features from the toolbar; select a feature to edit its parameters or Python block, then Apply and Save. The Python file remains the model definition. A failed rebuild leaves the previous geometry visible. Double-click a sketch to edit its geometry, constraints and dimensions. Unsupported sketch constructs remain editable through their Python block without losing information.

Frame parameters have coordinate editors, standard plane orientations, named frame
references, and placement from selected planar faces. Invalid inputs have an error
border and an inline explanation. Apply remains disabled until form validation
passes; geometry failures keep the edit open and preserve the previous model.
Cancel discards the edit.

Choosing a sketch-based feature without an existing sketch opens sketch setup.
Finish drawing to return to that feature with the new profile selected. Selecting
a sketch also exposes Extrude, Revolve and Surface directly in its properties.

The document toolbar provides New, Open, Save, Undo and Redo. Keyboard shortcuts
are Ctrl+N/O/S, Ctrl+Z/Y, and Ctrl+Enter to apply the current feature. Undo inside
a text field edits that field; outside it, Undo restores the previous model revision.
Switching features, saving or closing prompts before discarding unapplied edits.
The feature tree can be filtered. Frame placement offers **Pick planar face**;
select a face in the viewport, or press Esc to cancel picking.

Use **File → New assembly**, then **Insert** to choose a saved part or assembly.
The first component is fixed automatically. On the Assembly tab, choose a mate,
then pick its first and second references in the viewport. Repeated components
have distinct references. Face mates default to opposing normals. Preview checks
the solved placement; Create mate keeps it. Failed solves retain the last valid
assembly. The navigator groups Components and Constraints, including nested assemblies.
Select a component or mate to inspect it; double-click an authored mate to edit it.
Open component model edits its saved source; Save and Back to assembly rebuild the parent.
During sketch editing, the same navigator groups Curves, Constraints and Dimensions.
Click to select geometry or supply a constraint reference; double-click to edit.
Drawing tools, constraints and dimensions appear in the main Sketch ribbon. The
docked Properties panel edits the selection; the document toolbar and status bar
remain in place. Finish sketch and Cancel sketch return to part modeling.
Finish returns to the owning part and preserves the sketch's named placement.

Use **View → Measure**, then Ctrl-click two points, edges or faces to see their
minimum distance and X/Y/Z separation. **View → Section view** reveals the inside
of a part or assembly: choose an axis, align to the view or use a selected planar
face, then adjust the position or reverse the retained side. **Done** closes the
settings while keeping the cut visible; reopen Section view to disable it.
These inspection tools leave the model and its construction history unchanged.
Section clipping pauses during sketch editing and returns when the sketch closes.

Sketch setup offers **Pick planar face**, and point inputs offer **Pick point**.
Selections store named references, so edits resolve the current feature geometry
rather than freezing picked coordinates. Frame references are available in Python
as `solid.face_frame(face_name, inward=False)` and points as `solid.point(point_name)`.
**Save as** (Ctrl+Shift+S) preserves relative component links and undo history.

Detailed output is hidden initially to leave room for the model. **Python** opens
the source editor; **View → Output** toggles the message/source panel. Advanced
frame directions and optional arguments expand when needed, including on errors.
**Commands** or Ctrl+F1 toggles the ribbon to maximize the viewport.

The cube at the bottom left of the viewport follows the current orientation.
Click a face center, an edge strip or a corner block to change the view; zoom
and the view center stay unchanged. It is available in both viewer workflows.
Edges running along X are red, Y green and Z blue.

Required inputs appear first; expand Options for optional arguments. Existing
feature results are available as inputs, including individual pattern instances.
Coordinates and profile lists accept Python literals such as `(0, 0, 10)` and
`[first, second]`. Public API methods returning modeling objects automatically
receive forms; existing API categories organize the commands. Display calls and
numeric/data queries do not become modeling features.

The feature tree uses operation icons and nests consumed sketches beneath their
extrusions or other features. Select or double-click a feature to open its
parameter editor; Apply rebuilds dependent features. Expand a feature to select
its profile. Preview shows that feature's result; Show final restores the finished
part. Supported constrained sketches can be reopened without dropping their constraints.

For a complete source-backed example with ribs, a stepped bearing bore and four
counterbored mounting holes, open `python/examples/modeling/bearing_housing.py`
in modeling mode, or run that script directly.

For an assembly example with revolved, grooved races and nine rolling elements,
run `python python/examples/modeling/ball_bearing/bearing.py`. Select a race in
the assembly tree and choose **Open component model** to edit its sketches and
features. Save the component, then choose **Back to assembly** to rebuild it.

### Automated GUI testing

`camber.gui_testing.GuiDriver` sends mouse, keyboard and text events directly to
the pyglet window. It finds visible controls by label and panel, supports dragging
and scrolling, and captures screenshots without desktop automation. It needs
only pyglet; the driver contains no CAD logic. Control inspection is opt-in and
does not retain geometry during normal GUI use.

```python
from camber.gui_testing import GuiDriver

driver = GuiDriver(ui, draw_frame, capture=capture_image)
driver.click("Name", panel="Properties")
driver.text("Example", replace=True)
driver.key("TAB")
driver.click("Apply", panel="Properties")
driver.screenshot("gui.png")
```

Use the application's frame callback for `draw_frame`. `capture_image` is
optional; the CAD viewer's `capture_image()` includes the scene and GUI using
an offscreen render target. Native file dialogs are tested through their
selection callbacks. Run the GUI and workflow regression tests with
`CAMBER_TEST_GL=1`; they use hidden windows and need an OpenGL context.

To control an already running modeling workspace from another Python process,
explicitly enable its local test endpoint:

```python
import camber
camber.modeling.open("my_part.py", control_file="gui-control.json")
```

Then run small control scripts in another terminal:

```python
from camber.gui_remote import GuiControlClient

gui = GuiControlClient.from_file("gui-control.json")
print(gui.controls())  # Visible labels, panels, rectangles and enabled states.
gui.click("Solid##tab", panel="Model")
gui.screenshot("workspace.png")
# gui.click("Diameter", panel="Properties")  # Use the label from controls().
# gui.text("8", replace=True)
```

The client also supports `move`, `drag`, `scroll`, and `key`. Coordinates are
logical pixels measured from the top left. Responses arrive after the input
has been processed and the GUI redrawn. Screenshots are PNG bytes and are saved
by the client. Native OS file dialogs remain outside this bridge.

The endpoint listens only on `127.0.0.1`, requires its random session token,
and accepts a fixed set of GUI commands. No arbitrary Python command is accepted.
The connection file must not already exist and is removed on normal exit;
delete a stale file after an interrupted run. Keep this credential file private.
No additional dependencies are installed, and normal GUI startup has no server.

Other pyglet applications can use the same bridge without CAD logic:

```python
import pyglet
from camber.gui_remote import GuiControlServer

with GuiControlServer(driver) as server:  # Create on the GUI thread.
    print(server.connection)             # Pass these credentials to the client.
    pyglet.app.run()
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
python -m pip install "pyglet>=3.0.dev11,<4" numpy
```

The viewer currently uses the pyglet 3 development release for its graphics API
and native file dialogs. Pyglet 2 is no longer supported by this checkout.

Check:

```powershell
python -c "import camber, pyglet; print('ok', camber.__file__)"
```

If `import camber` fails, the wheel is not installed in **this** venv. Activate `.venv` again and retry step 3.

If `import pyglet` fails: `python -m pip install "pyglet>=3.0.dev11,<4" numpy`.

---

## 4. Run an example (no window)

From `Geo.Python`:

```powershell
python python\smoke.py
```

You should see `ok … triangles` and an STL path. That does **not** open a viewer.

---

## 5. Run with the OpenGL viewer (3D window)

The viewer's dockable UI is a reusable pyglet module (`python/camber/gui.py`).
Run `python python/examples/standalone/control_gallery.py` from `Geo.Python`
to try its tabs, splitters, text fields, choice controls, trees, and light/dark
themes without loading a CAD model. The gallery's theme button switches at
runtime; other pyglet consumers can call `ui.set_theme("light")` or
`ui.set_theme("dark")` on their `camber.gui.UI` instance.
In the Camber viewer, press `D` to switch the UI theme; text fields keep `D`
for typing while focused.

The reusable GUI also provides themed context menus. Open one explicitly on a
right-click (after distinguishing it from a camera drag), then draw it each
frame outside conditional panels:

```python
from camber.gui import MenuItem

# In the application's right-click handler:
ui.OpenContextMenu("actions")

# In the frame callback, before ui.render():
choice = ui.ContextMenu("actions", [
    MenuItem("Rename…", icon="✎", shortcut="F2"),
    None,  # separator
    MenuItem("Delete", enabled=False),
])
if choice == (0,):
    rename_selected()
```

The returned index path refers to the original lists, including separators:
`(0,)` is the first root action, `(0, 1)` is its second submenu action. Shortcut
labels are hints, not bindings. Menus support keyboard navigation, scrolling,
outside-click/Escape dismissal, and optional checkmarks. Right-click anywhere
in `control_gallery.py` to try them in either theme.

`MenuItem(..., children=(...))` creates a submenu. Nested menus share context
menu rendering and keyboard handling; they open to the left when there is not
enough room on the right. `ui.MenuBar("main", headings)` takes MenuItem headings
with children and returns the same index paths, including the heading index.
Left/right arrows navigate menu levels or switch headings; Escape closes one
level at a time. Menu bars also switch headings on hover while open.

Property inspectors are composed from existing controls, with aligned labels
and collapsible groups. No model-binding framework is required:

```python
if ui.PropertyGroup("Geometry"):
    changed, radius = ui.Property("Radius", radius, step=0.1)
    changed, name = ui.Property("Name", name)
if ui.PropertyGroup("Appearance"):
    changed, visible = ui.Property("Visible", visible)
    changed, material = ui.Property("Material", material,
                                    options=["Steel", "Aluminium"], editable=True)
    ui.Property("Identifier", identifier, read_only=True)
```

Group names scope property identities. Numbers are edited as floats; boolean,
text, preset, free-form preset and read-only rows reuse their corresponding
controls. The gallery's View menu opens the Properties demonstration.

Numeric fields support optional step buttons; editable combo boxes let users
choose a preset or type another value. Both follow the `(changed, value)` pattern:

```python
changed, radius = ui.InputDouble("Radius", radius, step=0.1, fast_step=1.0)
changed, material = ui.EditableComboBox("Material", material, ["Steel", "Aluminium"])
```

Hold Ctrl while stepping to use `fast_step`. A zero step keeps a plain numeric
field. Confirmation dialogs block background mouse and keyboard input, remain
open on outside clicks, and use the current theme:

```python
# When the action is requested:
ui.OpenMessageBox("Delete body?", "This removes the selected body.",
                  buttons=("Delete", "Cancel"), default="Cancel")

# Every frame, after the other controls and before ui.render():
response = ui.MessageBox("Delete body?")
if response == "Delete":
    delete_selected()
```

Tab/arrows move between buttons; Enter chooses the focused button. Escape
chooses the configured cancel button (automatically `Cancel` when present).
Long messages scroll with the mouse wheel. A missing cancel choice means
Escape does not dismiss the dialog.

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
cube = sk.extrude(2.0, name='cube')
hole = part.cylinder(origin=(1, 1, -0.5), radius=0.4, height=3.0, name='cyl')
(cube - hole).show()
"@
```

A window should open. Click a face/edge/point to copy its name. **Ctrl-click** = multi-select.

`Part` is a modelling document: it owns sketches, geometry, names, operation
history and the default tolerance. `Solid` is a solid body within that document.
A Part can contain multiple solids, including intermediate modelling results.

The Python API follows the geometry being edited: construct from a `Sketch`
(`sk.extrude(...)`, `sk.revolve(...)`), modify a `Solid`
(`cube.shell(...)`, `cube.fillet(...)`, `cube.hole(...)`), and combine solids
through their owning `Part` (`part.union(...)`, `part.batch_subtract(...)`).
The [API reference](python/docs/camber-api.html) groups methods from metadata
on their definitions.

Same example as a file:

```powershell
python python\view_smoke.py
```

Classic CSG example (sphere ∩ cube, minus three cylinders — like the screenshot). **Rebuild the wheel first** (`sphere` and `axis=` cylinders are new):

```powershell
.\publish-wheel.ps1
python -m pip install --force-reinstall (Get-ChildItem dist\camber*.whl | Select-Object -Last 1).FullName
python python\examples\standalone\csg_sphere_cube.py
```

Triangulate a sketch into an open sheet (not a volume) and show it with a cube:

```powershell
python python\examples\standalone\sketch_triangulate.py
```

Herringbone (double-helical) involute gear — Citroën chevron:

```powershell
python python\examples\standalone\v_gear.py
```

3:1 planetary gearbox (ring fixed, carrier output; involute sun / planets / internal ring):

```powershell
python python\examples\standalone\planetary_gearbox.py
```

Shell gallery (analytic cylinder and sphere offsets, CSG trimming, a filleted
shell rim, and the generic triangle-mesh fallback on a twisted loft):

```powershell
python python\examples\standalone\shell_showcase.py
```

Outward shelling defaults to sharp joins; pass `join="round"` to round
supported convex joins with the wall thickness as radius. The opening rim
stays sharp. Inward rounded joins and unsupported concave corners still fail
explicitly rather than returning an open or self-intersecting shell.

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
| `python/examples/standalone/` | Standalone modeling examples and their helpers |
| `NativePart.cs` | Thin C# FFI for the wheel |
| `publish-wheel.ps1` | Publish AOT + build `dist\*.whl` |
| `dist/` | Output wheels (generated) |

`PYTHONPATH` to `python\` is **not** enough: the native `_camber_native` library only arrives via the installed wheel.

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `No module named 'camber'` | Activate `.venv`, reinstall the `.whl` from `dist\` |
| `The OpenGL viewer needs pyglet` / import error on `show()` | `python -m pip install "pyglet>=3.0.dev11,<4" numpy` |
| Two Pythons / “works in IDE, not in terminal” | `where.exe python` — must be `Geo.Python\.venv\Scripts\python.exe` |
| Publish fails at link | Install VS **Desktop development with C++** |
| Old geometry after code changes | Rebuild wheel (`publish-wheel.ps1`) **and** `--force-reinstall` the new `.whl` |


## Complete racing bicycle and image feedback

The [engineering models gallery](python/examples/engineering_models/README.md)
also contains the gearbox, impeller, wrench, construction bricks, and connectors.

The [racing bicycle example](python/examples/engineering_models/racing_bike.md) builds a full sketch-driven
road bike using nested wheel, drivetrain and component assemblies. From the
repository root with a locally built wheel installed:

```bash
.venv/bin/python Geo.Python/python/examples/engineering_models/racing_bike.py
.venv/bin/python Geo.Python/python/examples/engineering_models/racing_bike.py --render output/racing-bike
```

`camber.render_views(obj, "views.png")` captures six orthographic views and two
isometrics in one image using the existing viewer. It needs no new rendering
packages. Both `render_views` and `camber.show` accept an optional `colors` mapping
from entity-name patterns to RGB triples.

Feature edges can be inspected with `solid.curve_names`. These are the named
edges accepted by `solid.fillet` and `solid.chamfer`, excluding mesh diagonals.
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

In the ordinary 3D viewer, press **I** to open Inspect. In its **Measure** tab,
Ctrl-click two named points, edges or faces and choose **Measure minimum distance**. All combinations
are supported, including edge–surface. Picking and closest-distance calculations
run on the CPU in C# using cached BVHs; Python only handles viewer data and UI.
Distances follow the displayed geometry's tessellation tolerance.

In the **Cross section** tab, turn on **Enable cross section**. Choose the plane's
normal axis, align to the view or a selected planar face, or set any orientation
with azimuth/elevation sliders and numeric fields (degrees). Move the position
slider or enter a distance in model units; zero is the model center. **Keep
opposite side** reverses the cut. Updates use GPU clipping and reuse the existing
mesh; they do not modify the CAD model. Section fill is intended for closed solids;
disable it when inspecting open sheets. Use `camber.section` below when an
actual capped geometry snapshot is needed.

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
`Curve3D.hermite` with one knot at the same endpoint of each section, in section
order. A line must cross those endpoints at the corresponding equally spaced
parameters. Guides are preserved between sections, not just at sampled points.
Duplicate, missed, reversed or conflicting guides raise an error. Interior guides
and arbitrary guide reparameterization are not supported yet. Rational sections
must have equal weights after degree/knot matching. Adjacent-face tangency and
curvature matching are not implied by the vector controls.

Run the constrained-sketch example (opens the existing viewer by default):

```bash
.venv/bin/python Geo.Python/python/examples/standalone/surface_loft.py
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
two-arc/two-line rounded slots. Rational sections are interpolated in homogeneous
coordinates, including differing weights. Each authored curve produces a separate selectable face named `<loft>-Side-<edge>`, plus
start/end caps. Side names come from the first section's authored curve names,
remain stable under dimension/tessellation changes, and retain analytic UV domains.
Explicit matched correspondence rejects incompatible sections; it does not silently
switch to perimeter-based correspondence. The existing perimeter-based loft remains
available for other section structures and currently retains one side face.

### Solid loft guides and end conditions

```python
body = part.loft(sections, guides=[leading_edge, trailing_edge])
body = part.loft(sections, start_tangent=(0, 0, 100), end_tangent=(0, 0, 100))
body = part.loft(sections, start_point=(0, 0, -20), end_point=(0, 0, 120))
# Choose a conical tip instead of the default rounded closure:
body = part.loft(sections, end_point=(0, 0, 120), end_closure="sharp")
```

Guides are lines or Hermite curves passing through corresponding contour joints
of every section, in section order. End tangents are world-space derivative
vectors along that order; their lengths control the bend. The longitudinal
parameter runs from zero to one with equal section intervals.

Tip points lie beyond the first or last section. Rounded closures join smoothly
to the loft and end with a rounded tip; sharp closures form cones. Without a tip
point, the existing flat cap remains. Rounded closures require a regular,
star-shaped end profile, an interior projected tip and compatible tangency.
Unsupported or conflicting geometry raises an explanatory error rather than
fitting or snapping the profiles. Supplying these controls without `options`
selects matched-vertex correspondence automatically.

In the full GUI, choose **Loft → End conditions** for tip coordinates, closure
styles and optional tangents. Guides and profile correspondence are under
**More options**. Invalid inputs prevent applying the feature.

The inspection example has four constrained rectangular sections, varying dimensions
and twist, four side faces and two caps:

```bash
.venv/bin/python Geo.Python/python/examples/standalone/loft_transition.py
.venv/bin/python Geo.Python/python/examples/standalone/loft_transition.py --sections
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

Python can choose the same alignment explicitly:
`part.loft(sections, options=LoftOptions(alignment="minimum_twist"))`.
The full GUI provides ordered profile selection and **Customize loft → Alignment →
Minimize twist**. This is useful for a rectangular-to-round transition whose
section start points do not correspond. Inspect the preview before applying.


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
holes = hole_tool.pattern_linear(count=4, step=(20, 0, 0))
plate = part.batch_subtract(plate, holes)
ring_tools = hole_tool.pattern_circular(count=6, axis=((0, 0, 0), (0, 0, 1)))
other_hand = bracket.mirror(plane=camber.Frame(), name="left_bracket")
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

A seed can be a direct part or a direct subassembly occurrence. Subassembly pattern
copies retain their hierarchy, local poses and internal mate definitions. Repeated
standard parts share their solid definition. Mirrors create opposite-handed geometry
and preserve outward faces.

`count` includes the original seed, returned as the first list item. Full circles
omit the duplicate endpoint; partial circular sweeps include both endpoints.
Angles are radians. A Frame's Z axis defines the circular axis and its XY plane
defines the mirror plane. Steps and planes use the owning assembly's coordinates.

Assembly copies are connected to the seed by recorded rigid placement mates. The
pattern or mirror determines their initial placement; subsequent movement follows
the seed rigidly. This is not a continuously reflected symmetry constraint across
a stationary world plane. By default, ``add_subassembly`` treats the child as rigid
and reuses its internal pose. Pass ``flexible=True`` when each occurrence needs its
own internal joint state; Camber makes an independent geometry/mate copy for it.

An assembly can contain a subassembly created by another `Part`. The child
keeps its own solver and coordinate lattice; the parent occurrence supplies
only the rigid placement, and parent sections/interference checks convert exact
mesh coordinates as needed.

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
assembly = part.assembly(name="components")
for index, solid in enumerate(solids):
    assembly.add_part(solid, position=(0, 0, 3 * index))
```

Assembly mutation occurs sequentially. Independent parts can use different
working boxes and tolerances; assembly placement preserves their exact mesh
coordinates and face provenance without requiring `copy_solid`. That operation
remains useful when geometry itself needs to enter another Part's feature or CSG
history.

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

Coplanar fusion keeps one face. Its current name joins the creation roots, for
example `left-ExtrudeTop&right-ExtrudeTop`. Each original name remains an alias
of that same face, so a later feature can still select it. Passing two aliases
selects the face once. A later cut that leaves an alias on more than one region
retires it: datum lookup then names the current faces instead of guessing.

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
