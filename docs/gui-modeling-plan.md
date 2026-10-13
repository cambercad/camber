# GUI modeling mode

## Goal and boundary

Add a separate, code-first modeling mode for parts. A user can create and edit a feature tree through a familiar CAD interface, and the saved document remains ordinary, readable Python. The existing Python-driven `Part.show()` viewer stays available. Both modes use the same renderer, sketch tools, picking, and reusable GUI controls.

The Python file is the only editable model definition. Kernel geometry and `Part.operations` are results of running it, not a second feature database. The GUI owns only blocks it can parse and regenerate exactly. Other valid Python remains executable and visible as code, without a misleading parameter form.

## Document format

Modeling-mode files define `build_model()` and return the Part plus the final displayed solid. Generated features have stable IDs and explicit result names:

```python
from camber import Part


def build_model():
    # camber:begin setup
    part = Part((-50, -50, -50), (50, 50, 50), tolerance=0.01)
    # camber:end setup

    # camber:begin profile
    profile = part.sketch("xy", name="profile")
    profile.add_rectangle((-10, -5), (10, 5))
    # camber:end profile

    # camber:begin pad
    pad = profile.extrude(8, name="pad")
    # camber:end pad

    return part, pad
```

Markers are Python comments, not start/end API calls. They have no runtime cost or hidden semantics. `tokenize` finds their exact source spans; `ast.parse` validates Python and recognizes supported statement patterns. `ast.literal_eval` reads literal parameters. `inspect.signature` supplies parameter names and defaults. Never use `eval` to populate a form, and never reformat the whole file to change one feature: regenerate only the selected GUI-owned block.

Stable feature IDs identify tree rows independently of Python variable and mesh names. The generated code also supplies explicit sketch, curve, and solid names where later features depend on them. Duplicate IDs, nesting, mismatched markers, unsupported statements, or ambiguous references make a block code-only until corrected; they must not trigger a best-guess rewrite.

The parser recognizes assignments and calls in supported blocks, including simple references to previously defined results. Literal numbers, strings, booleans, tuples, and explicitly recognized enum values get form controls. An expression such as `wall * 2` is displayed as source and stays code-only for that argument until a deliberate expression editor exists. Comments or hand edits inside a generated block must not be silently lost during regeneration.

## Workspace and interaction

The layout uses the existing docking system:

- **Top:** compact New/Open/Save, Undo/Redo, rebuild status, and modeling commands grouped by Sketch, Create, Modify, and Inspect.
- **Left:** a feature tree with origin planes, sketches, solids, and operations in source order. Indentation shows dependencies; selection highlights the corresponding geometry. Rows distinguish valid, failed, and code-only features.
- **Center:** the current 3D viewport, with the existing selection and camera behavior. Sketch editing enters the established sketch workspace and returns an updated sketch block.
- **Right:** a property panel for the selected feature or document setup. Inputs identify model units or angles, have clear labels, concise tooltips, validation, and explicit Apply/Cancel. Face, edge, plane, sketch, and solid references can be picked from the viewport or chosen by name.
- **Bottom:** concise diagnostics and a collapsible Python preview for the selected block. A rebuild error identifies the feature and source line; it does not replace the last valid display.

Use the existing light/dark theme and reusable controls rather than introducing another GUI toolkit. Match conventional CAD interaction: select a feature to inspect it, edit its parameters in place, preview a changed value when inexpensive, commit with Apply, and keep a visible undo path. The feature tree should have a calm neutral surface, one clear blue selection state, restrained separators, and readable row height. The property panel should use aligned labels, consistent input baselines, grouped parameters, and a fixed Apply/Cancel footer. Avoid a dense grid of unlabeled icon buttons. Keyboard focus, tab order, scaling, high-DPI text, hover states, and scrollbars are part of the initial quality bar.

## Editing and rebuild rules

1. A GUI command creates a typed feature description, generates one Python block, and inserts it after its dependencies.
2. Before editing, validate the current source and the selected block against its supported AST pattern. If it no longer matches, preserve it and switch to code-only mode.
3. Edits change a temporary source buffer. Parse the whole candidate file and validate feature IDs, names, references, argument types, and dependency order before running it.
4. Run `build_model()` to create a fresh Part. Rebuild from the start because the native feature ledger is append-only and has no general rollback API. Swap the displayed scene only after the candidate succeeds; otherwise keep the previous model and show the error.
5. Commit the source buffer only after a successful Apply. Keep source revisions for Undo/Redo; save to disk atomically on explicit Save. Do not rebuild for every keystroke.
6. Deleting or moving a feature is allowed only if its dependents remain valid. Initially refuse deletion with dependents and list them; do not silently remove downstream work.

Generated modeling files should keep imports and side effects outside `build_model()` minimal. Each Apply reruns that function, including any user-authored code inside it. The GUI must make the rebuild boundary explicit and never rerun it merely because a field has focus or a key is pressed.

The source-defined tree includes sketches and other non-solid steps that `Part.operations` does not record. The existing operation ledger is useful for cross-checking successful runtime results, but it cannot recreate Python arguments or code. `Part.solid(name)` and the existing viewer scene replacement can show a selected solid result. An unresolved topology name after rebuilding is a feature error with its source location, not a reason to guess a new face.

## Forms and supported operations

Do not generate every editor blindly from annotations. Signatures provide basic fields and defaults; a small operation-specific definition supplies units, pick mode, validation, and code generation. A plain numeric field works for extrusion height. Loft sections, face selections, frames, and sketch geometry need specialized interactions.

Deliver coverage in useful slices:

1. **Vertical slice:** create/open/save a modeling file; edit Part bounds and tolerance; create a sketch with lines, arcs, circles, rectangles, and existing constraints; extrude or revolve it; edit a literal dimension; rebuild, undo, and recover from an error.
2. **Core solids:** primitives, Boolean union/subtract/intersect, fillet/chamfer, shell, hole, transforms, and named reference picking.
3. **Advanced modeling:** multi-profile loft, sweep, surfaces, trim, patterns, and the remaining common part operations. Each becomes GUI-editable only when its selection and parameter semantics are specified and tested.

Assembly authoring is a later, separate slice. The initial mode must still display an assembly created by Python, but it should not claim to edit constraints that it cannot round-trip.

## Code organization

Start with `camber/modeling.py` as the workflow controller and entry point, separate from the existing `Part.show()` viewer. Keep the general control library in `camber/gui.py` free of Camber-specific concepts. Put the marker/AST logic in a small `camber/feature_source.py` once it is large enough to merit separation. The form definitions belong with the modeling mode, not in the kernel or the generic GUI. Reuse the current sketch editor and viewer instead of duplicating them. Adapt the sketcher's current clipboard-based Finish path so modeling mode receives the generated block directly; leave clipboard behavior intact in the Python-driven viewer.

The public entry point should be explicit, such as `camber.modeling.open(path)`. It should accept a file path, not pretend that an arbitrary live Part can be reverse-engineered into editable source.

No additional pip dependency is needed for parsing or forms: use the Python standard library and the current pyglet GUI.

## Implementation sequence

1. **Visual shell:** build the docked tree, viewport, properties, and diagnostics with representative real controls. Review light/dark screenshots, typography, spacing, scrollbars, focus, and high-DPI behavior before wiring many commands.
2. **Source engine:** implement marker discovery, AST validation, stable IDs, dependency extraction, single-block replacement, and byte-preserving round trips for unrelated code. Test malformed and custom blocks before connecting it to the UI.
3. **Usable modeling path:** connect document setup, sketch editing, extrude/revolve, rebuild, save, Undo/Redo, and selected-result display. Convert sketch Finish from clipboard-only output into a reusable code result while retaining its existing viewer behavior.
4. **Common feature coverage:** add forms and picking adapters for primitives, Booleans, edge treatments, shell, holes, and transforms. Treat every new adapter as a round-trip and rebuild test case.
5. **Advanced operations and performance:** add loft/sweep/surface workflows; profile rebuild times on a representative model. If a rebuild stalls the viewport, move execution to a worker process with explicit result transfer rather than threading native state through the render loop.

## Verification and completion criteria

- Round-trip generated files through create, save, reopen, edit, and rebuild without changing unrelated source bytes.
- Reject malformed markers and unsupported Python patterns without losing source or replacing the last good model.
- Verify dependency blocking for delete/reorder and stable explicit names across repeated rebuilds.
- Test sketches and representative solid operations against the real native API, including failed operations and invalid topology references.
- Exercise keyboard, mouse picking, undo/redo, focus, scrollbars, and docking in both themes; inspect screenshots at normal and high DPI.
- Measure rebuild and GUI frame times on a representative multi-feature part. Keep rendering responsive during ordinary editing, and introduce a worker process only if measured rebuild behavior requires it.

The first release is complete when a user can build a small part entirely through the GUI, save understandable Python, reopen it, edit an earlier sketch or extrusion, see downstream features rebuild correctly, and recover cleanly from a bad edit. Arbitrary Python round-tripping and incremental kernel recomputation are not prerequisites.
