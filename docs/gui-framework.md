# Reusing the GUI

`gui.py` provides pyglet controls, docking and batched rendering without CAD
dependencies. Window chrome and optional GUI automation are separate modules.
Application code owns its model, validation and commands.

Prefer scoped panels and styles for new code:

```python
ui.new_frame()
with ui.style(variables={ui.ImGuiStyleVar_WindowPadding: (12, 8)}):
    with ui.panel("Properties") as visible:
        if visible:
            with ui.child("Inputs", height=240):
                changed, name = ui.InputText("Name", name)
            if ui.Button("Apply", enabled=inputs_valid):
                apply_changes()
ui.render()
```

Scopes restore state on exceptions. Manual Begin/End and Push/Pop remain
available, but mismatched children, overlapping panels, underflow and leaked
styles raise clear errors. A zero-count Pop is a no-op.

Use `SetSelectedTab`, `GetItemRect` and `GetItemSpacing` for selection and layout;
do not modify the framework's panel or tab dictionaries. Coordinates are logical
pixels, top-down. Input and rendering run on the window's thread.

## Undo and redo

`history.py` is an independent standard-library utility shared by document and
sketch editing. Record successful transactions with `history.record(before,
after)`. A no-op retains redo; a new edit clears redo. Mutable snapshots are
copied, and exposed stacks are read-only copies.

```python
def restore(snapshot):
    candidate = try_build(snapshot)
    if candidate is None:
        return False  # Keep the existing model.
    install(candidate)
    return True

history.step("undo", current_snapshot, restore)
```

A restore must explicitly return True or False. History advances only on True;
exceptions and False leave both stacks unchanged. The application callback must
retain or restore its live state when it fails. Restoring into a disposable model
before installing it is preferred. Reentrant history changes are rejected.

Use `can_undo` and `can_redo` for command availability. Local text editing keeps
its bounded caret/selection history; document history should not intercept its
shortcuts. Sketch workflows additionally check whether an unfinished action
must be cancelled before changing geometry. Toolbar, ribbon and keyboard commands
use that same availability policy.
