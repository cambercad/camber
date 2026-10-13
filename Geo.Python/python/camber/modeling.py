"""Source-backed part modeling workspace. Open with ``camber.modeling.open(path)``."""

import ast
import copy
from contextvars import ContextVar
import inspect
import keyword
import math
import os
from pathlib import Path
import sys
import tempfile
import textwrap

from .feature_source import ModelDocument, _call
from .modeling_commands import CONSTRUCTORS, annotation_names, operations, is_angle_parameter
from .modeling_references import is_reference, selection_expression


_TEMPLATE = '''from camber import Part, Frame, Curve3D, LoftOptions


def build_model():
    # camber:begin setup
    part = Part((-50, -50, -50), (50, 50, 50), tolerance=0.01)
    # camber:end setup

    return part, part
'''

_ASSEMBLY_TEMPLATE = _TEMPLATE.replace("    return part, part", """    # camber:begin assembly
    assembly = part.assembly(name="assembly")
    # camber:end assembly

    return part, assembly""")
_MODEL_PATHS = ContextVar("camber_model_paths", default=())
_LOFT_FIELD_HELP = {
    "guides": "Choose curves passing through matching contour joints of every profile.",
    "start_point": "Close the solid at a tip beyond the first profile.",
    "end_point": "Close the solid at a tip beyond the last profile.",
    "start_tangent": "World-space direction along the section order. Vector length controls the bend.",
    "end_tangent": "World-space direction along the section order. Vector length controls the bend.",
    "start_closure": "Rounded joins smoothly to the profile and closes with a rounded tip.",
    "end_closure": "Rounded joins smoothly to the profile and closes with a rounded tip.",
}


def load_component(path: str) -> "Solid | Assembly":
    """Load the saved Solid or Assembly displayed by a Python modeling document.

    Relative paths resolve beside the document containing this call. Standalone
    calls rebuild from saved source. Modeling sessions reuse eligible solid
    definitions until their saved source or component dependencies change.
    """
    from .api import Solid, Assembly
    paths = _MODEL_PATHS.get()
    target = Path(path).expanduser()
    if not target.is_absolute() and paths:
        target = paths[-1].parent / target
    target = target.resolve()
    if target in paths:
        raise ValueError("Circular component reference: " + target.name)
    source = target.read_text(encoding="utf-8")
    document = ModelDocument(source)
    def build():
        _, result, _, _ = _run_model(document, target)
        if not isinstance(result, (Solid, Assembly)):
            raise ValueError(target.name + " must display a finished solid or assembly")
        return result
    from .modeling_components import component_cache
    cache = component_cache()
    return cache.load(target,source,document,build) if cache is not None else build()


_FIELD_LABELS = {
    "low": "Lower bound · model units", "high": "Upper bound · model units",
    "tolerance": "Tessellation tolerance", "height": "Height · model units",
    "angle": "Angle · degrees", "radians": "Angle · degrees", "taper_angle": "Taper angle · degrees",
    "included_angle": "Included angle · degrees", "twist": "Twist · degrees per unit length",
    "twist_center": "Twist center · sketch plane", "radius": "Radius · model units",
    "pose_or_min": "Minimum corner", "extents_or_max": "Maximum corner",
    "a": "First solid", "b": "Second solid", "name": "Result name",
    "plane": "Sketch plane", "frame": "Sketch frame", "edges": "Edges",
    "faces": "Faces", "sketches": "Profiles",
    "mouth": "Hole placement", "count": "Instance count", "step": "Translation per instance",
    "start_point": "Start tip", "end_point": "End tip",
    "start_tangent": "Start tangent · direction and strength",
    "end_tangent": "End tangent · direction and strength",
}


def _assignment(block):
    statements = ast.parse(textwrap.dedent(block.source)).body
    return _call(statements[0]) if statements else None


def _parameter_form(block):
    parsed = _assignment(block)
    operation = operations().get(block.operation)
    if parsed is None or operation is None:
        return []
    _, _, method, call = parsed
    parameters = operation.parameters
    fields = operation.defaults()
    def display(node):
        return node.value if isinstance(node, ast.Constant) and isinstance(node.value, str) else ast.unparse(node)
    for parameter, node in zip(parameters, call.args):
        fields[parameter.name] = display(node) if operation.is_text(parameter) or operation.choices(parameter) else ast.unparse(node)
    for kw in call.keywords:
        parameter = next(p for p in parameters if p.name == kw.arg)
        fields[kw.arg] = display(kw.value) if operation.is_text(parameter) or operation.choices(parameter) else ast.unparse(kw.value)
    return list(fields.items())


def _parameter_source(block, values, available=(), *, receiver=None):
    parsed = _assignment(block)
    if parsed is None:
        raise ValueError("Feature has no editable call")
    result, original_receiver, method, call = parsed
    operation = operations().get(block.operation)
    if operation is None:
        raise ValueError("Operation has no parameter editor")
    first = operation.source(original_receiver if receiver is None else receiver, result, values, available)
    assignment = ast.parse(textwrap.dedent(block.source)).body[0]
    rest = block.source.splitlines(keepends=True)[assignment.end_lineno:]
    return first + "".join(rest)


def _pack_result(part, result):
    from . import glview
    from .display import DisplayScene
    scene = DisplayScene()
    if isinstance(result, (tuple, list)):
        for item in result:
            scene.extend(_result_scene(part, item))
    else:
        scene = _result_scene(part, result)
    return glview.pack_scene(scene)


def _body_items(value):
    from .api import Solid, Surface
    if isinstance(value, (Solid, Surface)):
        return [value]
    if isinstance(value, (tuple, list)) and value and all(isinstance(item, (Solid, Surface)) for item in value):
        return list(value)
    return []


def _visible_body_entries(session):
    """Solids and surfaces that are still separate bodies, in feature order.

    A body drops out once a later solid or surface feature uses it, so a fillet
    replaces its input instead of drawing both on top of each other.
    """
    from .api import Assembly
    document = session.document
    if document is None or isinstance(session.displayed, Assembly):
        return []
    consumed = set()
    for block in document.blocks:
        operation = operations().get(block.operation)
        if operation is None or not operation.result_types & {"Solid", "Surface"}:
            continue
        consumed.update(block.dependencies)
    entries = []
    seen = set()
    for block in document.blocks:
        if not block.result or block.result in consumed:
            continue
        items = _body_items(session.values.get(block.result))
        names = [block.result] if len(items) == 1 else [f"{block.result}[{index}]" for index in range(len(items))]
        for name, item in zip(names, items):
            if name in consumed or id(item) in seen:
                continue
            seen.add(id(item))
            entries.append((name, item))
    return entries


def _prefix_scene(scene, owner):
    prefix = owner + ":"
    for collection in (scene.patches, scene.curves, scene.points):
        for item in collection:
            name = item.get("name")
            if name and not str(name).startswith(prefix):
                item["name"] = prefix + str(name)
            color = item.get("color_name")
            if color and not str(color).startswith(prefix):
                item["color_name"] = prefix + str(color)


def _combined_body_scene(session):
    """Draw every separate body when the returned result is only one of them."""
    entries = _visible_body_entries(session)
    displayed = {id(item) for item in _body_items(session.displayed)}
    if len(entries) <= 1 or {id(item) for _, item in entries} == displayed:
        return None
    from .display import DisplayScene
    scene = DisplayScene()
    owners = {}
    for name, body in entries:
        piece = _result_scene(session.part, body)
        _prefix_scene(piece, name)
        for collection in (piece.patches, piece.curves, piece.points):
            for item in collection:
                item_name = item.get("name")
                if item_name:
                    owners[item_name] = name
        scene.extend(piece)
    return scene, owners


def _result_scene(part, result):
    from . import glview
    from .api import Curve3D, Frame, LoftOptions, ProjectedSketch, Sketch, SketchCurve
    from .display import DisplayScene
    from .vec import _xyz
    from .api import Assembly
    if isinstance(result, Assembly):
        from .modeling_assembly import assembly_scene
        return assembly_scene(result)
    if result is None or isinstance(result, (Frame, LoftOptions, ProjectedSketch, SketchCurve)):
        return glview._as_scene(part)
    if isinstance(result, Sketch):
        from . import pick
        scene = DisplayScene()
        actions = result._solved_actions()
        for item in pick.catalog_from_sketch(actions, result.frame, 0, 0, result.name):
            if item["kind"] == pick.KIND_CURVE:
                scene.curves.append(item)
            else:
                scene.points.append({"name": item["name"], "position": item["world"]})
        if not actions:
            from .vec import _xy
            frame = result.frame
            for points in result.polylines():
                positions = [_xyz(frame.origin + frame.x*_xy(p)[0] + frame.y*_xy(p)[1]) for p in points]
                scene.curves.append({"name": result.name, "points": positions, "closed": False})
        return scene
    if isinstance(result, Curve3D):
        scene = DisplayScene()
        points = [_xyz(p) for p in result.tessellate()]
        scene.curves.append({"name": result.name, "points": points,
                             "closed": bool(points and points[0] == points[-1])})
        return scene
    return glview._as_scene(result)


def _run_model(document, path, *, before=None):
    """Build a fresh model. A failed build cannot mutate the displayed Part."""
    namespace = {"__name__": "__camber_model__", "__file__": str(path)}
    tree = copy.deepcopy(document.tree)
    function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "build_model")
    if before is not None:
        function.body = [n for n in function.body if n.end_lineno < before]
        function.body.append(ast.Return(value=ast.Tuple(elts=[ast.Name(id="part",ctx=ast.Load()),
                                                            ast.Name(id="part",ctx=ast.Load())],ctx=ast.Load())))
    names = [b.result for b in document.blocks if b.result]
    class CaptureReturns(ast.NodeTransformer):
        def visit_FunctionDef(self, node):
            return node

        visit_AsyncFunctionDef = visit_FunctionDef

        def visit_Return(self, node):
            node.value = ast.Tuple(elts=[node.value or ast.Constant(None), ast.Call(
                func=ast.Name(id="__camber_model_locals", ctx=ast.Load()), args=[], keywords=[])], ctx=ast.Load())
            return node

    capture = CaptureReturns()
    function.body = [capture.visit(statement) for statement in function.body]
    ast.fix_missing_locations(tree)
    parent = str(path.parent)
    token = _MODEL_PATHS.set((*_MODEL_PATHS.get(), Path(path).resolve()))
    sys.path.insert(0, parent)
    try:
        exec(compile(tree, str(path), "exec"), namespace)
        namespace["__camber_model_locals"] = locals
        result, local_values = namespace["build_model"]()
        values = {name: local_values[name] for name in names if name in local_values}
    finally:
        sys.path.remove(parent)
        _MODEL_PATHS.reset(token)
    if not isinstance(result, tuple) or len(result) != 2:
        raise TypeError("build_model() must return (part, displayed_result)")
    from .api import Part
    part, displayed = result
    if not isinstance(part, Part):
        raise TypeError("build_model() must return a Part first")
    from .api import Assembly
    if isinstance(displayed, Assembly):
        report = displayed.solve()
        if not report.converged or report.unsatisfied:
            labels = ", ".join(m.label for m in report.unsatisfied)
            raise ValueError("Assembly mates cannot be satisfied" + (": " + labels if labels else ": " + report.message))
    packed = _pack_result(part, displayed)
    return part, displayed, packed, values


def _sketch_for_edit(document, block, path):
    """Evaluate only the prefix through this sketch in a disposable Part."""
    tree = copy.deepcopy(document.tree)
    function = next(n for n in tree.body if isinstance(n, ast.FunctionDef)
                    and n.name == "build_model")
    function.body = [n for n in function.body if n.end_lineno < block.end]
    for statement in function.body:
        if (isinstance(statement, ast.Assign) and isinstance(statement.targets[0], ast.Name)
                and statement.targets[0].id == block.result and isinstance(statement.value, ast.Call)):
            call = statement.value
            if not isinstance(call.func, ast.Attribute) or call.func.attr != "sketch":
                raise ValueError("This derived sketch must be edited through its generating feature")
            call.keywords = [kw for kw in call.keywords if kw.arg != "constrained"]
            call.keywords.append(ast.keyword(arg="constrained", value=ast.Constant(True)))
    function.body.append(ast.Return(value=ast.Tuple(
        elts=[ast.Name(id="part", ctx=ast.Load()), ast.Name(id=block.result, ctx=ast.Load())],
        ctx=ast.Load())))
    ast.fix_missing_locations(tree)
    namespace = {"__name__": "__camber_sketch_edit__", "__file__": str(path)}
    parent = str(path.parent)
    sys.path.insert(0, parent)
    try:
        exec(compile(tree, str(path), "exec"), namespace)
        part, sketch = namespace["build_model"]()
        from .sketch_source import read_sketch_actions
        return part, sketch, read_sketch_actions(block.source, block.result, sketch)
    finally:
        sys.path.remove(parent)


def _feature_groups(document):
    """Show each consumed profile beneath its first solid or surface feature."""
    children, consumed = {}, set()
    for block in document.blocks:
        operation = operations().get(block.operation)
        if operation is None or not operation.result_types & {"Solid", "Surface"}:
            continue
        profiles = [prior for prior in document.blocks if prior.start < block.start
                    and prior.kind == "Sketch" and prior.result in block.dependencies
                    and prior.id not in consumed]
        if profiles:
            children[block.id] = profiles
            consumed.update(p.id for p in profiles)
    for assembly in document.blocks:
        if assembly.operation != "Part.assembly":
            continue
        members = [b for b in document.blocks if b.start > assembly.start
                   and b.operation.startswith("Assembly.") and _assignment(b)[1] == assembly.result]
        if members:
            children[assembly.id] = members
            consumed.update(b.id for b in members)
    return [(block, children.get(block.id, [])) for block in document.blocks if block.id not in consumed]


def _feature_icon(block):
    from . import modeling_icons as icons, sketch_icons
    if block.kind == "Sketch":
        return sketch_icons.draw_rectangle
    method = block.operation.rsplit(".", 1)[-1]
    return {"extrude": icons.extrude, "revolve": icons.revolve, "loft": icons.loft,
            "cylinder": icons.cylinder, "cuboid": icons.block, "sphere": icons.sphere,
            "union": icons.boolean, "subtract": icons.cut, "intersect": icons.common,
            "fillet": icons.fillet, "chamfer": icons.chamfer, "shell": icons.shell,
            "hole": icons.hole, "counterbore_hole": icons.counterbore,
            "countersink_hole": icons.countersink,"assembly":icons.assembly,
            "fix": icons.fix,"coincident":icons.coincident,"concentric":icons.concentric,
            "distance":icons.distance,"parallel":icons.parallel,
            "perpendicular":icons.perpendicular,"angle":icons.angle,
            "pattern_linear":icons.pattern,"pattern_circular":icons.circular_pattern,
            "add_subassembly":icons.assembly}.get(method, icons.block)


class ModelSession:
    """Document revisions and successful build, independent of window widgets."""

    def __init__(self, path):
        from .modeling_components import ComponentCache
        self.component_cache = ComponentCache()
        self.path = Path(path).resolve()
        self.source = self.path.read_text(encoding="utf-8") if self.path.exists() else _TEMPLATE
        try:
            self.document = ModelDocument(self.source)
            parse_error = ""
        except (SyntaxError, ValueError) as exc:
            self.document = None
            parse_error = str(exc)
        self.saved_source = self.source if self.path.exists() else None
        self._initial_source = self.source
        self.undo_stack = []
        self.redo_stack = []
        self.part = self.displayed = self.packed = None
        self.values = {}
        self.error = parse_error
        self.failed_line = None
        self.selected = (self.document.blocks[0].id
                         if self.document is not None and self.document.blocks else None)

    @property
    def dirty(self):
        return self.source != (self.saved_source if self.saved_source is not None else self._initial_source)

    def build(self, document, *, before=None, prune=False):
        with self.component_cache.scope(prune=prune):
            return _run_model(document,self.path,before=before)

    def rebuild(self, source=None, *, record=True):
        source = self.source if source is None else source
        try:
            document = ModelDocument(source)
            part, displayed, packed, values = self.build(document,prune=True)
            types = {name: type(value).__name__ for name, value in values.items()}
            for name, value in values.items():
                if isinstance(value, (list, tuple)):
                    types.update({f"{name}[{i}]": type(item).__name__ for i, item in enumerate(value)})
            document = ModelDocument(source, types=types)
        except Exception as exc:
            self.error = str(exc) or type(exc).__name__
            self.failed_line = None
            trace = exc.__traceback__
            while trace is not None:
                if trace.tb_frame.f_code.co_filename == str(self.path):
                    self.failed_line = trace.tb_lineno
                trace = trace.tb_next
            return False
        if source != self.source and record:
            self.undo_stack.append(self.source)
            self.redo_stack.clear()
        self.source, self.document = source, document
        self.part, self.displayed, self.packed = part, displayed, packed
        self.values = values
        self.error = ""
        self.failed_line = None
        return True

    def undo(self):
        if not self.undo_stack:
            return False
        previous = self.undo_stack[-1]
        current = self.source
        if not self.rebuild(previous, record=False):
            return False
        self.undo_stack.pop()
        self.redo_stack.append(current)
        return True

    def redo(self):
        if not self.redo_stack:
            return False
        candidate = self.redo_stack[-1]
        current = self.source
        if not self.rebuild(candidate, record=False):
            return False
        self.redo_stack.pop()
        self.undo_stack.append(current)
        return True

    def save(self):
        if self.path.exists():
            if self.saved_source is None:
                raise FileExistsError("Model file was created elsewhere; choose another path")
            if self.path.read_text(encoding="utf-8") != self.saved_source:
                raise OSError("Model file changed on disk; reopen it before saving")
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd, temporary = tempfile.mkstemp(prefix="." + self.path.name + ".", suffix=".tmp",
                                         dir=self.path.parent)
        try:
            with os.fdopen(fd, "w", encoding="utf-8", newline="") as stream:
                stream.write(self.source)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.path)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
        self.saved_source = self.source


class _Workspace:
    def __init__(self, session):
        from .host import Viewer
        self.model = session
        self.viewer = Viewer("Modeling (experimental) — " + session.path.name, size=(1920,1080), borderless=True)
        self.ui = self.viewer._ui
        from .gui_window import WindowChrome
        self.window_chrome = WindowChrome(self.viewer.window, self.ui, panel=None, height=40)
        self.ui.dock_panel("Status", "bottom", size=28, header=False)
        self.ui.dock_panel("Document", "top", size=40, header=False)
        self.ui.dock_panel("Model", "top", size=124, header=False)
        self.ui.dock_panel("Features", "left", size=240)
        self.ui.dock_panel("Properties", "right", size=360)
        self.ui.dock_panel("Output", "bottom", size=190)
        self.ui.set_dock_visible("Output", session.document is None)
        self.editor = ""
        self.source_editor = session.source
        self.file_path = str(session.path)
        self.file_action = None
        self.draft_for = None
        self.fields = {}
        self.frame_fields = {}
        self.vector_fields = {}
        self.angle_fields = {}
        self.form_build_error = ""
        self.pending = None
        self.receiver = ""
        self.result_name = ""
        self.operation_filter = ""
        self.operation_choice = "Part.sketch"
        self.picked = []
        self.status = ""
        self.feature_filter = ""
        self._feature_menu_id = None
        self._draft_baseline = None
        self._pending_action = None
        self._shortcut_action = None
        self._sketch_panels = {}
        self._output_tab = None
        self._field_errors = {}
        self._pick_reference = None
        self._closing = False
        self._queued_action = None
        self._sketch_close_requested = False
        self._sketch_next_operation = None
        self._assembly_references = None
        self._assembly_context_cache = {}
        self._assembly_context_revision = None
        self._tree_inspection = None
        self._return_paths = []
        self._pick_datum = None
        self._prior_values = None
        self._ribbon_tab = "Assembly" if type(session.displayed).__name__ == "Assembly" else "Solid"
        self.sketch_session = None
        self.viewer.allow_viewport_menu = lambda: self.sketch_session is None
        self.sketch_block = None
        self.sketch_transparency = None
        self._sketch_camera = None
        self._sketch_previous = None
        self._fit_key_down = False
        self._body_pick_owners = {}
        self._draft_preview = False
        self._preview_selection = None
        self._sync_scene(fit=True)
        self.viewer.set_user_callback(self.tick)
        self.viewer.window.push_handlers(on_key_press=self._on_key_press, on_close=self._on_close)

    def _draft_state(self):
        return self.receiver, dict(self.fields), self.result_name if self.pending else ""

    def _draft_dirty(self):
        return self._source_dirty() or ((self.pending is not None or self.draft_for is not None)
                and self._draft_baseline is not None and self._draft_state() != self._draft_baseline)

    def _source_dirty(self):
        if self.pending or self.draft_for is None or self.model.document is None:
            return False
        return self.editor != self.model.document.block(self.draft_for).source.rstrip("\r\n")

    def _request_action(self, kind, value=None):
        if kind == "resolve":
            self._resolve_action(value)
            return
        if kind == "select" and value == self.model.selected and self.pending is None:
            self.ui.set_dock_visible("Properties", True)
            return
        if self._draft_dirty() or kind in ("open", "new", "new_assembly", "close", "open_component", "return_parent") and self.model.dirty:
            self._pending_action = (kind,value)
            self._action_dialog(open=True)
        else:
            self._run_action(kind,value)

    def _queue_action(self, kind, value=None):
        # Read all field editors before deciding whether this action loses edits.
        self._queued_action = (kind,value)

    def _run_action(self, kind, value=None):
        if kind == "select":
            self._select(value)
        elif kind == "edit_sketch":
            self._select(value)
            self._start_sketch(self.model.document.block(value))
        elif kind == "inspect_component":
            self._select(self.model.selected)
            self._tree_inspection = value
            context = self._assembly_context(self._available_values())
            address, constraint_index = value
            path = (context.components[address].path if address in context.components else context.containers[address][0])
            if constraint_index is not None:
                constraint = context.constraints[address][constraint_index]
                paths = [path+"/"+datum.path.split("/", 1)[1] for datum in (constraint.first, constraint.second) if datum]
            else:
                paths = [path]
            self.picked = [name for name in self.model.packed["entity_types"]
                           if any(name.startswith(p+":") or name.startswith(p+"/") for p in paths)]
            self.viewer.highlight_solid(self.picked)
        elif kind in ("open_component", "return_parent"):
            target = Path(value) if kind == "open_component" else self._return_paths[-1]
            previous = self.model.path
            self.file_path = str(target)
            self._switch_file(new=False, discard=True)
            if self.model.path == target:
                if kind == "open_component":
                    self._return_paths.append(previous)
                else:
                    self._return_paths.pop()
        elif kind == "operation":
            self._sketch_next_operation = None
            self._choose_operation(value)
        elif kind in ("undo", "redo"):
            self._history(kind)
        elif kind == "save":
            self._save_model()
        elif kind == "save_as":
            self._save_as_dialog()
        elif kind == "new_assembly":
            self._file_dialog(new=True,discard=True,assembly=True)
        elif kind == "insert_component":
            self._insert_component(value) if value else self._component_dialog()
        elif kind == "ribbon":
            self.ui.set_dock_visible("Model",not self.ui.docks.visible.get("Model",True))
        elif kind in ("open", "new"):
            self._file_dialog(new=kind == "new", discard=True)
        elif kind == "close":
            # A button may request this during drawing. Close after the frame.
            if not self._closing:
                self._closing = True
                self.viewer._pyglet.clock.schedule_once(lambda dt:self.viewer.close(),0)

    def _save_model(self):
        try:
            self.model.save()
            self.status = "Saved " + self.model.path.name
            return True
        except (OSError, ValueError) as exc:
            self.status = str(exc)
            self.form_build_error = str(exc)
            return False

    def _action_dialog(self, *, open=False):
        if self._pending_action is None:
            return
        kind = self._pending_action[0]
        saving = kind in ("open", "new", "new_assembly", "close", "save", "save_as", "open_component", "return_parent")
        title = "Unsaved changes" if saving else "Unapplied changes"
        suffix = " and close" if kind == "close" else " and continue"
        buttons = (("Save" if saving else "Apply")+suffix, "Discard"+suffix, "Keep editing")
        message = ("Save your changes before closing this model?" if kind == "close" else
                   "Save your changes before continuing?" if saving else
                   "Apply your changes before switching?")
        if self.form_build_error:
            message += "\n\n"+self.form_build_error
        if open:
            self.ui.OpenMessageBox(title, message, buttons, default=buttons[0], cancel="Keep editing")
        choice = self.ui.MessageBox(title)
        if choice is not None:
            self._queue_action("resolve", dict(zip(buttons, ("apply", "discard", "stay")))[choice])

    def _resolve_action(self, choice):
        action = self._pending_action
        if choice == "stay":
            self._pending_action = None
            return
        if choice == "apply":
            if self._draft_dirty() and not self._commit_edits(start_sketch=False):
                self._action_dialog(open=True)
                return
            if action[0] in ("open", "new", "new_assembly", "close", "open_component", "return_parent") and not self._save_model():
                self._action_dialog(open=True)
                return
        self._pending_action = None
        self.pending, self.draft_for, self._draft_baseline = None, None, None
        self._pick_reference = None
        self._run_action(*action)

    def _on_close(self):
        from pyglet.event import EVENT_HANDLED
        if self.sketch_session is not None:
            if not self._sketch_close_requested:
                self.ui.OpenMessageBox("Close sketch?",
                    "Finish the current sketch to keep its changes, or discard it before closing.",
                    ("Keep editing", "Discard and close"), default="Keep editing", cancel="Keep editing")
            self._sketch_close_requested = True
        else:
            self._queue_action("close")
        return EVENT_HANDLED

    def _on_key_press(self, symbol, modifiers):
        from pyglet.window import key
        from pyglet.event import EVENT_HANDLED
        if symbol == key.ESCAPE and (self._pick_reference is not None or self._pick_datum is not None):
            self._pick_reference = None
            self._pick_datum = None
            self.status = "Face selection canceled"
            return EVENT_HANDLED
        if self.sketch_session is not None or not modifiers & key.MOD_CTRL:
            return
        action = {key.S:"save", key.O:"open", key.N:"new", key.F1:"ribbon"}.get(symbol)
        if symbol == key.S and modifiers & key.MOD_SHIFT:
            action = "save_as"
        if symbol in (key.Z,key.Y) and not self.ui.text_input_focused:
            action = "redo" if symbol == key.Y or modifiers & key.MOD_SHIFT else "undo"
        if symbol == key.ENTER:
            action = "apply"
        if action:
            self._shortcut_action = action
            return EVENT_HANDLED

    def _commit_edits(self, *, start_sketch=True):
        if self.pending:
            return self._new_feature(start_sketch=start_sketch)
        if self.model.document is None:
            return False
        block = self.model.document.block(self.model.selected)
        if self._source_dirty():
            if self._draft_baseline is not None and self._draft_state() != self._draft_baseline:
                self.form_build_error = "Both parameters and Python have edits. Cancel one editor's changes before applying the other."
                return False
            try:
                return self._apply(self.model.document.replace(block.id,self.editor,require_editable=False))
            except (ValueError,SyntaxError) as exc:
                self.form_build_error = str(exc)
                return False
        operation = operations().get(block.operation)
        if not block.editable or operation is None:
            return False
        available = self._available_values(block.start)
        errors = self._form_errors(operation,available,block.result or block.id)
        if errors:
            self.status = next(iter(errors.values()))
            return False
        try:
            return self._apply(self._candidate_source())
        except (ValueError,SyntaxError) as exc:
            self.status = str(exc)
            return False

    def _start_sketch(self, block):
        from .sketch_ui import begin_sketch_session
        try:
            part, sketch, actions = _sketch_for_edit(self.model.document, block, self.model.path)
            call = _assignment(block)[3]
            plane_node = call.args[0] if call.args else next((k.value for k in call.keywords if k.arg == "plane"), ast.Constant(value="xy"))
            plane = ast.literal_eval(plane_node)
            custom_frame = any(k.arg == "frame" and not (isinstance(k.value,ast.Constant) and k.value.value is None)
                               for k in call.keywords)
            self.sketch_transparency = (self.viewer._solid_transparent, self.viewer._transparency)
            self._sketch_camera = copy.copy(self.viewer.camera)
            self.viewer.set_solid(_pack_result(part, part))
            self.viewer.set_solid_transparent(True)
            self.sketch_session = begin_sketch_session(
                self.viewer, self.ui, part, plane=plane, name=block.result,
                  existing_sketch=sketch, emit_frame=custom_frame, initial_actions=actions,
                  register_context=False, panel_title="Properties")
            self.sketch_block = block.id
            self._ribbon_tab = "Sketch"
            self._sketch_panels = {title:self.ui.docks.visible.get(title,True)
                                   for title in ("Document","Status","Model","Features","Properties","Output")}
            for title in self._sketch_panels:
                self.ui.set_dock_visible(title, title != "Output")
        except Exception as exc:
            self.status = str(exc)
            self.sketch_session = None
            self.sketch_block = None
            if self.sketch_transparency is not None:
                self.viewer.set_solid_transparent(self.sketch_transparency[0])
                self.viewer.set_transparency_mode(self.sketch_transparency[1])
                self.sketch_transparency = None
            self._restore_sketch_view()
            self._sync_scene()

    def _restore_sketch_view(self):
        if self._sketch_camera is not None:
            for field in ("eye", "center", "up", "zoom", "ortho", "near", "far"):
                setattr(self.viewer.camera, field, getattr(self._sketch_camera, field))
            self._sketch_camera = None

    def _finish_sketch(self):
        from .sketch_ui import finalize_sketch_session, sketch_session_code
        session = self.sketch_session
        identifier = self.sketch_block
        block = self.model.document.block(identifier)
        next_operation = self._sketch_next_operation
        previous = self.model
        target = self._sketch_previous or previous
        applied = session["state"].get("done") == "finish"
        if applied:
            try:
                code = sketch_session_code(session, part_var="part", sketch_var=block.result)
                # Preserve named placement rather than replacing it with coordinates.
                creation = ast.parse(textwrap.dedent(block.source)).body[0]
                creation.value.keywords = [kw for kw in creation.value.keywords if kw.arg != "constrained"]
                creation.value.keywords.append(ast.keyword(arg="constrained", value=ast.Constant(True)))
                # Keep emitted curve metadata and comments when restoring placement.
                emitted_creation = ast.parse(code).body[0]
                code = ast.unparse(creation) + "\n" + "".join(code.splitlines(keepends=True)[emitted_creation.end_lineno:])
                candidate = self.model.document.replace(identifier, textwrap.indent(code, "    "))
                self.model = target
                if not self._apply(candidate):
                    raise ValueError(self.form_build_error)
            except (ValueError, SyntaxError) as exc:
                self.model = previous
                session["state"]["done"] = None
                session["state"]["status"] = "Cannot finish sketch: "+str(exc)
                self.form_build_error = str(exc)
                return False
        else:
            self.model = target
        finalize_sketch_session(self.viewer, session, part_var="part", sketch_var=block.result, copy=False)
        self.sketch_session = None
        self.sketch_block = None
        self._sketch_previous = None
        self._sketch_next_operation = None
        self._ribbon_tab = "Solid"
        for title, visible in self._sketch_panels.items():
            self.ui.set_dock_visible(title, visible)
        self.viewer.set_solid_transparent(self.sketch_transparency[0])
        self.viewer.set_transparency_mode(self.sketch_transparency[1])
        self.sketch_transparency = None
        self._restore_sketch_view()
        self._sync_scene()
        self._select(identifier if applied else self.model.selected)
        if applied and next_operation:
            self._choose_operation(next_operation)
        return True

    def _sync_scene(self, fit=False):
        from . import glview
        self._draft_preview = False
        self._preview_selection = None
        combined = _combined_body_scene(self.model)
        if combined is not None:
            scene, self._body_pick_owners = combined
            packed = glview.pack_scene(scene)
        else:
            self._body_pick_owners = {}
            packed = self.model.packed
        if packed is not None:
            self.viewer.set_solid(packed)
            if fit:
                self.viewer.camera.fit(*packed["bounds"])
        else:
            self.viewer.set_solid(None)

    def _switch_file(self, *, new, discard=False, assembly=False):
        if self.model.dirty and not discard:
            self.status = "Save the current model before switching files"
            return
        path = Path(self.file_path).expanduser().resolve()
        if path.suffix.lower() != ".py":
            self.status = "Choose a .py model file"
            return
        if new and path.exists():
            self.status = "File already exists; choose another path"
            return
        if not new and not path.exists():
            self.status = "File does not exist"
            return
        try:
            session = ModelSession(path)
            if not session.rebuild(_ASSEMBLY_TEMPLATE if new and assembly else None):
                self.status = session.error
                return
            if new:
                session._initial_source = session.source
        except (OSError, UnicodeError) as exc:
            self.status = str(exc)
            return
        self.model = session
        self._sketch_next_operation = None
        self._ribbon_tab = "Assembly" if type(session.displayed).__name__ == "Assembly" else "Solid"
        self.pending = None
        self.source_editor = session.source
        self.draft_for = None
        self.picked.clear()
        self._pick_reference = None
        self._pick_datum = None
        self._assembly_references = None
        self._prior_values = None
        self.viewer.window.set_caption("Modeling (experimental) — " + path.name)
        self.file_action = None
        self._sync_scene(fit=True)
        self.status = "New model" if new else "Opened model"
        if session.document is not None:
            displayed = next((b for b in session.document.blocks
                              if b.result and session.values.get(b.result) is session.displayed), None)
            if displayed is not None:
                self._select(displayed.id)

    def _apply(self, candidate):
        if self.model.rebuild(candidate):
            self.form_build_error = ""
            self._sync_scene()
            self.picked.clear()
            self._assembly_references = None
            self._prior_values = None
            self._pick_datum = None
            self.status = "Model rebuilt"
            self.draft_for = None
            self.source_editor = self.model.source
            if self.model.selected not in {b.id for b in self.model.document.blocks} and self.model.document.blocks:
                self.model.selected = self.model.document.blocks[0].id
            return True
        else:
            self.form_build_error = self.model.error
            self.status = "Build failed; previous model retained"
            return False

    def _select(self, identifier):
        self._tree_inspection = None
        self._sketch_next_operation = None
        self._pick_reference = None
        self._pick_datum = None
        self.form_build_error = ""
        self.model.selected = identifier
        self.draft_for = None
        self._draft_baseline = None
        self.pending = None
        self.file_action = None
        self.ui.set_dock_visible("Properties", True)
        if self.model.document is not None:
            block = self.model.document.block(identifier)
            from .api import Assembly, Solid, Surface
            selected = []
            if isinstance(self.model.displayed, Assembly):
                context = self._assembly_context(self._available_values())
                if context:
                    expressions = {ast.unparse(n) for n in ast.walk(ast.parse(textwrap.dedent(block.source)))
                                   if isinstance(n, (ast.Name, ast.Subscript, ast.Call))}
                    for (name, _), expression in context.entities.items():
                        owner = ast.unparse(ast.parse(expression, mode="eval").body.func.value)
                        produced = block.result and (owner == block.result or owner.startswith(block.result+"[") or owner.startswith(block.result+"."))
                        if expression in expressions or produced or owner in expressions and block.operation == "Assembly.fix":
                            selected.append(name)
            else:
                value = self.model.values.get(block.result)
                if isinstance(value, (Solid, Surface)):
                    shown = self.viewer._solid["packed"] if self.viewer._solid is not None else self.model.packed
                    entity_types = (shown or {}).get("entity_types") or {}
                    prefixed = [name for name in entity_types if name.startswith(block.result + ":")]
                    if prefixed:
                        selected = prefixed
                    else:
                        local = {name.rsplit(":", 1)[-1] for name in value.patch_names}
                        selected = [name for name in entity_types if name.rsplit(":", 1)[-1] in local]
            self.picked = []
            self.viewer.highlight_solid(list(dict.fromkeys(selected)))

    def _preview(self, block):
        result = self.model.values[block.result] if block.result else self.model.values[_assignment(block)[1]]
        self._body_pick_owners = {}
        self.viewer.set_solid(_pack_result(self.model.part, result))
        self.status = "Previewing " + block.id

    def _available_values(self, before=None):
        from .api import Assembly
        result = {}
        if self.model.document is None:
            return result
        if before is not None and self.model.document.blocks and before <= self.model.document.blocks[0].start:
            return result
        values = self.model.values
        if before is not None and any(isinstance(v,Assembly) for v in values.values()):
            key = (self.model.part,self.model.source,before)
            if self._prior_values is None or self._prior_values[0] != key:
                self._prior_values = (key,self.model.build(self.model.document,before=before)[3])
            values = self._prior_values[1]
        for block in self.model.document.blocks:
            if before is not None and block.start >= before:
                break
            if block.result not in values:
                continue
            value = values[block.result]
            result[block.result] = value
            if isinstance(value, (tuple, list)):
                result.update({f"{block.result}[{i}]": item for i, item in enumerate(value)})
        assembly = result.get(self.receiver)
        if isinstance(assembly,Assembly):
            context = self._assembly_context(result)
            result.update({name:c.part for name,c in context.components.items()})
        return result

    def _assembly_context(self, available):
        from .api import Assembly, AssemblyPart, AssemblyOccurrence
        from .modeling_assembly import AssemblyReferences
        name = self.receiver if isinstance(available.get(self.receiver),Assembly) else next(
            (name for name,value in available.items() if value is self.model.displayed and isinstance(value,Assembly)),None)
        if name is None:
            name = next((name for name,value in reversed(tuple(available.items())) if isinstance(value,Assembly)),None)
        if name is None:
            return None
        value = available[name]
        revision = (self.model.part, self.model.path, self.model.source)
        if self._assembly_context_revision != revision:
            self._assembly_context_cache.clear()
            self._assembly_context_revision = revision
        key = (id(value), name)
        cached = self._assembly_context_cache.get(key)
        if cached is not None:
            self._assembly_references = cached
            return cached[2]
        if self._assembly_references is None or self._assembly_references[:2] != (value,name):
            sources, counts = {}, {"parts":0,"subassemblies":0}
            for block in self.model.document.blocks:
                parsed = _assignment(block)
                if parsed is None or parsed[1] != name or block.result not in available:
                    continue
                result = available[block.result]
                items = list(enumerate(result))[1:] if isinstance(result,list) else [(None,result)]
                for index,item in items:
                    kind = "parts" if isinstance(item,AssemblyPart) else "subassemblies" if isinstance(item,AssemblyOccurrence) else None
                    if kind:
                        sources[name+f".{kind}[{counts[kind]}]"] = block.result+(f"[{index}]" if index is not None else "")
                        counts[kind] += 1
            self._assembly_references = (value,name,AssemblyReferences(value,name,sources))
            self._assembly_context_cache[key] = self._assembly_references
        return self._assembly_references[2]

    def _choose_operation(self, key):
        from .api import Assembly, Sketch, Solid, Surface
        self._tree_inspection = None
        self._pick_reference = None
        self._pick_datum = None
        self.form_build_error = ""
        self.frame_fields.clear()
        self.vector_fields.clear()
        self.angle_fields.clear()
        if self.model.document is None:
            self.status = "Repair the Python source before adding a feature"
            return
        operation = operations()[key]
        self.file_action = None
        stem = operation.method or operation.owner.__name__.lower()
        ids = {b.id for b in self.model.document.blocks}
        index = 1
        while stem + str(index) in ids:
            index += 1
        self.result_name = stem + str(index)
        self.pending = operation
        self.fields = operation.defaults()
        available = self._available_values()
        receivers = [name for name, value in available.items() if isinstance(value, operation.owner)]
        if operation.owner is Sketch and not receivers:
            self._sketch_next_operation = key
            self._choose_operation("Part.sketch")
            self.status = "Choose a plane and create the sketch; finish it to continue with " + operation.label.lower()
            return
        selected = next((b.result for b in self.model.document.blocks if b.id == self.model.selected), None)
        displayed = next((name for name in receivers if available[name] is self.model.displayed), None)
        self.receiver = (displayed if operation.owner is Assembly and displayed else
                         selected if selected in receivers else receivers[-1] if receivers else "")
        if "name" in self.fields:
            self.fields["name"] = self.result_name
        if key == "Part.sketch":
            self.fields["constrained"] = "True"
        if key in ("Sketch.revolve", "Sketch.revolve_surface"):
            self.fields["angle"] = "6.283185307179586"
        self._draft_baseline = self._draft_state()
        self.status = "Set the parameters, then Apply to create " + operation.label.lower()
        if key in ("Part.subtract", "Part.union", "Part.intersect") and len(_visible_body_entries(self.model)) > 1:
            self.status = "Click the base body, then the other body. Or choose both in the form."
        if key == "Part.sketch":
            self.status = "Choose XY, XZ or YZ, or select a planar face; then Create sketch"
            if any(isinstance(value, (Solid, Surface)) for value in available.values()):
                self._pick_reference = (key, "plane", "plane")
                if self.picked:
                    self._reference_from_selection("plane", "plane", available)
        if operation.owner.__name__ == "Assembly":
            if "opposite_normals" in self.fields:
                self.fields["opposite_normals"] = "True"
            self._draft_baseline = self._draft_state()
            datums = self._datum_fields(operation)
            if datums:
                self._pick_datum = (key, datums[0])
                self.status = "Select "+self._datum_caption(datums[0])+" in the viewport · Esc to stop picking"

    @staticmethod
    def _datum_fields(operation):
        return [p.name for p in operation.parameters
                if any(name.startswith("Assembly") and name.endswith("Datum")
                       for name in annotation_names(p.annotation))]

    @staticmethod
    def _datum_caption(label):
        return {"a": "the first reference", "b": "the second reference"}.get(label, "the "+label.replace("_", " "))

    def _accept_datum_pick(self, name, available):
        key, label = self._pick_datum
        operation = operations()[key]
        context = self._assembly_context(available)
        parameter = next(p for p in operation.parameters if p.name == label)
        names = annotation_names(parameter.annotation)
        kinds = tuple(kind for kind in ("plane", "axis", "point") if "Assembly"+kind.capitalize()+"Datum" in names)
        expression = context.from_pick(name, kinds) if context else None
        if not expression:
            self.status = "Select a compatible "+" or ".join(kinds)+" reference"
            return False
        fields = self._datum_fields(operation)
        for other in fields:
            value = self.fields[other]
            if other != label and value:
                try:
                    call = ast.parse(value, mode="eval").body
                    selected = ast.parse(expression, mode="eval").body
                    if isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute) and ast.unparse(call.func.value) == ast.unparse(selected.func.value):
                        self.status = "Select a reference on another component"
                        return False
                except SyntaxError:
                    pass
        self.fields[label] = expression
        remaining = [field for field in fields[fields.index(label)+1:]
                     if not self.fields[field] or self.fields[field] == "None"]
        self._pick_datum = (key, remaining[0]) if remaining else None
        self.status = ("Select "+self._datum_caption(remaining[0])+" in the viewport" if remaining else
                       "References selected · Preview or Create mate")
        return True

    def _body_reference_fields(self, operation):
        fields = []
        for parameter in operation.parameters:
            names = annotation_names(parameter.annotation)
            if names & {"Sequence", "Iterable", "list", "tuple", "Point3Like", "Frame", "Sketch"}:
                continue
            if names & {"Solid", "Surface"}:
                fields.append(parameter.name)
        return fields

    def _assign_clicked_body(self, owner):
        """Fill the next empty solid input from a click on that body."""
        if self._pick_reference or self._pick_datum:
            return False
        operation = self.pending
        if operation is None:
            block = next((item for item in self.model.document.blocks if item.id == self.model.selected), None)
            operation = operations().get(block.operation) if block is not None else None
        if operation is None:
            return False
        fields = self._body_reference_fields(operation)
        if not fields:
            return False
        if owner in {(self.fields.get(name) or "").strip() for name in fields}:
            return False
        for name in fields:
            current = (self.fields.get(name) or "").strip()
            if current in ("", "None"):
                self.fields[name] = owner
                remaining = [other for other in fields[fields.index(name) + 1:]
                             if (self.fields.get(other) or "").strip() in ("", "None")]
                role = {"a": "base", "b": "body to remove"}.get(name, name.replace("_", " "))
                self.status = owner + " set as the " + role
                if remaining:
                    self.status += " · click the next body"
                return True
        return False

    def _candidate_source(self):
        from .api import Solid
        doc = self.model.document
        block = None if self.pending else doc.block(self.model.selected)
        operation = self.pending or operations()[block.operation]
        name = self.result_name if self.pending else block.result or block.id
        available = self._available_values(None if self.pending else block.start)
        errors = self._form_errors(operation,available,name)
        if errors:
            raise ValueError(next(iter(errors.values())))
        body = operation.source(self.receiver,name,self.fields,available) if self.pending else _parameter_source(block,self.fields,available,receiver=self.receiver)
        candidate = doc.insert(name,body) if self.pending else doc.replace(block.id,body)
        imports = {n.id for n in ast.walk(ast.parse(textwrap.dedent(body)))
                   if isinstance(n,ast.Name) and n.id in CONSTRUCTORS}
        if operation.call_name:
            imports.add(operation.call_name)
        elif operation.static:
            imports.add(operation.owner.__name__)
        candidate = ModelDocument(candidate).with_imports(imports)
        if self.pending:
            if operation.owner.__name__ == "Assembly":
                candidate = ModelDocument(candidate).display_result(self.receiver)
            elif (operation.result_types & {"Solid","Surface","Sketch","Curve3D","ProjectedSketch","Assembly"}
                  and not (operation.key == "Part.section_sketch" and isinstance(self.model.displayed, Solid))):
                candidate = ModelDocument(candidate).display_result(name)
        return candidate

    def _selection_signature(self):
        return (self.receiver, self.fields.get("edges"), self.fields.get("faces"))

    def _input_entity_names(self, body, kind):
        if body is None:
            return set()
        return set(body.patch_names if kind == "faces" else getattr(body, "curve_names", ()) or ())

    def _resolve_input_entity(self, name, body, kind):
        """Map a name on the previewed result back to the solid being edited."""
        names = self._input_entity_names(body, kind)
        if not isinstance(name, str) or name in names or not names:
            return name
        local = name.split(":", 1)[-1]
        prefixed = (self.receiver + ":" + local) if self.receiver else local
        if prefixed in names:
            return prefixed
        if kind == "edges" and local.startswith("BlendEdge_"):
            edge = (self.receiver + ":" if self.receiver else "") + local[len("BlendEdge_"):]
            if edge in names:
                return edge
        for candidate in names:
            if candidate.split(":", 1)[-1] == local:
                return candidate
        return name

    def _remove_entity(self, label, name):
        try:
            current = ast.literal_eval(self.fields.get(label) or "()")
        except (ValueError, SyntaxError):
            return False
        current = (current,) if isinstance(current, str) else tuple(current or ())
        if name not in current:
            return False
        remaining = tuple(item for item in current if item != name)
        self.fields[label] = repr(remaining) if remaining else "()"
        return True

    def _apply_entity_pick(self, name, available, ray=None):
        """Add or remove one face or edge. Preview names map back to the input solid."""
        _, label, kind = self._pick_reference
        if kind not in ("faces", "edges"):
            return self._reference_from_selection(label, kind, available, ray=ray)
        body = available.get(self.receiver)
        resolved = self._resolve_input_entity(name, body, kind)
        if resolved not in self._input_entity_names(body, kind):
            self.status = "Select " + kind + " on the input solid"
            return False
        visible = list(self.picked)
        try:
            current = ast.literal_eval(self.fields.get(label) or "()")
        except (ValueError, SyntaxError):
            current = ()
        current = (current,) if isinstance(current, str) else tuple(current or ())
        if resolved in current and self._pick_reference[0] == "Solid.shell":
            self.status = "Already selected: " + resolved
            changed = False
        elif resolved in current:
            self._remove_entity(label, resolved)
            self.status = "Removed " + resolved
            changed = True
        else:
            self.picked = [resolved]
            changed = self._reference_from_selection(label, kind, available, ray=ray)
            self.picked = visible or [name]
        if changed and self._draft_preview:
            self._preview_draft()
        return changed

    def _preview_draft(self):
        try:
            _, _, packed, _ = self.model.build(ModelDocument(self._candidate_source()))
            self._body_pick_owners = {}
            self.viewer.set_solid(packed)
            self.form_build_error = ""
            self._draft_preview = True
            self._preview_selection = self._selection_signature()
            self.status = "Preview · Apply to keep, Cancel to discard"
            return True
        except Exception as exc:
            self.form_build_error = str(exc)
            self.status = "Preview failed; previous model retained"
            self._sync_scene()
            return False

    def _new_feature(self, *, start_sketch=True):
        operation, name = self.pending, self.result_name
        try:
            candidate = self._candidate_source()
        except (ValueError, SyntaxError) as exc:
            self.status = str(exc)
            return
        previous = self.model
        previous_displayed = previous.displayed
        if start_sketch and operation.key == "Part.sketch":
            self.model = copy.copy(previous)
            self.model.undo_stack = previous.undo_stack.copy()
            self.model.redo_stack = previous.redo_stack.copy()
            self._sketch_previous = previous
        if self._apply(candidate):
            next_operation = self._sketch_next_operation
            self._select(name)
            from .api import Solid, Surface
            if isinstance(self.model.displayed, (Solid, Surface)) and not isinstance(previous_displayed, (Solid, Surface)):
                self._set_standard_view("Isometric")
                self._sync_scene(fit=True)
            if start_sketch and operation.key == "Part.sketch":
                self._sketch_next_operation = next_operation
                self._start_sketch(self.model.document.block(name))
                if self.sketch_session is None and self._sketch_previous is not None:
                    self.model = self._sketch_previous
                    self._sketch_previous = None
                    self._select(self.model.selected)
                    self._sync_scene()
                    return False
            if operation.key == "Part.assembly":
                self._ribbon_tab = "Assembly"
                self.status = "Assembly created · insert a saved part to begin"
            return True
        if self._sketch_previous is not None:
            self.model = self._sketch_previous
            self._sketch_previous = None
        return False

    def _feature_name(self, block):
        return block.id.replace("_", " ").capitalize()

    def _delete_feature(self, identifier):
        document = self.model.document
        block = document.block(identifier)
        if block.kind == "Setup":
            return
        try:
            candidate = document.delete(identifier)
            if block.result:
                remaining = [item for item in ModelDocument(candidate).blocks if item.result and item.kind != "Setup"]
                assemblies = [item for item in remaining if item.operation == "Part.assembly"]
                shown = (assemblies or remaining)[-1].result if remaining else "part"
                candidate = ModelDocument(candidate).display_result(shown)
            if self._apply(candidate):
                self._select(self.model.document.blocks[0].id)
        except ValueError as exc:
            self.status = str(exc)

    def _feature_context_menu(self):
        ui = self.ui
        identifier = self._feature_menu_id
        label = ""
        if identifier and self.model.document is not None:
            try:
                label = self._feature_name(self.model.document.block(identifier))
            except KeyError:
                self._feature_menu_id = None
        if ui.ContextMenu("Feature", [label] if label else ()) == (0,) and self._feature_menu_id:
            self._delete_feature(self._feature_menu_id)
            self._feature_menu_id = None

    def _feature_tree(self):
        ui = self.ui
        if not ui.Begin("Features"):
            ui.End()
            self._feature_context_menu()
            return
        ui.Heading(self.model.path.stem.replace("_", " ").capitalize(),16)
        from .api import Assembly
        assembly_mode = isinstance(self.model.displayed, Assembly)
        ui.TextMuted("Assembly" if assembly_mode else str(len(self.model.document.blocks) if self.model.document else 0)+" features")
        _, self.feature_filter = ui.InputText("Filter features",self.feature_filter,
                                             width=max(1,ui.GetContentRegionAvail()[0]),hint="Filter features")
        ui.Separator()
        def row(block, depth=0, children=()):
            label = self._feature_name(block)
            if not block.editable:
                label += "  [code]"
            clicked, expanded = ui.TreeNode(label + "##" + block.id,
                                            depth=depth, selected=self.model.selected == block.id and self.pending is None and self._tree_inspection is None,
                                            leaf=not children, drawer=_feature_icon(block),force_open=bool(self.feature_filter) or any(c.id == self.model.selected for c in children))
            if ui.IsItemHovered() and ui.IsMouseDoubleClicked(0) and block.kind == "Sketch":
                self._queue_action("edit_sketch", block.id)
            elif clicked or ui.IsItemHovered() and ui.IsMouseDoubleClicked(0):
                self._queue_action("select",block.id)
            if block.kind != "Setup" and ui.IsItemHovered() and ui.IsMouseClicked(1):
                self._feature_menu_id = block.id
                self._queue_action("select", block.id)
                ui.OpenContextMenu("Feature")
            if expanded:
                for child in children:
                    row(child, depth+1)
        context = self._assembly_context(self._available_values()) if assembly_mode else None
        blocks = {b.result: b for b in self.model.document.blocks if b.result} if self.model.document else {}
        def component(address, depth):
            block = blocks.get(address)
            if block:
                row(block, depth)
            else:
                item = context.components.get(address)
                label = (item.path if item else context.containers[address][0]).rsplit("/", 1)[-1]
                from . import modeling_icons as icons
                origin = blocks.get(address.split("[", 1)[0])
                if item and origin and origin.operation in ("Assembly.pattern_linear", "Assembly.pattern_circular"):
                    seed = dict(_parameter_form(origin)).get("seed", "Component")
                    number = int(address.rsplit("[", 1)[1][:-1])+1
                    label = seed.replace("_", " ").capitalize()+" · "+str(number)
                clicked, _ = ui.TreeNode(label+"##"+address, depth=depth, leaf=True,
                                        selected=self._tree_inspection == (address, None),
                                        drawer=icons.block if item else icons.assembly)
                if clicked:
                    self._queue_action("inspect_component", (address, None))
            if address in context.containers:
                _, expanded = ui.TreeNode("Components##"+address, depth=depth+1)
                if expanded:
                    for child in context.children[address]:
                        component(child, depth+2)
                _, expanded = ui.TreeNode("Constraints##"+address, depth=depth+1)
                if expanded:
                    for index, constraint in enumerate(context.constraints[address]):
                        clicked, _ = ui.TreeNode(constraint.label+"##"+address+str(index), depth=depth+2, leaf=True,
                                                selected=self._tree_inspection == (address,index))
                        if clicked:
                            self._queue_action("inspect_component", (address,index))
        for block, children in _feature_groups(self.model.document) if self.model.document is not None else ():
            query = self.feature_filter.casefold()
            if query and query not in (block.id+" "+block.kind).replace("_"," ").casefold():
                children = [child for child in children if query in (child.id+" "+child.kind).replace("_"," ").casefold()]
                if not children:
                    continue
            if assembly_mode and block.operation == "Part.assembly":
                row(block)
                for title, members in (("Components", [b for b in children if not operations()[b.operation].mutation]),
                                       ("Constraints", [b for b in children if operations()[b.operation].mutation])):
                    _, expanded = ui.TreeNode(title + "##" + block.id, depth=1, default_open=True,
                                              force_open=any(b.id == self.model.selected for b in members))
                    if expanded:
                        if title == "Components" and context and block.result in context.children:
                            for address in context.children[block.result]:
                                if not query or query in address.casefold() or query in (context.components[address].path if address in context.components else context.containers[address][0]).casefold():
                                    component(address, 2)
                            for member in members:
                                if member.result not in context.components and member.result not in context.containers:
                                    row(member, depth=2)
                        else:
                            for member in members:
                                row(member, depth=2)
            else:
                row(block, children=children)
        if self.picked:
            ui.Separator()
            ui.TextWrapped("Picked: " + ", ".join(self.picked))
        ui.End()
        self._feature_context_menu()

    def _component_file(self, expression):
        root = expression.split(".", 1)[0].split("[", 1)[0]
        block = next((b for b in self.model.document.blocks if b.result == root), None)
        if block:
            if block.operation in ("Assembly.pattern_linear", "Assembly.pattern_circular", "Assembly.mirror"):
                seed = dict(_parameter_form(block)).get("seed", "")
                return self._component_file(seed)
            definition = next((b for b in self.model.document.blocks
                               if b.operation == "load_component" and b.result in block.dependencies), None)
            if definition:
                return (self.model.path.parent / dict(_parameter_form(definition))["path"]).resolve()
        return None

    def _form_errors(self, operation, available, result):
        errors = operation.field_errors(self.fields, available)
        from .modeling_fields import OrientationFields
        for parameter in operation.parameters:
            draft = self.vector_fields.get(parameter.name+"##"+operation.key)
            if isinstance(draft, OrientationFields) and draft.error():
                errors[parameter.name] = draft.error()
        if not result or not result.isidentifier() or keyword.iskeyword(result):
            errors["__result"] = "Use a Python name: letters, digits and underscores; no spaces"
        elif self.pending and any(b.result == result or b.id == result for b in self.model.document.blocks):
            errors["__result"] = "A feature with this name already exists"
        if not operation.static and not isinstance(available.get(self.receiver), operation.owner):
            errors["__receiver"] = "Select an input " + operation.owner.__name__.lower()
        if operation.owner.__name__ == "Assembly" and operation.method in ("coincident","parallel","perpendicular","distance"):
            try:
                a, b = (ast.parse(self.fields[name],mode="eval").body for name in ("a","b"))
                if isinstance(a,ast.Call) and isinstance(b,ast.Call) and isinstance(a.func,ast.Attribute) and isinstance(b.func,ast.Attribute):
                    if a.func.attr != b.func.attr:
                        errors["b"] = "Both references must have the same type"
                    elif ast.unparse(a.func.value) == ast.unparse(b.func.value):
                        errors["b"] = "Select a reference on another component"
                    if a.func.attr != "plane" and "opposite_normals" in self.fields:
                        self.fields["opposite_normals"] = "None"
            except (SyntaxError,KeyError):
                pass
        return errors

    def _frame_field(self, identity, value, available, optional):
        from .api import Frame
        from .modeling_fields import FrameFields
        ui = self.ui
        references = [name for name, item in available.items() if isinstance(item, Frame)]
        current = value if value in references or optional and value in ("", "None") else "Custom frame"
        options = ["Custom frame"] + references + (["None"] if optional else [])
        changed, selected = (False, "Custom frame") if len(options) == 1 else ui.ComboBox(
            identity+"/mode", current, options, width=max(1, ui.GetContentRegionAvail()[0]))
        if changed:
            value = "Frame()" if selected == "Custom frame" else selected
        if selected != "Custom frame":
            return value
        draft = self.frame_fields.get(identity)
        if draft is None or draft.source != value:
            try:
                draft = self.frame_fields[identity] = FrameFields(value or "Frame()", available)
            except (ValueError, SyntaxError, TypeError, NameError, AttributeError):
                # Preserve expressions that cannot be decomposed into coordinates.
                _, value = ui.InputText(identity, value, width=max(1, ui.GetContentRegionAvail()[0]),
                                        hint="Frame expression")
                return value
        value = draft.draw(ui, identity)
        return value

    def _field(self, operation, parameter, available):
        from . import api
        ui = self.ui
        label = parameter.name
        value = self.fields[label]
        caption = _FIELD_LABELS.get(label, label.replace("_", " ").capitalize())
        if is_angle_parameter(parameter) and "degrees" not in caption:
            caption += " · degrees"
        names = annotation_names(parameter.annotation)
        if any(name.startswith("Assembly") and name.endswith("Datum") for name in names):
            caption = {"a":"First reference","b":"Second reference"}.get(label,caption)
        identity = label + "##" + operation.key
        width = max(1, ui.GetContentRegionAvail()[0])
        choices = operation.choices(parameter)
        error = self._field_errors.get(label, "")
        if (operation.key == "Part.loft" and label in ("start_closure", "end_closure")
                and self.fields.get(label.replace("closure", "point"), "None") in ("", "None") and not error):
            return
        if label in ("faces", "edges") and "EntitySelection" in names:
            ui.TextUnformatted("Opening faces" if operation.key == "Solid.shell" else caption)
            try:
                selected = ast.literal_eval(value) if value else None
                selected = (selected,) if isinstance(selected, str) else tuple(selected or ())
                if not all(isinstance(n, str) for n in selected):
                    raise ValueError("Select named " + label)
            except (ValueError, SyntaxError, TypeError):
                _, self.fields[label] = ui.InputText(identity, value, width=width, error=error)
                return
            remaining = []
            for name in selected:
                _, checked = ui.Checkbox(name+"##"+identity, True)
                if checked:
                    remaining.append(name)
            self.fields[label] = repr(tuple(remaining)) if remaining else "None" if "None" in names else "()"
            if not selected:
                ui.TextMuted("No " + label + " selected. Pick them in the viewport.", wrap=True)
            selecting = self._pick_reference == (operation.key, label, label)
            if ui.Button(("Stop picking" if selecting else "Pick " + label)+"##"+identity, height=28,
                         enabled=isinstance(available.get(self.receiver), (api.Solid, api.Surface))):
                self._pick_reference = None if selecting else (operation.key, label, label)
                self.status = "Select " + label + " in the viewport · Esc to stop"
            if self.picked and ui.Button("Use selection##"+identity, height=26):
                self._reference_from_selection(label, label, available)
            if error:
                ui.ValidationMessage(error)
            return
        if names == {"bool"}:
            _, checked = ui.Checkbox(caption + "##" + operation.key, value == "True")
            value = str(checked)
        else:
            optional_vector = "Point3Like" in names and "None" in names and not names & {"Sequence", "Iterable", "list", "tuple", "Frame"}
            if not optional_vector:
                ui.TextUnformatted(caption)
            reference_types = tuple(getattr(api, name) for name in names
                                    if isinstance(getattr(api,name,None),type) and name != "EntityName")
            references = [name for name, item in available.items()
                          if reference_types and isinstance(item, reference_types)]
            sequence = bool(names & {"Sequence", "Iterable", "list", "tuple"})
            if reference_types and sequence:
                references += ["*" + name for name,item in available.items()
                               if isinstance(item,(list,tuple)) and item and
                               all(isinstance(member,reference_types) for member in item)]
            frame_input = "Frame" in names and ("Point3Like" not in names or (value.lstrip().startswith("Frame(") or ".face_frame(" in value) or isinstance(available.get(value),api.Frame))
            kinds = tuple(kind for kind,typ in (("plane",api.AssemblyPlaneDatum),
                          ("axis",api.AssemblyAxisDatum),("point",api.AssemblyPointDatum)) if typ.__name__ in names)
            if "Point3Like" in names and "None" in names and not sequence and not frame_input:
                toggle = ("Close " + label.split("_")[0] + " to a point" if operation.key == "Part.loft"
                          and label in ("start_point", "end_point") else "Specify " + caption.lower())
                _, enabled = ui.Checkbox(toggle+"##"+identity+"/enabled", value not in ("", "None"))
                if not enabled:
                    self.fields[label] = "None"
                    if operation.key == "Part.loft" and label in ("start_point", "end_point"):
                        self.fields[label.replace("point", "closure")] = "None"
                    return
                if value in ("", "None"):
                    seed = (0.0, 0.0, 0.0)
                    if operation.key == "Part.loft" and label in ("start_point", "end_point"):
                        from .modeling_commands import field_value
                        from .vec import _xyz
                        try:
                            sections = field_value(ast.parse(self.fields["sketches"], mode="eval").body, available)
                            section, neighbor = (sections[0], sections[1]) if label == "start_point" else (sections[-1], sections[-2])
                            origin, adjacent = _xyz(section.frame.origin), _xyz(neighbor.frame.origin)
                            seed = tuple(a+.25*(a-b) for a,b in zip(origin, adjacent))
                        except (ValueError, SyntaxError, KeyError, IndexError, AttributeError):
                            pass
                    value = repr(seed)
            if kinds:
                context = self._assembly_context(available)
                if context:
                    value = context.draw_datum(ui,identity,value,kinds,picked=self.picked)
                    selecting = self._pick_datum == (operation.key,label)
                    if ui.Button(("Stop picking" if selecting else "Pick in viewport")+"##"+identity,height=26):
                        self._pick_datum = None if selecting else (operation.key,label)
                        self.status = "Select " + caption.lower() + " in the viewport" if not selecting else "Picking stopped"
                else:
                    ui.ValidationMessage("Create an assembly and insert components first")
            elif label == "opposite_normals":
                labels = {"Opposed (faces meet)":"True","Aligned":"False","Automatic":"None"}
                current = next((k for k,v in labels.items() if v == value),"Automatic")
                changed,selected = ui.ComboBox(identity,current,tuple(labels),width=width)
                if changed:
                    value = labels[selected]
            elif is_reference(value) and (frame_input or "Point3Like" in names) and not sequence:
                reference_node = ast.parse(value, mode="eval").body
                if reference_node.func.attr == "to_global":
                    reference_node = reference_node.func.value
                if frame_input:
                    value = self._frame_field(identity, value, available, False)
                else:
                    ui.TextWrapped("Point: " + reference_node.args[0].value)
                if ui.Button("Use coordinates##"+identity,height=26):
                    from .modeling_commands import field_value
                    from .vec import _xyz
                    reference = field_value(ast.parse(value,mode="eval").body,available)
                    value = ("Frame(" + ", ".join(k+"="+repr(_xyz(getattr(reference,k)))
                             for k in ("origin","x","y","z")) + ")") if frame_input else repr(_xyz(reference))
            elif frame_input and not sequence:
                value = self._frame_field(identity, value, available, "None" in names)
            elif label == "orientation" and operation.key in ("Assembly.add_part", "Assembly.add_subassembly"):
                from .modeling_fields import OrientationFields
                draft = self.vector_fields.get(identity)
                if draft is None or draft.source != value:
                    try:
                        draft = self.vector_fields[identity] = OrientationFields(value, available)
                    except (ValueError, SyntaxError, TypeError, NameError):
                        draft = None
                if draft is not None:
                    value = draft.draw(ui, identity)
                else:
                    _, value = ui.InputText(identity, value, width=width, error=error)
            elif label == "twist_center":
                _, enabled = ui.Checkbox("Twist about a sketch point##"+identity+"/enabled", value not in ("", "None"))
                if not enabled:
                    value = "None"
                else:
                    if value in ("", "None"):
                        value = "(0, 0)"
                    _, value = ui.InputText(identity, value, width=width, hint="(x, y) on the sketch plane", error=error)
            elif sequence and names & {"Point2Like", "Point3Like"}:
                from .modeling_fields import PointListFields
                draft = self.vector_fields.get(identity)
                if draft is None or draft.source != value:
                    try:
                        draft = self.vector_fields[identity] = PointListFields(value, 3 if "Point3Like" in names else 2)
                    except (ValueError, SyntaxError, TypeError):
                        draft = None
                if draft is not None:
                    value = draft.draw(ui, identity, error)
                else:
                    _, value = ui.InputText(identity, value, width=width, error=error)
            elif "Point3Like" in names and not sequence:
                from .modeling_fields import VectorFields
                draft = self.vector_fields.get(identity)
                if draft is None or draft.source != value:
                    try:
                        draft = self.vector_fields[identity] = VectorFields(value,available)
                    except (ValueError,SyntaxError,TypeError,NameError,IndexError,AttributeError):
                        draft = None
                if draft is not None:
                    value = draft.draw(ui,identity,error)
                else:
                    _,value = ui.InputText(identity,value,width=width,error=error)
            elif is_angle_parameter(parameter) and value != "None":
                from .modeling_fields import AngleField
                draft = self.angle_fields.get(identity)
                if draft is None or draft.source != value:
                    try:
                        draft = self.angle_fields[identity] = AngleField(value, available)
                    except (ValueError, SyntaxError, TypeError, NameError):
                        draft = None
                if draft is not None:
                    value = draft.draw(ui, identity, error)
                else:
                    _, value = ui.InputText(identity, value, width=width, error=error)
            elif operation.key == "Part.sketch" and label == "plane":
                options = ["xy", "xz", "yz"]
                if value not in options:
                    options.append(value)
                _, value = ui.ComboBox(identity, value, options, width=width)
            elif operation.key == "Part.loft" and label in ("start_closure", "end_closure"):
                labels = {"Automatic (rounded)": "None", "Rounded": "'rounded'", "Sharp": "'sharp'"}
                current = next((caption for caption, source in labels.items() if value in (source, source.replace("'", '"'))), "Automatic (rounded)")
                changed, selected = ui.ComboBox(identity, current, tuple(labels), width=width)
                if changed:
                    value = labels[selected]
            elif choices:
                options = [str(v) for v in choices]
                if operation.key == "Part.sketch" and label == "plane" and value not in options:
                    options.append(value)
                if "None" in names:
                    options.append("None")
                _, value = ui.ComboBox(identity, value.strip("\"'"), options, width=width)
            elif operation.key == "Part.loft" and label == "first_curves":
                from .modeling_fields import loft_starting_curves
                from .modeling_commands import field_value
                try:
                    profiles = ast.parse(self.fields.get("sketches", "[]"), mode="eval").body
                    sections = field_value(profiles, available)
                    labels = [ast.unparse(item) for item in profiles.elts]
                    options = field_value(ast.parse(self.fields.get("options", "None"), mode="eval").body, available)
                    required = options is not None and options.correspondence == "vertices"
                    value = loft_starting_curves(ui, identity, value, list(zip(labels, sections)), required=required)
                except (ValueError, SyntaxError, TypeError, AttributeError, KeyError):
                    ui.ValidationMessage("Choose loft profiles before setting their starting curves")
            elif "LoftOptions" in names and (value in ("", "None") or value.lstrip().startswith("LoftOptions(")):
                from .modeling_fields import loft_settings
                try:
                    value = loft_settings(ui, identity, value)
                except (ValueError, SyntaxError, TypeError):
                    _, value = ui.InputText(identity, value, width=width, error=error)
            elif reference_types and sequence and names <= set(t.__name__ for t in reference_types) | {"None", "Sequence", "Iterable", "list", "tuple"}:
                from .modeling_fields import reference_sequence
                try:
                    value = reference_sequence(ui, identity, value, references, optional="None" in names)
                except (ValueError, SyntaxError):
                    _, value = ui.InputText(identity, value, width=width, error=error)
            elif references and not sequence and names <= set(t.__name__ for t in reference_types) | {"None"}:
                options = references + (["None"] if "None" in names else [])
                _, value = ui.ComboBox(identity, value, options, width=width)
            else:
                hint = "Required" if parameter.default is inspect.Parameter.empty else "Blank uses API default"
                _, value = ui.InputText(identity, value, width=width, hint=hint, error=error)
            if ui.IsItemHovered():
                ui.SetTooltip(_LOFT_FIELD_HELP[label] if operation.key == "Part.loft" and label in _LOFT_FIELD_HELP else
                              str(parameter.annotation) + (" · Python literal or feature reference" if not operation.is_text(parameter) else ""))
        self.fields[label] = value
        error = self._field_errors.get(label, "")
        if error:
            ui.ValidationMessage(error)
        if self.picked and label in ("edges", "faces", "patch_name", "curve", "curves", "point", "origin_anchor_name"):
            if ui.Button("Use selection##" + identity, height=26):
                selected = tuple(self.picked)
                self.fields[label] = repr(selected) if sequence or label in ("edges", "faces", "curves") else selected[0] if operation.is_text(parameter) else repr(selected[0])
        kind = ("plane" if operation.key == "Part.sketch" and label == "plane" else
                "frame" if "Frame" in names else "point" if "Point3Like" in names
                and label not in ("start_tangent", "end_tangent", "reference_direction") else None)
        if kind and not sequence:
            selecting = self._pick_reference == (operation.key,label,kind)
            caption = "Pick point" if kind == "point" else "Pick planar face"
            if ui.Button(("Stop picking" if selecting else caption)+"##"+identity,height=28):
                self._pick_reference = None if selecting else (operation.key,label,kind)
                self.status = (caption + " in the viewport - Esc to cancel") if not selecting else "Picking stopped"

    def _reference_from_selection(self, label, kind, available, ray=None):
        try:
            if kind in ("faces", "edges"):
                from .naming import parse_sketch_curve_address
                body = available.get(self.receiver)
                if body is None:
                    raise ValueError("Select an input solid first")
                names = self._input_entity_names(body, kind)
                picked = []
                for name in self.picked:
                    name = self._resolve_input_entity(name, body, kind)
                    address = parse_sketch_curve_address(name) if kind == "edges" else None
                    picked.append(address["curve"] if address and address["curve"] in names else name)
                if any(name not in names for name in picked):
                    raise ValueError("Select " + kind + " on the input solid")
                current = ast.literal_eval(self.fields[label]) if self.fields[label] else None
                current = (current,) if isinstance(current, str) else tuple(current or ())
                self.fields[label] = repr(tuple(dict.fromkeys((*current, *picked))))
                self.status = "Selected " + kind + " · pick more or create the feature"
                return True
            thread_pose = (label == "pose" and self._pick_reference is not None and self._pick_reference[0] in
                           {"Part.create_metric_thread_for_hole_negative", "Part.create_metric_thread_for_nut_negative"})
            expression = selection_expression(available,self.picked[-1],kind,inward=label == "mouth" or thread_pose,
                                              ray=ray if kind == "frame" else None)
            self.fields[label] = ast.literal_eval(expression) if kind == "plane" else expression
            if kind == "plane":
                self.fields["frame"] = "None"
            self._pick_reference = None
            self.status = "Reference selected: " + self.picked[-1]
            return True
        except Exception as exc:
            self.status = str(exc)
            return False

    def _properties(self):
        ui = self.ui
        ui.PushStyleVar(ui.ImGuiStyleVar_WindowPadding, (14, 12))
        visible = ui.Begin("Properties", flags=ui.ImGuiWindowFlags_NoScrollWithMouse)
        ui.PopStyleVar()
        if not visible:
            ui.End()
            return
        if self._tree_inspection is not None and self._pending_action is None:
            address, index = self._tree_inspection
            context = self._assembly_context(self._available_values())
            item = context.components.get(address)
            path = item.path if item else context.containers[address][0]
            ui.Heading(path.rsplit("/", 1)[-1], 16)
            if index is not None:
                constraint = context.constraints[address][index]
                ui.TextWrapped(constraint.label)
                if constraint.value is not None:
                    ui.TextUnformatted("Value: "+format(constraint.value, ".6g"))
                for datum in (constraint.first, constraint.second):
                    if datum:
                        ui.TextWrapped(datum.path)
            else:
                ui.TextMuted("Component" if item else "Subassembly")
            ui.Separator()
            ui.TextMuted("Defined in an inserted model. Open that model to edit its features or constraints.", wrap=True)
            source = self._component_file(address)
            if source and ui.Button("Open component model", primary=True):
                self._queue_action("open_component", source)
            ui.End()
            return
        if self.file_action:
            ui.TextUnformatted(self.file_action + " model")
            ui.Separator()
            ui.TextWrapped("Choose a Python model file. A new model is written only when you save it.")
            ui.Spacing(4)
            ui.TextUnformatted("File path")
            _, self.file_path = ui.InputText("File path", self.file_path,
                                             width=max(1, ui.GetContentRegionAvail()[0]))
            ui.Spacing(4)
            if ui.Button(self.file_action, width=100, height=30, primary=True):
                self._switch_file(new=self.file_action == "New")
            ui.SameLine()
            if ui.Button("Cancel", width=100, height=30):
                self.file_action = None
            ui.End()
            return
        if self.model.document is None:
            ui.TextWrapped("Repair the Python source in Output, then rebuild.")
            ui.End()
            return
        block = next((b for b in self.model.document.blocks if b.id == self.model.selected), None)
        if self.pending:
            block = None
        elif block is None:
            ui.Heading("Start modeling")
            ui.TextWrapped("Create a sketch, choose its plane, then draw your profile.")
            if ui.Button("Create sketch",height=32,primary=True):
                self._queue_action("operation","Part.sketch")
            ui.Spacing(8)
            ui.TextMuted("Or select an existing feature to edit it.",wrap=True)
            ui.End()
            return
        operation = self.pending or operations().get(block.operation)
        if block and self.draft_for != block.id:
            self.editor = block.source.rstrip("\r\n")
            self.fields = dict(_parameter_form(block)) if block.editable else {}
            self.receiver = _assignment(block)[1] if block.editable else ""
            self.draft_for = block.id
            self.frame_fields.clear()
            self.vector_fields.clear()
            self.angle_fields.clear()
            self._draft_baseline = self._draft_state()
        editable = self.pending is not None or block.editable
        ui.Heading("New " + operation.label.lower() if self.pending else block.id.replace("_"," ").capitalize())
        ui.TextMuted(operation.label if operation else block.kind)
        ui.Separator()
        if not self.pending and block.operation in ("Assembly.add_part", "Assembly.add_subassembly"):
            source = self._component_file(block.result)
            if source and ui.Button("Open component model", height=28):
                self._queue_action("open_component", source)
            ui.Separator()
        if not self.pending and block.kind == "Setup" and len(self.model.document.blocks) == 1:
            ui.TextMuted("Start with a sketch on a plane.")
            if ui.Button("Create sketch",height=32,primary=True):
                self._queue_action("operation","Part.sketch")
            ui.SameLine()
            if ui.Button("Create assembly",height=32):
                self._queue_action("operation","Part.assembly")
            ui.Separator()
        if not self.pending and block.kind == "Sketch":
            ui.TextMuted("Use this sketch")
            width = max(1, (ui.GetContentRegionAvail()[0]-16)/3)
            for index, (label, key) in enumerate((("Extrude", "Sketch.extrude"),
                                                 ("Revolve", "Sketch.revolve"),
                                                 ("Surface", "Sketch.surface"))):
                if index:
                    ui.SameLine()
                if ui.Button(label, width=width, height=28):
                    self._queue_action("operation",key)
            ui.Separator()
        available = self._available_values(None if self.pending else block.start)
        if not self.pending and block.operation == "Part.assembly":
            ui.TextMuted("Insert saved parts, then select references to mate them.",wrap=True)
            if ui.Button("Insert component",height=32,primary=True):
                self._queue_action("insert_component")
            ui.Separator()
        ui.BeginChild("Parameters", height=max(30, ui.GetContentRegionAvail()[1]-82))
        if editable and operation:
            form_errors = self._form_errors(operation, available, self.result_name if self.pending else block.result or block.id)
            self._field_errors = form_errors
            ui.TextMuted(operation.description,wrap=True)
            ui.Spacing(4)
            if not operation.static:
                receivers = [name for name, value in available.items() if isinstance(value, operation.owner)]
                ui.TextUnformatted("Input " + operation.owner.__name__.lower())
                if receivers:
                    _, self.receiver = ui.ComboBox("Receiver##" + operation.key, self.receiver,
                                                   receivers, width=max(1, ui.GetContentRegionAvail()[0]))
                else:
                    ui.TextWrapped("Create a " + operation.owner.__name__.lower() + " first.")
                ui.Spacing(2)
            if self.pending:
                ui.TextUnformatted("Mate name" if operation.mutation else "Feature name")
                changed, result_name = ui.InputText("Feature name", self.result_name,
                                                    width=max(1, ui.GetContentRegionAvail()[0]), error=form_errors.get("__result", ""))
                if changed and self.fields.get("name") == self.result_name:
                    self.fields["name"] = result_name
                self.result_name = result_name
                if "__result" in form_errors:
                    ui.ValidationMessage(form_errors["__result"])
                ui.Spacing(2)
            primary = [p for p in operation.parameters if p.default is inspect.Parameter.empty
                       or p.name != "max_deviation" and annotation_names(p.annotation)-{"None"} in ({"float"}, {"int"})
                       or operation.key == "Part.sketch" and p.name == "plane"
                       or operation.key == "Solid.shell" and p.name == "faces"
                       or operation.owner.__name__ == "Assembly" and p.name == "position"
                       or p.name == "opposite_normals"]
            for parameter in primary:
                self._field(operation, parameter, available)
            end_fields = [p for p in operation.parameters if operation.key == "Part.loft"
                          and p.name in ("start_point", "start_closure", "start_tangent",
                                         "end_point", "end_closure", "end_tangent")]
            if end_fields:
                _, expanded = ui.TreeNode("End conditions##Part.loft", default_open=False,
                                         force_open=any(p.name in form_errors for p in end_fields))
                if expanded:
                    for end in ("start", "end"):
                        ui.Heading(end.capitalize())
                        for parameter in sorted((p for p in end_fields if p.name.startswith(end)),
                                                key=lambda p: ("point", "closure", "tangent").index(p.name.split("_", 1)[1])):
                            self._field(operation, parameter, available)
            optional = [p for p in operation.parameters if p not in primary and p not in end_fields]
            if optional:
                _, expanded = ui.TreeNode("More options##" + operation.key, default_open=False,
                                         force_open=any(p.name in form_errors for p in optional))
                if expanded:
                    for parameter in optional:
                        self._field(operation, parameter, available)
        else:
            ui.TextWrapped(block.reason)
        if self.form_build_error:
            ui.ValidationMessage("Cannot apply: " + self.form_build_error)
        if block:
            users = self.model.document.dependents(block.id)
            ui.Spacing(4)
            if users:
                ui.TextMuted("Used by: " + ", ".join(users),wrap=True)
            elif block.kind != "Setup" and ui.Button("Delete feature"):
                self._delete_feature(block.id)
        ui.EndChild()
        ui.Separator()
        errors = self._form_errors(operation, available, self.result_name if self.pending else block.result or block.id) if editable and operation else {}
        if errors and self._pick_reference is None and self._pick_datum is None:
            self.status = "Check inputs: " + "; ".join(key.replace("__", "").replace("_", " ")+": "+message for key,message in errors.items())
        elif self.status.startswith("Check inputs:"):
            self.status = "Inputs valid"
        width = max(1, (ui.GetContentRegionAvail()[0]-8)/2)
        action_label = "Create mate" if self.pending and operation.mutation else "Create sketch" if self.pending and operation.key == "Part.sketch" else "Create feature" if self.pending else "Apply"
        if ui.Button(action_label, width=width, height=32, primary=True,
                     enabled=editable and not errors and (self.pending is not None or self._draft_dirty())):
            self._commit_edits()
        ui.SameLine()
        if ui.Button("Cancel", width=width, height=32):
            self._sketch_next_operation = None
            self.pending = None
            self.draft_for = None
            self._draft_baseline = None
            self._pending_action = None
            self._pick_reference = None
            self._pick_datum = None
            self.form_build_error = ""
            self.picked = []
            self._sync_scene()
            self.viewer.highlight_solid([])
            self.status = "Edit discarded"
        if self.pending:
            if ui.Button("Preview",width=width,height=30,enabled=not errors):
                self._preview_draft()
            ui.SameLine()
            if ui.Button("Show final",width=width,height=30):
                self._sync_scene()
        elif block:
            if block.kind == "Sketch" and ui.Button("Edit geometry", width=width, height=30, enabled=block.editable):
                self._queue_action("edit_sketch", block.id)
            elif block.kind != "Sketch" and ui.Button("Preview", width=width, height=30):
                if block.editable:
                    self._preview_draft()
                else:
                    try:
                        self._preview(block)
                    except Exception as exc:
                        self.status = str(exc)
            ui.SameLine()
            if ui.Button("Show final", width=width, height=30):
                self._sync_scene()
        if self._draft_preview and self._preview_selection != self._selection_signature():
            self._preview_draft()
        ui.End()

    def _diagnostics(self):
        ui = self.ui
        if not ui.Begin("Output"):
            ui.End()
            return
        if self._output_tab:
            ui._tabs[(id(ui._current),"Model output")] = self._output_tab
            self._output_tab = None
        ui.BeginTabBar("Model output")
        if ui.BeginTabItem("Messages"):
            if self.model.error:
                ui.TextWrapped("Build error" +
                               (f" at line {self.model.failed_line}" if self.model.failed_line else ""))
                ui.TextWrapped(self.model.error)
            else:
                ui.TextWrapped(self.status or "Model ready")
            ui.EndTabItem()
        if ui.BeginTabItem("Python"):
            if self.model.document is None:
                _, self.source_editor = ui.InputTextMultiline(
                    "Repair source", self.source_editor,
                    width=max(180, ui.GetContentRegionAvail()[0]), height=125)
                if ui.Button("Rebuild source", primary=True):
                    self._apply(self.source_editor)
            else:
                try:
                    block = self.model.document.block(self.model.selected)
                except KeyError:
                    ui.TextUnformatted("Select a feature to see its Python source.")
                else:
                    ui.TextUnformatted(block.id + "  ·  " + block.kind)
                    _, self.editor = ui.InputTextMultiline(
                        "Source##feature", self.editor,
                        width=max(180, ui.GetContentRegionAvail()[0]), height=115)
                    if ui.Button("Apply Python", primary=True):
                        self._commit_edits()
                    ui.SameLine()
                    if ui.Button("Cancel"):
                        self.editor = block.source.rstrip("\r\n")
            ui.EndTabItem()
        ui.EndTabBar()
        ui.End()

    def _history(self, direction):
        if getattr(self.model, direction)():
            self._tree_inspection = None
            self.pending = None
            self.draft_for = None
            self.source_editor = self.model.source
            if self.model.selected not in {b.id for b in self.model.document.blocks}:
                self.model.selected = self.model.document.blocks[0].id
            self.picked.clear()
            self._sync_scene()
            self.status = "Undo" if direction == "undo" else "Redo"

    def _document_bar(self):
        from . import modeling_icons as icons
        from . import sketch_icons
        ui = self.ui
        sketching = self.sketch_session is not None
        ui.PushStyleVar(ui.ImGuiStyleVar_WindowPadding,(12,4))
        visible = ui.Begin("Document",flags=ui.ImGuiWindowFlags_NoScrollWithMouse)
        ui.PopStyleVar()
        if visible:
            from .gui import MenuItem
            ui.TextEllipsis("CamberCAD",110,height=28)
            for action, drawer, tip, enabled in (
                    ("new",icons.file_new,"New model · Ctrl+N",True),
                    ("open",icons.file_open,"Open model · Ctrl+O",True)):
                ui.SameLine()
                if ui.IconButton(action,drawer,tooltip="Finish or cancel the sketch first" if sketching else tip,enabled=enabled and not sketching):
                    self._queue_action(action)
            ui.SameLine()
            save_choice = ui.MenuButton(
                "Save##document",
                (MenuItem("Save", shortcut="Ctrl+S"), MenuItem("Save as", shortcut="Ctrl+Shift+S")),
                drawer=icons.file_save, enabled=not sketching,
                tooltip="Finish or cancel the sketch first" if sketching else "Save or save as · Ctrl+S")
            if save_choice == (0,):
                self._queue_action("save")
            elif save_choice == (1,):
                self._queue_action("save_as")
            for action, drawer, tip, enabled in (
                    ("undo",icons.undo,"Undo feature change · Ctrl+Z",bool(self.model.undo_stack)),
                    ("redo",icons.redo,"Redo feature change · Ctrl+Y",bool(self.model.redo_stack))):
                ui.SameLine()
                if ui.IconButton(action,drawer,tooltip="Finish or cancel the sketch first" if sketching else tip,enabled=enabled and not sketching):
                    self._queue_action(action)
            if self._return_paths:
                ui.SameLine()
                if ui.Button("Back to assembly", width=140, height=28, enabled=not sketching):
                    self._queue_action("return_parent")
                if ui.IsItemHovered():
                    ui.SetTooltip("Return to "+self._return_paths[-1].name+" and rebuild its components.")
            ui.SameLine()
            ui.TextEllipsis(self.model.path.name+(" · Sketch: "+self.sketch_session["sketch_name"] if sketching else "")+("  •" if self.model.dirty else ""),
                            max(1,ui.GetContentRegionAvail()[0]-312-3*self.window_chrome.button_width),muted=True,height=28)
            title_rect = ui._last
            self.window_chrome.drag_rect = (title_rect[0], 0, title_rect[2], 40)
            ui.SameLine()
            if ui.IconButton("Fit model",sketch_icons.draw_normal):
                if sketching:
                    self.sketch_session["fit"]()
                elif self.viewer._solid is not None:
                    self.viewer.camera.fit(*self.viewer._solid["packed"]["bounds"])
            ui.SameLine()
            if ui.Button("Commands",width=80,height=28,enabled=not sketching):
                self._run_action("ribbon")
            if ui.IsItemHovered():
                ui.SetTooltip("Show or hide the command ribbon · Ctrl+F1")
            ui.SameLine()
            if ui.Button("Python",width=72,height=28,enabled=not sketching):
                visible = not ui.docks.visible.get("Output",False)
                ui.set_dock_visible("Output",visible)
                if visible:
                    self._output_tab = "Python"
            ui.SameLine()
            if ui.Button("Light" if ui.theme_name == "dark" else "Dark",width=64,height=28):
                ui.set_theme("light" if ui.theme_name == "dark" else "dark")
            self.window_chrome.draw_buttons()
        ui.End()

    def _status_bar(self):
        ui = self.ui
        ui.PushStyleVar(ui.ImGuiStyleVar_WindowPadding,(12,2))
        visible = ui.Begin("Status",flags=ui.ImGuiWindowFlags_NoScrollWithMouse)
        ui.PopStyleVar()
        if visible:
            message = self.status or "Ready"
            if self.sketch_session is not None:
                message = self.sketch_session["state"].get("status") or "Editing sketch"
            elif self.model.error:
                message = "Build failed · " + self.model.error
            elif self._draft_dirty() and not message.startswith("Check inputs:"):
                message = "Unapplied changes · Apply to rebuild the model"
            ui.TextEllipsis(message,max(1,ui.GetContentRegionAvail()[0]-268),height=22)
            ui.SameLine()
            state = "Unsaved changes" if self.model.dirty else "New model" if self.model.saved_source is None else "Saved"
            ui.TextEllipsis(state+"  ·  Experimental modeling",
                            muted=True,height=22)
        ui.End()

    def _ribbon_group(self, label, commands):
        from . import modeling_icons as icons
        ui = self.ui
        ui.BeginRibbonGroup(label)
        compact_width = max((ui.CalcTextSize(caption)[0]+36 for _,caption,_ in commands[1:]),default=0)
        for index, (key, caption, icon) in enumerate(commands):
            compact = index > 0
            if index == 1:
                ui.BeginRibbonColumn()
            operation = operations()[key]
            enabled = operation.static or any(isinstance(v, operation.owner) for v in self.model.values.values()) or (
                operation.owner.__name__ == "Sketch" and self.model.part is not None)
            if ui.RibbonButton(caption, getattr(icons, icon), width=compact_width if compact else 76,
                               height=22 if compact else 66, icon_size=16 if compact else 32,
                               compact=compact, tooltip=operation.description,
                               selected=self.pending is not None and self.pending.key == key, enabled=enabled and self.sketch_session is None):
                self._queue_action("operation",key)
        if len(commands) > 1:
            ui.EndRibbonColumn()
        ui.EndRibbonGroup()

    def _file_dialog(self, new=False, discard=False, assembly=False):
        from pyglet.window.dialog import FileOpenDialog, FileSaveDialog
        if self.model.dirty and not discard:
            self.status = "Save the current model before opening another file"
            return
        options = dict(title="New assembly" if new and assembly else "New part" if new else "Open model", initial_dir=self.model.path.parent,
                       filetypes=[("Python models", "*.py")])
        self.dialog = FileSaveDialog(default_ext=".py", initial_file="assembly.py" if assembly else "part.py", **options) if new else FileOpenDialog(**options)
        def selected(paths):
            filename = paths if new else paths[0] if paths else ""
            if filename:
                self.file_path = str(filename)
                if assembly:
                    self._switch_file(new=new,discard=discard,assembly=True)
                else:
                    self._switch_file(new=new,discard=discard)
            self.dialog = None
        self.dialog.push_handlers(**{"on_dialog_save" if new else "on_dialog_open": selected})
        self.dialog.open()

    def _component_dialog(self):
        from pyglet.window.dialog import FileOpenDialog
        self.dialog = FileOpenDialog(title="Insert saved part or assembly",initial_dir=self.model.path.parent,
                                     filetypes=[("Python models","*.py")])
        def selected(paths):
            self.dialog = None
            if paths:
                self._queue_action("insert_component",str(paths[0]))
        self.dialog.push_handlers(on_dialog_open=selected)
        self.dialog.open()

    def _unique_name(self, stem, document=None):
        import re
        stem = re.sub(r"\W", "_",stem)
        if not stem or stem[0].isdigit() or keyword.iskeyword(stem):
            stem = "component_"+stem
        names = {b.id for b in (document or self.model.document).blocks}
        name, index = stem, 1
        while name in names:
            name, index = stem+str(index), index+1
        return name

    def _insert_component(self, path):
        from .api import Assembly
        context = self._assembly_context(self._available_values())
        if context is None:
            self.status = "Create an assembly before inserting a component"
            return False
        assembly_name = self._assembly_references[1]
        try:
            relative = os.path.relpath(Path(path).resolve(),self.model.path.parent)
            source = self.model.document.with_imports({"load_component"})
            definition = self._unique_name(Path(path).stem,ModelDocument(source))
            source = ModelDocument(source).insert(definition,"    "+definition+" = load_component("+repr(relative)+")\n")
            # Resolve its type in a disposable build before choosing insertion syntax.
            _, _, _, values = self.model.build(ModelDocument(source))
            nested = isinstance(values[definition],Assembly)
            position = (0,0,0)
            bounds = context.assembly.bounds()
            if bounds is not None:
                placed = _result_scene(self.model.part,values[definition])
                coordinates = [v[0] for patch in placed.patches for v in patch["vertices"]]
                if coordinates:
                    span = max(coordinates)-min(coordinates)
                    position = (bounds[1].x-min(coordinates)+max(2,span*.15),0,0)
            occurrence = self._unique_name(definition+"_instance",ModelDocument(source))
            source = ModelDocument(source).insert(occurrence,"    "+occurrence+" = "+assembly_name+
                (".add_subassembly(" if nested else ".add_part(")+definition+", position="+repr(position)+")\n")
            if not context.assembly.parts and not context.assembly.subassemblies:
                fixed = self._unique_name("fixed_base",ModelDocument(source))
                source = ModelDocument(source).insert(fixed,"    "+assembly_name+".fix("+occurrence+")\n")
            source = ModelDocument(source).display_result(assembly_name)
            if not self._apply(source):
                return False
            self._select(occurrence)
            self._sync_scene(fit=True)
            self.status = "Component inserted · select references to add a mate"
            self._ribbon_tab = "Assembly"
            return True
        except (ValueError,OSError,SyntaxError,TypeError) as exc:
            self.form_build_error = str(exc)
            self.status = "Cannot insert component: "+str(exc)
            return False

    def _save_as_dialog(self):
        from pyglet.window.dialog import FileSaveDialog
        self.dialog = FileSaveDialog(title="Save model as",initial_dir=self.model.path.parent, initial_file=self.model.path.name,
                                     filetypes=[("Python models","*.py")],default_ext=".py")
        def selected(path):
            self.dialog = None
            if not path:
                return
            target = Path(path).resolve()
            previous = self.model.path
            if target == previous:
                self._save_model()
                return
            try:
                def relocate(source):
                    for block in ModelDocument(source).blocks:
                        if block.operation == "load_component" and block.editable:
                            fields = dict(_parameter_form(block))
                            component = (previous.parent/fields["path"]).resolve()
                            fields["path"] = os.path.relpath(component,target.parent)
                            source = ModelDocument(source).replace(block.id,_parameter_source(block,fields))
                    return source
                source = relocate(self.model.source)
                candidate = ModelSession(target)
                if not candidate.rebuild(source):
                    raise ValueError(candidate.error)
                candidate.save()
                candidate.undo_stack = [relocate(s) for s in self.model.undo_stack]
                candidate.redo_stack = [relocate(s) for s in self.model.redo_stack]
                candidate.selected = self.model.selected
                self.model = candidate
                self.file_path = str(target)
                self._select(candidate.selected)
                self._prior_values = self._assembly_references = None
                self._sync_scene()
                self.viewer.window.set_caption("Modeling (experimental) — "+target.name)
                self.status = "Saved "+target.name
            except (OSError,ValueError) as exc:
                self.form_build_error = self.status = str(exc)
        self.dialog.push_handlers(on_dialog_save=selected)
        self.dialog.open()

    def _ribbon(self):
        from . import modeling_icons as icons
        from . import sketch_icons
        ui = self.ui
        ui.PushStyleVar(ui.ImGuiStyleVar_WindowPadding, (12, 4))
        visible = ui.Begin("Model")
        ui.PopStyleVar()
        if not visible:
            ui.End()
            return
        if self._ribbon_tab:
            ui._tabs[(id(ui._current),"Modeling commands")] = self._ribbon_tab
            self._ribbon_tab = None
        ui._tabs.setdefault((id(ui._current), "Modeling commands"), "Solid")
        ui.BeginTabBar("Modeling commands")
        if ui.BeginTabItem("File"):
            ui.BeginRibbonGroup("Document")
            for index, (label, icon, action) in enumerate((
                    ("New part", icons.file_new, "new"), ("New assembly", icons.assembly, "new_assembly"),
                    ("Open", icons.file_open, "open"),("Save", icons.file_save, "save"),
                    ("Save as", icons.file_save, "save_as"))):
                if index:
                    ui.SameLine()
                if ui.RibbonButton(label, icon, width=max(76, ui.CalcTextSize(label)[0]+16),
                                   enabled=self.sketch_session is None, tooltip=label + " a Python model file."):
                    self._queue_action(action)
            ui.EndRibbonGroup()
            ui.BeginRibbonGroup("History")
            for index, (label, icon, revisions) in enumerate((
                    ("Undo", icons.undo, self.model.undo_stack), ("Redo", icons.redo, self.model.redo_stack))):
                if index:
                    ui.SameLine()
                if ui.RibbonButton(label, icon, enabled=bool(revisions) and self.sketch_session is None, tooltip="Restore the " + label.lower() + " revision."):
                    self._queue_action(label.lower())
            ui.EndRibbonGroup()
            ui.EndTabItem()
        if ui.BeginTabItem("Assembly"):
            from .api import Assembly
            present = any(isinstance(v,Assembly) for v in self.model.values.values())
            ui.BeginRibbonGroup("Components")
            if ui.RibbonButton("Assembly",icons.assembly,enabled=self.sketch_session is None,tooltip="Create an assembly in this document."):
                self._queue_action("operation","Part.assembly")
            ui.SameLine()
            if ui.RibbonButton("Insert",icons.file_open,enabled=present and self.sketch_session is None,tooltip="Insert a saved part or assembly. The first component is fixed as the base."):
                self._queue_action("insert_component")
            ui.EndRibbonGroup()
            for title,commands in (
                ("Placement",(("Assembly.fix","Fix","fix"),
                               ("Assembly.coincident","Coincident","coincident"),
                               ("Assembly.concentric","Concentric","concentric"))),
                ("Orientation",(("Assembly.parallel","Parallel","parallel"),
                                 ("Assembly.perpendicular","Perpendicular","perpendicular"),
                                 ("Assembly.angle","Angle","angle"))),
                ("Position",(("Assembly.distance","Distance","distance"),
                              ("Assembly.on_plane","On plane","surface"),
                              ("Assembly.contact","Contact","surface"))),
                ("Pattern",(("Assembly.pattern_linear","Linear","pattern"),
                             ("Assembly.pattern_circular","Circular","circular_pattern"),
                             ("Assembly.mirror","Mirror","mirror")))):
                self._ribbon_group(title,commands)
            ui.EndTabItem()
        if ui.BeginTabItem("Solid"):
            groups = (
                ("Profiles", (("Part.sketch", "Sketch", "offset_sketch"),
                              ("Part.section_sketch", "Section", "surface"))),
                ("Features", (("Sketch.extrude", "Extrude", "extrude"),
                              ("Sketch.revolve", "Revolve", "revolve"),
                              ("Part.loft", "Loft", "loft"),
                              ("Sketch.extrude_along_curve", "Sweep", "sweep"))),
                ("Combine", (("Part.union", "Union", "boolean"),
                             ("Part.subtract", "Subtract", "cut"),
                             ("Part.intersect", "Intersect", "common"))),
                ("Dress-up", (("Solid.fillet", "Fillet", "fillet"),
                              ("Solid.chamfer", "Chamfer", "chamfer"),
                              ("Solid.shell", "Shell", "shell"),
                              ("Solid.draft_faces", "Draft", "draft"))),
                ("Holes", (("Solid.hole", "Hole", "hole"),
                           ("Solid.counterbore_hole", "Counterbore", "counterbore"),
                           ("Solid.countersink_hole", "Countersink", "countersink"))),
                ("Pattern", (("Solid.pattern_linear", "Linear", "pattern"),
                             ("Solid.pattern_circular", "Circular", "circular_pattern"),
                             ("Solid.mirror", "Mirror", "mirror"))),
            )
            for label, commands in groups:
                self._ribbon_group(label, commands)
            ui.EndTabItem()
        if ui.BeginTabItem("Sketch"):
            self._sketch_commands()
            ui.EndTabItem()
        if ui.BeginTabItem("Surface"):
            for label, commands in (
                    ("Create", (("Sketch.surface", "Planar", "surface"),
                                ("Sketch.extrude_surface", "Extrude", "extrude"),
                                ("Sketch.revolve_surface", "Revolve", "revolve"),
                                ("Part.loft_surface", "Loft", "loft"))),
                    ("Modify", (("Surface.trim", "Trim", "cut"),
                                ("Surface.split", "Split", "surface"),
                                ("Surface.intersection_curves", "Intersection", "common"))),
                    ("Finish", (("Surface.thicken", "Thicken", "shell"),
                                ("Part.sew", "Sew", "boolean"),
                                ("Surface.cap_planar_boundaries", "Cap", "surface"))),
                    ("Extract", (("Solid.face_surface", "Face", "surface"),
                                 ("Part.section_sketch", "Section", "offset_sketch")))):
                self._ribbon_group(label, commands)
            ui.EndTabItem()
        if ui.BeginTabItem("Create"):
            self._ribbon_group("Primitives", (("Part.cuboid", "Block", "block"),
                                              ("Part.cylinder", "Cylinder", "cylinder"),
                                              ("Part.sphere", "Sphere", "sphere")))
            self._ribbon_group("Import", (("Part.load_stl", "STL", "file_open"),
                                          ("Part.load_obj", "OBJ", "file_open"),
                                          ("Part.load_off", "OFF", "file_open")))
            ui.EndTabItem()
        if ui.BeginTabItem("Operations"):
            _, self.operation_filter = ui.InputText("Find operation", self.operation_filter,
                                                    width=200, hint="Search commands")
            ui.SameLine()
            catalog = {operation.owner.__name__ + " · " + operation.group + " · " + operation.label: key
                       for key, operation in operations().items()
                       if key != "Part" and self.operation_filter.casefold() in
                       (key + " " + operation.label + " " + operation.group).casefold()}
            labels = sorted(catalog)
            current = next((label for label in labels if catalog[label] == self.operation_choice), "")
            if labels:
                changed, chosen = ui.ComboBox("Operation", current, labels,
                                             width=min(440, max(100, ui.GetContentRegionAvail()[0]-110)))
                if changed:
                    self.operation_choice = catalog[chosen]
            else:
                ui.TextUnformatted("No matching operations")
            ui.SameLine()
            if ui.Button("Set up", width=90, height=26, enabled=bool(labels) and self.sketch_session is None):
                self._queue_action("operation",self.operation_choice if current else catalog[labels[0]])
            if self.operation_choice in operations():
                ui.TextWrapped(operations()[self.operation_choice].description)
            ui.EndTabItem()
        if ui.BeginTabItem("View"):
            ui.BeginRibbonGroup("Viewport")
            if ui.RibbonButton("Fit", sketch_icons.draw_normal, tooltip="Fit the displayed model in the viewport."):
                if self.sketch_session is not None:
                    self.sketch_session["fit"]()
                elif self.viewer._solid is not None:
                    self.viewer.camera.fit(*self.viewer._solid["packed"]["bounds"])
            ui.SameLine()
            if ui.RibbonButton("Theme", sketch_icons.draw_circle, tooltip="Switch between light and dark appearance."):
                ui.set_theme("dark" if ui.theme_name == "light" else "light")
            ui.EndRibbonGroup()
            ui.BeginRibbonGroup("Standard views")
            views = ("Isometric", "Top", "Bottom", "Front", "Back", "Right", "Left")
            for index, view in enumerate(views):
                if index % 3 == 0:
                    ui.BeginRibbonColumn()
                if ui.RibbonButton(view, icons.surface, compact=True, width=106, height=22, icon_size=16,
                                   enabled=self.sketch_session is None,
                                   tooltip="Look at the model from the " + view.lower() + " view."):
                    self._set_standard_view(view)
                if index % 3 == 2 or index == len(views)-1:
                    ui.EndRibbonColumn()
            ui.EndRibbonGroup()
            ui.BeginRibbonGroup("Workspace")
            for index,title in enumerate(("Features","Properties","Output")):
                if index:
                    ui.SameLine()
                shown = ui.docks.visible.get(title,True)
                if ui.RibbonButton(title,icons.surface,selected=shown,tooltip="Show or hide "+title.lower()+"."):
                    ui.set_dock_visible(title,not shown)
            ui.EndRibbonGroup()
            ui.EndTabItem()
        ui.EndTabBar()
        ui.End()

    def _set_standard_view(self, name):
        directions = {
            "Isometric": (1, -1, 1), "Top": (0, 0, 1), "Bottom": (0, 0, -1),
            "Front": (0, -1, 0), "Back": (0, 1, 0),
            "Right": (1, 0, 0), "Left": (-1, 0, 0),
        }
        self.viewer.camera.look_from(directions[name])

    def _sketch_commands(self):
        from . import sketch_icons as icons, sketch_ui
        from . import modeling_icons
        ui = self.ui
        active = self.sketch_session is not None
        state = self.sketch_session["state"] if active else {}
        block = next((b for b in self.model.document.blocks if b.id == self.model.selected), None) if self.model.document else None
        groups = (("Draw", icons.CURVE_TOOLS, ""),
                  ("Constraints", icons.CONSTRAINT_TOOLS, "con_"),
                  ("Dimensions", icons.DIMENSION_TOOLS, "dim_"))
        columns = sum((len(tools)+2)//3 for _, tools, _ in groups)
        tool_width = max(36, min(122, (ui.GetContentRegionAvail()[0]-620)/columns))
        ui.BeginRibbonGroup("Sketch")
        ui.BeginRibbonColumn()
        if ui.RibbonButton("New sketch", icons.draw_rectangle, compact=True, width=122, height=30,
                           icon_size=20, enabled=not active and self.model.part is not None,
                           tooltip="Choose a plane or planar face and draw a new profile."):
            self._queue_action("operation", "Part.sketch")
        if ui.RibbonButton("Edit sketch", icons.draw_line, compact=True, width=122, height=30,
                           icon_size=20, enabled=not active and block is not None and block.kind == "Sketch" and block.editable,
                           tooltip="Edit the selected sketch's geometry, constraints and dimensions."):
            self._queue_action("edit_sketch", block.id)
        ui.EndRibbonColumn()
        ui.EndRibbonGroup()
        for title, tools, prefix in groups:
            ui.BeginRibbonGroup(title)
            for index, (tool, label, drawer) in enumerate(tools):
                if index % 3 == 0:
                    ui.BeginRibbonColumn()
                caption = {"polyline":"Polyline", "rectangle":"Rectangle", "coincident":"Coincident",
                           "horizontal":"Horizontal", "vertical":"Vertical", "parallel":"Parallel",
                           "perpendicular":"Perpendicular", "tangent":"Tangent", "concentric":"Concentric",
                           "midpoint":"Midpoint", "distance":"Distance"}.get(tool, label)
                selected = state.get("pending_constraint" if prefix == "con_" else "pending_dimension" if prefix else "tool") == tool
                if ui.RibbonButton(caption+"##sketch_"+prefix+tool, drawer, compact=True, width=tool_width,
                                   height=22, icon_size=16, selected=selected, enabled=active,
                                   tooltip=icons._TOOLTIPS.get(prefix+tool, caption)):
                    if prefix == "con_":
                        sketch_ui._request_constraint(state, tool)
                    elif prefix == "dim_":
                        sketch_ui._request_dimension(state, tool)
                    else:
                        sketch_ui._set_curve_tool(state, tool)
                if index % 3 == 2 or index == len(tools)-1:
                    ui.EndRibbonColumn()
            ui.EndRibbonGroup()
        ui.BeginRibbonGroup("Close sketch")
        ui.BeginRibbonColumn()
        editing = any(state.get(k) is not None for k in ("curve_edit", "profile_edit", "constraint_edit", "dimension"))
        if ui.RibbonButton("Finish sketch", modeling_icons.check, width=128, height=30, compact=True, icon_size=20,
                           enabled=active and not editing and not state.get("edit_error") and not state.get("live_invalid"),
                           tooltip="Keep the sketch and return to the part feature."):
            state["done"] = "finish"
        if ui.RibbonButton("Cancel sketch", icons.draw_cancel, width=128, height=30, compact=True, icon_size=20, enabled=active,
                           tooltip="Discard sketch changes and return to the part."):
            state["done"] = "cancel"
        ui.EndRibbonColumn()
        ui.EndRibbonGroup()
        ui.BeginRibbonGroup("View and editing")
        if ui.RibbonButton("Fit sketch", icons.draw_normal, enabled=active, tooltip="Center and fit this sketch in the viewport."):
            self.sketch_session["fit"]()
        ui.SameLine()
        if ui.RibbonButton("Undo", modeling_icons.undo, enabled=active, tooltip="Undo the last sketch action or cancel the active edit."):
            sketch_ui._undo_pending_or_action(state)
        ui.EndRibbonGroup()

    def _pointer_in_viewport(self):
        pos = getattr(self.ui.GetIO(), "MousePos", None)
        rect = getattr(self.ui, "viewport_rect", None)
        if pos is None or rect is None:
            return False
        x, y, w, h = rect
        return w > 1 and h > 1 and x <= pos[0] < x + w and y <= pos[1] < y + h

    def _maybe_fit_view(self):
        from pyglet.window import key
        io = self.ui.GetIO()
        down = bool(io.KeysDown.get(key.F, False)) and not io.KeyCtrl and not io.KeyAlt
        if not down:
            self._fit_key_down = False
            return
        if self._fit_key_down:
            return
        self._fit_key_down = True
        if not self._pointer_in_viewport():
            return
        if self.sketch_session is not None:
            fit = self.sketch_session.get("fit")
            if fit is not None:
                fit()
            return
        solid = self.viewer._solid
        if solid is not None:
            self.viewer.camera.fit(*solid["packed"]["bounds"])

    def tick(self):
        from .view import tick_cad_camera, _want_mouse, _left_click_released
        from . import glview
        ui = self.ui
        io = ui.GetIO()
        self._maybe_fit_view()
        if self.sketch_session is not None:
            from .sketch_ui import draw_sketch_tree
            self._document_bar()
            self._ribbon()
            if ui.Begin("Features"):
                draw_sketch_tree(ui, self.sketch_session["state"])
            ui.End()
            self.sketch_session["tick"]()
            self._status_bar()
            if self._sketch_close_requested:
                choice = ui.MessageBox("Close sketch?")
                if choice == "Keep editing":
                    self._sketch_close_requested = False
                elif choice == "Discard and close":
                    self._sketch_close_requested = False
                    self.sketch_session["state"]["done"] = "cancel"
                    self._queue_action("close")
            if self.sketch_session["state"]["done"] is not None:
                if self._sketch_close_requested:
                    self._queue_action("close")
                    self._sketch_close_requested = False
                self._finish_sketch()
            return
        want_mouse = _want_mouse(ui, io)
        tick_cad_camera(self.viewer, ui, io, want_mouse)
        released = _left_click_released(ui, io)
        packed = self.viewer._solid["packed"] if self.viewer._solid is not None else None
        if released is not None and not want_mouse and packed is not None:
            mx, my = io.MousePos
            width, height = io.DisplaySize
            if width > 1 and height > 1:
                origin, direction = self.viewer.camera.ray(
                    2 * mx / width - 1, 1 - 2 * my / height)
                point_radius, curve_radius = glview.pick_radii(self.viewer.camera)
                name = glview.pick_name(packed, origin, direction,
                                        point_radius, curve_radius)
                owner = self._body_pick_owners.get(name) if name else None
                assigned_body = bool(owner and self._assign_clicked_body(owner))
                if assigned_body:
                    self.picked = [picked for picked, who in self._body_pick_owners.items() if who == owner]
                    name = None
                if name:
                    if bool(getattr(io, "KeyCtrl", False)):
                        if name in self.picked:
                            self.picked.remove(name)
                        else:
                            self.picked.append(name)
                    else:
                        self.picked = [name]
                elif not assigned_body and not bool(getattr(io, "KeyCtrl", False)):
                    self.picked.clear()
                self.viewer.highlight_solid(self.picked)
                if name and self._pick_reference:
                    block = next((b for b in self.model.document.blocks if b.id == self.model.selected),None)
                    available = self._available_values(None if self.pending or block is None else block.start)
                    self._apply_entity_pick(name, available, ray=(origin, direction))
                if name and self._pick_datum:
                    block = next((b for b in self.model.document.blocks if b.id == self.model.selected), None)
                    available = self._available_values(None if self.pending or block is None else block.start)
                    self._accept_datum_pick(name, available)
        self._document_bar()
        self._ribbon()
        self._feature_tree()
        self._properties()
        self._diagnostics()
        self._status_bar()
        self._action_dialog()
        if self._queued_action:
            action,self._queued_action = self._queued_action,None
            self._request_action(*action)
        if self._shortcut_action:
            action,self._shortcut_action = self._shortcut_action,None
            if action == "ribbon":
                self._run_action(action)
            elif action == "apply":
                self._commit_edits()
            else:
                self._request_action(action)

    def show(self):
        try:
            self.viewer.show()
        finally:
            self.viewer.close()


def open(path, *, control_file=None):
    """Open a Python model in the visual modeling workspace.

    A missing path creates an unsaved starter model. Save writes ordinary Python.
    ``control_file`` explicitly enables local GUI automation and writes its
    connection credentials to a new JSON file, removed when the workspace exits.
    """
    session = ModelSession(path)
    session.rebuild()
    workspace = _Workspace(session)
    if control_file is None:
        workspace.show()
    else:
        import json
        from .gui_testing import GuiDriver
        from .gui_remote import GuiControlServer
        endpoint = Path(control_file)
        try:
            driver = GuiDriver(workspace.ui, workspace.viewer._draw_frame,
                               capture=workspace.viewer.capture_image)
            with GuiControlServer(driver) as server:
                with endpoint.open("x", encoding="utf-8") as output:
                    json.dump(server.connection, output)
                try:
                    workspace.show()
                finally:
                    try:
                        if json.loads(endpoint.read_text(encoding="utf-8")) == server.connection:
                            endpoint.unlink()
                    except (OSError, ValueError):
                        pass
        finally:
            workspace.viewer.close()
    return session
