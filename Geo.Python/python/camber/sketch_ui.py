"""Interactive sketch tools. Finish copies camber Python to the clipboard."""

import math

from .clipboard import copy_to_clipboard
from .sketch_emit import (
    apply_actions, emit_sketch_python, next_curve_name, _set_solve_after_every_constraint,
)
from . import naming as _naming
from . import pick as _pick
from . import sketch_constraints_viz as _cons_viz
from . import sketch_icons as _icons

_TOOLS = ("line", "polyline", "circle", "arc", "rectangle")
_PLANE_NAMES = {"xy": "xy", "yz": "yz", "zx": "zx", "xz": "zx"}
_PLANE_COLOR = (0.52, 0.54, 0.60)
_PLANE_ALPHA = 0.36
_CURVE_BLUE = (0.02, 0.28, 1.0)
_CONSTRUCTION_GRAY = (0.52, 0.52, 0.54)
_PREVIEW_BLUE = (0.22, 0.55, 1.0)
_SELECT_ORANGE = (1.0, 0.48, 0.06)
_CONSTRAINT_PURPLE = _cons_viz.CONSTRAINT_COLOR
_DIMENSION_ORANGE = _cons_viz.DIMENSION_COLOR
_ARROW_PX_LEN = 8.0
_ARROW_PX_WIDTH = 5.0
_LABEL_SCALE = _cons_viz.LABEL_SCALE
_GEOM_KINDS = ("line", "circle", "arc", "rectangle")
_PICK_ANY = _pick.MODE_ANY
_PICK_POINTS = _pick.MODE_POINTS
_PICK_LINES = _pick.MODE_LINES
_PICK_CIRCLES = _pick.MODE_CIRCLES
_PICK_LINE_OR_CIRCLE = _pick.MODE_LINE_OR_CIRCLE
_PICK_POINT_AND_LINE = _pick.MODE_POINT_AND_LINE
# Slot-based placement (same idea as the C# helper, without a second window).
_CONSTRAINT_SLOTS = {
    "coincident": (
        {"kind": "point", "label": "first point", "mode": _PICK_POINTS},
        {"kind": "point", "label": "second point", "mode": _PICK_POINTS},
    ),
    "fix": (
        {"kind": "point", "label": "point", "mode": _PICK_POINTS},
    ),
    "horizontal": (
        {"kind": "line", "label": "line", "mode": _PICK_LINES},
    ),
    "vertical": (
        {"kind": "line", "label": "line", "mode": _PICK_LINES},
    ),
    "parallel": (
        {"kind": "line", "label": "first line", "mode": _PICK_LINES},
        {"kind": "line", "label": "second line", "mode": _PICK_LINES},
    ),
    "perpendicular": (
        {"kind": "line", "label": "first line", "mode": _PICK_LINES},
        {"kind": "line", "label": "second line", "mode": _PICK_LINES},
    ),
    "tangent": (
        {"kind": "line", "label": "line", "mode": _PICK_LINES},
        {"kind": "circle", "label": "circle or arc", "mode": _PICK_CIRCLES},
    ),
    "equal": (
        {"kind": "curve", "label": "first curve", "mode": _PICK_LINE_OR_CIRCLE},
        {"kind": "curve", "label": "second curve", "mode": _PICK_LINE_OR_CIRCLE},
    ),
    "midpoint": (
        {"kind": "point", "label": "point", "mode": _PICK_POINTS},
        {"kind": "line", "label": "line", "mode": _PICK_LINES},
    ),
    "concentric": (
        {"kind": "circle", "label": "first circle or arc", "mode": _PICK_CIRCLES},
        {"kind": "circle", "label": "second circle or arc", "mode": _PICK_CIRCLES},
    ),
}
_DIMENSION_SLOTS = {
    "length": (
        {"kind": "line", "label": "line", "mode": _PICK_LINES},
    ),
    "radius": (
        {"kind": "circle", "label": "circle or arc", "mode": _PICK_CIRCLES},
    ),
    "distance": (
        {"kind": "point", "label": "first point", "mode": _PICK_POINTS},
        {"kind": "point", "label": "second point", "mode": _PICK_POINTS},
    ),
    "angle": (
        {"kind": "line", "label": "first line", "mode": _PICK_LINES},
        {"kind": "line", "label": "second line", "mode": _PICK_LINES},
    ),
}


def _sketch_registered_name(sketch):
    try:
        name = sketch.name
        if name:
            return str(name)
    except Exception:
        pass
    return ""


def _is_sketch_name_collision(exc):
    return "already registered" in str(exc).lower()


def _create_constraint_sketch(part, plane, frame, emit_frame, name):
    def create(n):
        if frame is not None:
            try:
                return part.sketch(frame=frame, name=n, constrained=True)
            except Exception:
                if emit_frame:
                    raise
        return part.sketch(plane, name=n, constrained=True)

    if name:
        try:
            return create(name)
        except Exception as ex:
            if not _is_sketch_name_collision(ex):
                raise
    last = None
    for _ in range(24):
        try:
            return create(None)
        except Exception as ex:
            last = ex
            if not _is_sketch_name_collision(ex):
                raise
    if last is not None:
        raise last
    return create(None)


def sketch_interactive(
    part,
    plane="xy",
    name="",
    part_var="part",
    sketch_var="sk",
    title="camber sketch",
    frame=None,
    emit_frame=False,
):
    """Draw on a plane, then Finish copies a sketch code block and returns (Sketch, code).

    Cancel returns (None, None). Requires pyglet + imgui.
    """
    from .host import Viewer
    from .imgui_compat import import_imgui

    imgui = import_imgui()
    ps = Viewer(title)
    session = begin_sketch_session(
        ps, imgui, part, plane=plane, name=name, frame=frame, emit_frame=emit_frame,
        register_context=True, look_at=True,
    )

    def callback():
        session["tick"]()
        if session["state"]["done"] is not None:
            ps.unshow()

    ps.set_user_callback(callback)
    ps.show()
    return finalize_sketch_session(ps, session, part_var=part_var, sketch_var=sketch_var)


def show_sketch(sketch, title="Camber", plane="xy"):
    """Open the sketch viewer for an existing constrained sketch. Does not extrude."""
    part = getattr(sketch, "_part", None)
    if part is None:
        raise TypeError("show() expects a Sketch created by Part.sketch(..., constrained=True)")
    from .host import Viewer
    from .imgui_compat import import_imgui

    imgui = import_imgui()
    ps = Viewer(title)
    session = begin_sketch_session(
        ps, imgui, part,
        plane=plane or "xy",
        name=getattr(sketch, "name", None) or "",
        register_context=True,
        look_at=True,
        existing_sketch=sketch,
    )

    def callback():
        session["tick"]()
        if session["state"]["done"] is not None:
            ps.unshow()

    ps.set_user_callback(callback)
    ps.show()
    finalize_sketch_session(ps, session)
    return sketch


def begin_sketch_session(
        ps, imgui, part, plane="xy", name="", frame=None, emit_frame=False,
        register_context=True, look_at=True, on_selection=None, existing_sketch=None):
    """Attach sketch overlays to the current viewer. Does not show() or unshow()."""
    from .view import (
        _as_scene,
        _cad_frame,
        _curve_pick_verts,
        _diamond_batch_verts,
        _is_ortho,
        _quad_batch_faces,
        _rect_batch_verts,
        _world_per_pixel,
    )

    plane_key = (plane or "xy").lower()
    plane = _PLANE_NAMES.get(plane_key, plane or "xy")
    if frame is None:
        if plane_key in _PLANE_NAMES:
            frame = _principal_frame(plane)
        else:
            frame = part.plane_frame(plane)
    scene = None
    try:
        scene = _as_scene(part)
    except Exception:
        scene = None

    seed = []
    if existing_sketch is not None:
        try:
            seed = list(existing_sketch._solved_actions() or [])
        except Exception:
            seed = []
        if frame is None:
            try:
                frame = existing_sketch.frame
            except Exception:
                frame = None
        if not name:
            name = getattr(existing_sketch, "name", None) or ""
        try:
            sketch_frame = existing_sketch.frame
        except Exception:
            sketch_frame = None
        if sketch_frame is not None:
            frame = sketch_frame

    bounds = _plane_bounds(scene, plane, frame, default=20.0)
    if seed:
        bounds = _actions_bounds(seed, bounds)
    extent = max(bounds[1] - bounds[0], bounds[3] - bounds[2], 1.0)
    plane_lift = max(0.35, 0.004 * extent)
    geometry_lift = plane_lift + max(0.01, 0.0001 * extent)
    point_lift = geometry_lift + max(0.02, 0.0002 * extent)
    try:
        ps.set_transparency_mode("pretty")
    except Exception:
        pass

    viz = {
        "lift": geometry_lift,
        "point_lift": point_lift,
        "plane_lift": plane_lift,
        "frame": frame,
        "context_names": [],
    }
    if register_context:
        _register_context(ps, scene, viz["context_names"])
    _register_plane(ps, frame, bounds, plane_lift, viz)
    viz["origin"] = [_uv_to_world((0.0, 0.0), frame, point_lift)]
    if look_at:
        _look_at_plane(ps, frame, extent)

    state = {
        "tool": "line",
        "pending": [],
        "pending_refs": [],
        "actions": [],
        "snap_grid": False,
        "grid": _nice_grid(extent),
        "done": None,
        "escape_armed": False,
        "status": "Finish copies Python and closes the sketch. Cancel discards it.",
        "click_latch": False,
        "right_press": None,
        "drag": None,
        "keys": {},
        "selected": [],
        "selection_dirty": False,
        "dimension": None,
        "pending_constraint": None,
        "pending_dimension": None,
        "slots": [],
        "sketch": None,
        "solved": None,
        "needs_rebuild": False,
        "batches": [],
        "sketch_name": name or "",
        "hover_hit": None,
        "last_picked_name": "",
        "part": part,
        "plane": plane,
        "frame": frame,
        "emit_frame": emit_frame,
        "on_selection": on_selection,
        "pick_catalog": [],
        "part_points": [],
        "scene": scene,
        "keep_sketch": existing_sketch is not None,
    }

    if existing_sketch is not None:
        state["sketch"] = existing_sketch
        state["sketch_name"] = name or getattr(existing_sketch, "name", None) or ""
        state["actions"] = seed
        state["solved"] = list(seed)
        state["status"] = "Viewing sketch. Finish or close the window."
    _ensure_live_sketch(state)
    _rebuild_pick_catalog(state, viz)
    _refresh_overlays(ps, state, frame, viz)
    _emit_selection(state)

    def tick():
        _draw_ui(imgui, state, plane, lambda: _look_at_plane(ps, frame, extent))
        if state["selection_dirty"]:
            _rebuild_pick_catalog(state, viz)
            _refresh_overlays(ps, state, frame, viz)
            state["selection_dirty"] = False
        if state["needs_rebuild"]:
            _adopt_solved(state, commit=False)
            _rebuild_pick_catalog(state, viz)
            _refresh_overlays(ps, state, frame, viz)
            state["needs_rebuild"] = False
        _handle_keys(imgui, state)
        try:
            _tick_sketch_input(ps, imgui, state, frame, viz)
        finally:
            _tick_sketch_draw(
                ps, imgui, viz, _cad_frame, _is_ortho, _world_per_pixel,
                _curve_pick_verts, _diamond_batch_verts, _rect_batch_verts, _quad_batch_faces,
            )

    return {
        "tick": tick,
        "state": state,
        "viz": viz,
        "part": part,
        "plane": plane,
        "frame": frame,
        "emit_frame": emit_frame,
        "sketch_name": state["sketch_name"],
        "extent": extent,
    }


def finalize_sketch_session(ps, session, part_var="part", sketch_var="sk"):
    state = session["state"]
    part = session["part"]
    teardown_sketch_graphics(ps, session.get("viz"))
    sketch = state.get("sketch")
    code = None
    if state.get("done") == "finish":
        code = emit_sketch_python(
            state["actions"],
            part_var=part_var,
            sketch_var=sketch_var,
            plane=session.get("plane"),
            name=state.get("sketch_name") or session.get("sketch_name") or "",
            frame=session.get("frame") if session.get("emit_frame") else None,
        )
        copy_to_clipboard(code)
    if not state.get("keep_sketch"):
        _unregister_session_sketch(part, sketch)
        state["sketch"] = None
    if code is None:
        return None, None
    return sketch, code


def _unregister_session_sketch(part, sketch):
    if part is None or sketch is None:
        return
    try:
        part._unregister_sketch(sketch)
    except Exception:
        pass


def _seed_live_from_actions(state):
    """Apply already-recorded actions onto a new live sketch. Used by tests."""
    created = state.get("sketch") is None
    live = _ensure_live_sketch(state)
    if created and state.get("actions"):
        apply_actions(live, state["actions"], start=0)
    _adopt_solved(state, commit=False)
    return live


def _ensure_live_sketch(state):
    """Create the session sketcher once. Never replaces an existing one."""
    live = state.get("sketch")
    if live is not None:
        return live
    live = _create_constraint_sketch(
        state.get("part"),
        state.get("plane"),
        state.get("frame"),
        state.get("emit_frame"),
        state.get("sketch_name") or None,
    )
    _set_solve_after_every_constraint(live, False)
    actual = _sketch_registered_name(live)
    if actual:
        state["sketch_name"] = actual
    state["sketch"] = live
    return live


def _sketch_counts(sketch):
    try:
        return int(sketch.curve_count), int(sketch.constraint_count)
    except Exception:
        native = getattr(sketch, "_n", None)
        if native is None:
            return None
        try:
            return int(native.constraint_curve_count), int(native.constraint_count)
        except Exception:
            return None


def _adopt_solved(state, commit=False):
    live = state.get("sketch")
    if live is None or not state.get("actions"):
        state["solved"] = []
        return
    try:
        dumped = live._solved_actions()
    except Exception:
        dumped = []
    state["solved"] = dumped if dumped else []
    if commit and dumped and len(dumped) == _recorded_curve_count(state["actions"]):
        _commit_solved_to_actions(state)


def _append_and_solve(state, new_actions):
    """Add new curves/constraints to the one live sketch and solve it once."""
    if not new_actions:
        return True
    created = state.get("sketch") is None
    live = _ensure_live_sketch(state)
    start = len(state["actions"])
    before = _sketch_counts(live)
    state["actions"].extend(new_actions)
    try:
        apply_actions(live, state["actions"], start=(0 if created else start))
        after = _sketch_counts(live)
        batch = {"actions": list(new_actions)}
        if before is not None and after is not None:
            batch["curves"] = after[0] - before[0]
            batch["constraints"] = after[1] - before[1]
        state.setdefault("batches", []).append(batch)
        _adopt_solved(state, commit=True)
        return True
    except Exception as ex:
        del state["actions"][start:]
        state["status"] = "Solve failed: " + str(ex)
        return False


def teardown_sketch_graphics(ps, viz):
    _remove_mesh(ps, "__sketch_plane")
    for key in ("x", "y", "geom", "build", "preview", "sel", "cons", "dim"):
        _remove_mesh(ps, "__sk_edge_" + key)
        _remove_network(ps, "__sk_net_" + key)
    for key in ("origin", "geom_pts", "preview_pts", "sel_pts", "fixed_pts"):
        _remove_mesh(ps, "__sk_pt_" + key)
    _remove_mesh(ps, "__sk_tri_dim_arrows")
    for name in (viz or {}).get("context_names", []):
        _remove_mesh(ps, name)
    if viz is not None:
        viz.clear()


def _tick_sketch_input(ps, imgui, state, frame, viz):
    from .view import _CAD, _left_click_released, tick_cad_camera

    if state["done"] is not None:
        return
    io = imgui.GetIO()
    want_ui = _want_mouse(imgui)
    tick_cad_camera(ps, imgui, io, want_ui)
    ray = None
    hover = None
    if not want_ui:
        ray, hover = _pick_ray_and_uv(ps, imgui, frame, viz["lift"])
    selecting = (
        state["tool"] == "select"
        or bool(state.get("pending_constraint"))
        or bool(state.get("pending_dimension"))
    )
    slot_exclude = None
    if state.get("pending_constraint") or state.get("pending_dimension"):
        slot_exclude = state.get("slots") or []
    if ray is not None:
        if selecting:
            state["hover_hit"] = _hit_at(
                state, ray, _pick_tolerance(ps, imgui, state), _pick_mode(state),
                exclude=slot_exclude)
        elif state["tool"] in _TOOLS:
            state["hover_hit"] = _hit_at(
                state, ray, _pick_tolerance(ps, imgui, state), _PICK_POINTS)
        else:
            state["hover_hit"] = None
    else:
        state["hover_hit"] = None
    if (
            _mouse_double_clicked(imgui, 0)
            and ray is not None
            and not want_ui
            and not _CAD.get("orbiting")):
        drag = state.get("drag")
        if drag is not None and not drag.get("moved"):
            state["drag"] = None
        if state.get("drag") is None and _try_edit_constraint(ps, imgui, state, viz, ray, hover):
            state["consume_click"] = True
            _refresh_overlays(ps, state, frame, viz)
            _refresh_preview(ps, state, frame, viz, hover=hover)
            return
    if state.get("drag") is not None:
        left = _mouse_down(imgui, 0)
        if want_ui or _CAD.get("orbiting") or not left:
            _finish_handle_drag(
                state, ray,
                select_if_click=(not want_ui and not _CAD.get("orbiting")),
                tolerance=_pick_tolerance(ps, imgui, state))
            _refresh_overlays(ps, state, frame, viz)
            _refresh_preview(ps, state, frame, viz, hover=hover)
            return
        if hover is not None:
            _update_handle_drag(state, hover, _pick_tolerance(ps, imgui, state))
            _refresh_overlays(ps, state, frame, viz)
        return
    if want_ui:
        return
    if _mouse_down(imgui, 2):
        return
    if _right_click_released(imgui, io, state):
        _undo_pending_or_action(state)
        _refresh_preview(ps, state, frame, viz, hover=hover)
        return
    can_drag = (
        state["tool"] == "select"
        and not state.get("pending_constraint")
        and not state.get("pending_dimension")
        and state.get("dimension") is None
    )
    if can_drag and _mouse_clicked(imgui, 0) and ray is not None and not _mouse_double_clicked(imgui, 0):
        if _hit_dimension_action(ps, imgui, viz, ray, hover) < 0:
            handle = _hit_drag_handle(state, ray, _pick_tolerance(ps, imgui, state))
            if handle is not None:
                state["drag"] = {
                    "curve": handle[0],
                    "role": handle[1],
                    "moved": False,
                    "native": False,
                    "start": hover,
                }
                return
    released = _left_click_released(imgui, io)
    if released is None:
        _refresh_preview(ps, state, frame, viz, hover=hover)
        return
    if state.pop("consume_click", False):
        _refresh_preview(ps, state, frame, viz, hover=hover)
        return
    if ray is None:
        ray, hover = _pick_ray_and_uv(ps, imgui, frame, viz["lift"])
    uv = hover
    if uv is None and ray is None:
        return
    if selecting:
        ctrl = False
        try:
            ctrl = bool(io.KeyCtrl)
        except Exception:
            pass
        if state.get("pending_constraint") or state.get("pending_dimension"):
            hit = _hit_at(
                state, ray, _pick_tolerance(ps, imgui, state), _pick_mode(state),
                exclude=state.get("slots") or [])
            if hit is not None:
                _accept_slot(state, hit)
        else:
            _select_at(
                state,
                _hit_at(state, ray, _pick_tolerance(ps, imgui, state), _pick_mode(state)),
                ctrl)

        _refresh_overlays(ps, state, frame, viz)
        _refresh_preview(ps, state, frame, viz, hover=hover)
        return
    uv, ref = _snap_ref(uv, state)
    added = _on_point(state, uv, ref)
    if added:
        _append_and_solve(state, added)
    _refresh_overlays(ps, state, frame, viz)
    _refresh_preview(ps, state, frame, viz)


def _draw_ui(imgui, state, plane, look_at_plane):
    from .view import _pop_style_color, _push_constraint_window_colors

    pushed = _push_constraint_window_colors(imgui)
    opened = _icons.begin_palette(imgui, "Sketch")
    if isinstance(opened, tuple):
        visible = bool(opened[0])
        if not visible:
            _icons.end_palette(imgui)
            _pop_style_color(imgui, pushed)
            return
    elif not opened:
        _icons.end_palette(imgui)
        _pop_style_color(imgui, pushed)
        return
    try:
        imgui.TextUnformatted("Sketch")
    except Exception:
        pass
    name = (state.get("sketch_name") or "").strip()
    head = (plane or "").strip()
    if name and name != head:
        head = (head + "  —  " + name) if head else name
    status = (state.get("status") or "").strip()
    line = (head + "  —  " + status) if head and status else (head or status)
    _icons.wrapped_text(imgui, line)
    imgui.Separator()
    try:
        imgui.TextUnformatted("Curves")
    except Exception:
        pass

    for i, (tool, label, drawer) in enumerate(_icons.CURVE_TOOLS):
        if i % _icons.COLS != 0:
            imgui.SameLine()
        if _icons.icon_button(imgui, tool, label, drawer, state["tool"] == tool):
            state["tool"] = tool
            state["pending"] = []
            state["pending_refs"] = []
            state["pending_constraint"] = None
            state["pending_dimension"] = None
            state["dimension"] = None
            state["slots"] = []
            _clear_selection(state)
            if tool == "select":
                state["status"] = "Select: click to select; drag ends, mids, or radius handles"
            else:
                state["status"] = "tool: " + tool
            state["escape_armed"] = False

    imgui.Separator()
    try:
        imgui.TextUnformatted("Constraints")
    except Exception:
        pass
    for i, (tool, label, drawer) in enumerate(_icons.CONSTRAINT_TOOLS):
        if i % _icons.COLS != 0:
            imgui.SameLine()
        if _icons.icon_button(
                imgui, "con_" + tool, label, drawer,
                state.get("pending_constraint") == tool):
            _request_constraint(state, tool)
    if state.get("pending_constraint") or state.get("pending_dimension"):
        _icons.wrapped_text(imgui, state.get("status") or "")

    imgui.Separator()
    try:
        imgui.TextUnformatted("Dimensions")
    except Exception:
        pass
    for i, (tool, label, drawer) in enumerate(_icons.DIMENSION_TOOLS):
        if i % _icons.COLS != 0:
            imgui.SameLine()
        if _icons.icon_button(
                imgui, "dim_" + tool, label, drawer,
                state.get("pending_dimension") == tool):
            _request_dimension(state, tool)

    if state["dimension"] is not None:
        request = state["dimension"]
        focus = bool(state.pop("dimension_focus", False))
        changed, value, apply_clicked, cancel_clicked = _icons.dimension_editor(
            imgui, request.get("kind"), request["value"], focus=focus)
        if changed:
            request["value"] = float(value)
        if apply_clicked:
            _commit_dimension(state)
        if cancel_clicked:
            state["dimension"] = None

    imgui.Separator()
    try:
        imgui.TextUnformatted("View")
    except Exception:
        pass
    if _icons.icon_button(imgui, "fit", "Fit", _icons.draw_normal, False):
        if look_at_plane is not None:
            look_at_plane()
    imgui.SameLine()
    if _icons.icon_button(imgui, "undo", "Undo", _icons.draw_undo, False):
        _undo_pending_or_action(state)

    _draw_construction_toggle(imgui, state)

    imgui.Separator()
    try:
        changed, grid_on = imgui.Checkbox("Snap to grid", state["snap_grid"])
        if changed:
            state["snap_grid"] = bool(grid_on)
    except Exception:
        if imgui.Button("Snap grid: " + ("on" if state["snap_grid"] else "off")):
            state["snap_grid"] = not state["snap_grid"]

    imgui.Separator()
    if _icons.action_button(imgui, "Finish", button_id="sketch_finish"):
        state["done"] = "finish"
    imgui.SameLine()
    if _icons.action_button(imgui, "Cancel", button_id="sketch_cancel"):
        state["done"] = "cancel"

    _icons.end_palette(imgui)
    _pop_style_color(imgui, pushed)


def _handle_keys(imgui, state):
    io = imgui.GetIO()
    want = False
    try:
        want = bool(io.WantCaptureKeyboard)
    except Exception:
        pass
    if want:
        return

    def down(name, *alts):
        pressed = False
        for alt in alts:
            if _key_down(imgui, io, alt):
                pressed = True
                break
        prev = state["keys"].get(name, False)
        state["keys"][name] = pressed
        return pressed and not prev

    if down("V", "V", "v"):
        state["tool"] = "select"
        state["pending"] = []
        state["pending_refs"] = []
        state["pending_constraint"] = None
        state["pending_dimension"] = None
        state["dimension"] = None
        state["slots"] = []
        _clear_selection(state)
        state["status"] = "Select: click to select; drag ends, mids, or radius handles"
    elif down("L", "L", "l"):
        _set_curve_tool(state, "line")
    elif down("P", "P", "p"):
        _set_curve_tool(state, "polyline")
    elif down("C", "C", "c"):
        _set_curve_tool(state, "circle")
    elif down("A", "A", "a"):
        _set_curve_tool(state, "arc")
    elif down("R", "R", "r"):
        _set_curve_tool(state, "rectangle")
    elif down("X", "X", "x"):
        indices = _selected_curve_indices(state)
        if indices:
            _set_curves_construction(state, indices, not _curves_are_construction(state, indices))
            state["status"] = "Construction" if _curves_are_construction(state, indices) else "Sketch geometry"
    elif down("Enter", "Enter", "\r"):
        if state.get("dimension") is not None:
            _commit_dimension(state)
        elif state["tool"] == "polyline" and len(state["pending"]) > 0:
            state["pending"] = []
            state["pending_refs"] = []
            state["status"] = "polyline stopped"
        else:
            state["done"] = "finish"
    elif down("Escape", "Escape", "\x1b"):
        if state["pending"]:
            state["pending"] = []
            state["pending_refs"] = []
            state["escape_armed"] = False
            state["status"] = "cleared points"
        elif state.get("dimension") is not None:
            state["dimension"] = None
            state["escape_armed"] = False
            state["status"] = "cancelled dimension"
        elif state.get("pending_constraint") or state.get("pending_dimension") or state.get("slots"):
            state["pending_constraint"] = None
            state["pending_dimension"] = None
            state["slots"] = []
            _clear_selection(state)
            state["escape_armed"] = False
            state["status"] = "cleared constraint"
        else:
            _clear_selection(state)
            state["escape_armed"] = False
            state["status"] = "cleared selection"
    elif down("Backspace", "Backspace"):
        _undo_pending_or_action(state)


def _key_down(imgui, io, key):
    try:
        if key == "Enter":
            return bool(imgui.IsKeyDown(imgui.ImGuiKey_Enter))
        if key == "Escape":
            return bool(imgui.IsKeyDown(imgui.ImGuiKey_Escape))
        if key == "Backspace":
            return bool(imgui.IsKeyDown(imgui.ImGuiKey_Backspace))
    except Exception:
        pass
    if len(key) == 1:
        letters = getattr(imgui, "_camber_letters", None)
        if letters and key.upper() in letters:
            return True
        try:
            return bool(io.KeysDown[ord(key)])
        except Exception:
            return False
    return False


def _want_mouse(imgui):
    try:
        return bool(imgui.GetIO().WantCaptureMouse)
    except Exception:
        return False


def _mouse_down(imgui, button):
    try:
        return bool(imgui.IsMouseDown(button))
    except Exception:
        try:
            return bool(imgui.GetIO().MouseDown[button])
        except Exception:
            return False


def _mouse_double_clicked(imgui, button):
    try:
        return bool(imgui.IsMouseDoubleClicked(button))
    except Exception:
        pass
    try:
        return bool(imgui.GetIO().MouseDoubleClicked[button])
    except Exception:
        return False


def _mouse_clicked(imgui, button):
    try:
        return bool(imgui.IsMouseClicked(button))
    except Exception:
        try:
            return bool(imgui.GetIO().MouseClicked[button])
        except Exception:
            return False


def _mouse_released(imgui, button):
    try:
        return bool(imgui.IsMouseReleased(button))
    except Exception:
        try:
            return bool(imgui.GetIO().MouseReleased[button])
        except Exception:
            return False


def _right_click_released(imgui, io, state):
    from .view import _mouse_pos, _PICK_DRAG_PX
    pos = _mouse_pos(io)
    if _mouse_clicked(imgui, 1):
        state["right_press"] = pos
        return False
    if not _mouse_released(imgui, 1):
        return False
    press = state.get("right_press")
    state["right_press"] = None
    if press is None or pos is None:
        return False
    dx = pos[0] - press[0]
    dy = pos[1] - press[1]
    return dx * dx + dy * dy <= _PICK_DRAG_PX * _PICK_DRAG_PX


def _hit_drag_handle(state, ray, tolerance):
    name = _hit_at(state, ray, tolerance, _PICK_POINTS)
    ref = _catalog_ref(state, name)
    if ref is None or ref[0] != "point":
        return None
    return (ref[1], ref[2])


def _update_handle_drag(state, uv, tolerance):
    drag = state.get("drag")
    if drag is None:
        return
    if not drag["moved"] and _dist(uv, drag["start"]) < max(tolerance * 0.2, 1e-9):
        return
    used_native, ok = _apply_handle_drag(state, drag["curve"], drag["role"], uv)
    if not ok:
        if drag["moved"]:
            state["status"] = "Fully constrained — cannot drag."
        return
    drag["moved"] = True
    drag["native"] = used_native
    state["status"] = "Dragging sketch handle…"


def _finish_handle_drag(state, ray, select_if_click, tolerance=None):
    drag = state.get("drag")
    state["drag"] = None
    if drag is None:
        return
    if drag["moved"]:
        _adopt_solved(state, commit=True)
        state["status"] = "Drag finished."
        return
    if select_if_click and ray is not None:
        if tolerance is None or tolerance <= 0.0:
            tolerance = max(state.get("grid") or 0.0, 1e-6) * 0.35
        _select_at(state, _hit_at(state, ray, tolerance, _pick_mode(state)), False)


def _native_try_drag(sketch, curve_index, role, uv):
    native = getattr(sketch, "_n", None) if sketch is not None else None
    if native is None:
        return None
    for name in ("try_drag_sketch_point", "TryDragSketchPoint"):
        fn = getattr(native, name, None)
        if fn is None:
            continue
        try:
            return int(fn(int(curve_index), int(role), float(uv[0]), float(uv[1]))) != 0
        except Exception:
            continue
    return None


def _apply_handle_drag(state, curve_index, role, uv):
    live = state.get("sketch")
    native_ok = _native_try_drag(live, curve_index, role, uv)
    if native_ok is True:
        _adopt_solved(state, commit=True)
        return True, True
    if native_ok is False:
        return True, False
    try_drag = getattr(live, "_try_drag_point", None) if live is not None else None
    if try_drag is not None:
        try:
            if try_drag(curve_index, role, uv):
                _adopt_solved(state, commit=True)
                return True, True
            return True, False
        except Exception:
            return True, False
    return True, False


def _mutate_handle(action, role, uv):
    kind = action["kind"]
    if kind == "line":
        if role == 0:
            action["p0"] = uv
            return True
        if role == 1:
            action["p1"] = uv
            return True
        if role == 3:
            mid = (
                (action["p0"][0] + action["p1"][0]) * 0.5,
                (action["p0"][1] + action["p1"][1]) * 0.5,
            )
            dx = uv[0] - mid[0]
            dy = uv[1] - mid[1]
            action["p0"] = (action["p0"][0] + dx, action["p0"][1] + dy)
            action["p1"] = (action["p1"][0] + dx, action["p1"][1] + dy)
            return True
        return False
    if kind == "circle":
        if role == 2:
            action["center"] = uv
            return True
        if role in (3, 4, 5, 6):
            action["radius"] = max(_dist(action["center"], uv), 1e-9)
            return True
        return False
    if kind == "arc":
        if role == 0:
            action["start"] = uv
            return True
        if role == 1:
            action["end"] = uv
            return True
        if role == 3:
            action["mid"] = uv
            return True
        return False
    return False


def _commit_solved_to_actions(state):
    solved = state.get("solved")
    if not solved:
        return
    old_geom = _recorded_geometry(state["actions"])
    rest = [a for a in state["actions"] if a["kind"] not in ("line", "circle", "arc", "rectangle")]
    merged = []
    for index, dumped in enumerate(solved):
        action = dict(dumped)
        if index < len(old_geom):
            previous = old_geom[index]
            if previous.get("name") and not action.get("name"):
                action["name"] = previous["name"]
            for key in ("p0_ref", "p1_ref", "center_ref", "start_ref", "mid_ref", "end_ref", "construction"):
                if previous.get(key):
                    action[key] = previous[key]
        merged.append(action)
    state["actions"] = merged + rest


def _set_curve_tool(state, tool):
    state["tool"] = tool
    state["pending"] = []
    state["pending_refs"] = []
    state["pending_constraint"] = None
    state["pending_dimension"] = None
    state["dimension"] = None
    state["slots"] = []
    _clear_selection(state)
    state["status"] = "tool: " + tool
    state["escape_armed"] = False


def _on_point(state, uv, ref=None):
    state["escape_armed"] = False
    pending = state["pending"]
    refs = state.setdefault("pending_refs", [])
    pending.append(uv)
    refs.append(ref)
    tool = state["tool"]
    added = []
    if tool == "line" and len(pending) >= 2:
        name = next_curve_name(state["actions"], "line")
        added.append(_named_line(pending[0], pending[1], name, refs[0], refs[1]))
        pending[:] = []
        refs[:] = []
        state["status"] = "line added"
    elif tool == "polyline" and len(pending) >= 2:
        name = next_curve_name(state["actions"], "line")
        added.append(_named_line(pending[0], pending[1], name, refs[0], refs[1]))
        if _recorded_curve_count(state["actions"]) > 0:
            previous = _recorded_geometry(state["actions"])[-1]
            if previous["kind"] == "line" and _dist(previous["p1"], pending[0]) < 1e-9:
                added.append({
                    "kind": "coincident",
                    "points": [
                        _handle_name_for(previous, 1, _recorded_geometry(state["actions"] + added)),
                        name + "@0.000",
                    ],
                })
        pending[:] = [pending[1]]
        refs[:] = [refs[1]]
        state["status"] = "polyline segment — click next, Enter to stop"
    elif tool == "circle" and len(pending) >= 2:
        r = _dist(pending[0], pending[1])
        if r > 1e-9:
            added.append({
                "kind": "circle",
                "center": pending[0],
                "radius": r,
                "center_ref": refs[0],
                "name": next_curve_name(state["actions"], "circle"),
            })
            state["status"] = "circle added"
        pending[:] = []
        refs[:] = []
    elif tool == "arc" and len(pending) >= 3:
        start, through, end = _arc_click_points(pending)
        start_ref, through_ref, end_ref = _arc_click_points(refs)
        added.append({
            "kind": "arc",
            "start": start,
            "mid": through,
            "end": end,
            "start_ref": start_ref,
            "mid_ref": through_ref,
            "end_ref": end_ref,
            "name": next_curve_name(state["actions"], "arc"),
        })
        pending[:] = []
        refs[:] = []
        state["status"] = "arc added"
    elif tool == "rectangle" and len(pending) >= 2:
        x0, y0 = pending[0]
        x1, y1 = pending[1]
        corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        names = []
        for i in range(4):
            names.append(next_curve_name(state["actions"] + added, "line"))
            added.append(_named_line(corners[i], corners[(i + 1) % 4], names[i], None, None))
        for i in range(4):
            added.append({
                "kind": "coincident",
                "points": [names[i] + "@1.000", names[(i + 1) % 4] + "@0.000"],
            })
        added.append({"kind": "parallel", "curves": [names[0], names[2]]})
        added.append({"kind": "parallel", "curves": [names[1], names[3]]})
        added.append({"kind": "perpendicular", "curves": [names[0], names[1]]})
        pending[:] = []
        refs[:] = []
        state["status"] = "rectangle added"
    else:
        if tool == "arc":
            hint = ("click start", "click the other end", "click a point on the arc")[min(len(pending), 2)]
            state["status"] = "arc: {0} ({1}/3)".format(hint, len(pending))
        else:
            need = {"line": 2, "polyline": 2, "circle": 2, "arc": 3, "rectangle": 2}.get(tool, 2)
            state["status"] = "{0}: {1}/{2} points".format(tool, len(pending), need)
    return added


def _arc_click_points(clicks):
    """Click 1 = start, click 2 = end, click 3 = through-point."""
    return clicks[0], clicks[2], clicks[1]


def _named_line(p0, p1, name, p0_ref, p1_ref):
    action = {"kind": "line", "p0": p0, "p1": p1, "name": name}
    if p0_ref:
        action["p0_ref"] = p0_ref
    if p1_ref:
        action["p1_ref"] = p1_ref
    return action


def _handle_name_for(action, role, geoms):
    index = 0
    for i, item in enumerate(geoms):
        if item is action:
            index = i
            break
    return _pick.sketch_curve_display_name(action, index) + {
        0: "@0.000",
        1: "@1.000",
        2: "@center",
        3: "@0.500",
    }.get(role, "")


def _recorded_geometry(actions):
    return [a for a in actions if a.get("kind") in ("line", "circle", "arc")]


def _recorded_curve_count(actions):
    count = 0
    for action in actions:
        kind = action.get("kind")
        if kind in ("line", "circle", "arc"):
            count += 1
        elif kind == "rectangle":
            count += 4
    return count


def _geometry_actions(state):
    solved = state.get("solved")
    recorded = _recorded_geometry(state["actions"])
    if not solved:
        return recorded
    out = []
    for index, dumped in enumerate(solved):
        action = dict(dumped)
        if index < len(recorded):
            previous = recorded[index]
            if previous.get("name") and not action.get("name"):
                action["name"] = previous["name"]
            for key in ("p0_ref", "p1_ref", "center_ref", "start_ref", "mid_ref", "end_ref", "construction"):
                if previous.get(key):
                    action[key] = previous[key]
        out.append(action)
    return out


def _curve_kind(state, index):
    action = _curve_action(state, index)
    return action["kind"] if action else ""


def _curve_action(state, index):
    curves = _geometry_actions(state)
    if isinstance(index, str):
        key = _local_entity_name(index, state.get("sketch_name") or "")
        for i, action in enumerate(curves):
            if _curve_display_name(action, i) == key:
                return action
        return None
    try:
        index = int(index)
    except (TypeError, ValueError):
        return None
    if 0 <= index < len(curves):
        return curves[index]
    return None


def _clear_selection(state):
    if state.get("selected"):
        state["selected"] = []
        state["selection_dirty"] = True
        _emit_selection(state)


def _emit_selection(state):
    callback = state.get("on_selection")
    if callback is None:
        return
    try:
        callback(list(state.get("selected") or []))
    except Exception:
        pass


def _curve_display_name(action, index):
    return _pick.sketch_curve_display_name(action, index)


def _catalog_entry(state, name):
    return _pick.entry_by_name(state.get("pick_catalog"), name)


def _catalog_ref(state, name):
    entry = _catalog_entry(state, name)
    if entry is None:
        return None
    return entry.get("ref")


def _rebuild_pick_catalog(state, viz=None):
    frame = state.get("frame")
    if frame is None:
        state["pick_catalog"] = []
        return
    lift = 0.0
    point_lift = 0.0
    if viz is not None:
        lift = viz.get("lift") or 0.0
        point_lift = viz.get("point_lift") or lift
    state["pick_catalog"] = build_sketch_pick_catalog(
        _geometry_actions(state),
        state.get("part_points") or (),
        frame,
        lift,
        point_lift,
        state.get("sketch_name") or "",
        scene=state.get("scene"),
    )
    state["part_points"] = _pick.part_point_snaps(state["pick_catalog"])


def build_sketch_pick_catalog(actions, part_points, frame, lift, point_lift, sketch_name, scene=None):
    """Named 3D pick targets: sketch handles plus every visible model point."""
    return _pick.build_sketch_pick_catalog(
        actions, part_points, frame, lift, point_lift, sketch_name, scene=scene,
    )


def _target_allowed(target, mode):
    return _pick.allowed(target, mode)


def _active_slot_spec(state):
    kind = state.get("pending_constraint")
    specs = _CONSTRAINT_SLOTS.get(kind) if kind else None
    if specs is None:
        kind = state.get("pending_dimension")
        specs = _DIMENSION_SLOTS.get(kind) if kind else None
    if not specs:
        return None, None
    filled = state.get("slots") or []
    if len(filled) >= len(specs):
        return None, specs
    return specs[len(filled)], specs


def _pick_mode(state):
    spec, _specs = _active_slot_spec(state)
    if spec is not None:
        return spec["mode"]
    return _PICK_ANY


def _slot_status(kind, specs, filled, prefix):
    if not specs:
        return prefix
    if len(filled) >= len(specs):
        return prefix + " ready"
    spec = specs[len(filled)]
    return "{0}: click {1} ({2}/{3})".format(prefix, spec["label"], len(filled) + 1, len(specs))


def _begin_slots(state, kind, is_dimension):
    state["tool"] = "select"
    state["pending"] = []
    state["pending_refs"] = []
    state["dimension"] = None
    state["escape_armed"] = False
    state["slots"] = []
    _clear_selection(state)
    if is_dimension:
        state["pending_constraint"] = None
        if state.get("pending_dimension") == kind:
            state["pending_dimension"] = None
            state["status"] = "Select curves or points"
            return
        state["pending_dimension"] = kind
        state["status"] = _slot_status(kind, _DIMENSION_SLOTS.get(kind), [], kind)
        return
    state["pending_dimension"] = None
    if state.get("pending_constraint") == kind:
        state["pending_constraint"] = None
        state["status"] = "Select curves or points"
        return
    state["pending_constraint"] = kind
    state["status"] = _slot_status(kind, _CONSTRAINT_SLOTS.get(kind), [], kind)


def _request_constraint(state, kind):
    _begin_slots(state, kind, False)


def _request_dimension(state, kind):
    _begin_slots(state, kind, True)


def _hit_as_point(state, hit):
    if isinstance(hit, str):
        if _naming.is_point_address(hit):
            return _local_entity_name(hit, state.get("sketch_name") or "")
        entry = _catalog_entry(state, hit)
        if entry is None:
            return None
        if entry.get("kind") == _pick.KIND_POINT:
            return _local_entity_name(hit, state.get("sketch_name") or "")
        ref = entry.get("ref")
        if ref is None:
            return None
        if ref[0] in ("point", "xy"):
            return _local_entity_name(hit, state.get("sketch_name") or "")
        return None
    ref = hit
    if ref is None:
        return None
    if ref[0] == "point":
        return (ref[1], ref[2])
    if ref[0] == "xy":
        return ("xy", ref[1], ref[2])
    return None


def _hit_curve_index(state, hit):
    name = _hit_curve_name(state, hit)
    if name:
        return name
    ref = _catalog_ref(state, hit) if isinstance(hit, str) else hit
    if ref is not None and ref[0] == "curve":
        return ref[1]
    return None


def _hit_curve_name(state, hit):
    if not isinstance(hit, str):
        return None
    ref = _catalog_ref(state, hit)
    if ref is not None and ref[0] == "curve":
        curves = _geometry_actions(state)
        if 0 <= ref[1] < len(curves):
            return _curve_display_name(curves[ref[1]], ref[1])
    if _naming.is_curve_address(hit):
        return _local_entity_name(hit, state.get("sketch_name") or "")
    return None


def _local_entity_name(name, sketch_name):
    owner, local = _naming.parse_qualified(name)
    if _naming.is_origin_name(local or name):
        return _naming.ORIGIN
    if owner and sketch_name and owner == sketch_name:
        return local
    if not owner:
        return local or name
    return name


def _accept_slot(state, hit):
    spec, specs = _active_slot_spec(state)
    if spec is None or specs is None:
        return
    slots = state.get("slots")
    if slots is None:
        slots = []
        state["slots"] = slots
    if hit in slots:
        return
    slots.append(hit)
    state["last_picked_name"] = hit
    state["selected"] = list(slots)
    state["selection_dirty"] = True
    _emit_selection(state)
    kind = state.get("pending_constraint") or state.get("pending_dimension")
    state["status"] = _slot_status(kind, specs, slots, kind)
    if len(slots) < len(specs):
        return
    if state.get("pending_constraint"):
        _finish_constraint_slots(state)
    else:
        _finish_dimension_slots(state)


def _finish_constraint_slots(state):
    kind = state["pending_constraint"]
    action = _action_from_slots(state, kind, state["slots"])
    if action is None or not _append_and_solve(state, [action]):
        if action is None:
            state["status"] = "Could not apply " + kind
        state["slots"] = []
        _clear_selection(state)
        return
    state["slots"] = []
    _clear_selection(state)
    state["pending_constraint"] = None
    state["status"] = kind + " solved — click next or Esc"
    state["selection_dirty"] = True


def _finish_dimension_slots(state):
    kind = state["pending_dimension"]
    request = _dimension_from_slots(state, kind, state["slots"])
    if request is None:
        state["status"] = "Could not read " + kind
        state["slots"] = []
        _clear_selection(state)
        return
    state["dimension"] = request
    state["pending_dimension"] = None
    state["slots"] = []
    state["status"] = "Enter " + kind + " dimension"


def _action_from_slots(state, kind, slots):
    points = [p for p in (_hit_as_point(state, hit) for hit in slots) if p is not None]
    curves = [c for c in (_hit_curve_index(state, hit) for hit in slots) if c is not None]
    if kind in ("horizontal", "vertical"):
        if len(curves) == 1 and _curve_kind(state, curves[0]) == "line":
            return {"kind": kind, "curves": curves}
    elif kind == "coincident":
        if len(points) == 2 and not (
                _is_free_xy(points[0]) and _is_free_xy(points[1])):
            return {"kind": kind, "points": points}
    elif kind in ("parallel", "perpendicular"):
        if len(curves) == 2 and all(_curve_kind(state, i) == "line" for i in curves):
            return {"kind": kind, "curves": curves}
    elif kind == "tangent":
        line = [i for i in curves if _curve_kind(state, i) == "line"]
        circular = [i for i in curves if _curve_kind(state, i) in ("circle", "arc")]
        if len(line) == 1 and len(circular) == 1:
            return {"kind": kind, "curves": [line[0], circular[0]]}
    elif kind == "equal":
        if len(curves) == 2:
            kinds = [_curve_kind(state, i) for i in curves]
            if all(k == "line" for k in kinds) or all(k in ("circle", "arc") for k in kinds):
                return {"kind": kind, "curves": curves}
    elif kind == "midpoint":
        lines = [i for i in curves if _curve_kind(state, i) == "line"]
        if len(points) == 1 and len(lines) == 1:
            return {"kind": kind, "points": points, "curves": lines}
    elif kind == "concentric":
        if len(curves) == 2 and all(_curve_kind(state, i) in ("circle", "arc") for i in curves):
            return {"kind": kind, "curves": curves}
    elif kind == "fix":
        if len(points) == 1 and not _is_free_xy(points[0]):
            return {"kind": kind, "points": points}
    return None


def _is_free_xy(point):
    return (
        point is not None
        and not isinstance(point, str)
        and hasattr(point, "__len__")
        and len(point) >= 3
        and point[0] == "xy"
    )


def _dimension_from_slots(state, kind, slots):
    points = [p for p in (_hit_as_point(state, hit) for hit in slots) if p is not None]
    curves = [c for c in (_hit_curve_index(state, hit) for hit in slots) if c is not None]
    value = None
    if kind == "length" and len(curves) == 1 and _curve_kind(state, curves[0]) == "line":
        action = _curve_action(state, curves[0])
        if action is None:
            return None
        value = _dist(action["p0"], action["p1"])
    elif kind == "radius" and len(curves) == 1 and _curve_kind(state, curves[0]) in ("circle", "arc"):
        action = _curve_action(state, curves[0])
        if action is None:
            return None
        value = action["radius"] if action["kind"] == "circle" else _arc_radius(action)
    elif kind == "distance" and len(points) == 2:
        value = _dist(_point_position(state, points[0]), _point_position(state, points[1]))
    elif kind == "angle" and len(curves) == 2 and all(_curve_kind(state, i) == "line" for i in curves):
        a = _curve_action(state, curves[0])
        b = _curve_action(state, curves[1])
        if a is None or b is None:
            return None
        da = math.atan2(a["p1"][1] - a["p0"][1], a["p1"][0] - a["p0"][0])
        db = math.atan2(b["p1"][1] - b["p0"][1], b["p1"][0] - b["p0"][0])
        value = abs(math.degrees(db - da)) % 180.0
    if value is None:
        return None
    return {
        "kind": kind,
        "curves": curves,
        "points": points,
        "value": float(value),
    }


def _constraint_hint(kind):
    spec, specs = None, _CONSTRAINT_SLOTS.get(kind)
    if specs:
        return _slot_status(kind, specs, [], kind)
    return "Select entities for " + kind


def _dimension_hint(kind):
    specs = _DIMENSION_SLOTS.get(kind)
    if specs:
        return _slot_status(kind, specs, [], kind)
    return "Select entities for " + kind


def _commit_dimension(state):
    request = state["dimension"]
    if request is None or request["value"] <= 0.0:
        state["status"] = "Dimension must be positive"
        return
    edit_index = request.get("edit_index")
    if edit_index is not None:
        ok = _update_dimension(state, int(edit_index), float(request["value"]))
        state["dimension"] = None
        state["selected"] = []
        state["selection_dirty"] = True
        if ok:
            state["status"] = request["kind"] + " updated"
        return
    action = {
        "kind": request["kind"],
        "curves": list(request["curves"]),
        "points": list(request["points"]),
        "value": float(request["value"]),
    }
    ok = _append_and_solve(state, [action])
    state["dimension"] = None
    state["selected"] = []
    state["selection_dirty"] = True
    if ok:
        state["status"] = action["kind"] + " solved"


def _begin_dimension_edit(state, action_index):
    actions = state.get("actions") or []
    if action_index < 0 or action_index >= len(actions):
        return False
    action = actions[action_index]
    if not _cons_viz.is_dimension_action(action):
        return False
    state["tool"] = "select"
    state["pending"] = []
    state["pending_constraint"] = None
    state["pending_dimension"] = None
    state["slots"] = []
    state["dimension"] = {
        "kind": action["kind"],
        "curves": list(action.get("curves") or []),
        "points": list(action.get("points") or []),
        "value": float(action.get("value") or 0.0),
        "edit_index": action_index,
    }
    state["dimension_focus"] = True
    state["escape_armed"] = False
    state["status"] = "Edit " + action["kind"] + " dimension"
    return True


def _try_edit_constraint(ps, imgui, state, viz, ray, hover):
    if state.get("pending_constraint") or state.get("pending_dimension") or state.get("slots"):
        return False
    action_index = _hit_dimension_action(ps, imgui, viz, ray, hover)
    if action_index < 0:
        return False
    return _begin_dimension_edit(state, action_index)


def _hit_constraint_action(ps, imgui, viz, ray, hover):
    return _hit_dimension_action(ps, imgui, viz, ray, hover)


def _hit_dimension_action(ps, imgui, viz, ray, hover):
    labels = viz.get("cons_labels") or []
    from .view import _mouse_pos
    screen = _mouse_pos(imgui.GetIO())
    if screen is not None and labels:
        best = -1
        best_d = None
        for text, world, kind, action_index in labels:
            if kind != _cons_viz.KIND_DIM:
                continue
            projected = _world_to_screen(ps, imgui, world)
            if projected is None:
                continue
            tw, th = _label_text_size(imgui, text)
            dx = abs(screen[0] - projected[0])
            dy = abs(screen[1] - projected[1])
            if dx <= tw * 0.5 + 10.0 and dy <= th * 0.5 + 10.0:
                d = dx * dx + dy * dy
                if best_d is None or d < best_d:
                    best_d = d
                    best = action_index
        if best >= 0:
            return best
    overlay = viz.get("cons_overlay")
    if overlay is None:
        return -1
    uv = hover
    if uv is None:
        return -1
    return _cons_viz.nearest_dimension(overlay, uv)


def _update_dimension(state, action_index, value):
    actions = state.get("actions") or []
    if action_index < 0 or action_index >= len(actions):
        state["status"] = "Could not update dimension"
        return False
    action = actions[action_index]
    if not _cons_viz.is_dimension_action(action):
        state["status"] = "Could not update dimension"
        return False
    action["value"] = float(value)
    live = state.get("sketch")
    if live is None:
        return True
    batch_index, start = _batch_covering(state, action_index)
    if batch_index is None:
        state["status"] = "Updated record; live sketch kept"
        return True
    tail = state["batches"][batch_index:]
    for batch in reversed(tail):
        if not _revert_live_batch(live, batch):
            state["status"] = "Could not update live dimension"
            return False
    cursor = start
    try:
        for batch in tail:
            n = len(batch.get("actions") or ())
            before = _sketch_counts(live)
            apply_actions(live, state["actions"], start=cursor, end=cursor + n)
            after = _sketch_counts(live)
            if before is not None and after is not None:
                batch["curves"] = after[0] - before[0]
                batch["constraints"] = after[1] - before[1]
            cursor += n
        _adopt_solved(state, commit=True)
        return True
    except Exception as ex:
        state["status"] = "Solve failed: " + str(ex)
        return False


def _batch_covering(state, action_index):
    offset = 0
    for index, batch in enumerate(state.get("batches") or []):
        count = len(batch.get("actions") or ())
        if offset <= action_index < offset + count:
            return index, offset
        offset += count
    return None, None


def _revert_live_batch(live, batch):
    remove_constraint = getattr(live, "remove_last_constraint", None)
    remove_curve = getattr(live, "remove_last_curve", None)
    if remove_constraint is None or remove_curve is None:
        return False
    for _ in range(max(int(batch.get("constraints") or 0), 0)):
        remove_constraint()
    for _ in range(max(int(batch.get("curves") or 0), 0)):
        remove_curve()
    if batch.get("constraints") and getattr(live, "solve", None) is not None:
        live.solve()
    return True


def _undo_pending_or_action(state, rebuild_live=None):
    if state["pending"]:
        state["pending"].pop()
        refs = state.get("pending_refs")
        if refs:
            refs.pop()
        state["status"] = "undid point"
        return
    if state.get("dimension") is not None:
        state["dimension"] = None
        state["status"] = "cancelled dimension"
        return
    if state.get("slots") or state.get("pending_constraint") or state.get("pending_dimension"):
        if state.get("slots"):
            state["slots"] = []
            _clear_selection(state)
            kind = state.get("pending_constraint") or state.get("pending_dimension")
            state["status"] = _slot_status(
                kind,
                (_CONSTRAINT_SLOTS if state.get("pending_constraint") else _DIMENSION_SLOTS).get(kind),
                [],
                kind,
            )
            return
        state["pending_constraint"] = None
        state["pending_dimension"] = None
        _clear_selection(state)
        state["status"] = "cleared constraint"
        return
    batches = state.get("batches") or []
    if not batches:
        return
    batch = batches.pop()
    n = len(batch.get("actions") or ())
    if n:
        del state["actions"][-n:]
    live = state.get("sketch")
    if live is not None and not _revert_live_batch(live, batch):
        state["status"] = "Undid record; live sketch kept"
        _adopt_solved(state, commit=False)
        return
    _adopt_solved(state, commit=False)
    state["status"] = "undid last action"


def _hit_at(state, ray, tolerance, mode=None, exclude=None):
    if ray is None:
        return None
    if mode is None:
        mode = _PICK_ANY
    if tolerance is None or tolerance <= 0.0:
        tolerance = max(state["grid"] * 0.35, 1e-6)
    return _pick.hit(
        state.get("pick_catalog") or (),
        ray[0],
        ray[1],
        tolerance,
        tolerance,
        mode=mode,
        exclude=exclude,
    )


def _select_at(state, name, extend):
    selected = state["selected"]
    accumulating = extend or bool(state.get("pending_constraint") or state.get("pending_dimension"))
    if not name:
        if not accumulating:
            selected[:] = []
        _emit_selection(state)
        if state.get("pending_constraint"):
            state["status"] = _constraint_hint(state["pending_constraint"]) + "  ({0} selected)".format(len(selected))
        elif state.get("pending_dimension"):
            state["status"] = _dimension_hint(state["pending_dimension"]) + "  ({0} selected)".format(len(selected))
        else:
            state["status"] = "Nothing selected"
        return
    if accumulating:
        if name in selected:
            selected.remove(name)
        else:
            selected.append(name)
    else:
        selected[:] = [name]
    state["last_picked_name"] = name
    state["selection_dirty"] = True
    _emit_selection(state)
    if state.get("pending_constraint"):
        state["status"] = _constraint_hint(state["pending_constraint"]) + "  ({0} selected)".format(len(selected))
    elif state.get("pending_dimension"):
        state["status"] = _dimension_hint(state["pending_dimension"]) + "  ({0} selected)".format(len(selected))
    else:
        state["status"] = "Selected {0}".format(name)


def _pickable_points(action):
    return _pick.sketch_handle_uvs(action)


def _all_anchor_uvs(action):
    return [p for _role, p in _pickable_points(action)]


def _point_position(state, point):
    if isinstance(point, str):
        return _named_point_xy(state, point)
    if point[0] == "xy":
        return (float(point[1]), float(point[2]))
    action = _geometry_actions(state)[point[0]]
    for point_index, position in _pickable_points(action):
        if point_index == point[1]:
            return position
    raise ValueError("invalid sketch point reference")


def _named_point_xy(state, name):
    sketch_name = state.get("sketch_name") or ""
    local = _local_entity_name(name, sketch_name)
    qualified = _naming.qualify(sketch_name, local) if sketch_name and ":" not in local else name
    for candidate in (name, local, qualified):
        entry = _catalog_entry(state, candidate)
        if entry is None:
            continue
        ref = entry.get("ref")
        if ref is None:
            continue
        if ref[0] == "xy":
            return (float(ref[1]), float(ref[2]))
        if ref[0] == "point":
            action = _geometry_actions(state)[ref[1]]
            for role, uv in _pickable_points(action):
                if role == ref[2]:
                    return uv
    if _naming.is_origin_name(local):
        return (0.0, 0.0)
    parsed = _naming.parse_sketch_curve_address(local)
    if parsed:
        action = _curve_action(state, parsed["curve"])
        if action is not None:
            role = _role_from_address(action.get("kind"), parsed)
            for handle_role, uv in _pickable_points(action):
                if handle_role == role:
                    return uv
    raise ValueError("invalid sketch point reference")


def _role_from_address(kind, parsed):
    if parsed.get("center"):
        return 2
    uniform = float(parsed.get("uniform") or 0.0)
    if kind == "circle":
        if abs(uniform - 0.0) < 1e-9:
            return 3
        if abs(uniform - 0.25) < 1e-9:
            return 4
        if abs(uniform - 0.5) < 1e-9:
            return 5
        if abs(uniform - 0.75) < 1e-9:
            return 6
        return 3
    if abs(uniform - 0.5) < 1e-9:
        return 3
    if abs(uniform - 1.0) < 1e-9:
        return 1
    return 0


def _arc_radius(action):
    a, b, c = action["start"], action["mid"], action["end"]
    d = 2.0 * (a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1]))
    if abs(d) < 1e-12:
        return max(_dist(a, b), _dist(b, c))
    ux = (
        (a[0] * a[0] + a[1] * a[1]) * (b[1] - c[1])
        + (b[0] * b[0] + b[1] * b[1]) * (c[1] - a[1])
        + (c[0] * c[0] + c[1] * c[1]) * (a[1] - b[1])
    ) / d
    uy = (
        (a[0] * a[0] + a[1] * a[1]) * (c[0] - b[0])
        + (b[0] * b[0] + b[1] * b[1]) * (a[0] - c[0])
        + (c[0] * c[0] + c[1] * c[1]) * (b[0] - a[0])
    ) / d
    return _dist(a, (ux, uy))


def _snap_ref(uv, state):
    hit = state.get("hover_hit")
    if hit:
        entry = _catalog_entry(state, hit)
        if entry is not None and entry.get("kind") == _pick.KIND_POINT:
            ref = entry.get("ref")
            name = _local_entity_name(hit, state.get("sketch_name") or "")
            if ref is not None and ref[0] == "point":
                return _point_position(state, (ref[1], ref[2])), name
            if ref is not None and ref[0] == "xy":
                return (float(ref[1]), float(ref[2])), name
    snapped = _snap(uv, state)
    return snapped, _name_for_uv(state, snapped)


def _name_for_uv(state, uv):
    if uv is None:
        return None
    if abs(float(uv[0])) < 1e-12 and abs(float(uv[1])) < 1e-12:
        return _naming.ORIGIN
    sketch_name = state.get("sketch_name") or ""
    for entry in state.get("pick_catalog") or ():
        if entry.get("kind") != _pick.KIND_POINT:
            continue
        ref = entry.get("ref")
        if ref is None:
            continue
        try:
            if ref[0] == "xy":
                pos = (float(ref[1]), float(ref[2]))
            elif ref[0] == "point":
                pos = _point_position(state, (ref[1], ref[2]))
            else:
                continue
        except Exception:
            continue
        if _dist(uv, pos) < 1e-9:
            return _local_entity_name(entry.get("name") or "", sketch_name)
    return None


def _snap(uv, state):
    x, y = uv
    pts = list(state["pending"])
    pts.append((0.0, 0.0))
    for a in _geometry_actions(state):
        pts.extend(_all_anchor_uvs(a))
    for item in state.get("part_points") or ():
        pts.append(item[0])
    best = None
    best_d = state["grid"] * 0.35
    for p in pts:
        d = _dist(uv, p)
        if d < best_d:
            best_d = d
            best = p
    if best is not None:
        return best
    if state["snap_grid"]:
        g = state["grid"]
        if g > 0:
            x = round(x / g) * g
            y = round(y / g) * g
    return (x, y)


def _pick_tolerance(ps, imgui, state):
    from .view import _world_per_pixel
    wpp = _world_per_pixel(ps, imgui)
    pixel = _pick.PICK_PX * wpp if wpp is not None and wpp > 0.0 else 0.0
    return max(pixel, state["grid"] * 0.2, 1e-6)


def _pick_ray_and_uv(ps, imgui, frame, lift=0.0):
    from .view import camera_ray
    origin, direction = camera_ray(ps, imgui)
    if origin is None or direction is None:
        return None, None
    plane_origin = _uv_to_world((0.0, 0.0), frame, lift)
    normal = _xyz(frame.z)
    denom = _dot(direction, normal)
    if abs(denom) < 1e-12:
        return (origin, direction), None
    delta = (
        plane_origin[0] - origin[0],
        plane_origin[1] - origin[1],
        plane_origin[2] - origin[2],
    )
    t = _dot(delta, normal) / denom
    hit = (
        origin[0] + direction[0] * t,
        origin[1] + direction[1] * t,
        origin[2] + direction[2] * t,
    )
    return (origin, direction), _world_to_uv(hit, frame)


def _world_to_uv(p, frame):
    return _pick.project_to_uv(p, frame)


def _uv_to_world(uv, frame, lift=0.0):
    return _pick.uv_to_world(uv, frame, lift)


def _register_context(ps, scene, names):
    if scene is None:
        return
    ghost = (0.55, 0.55, 0.58)
    for i, patch in enumerate(scene.patches):
        name = "__ctx_s{0}".format(i)
        try:
            m = ps.register_surface_mesh(
                name,
                patch["vertices"],
                patch["faces"],
                smooth_shade=True,
                color=ghost,
                transparency=0.85,
                back_face_policy="identical",
            )
            try:
                m.set_enabled(True)
            except Exception:
                pass
            names.append(name)
        except Exception:
            pass


def _look_at_plane(ps, frame, extent):
    e = max(extent, 1.0)
    try:
        from .view import _cad_commit

        origin = _xyz(frame.origin)
        normal = _xyz(frame.z)
        up = _xyz(frame.y)
        eye = (
            origin[0] + normal[0] * e * 2.5,
            origin[1] + normal[1] * e * 2.5,
            origin[2] + normal[2] * e * 2.5,
        )
        _cad_commit(ps, eye, origin, up)
        cam = getattr(ps, "camera", None)
        if cam is not None:
            cam.zoom = max(e * 0.7, 1e-3)
            clip = getattr(cam, "_clip", None)
            if callable(clip):
                clip(e, e * 2.5)
    except Exception:
        pass


def _register_plane(ps, frame, bounds, lift, viz):
    u0, u1, v0, v1 = bounds
    corners_uv = [(u0, v0), (u1, v0), (u1, v1), (u0, v1)]
    verts = [_uv_to_world(c, frame, lift) for c in corners_uv]
    mesh = ps.register_surface_mesh(
        "__sketch_plane",
        verts,
        [(0, 1, 2), (0, 2, 3)],
        color=_PLANE_COLOR,
        transparency=_PLANE_ALPHA,
        smooth_shade=False,
        back_face_policy="identical",
    )
    try:
        mesh.set_selection_mode("faces_only")
    except Exception:
        pass
    viz["x_nodes"] = [verts[0], verts[1], verts[3], verts[2]]
    viz["x_edges"] = [(0, 1), (2, 3)]
    viz["y_nodes"] = [verts[0], verts[3], verts[1], verts[2]]
    viz["y_edges"] = [(0, 1), (2, 3)]
    origin_lift = viz.get("point_lift", lift)
    viz["origin"] = [_uv_to_world((0, 0), frame, origin_lift)]


def _refresh_overlays(ps, state, frame, viz):
    _rebuild_pick_catalog(state, viz)
    _refresh_committed(ps, state, frame, viz)
    _refresh_selection(ps, state, frame, viz)
    _refresh_constraints(ps, state, frame, viz)


def _refresh_constraints(ps, state, frame, viz):
    overlay = _cons_viz.build_constraint_overlay(state.get("actions"), _geometry_actions(state))
    lift = viz["lift"]
    cons_nodes, cons_edges = _segs_to_nodes_edges(overlay.cons_segs, frame, lift)
    dim_nodes, dim_edges = _segs_to_nodes_edges(overlay.dim_segs, frame, lift)
    arrows = []
    for tip_uv, inward_uv in overlay.arrows:
        tip = _uv_to_world(tip_uv, frame, lift)
        inward = _uv_dir_to_world(inward_uv, frame)
        arrows.append((tip, inward))
    labels = []
    for text, uv, kind, action_index in overlay.labels:
        labels.append((text, _uv_to_world(uv, frame, lift), kind, action_index))
    viz["cons_nodes"] = cons_nodes
    viz["cons_edges"] = cons_edges
    viz["dim_nodes"] = dim_nodes
    viz["dim_edges"] = dim_edges
    viz["dim_arrows"] = arrows
    viz["cons_labels"] = labels
    viz["cons_overlay"] = overlay
    viz["cam"] = None


def _segs_to_nodes_edges(segs, frame, lift):
    nodes = []
    edges = []
    for a, b in segs:
        base = len(nodes)
        nodes.append(_uv_to_world(a, frame, lift))
        nodes.append(_uv_to_world(b, frame, lift))
        edges.append((base, base + 1))
    return nodes, edges


def _uv_dir_to_world(direction, frame):
    x_axis = _xyz(frame.x)
    y_axis = _xyz(frame.y)
    return (
        direction[0] * x_axis[0] + direction[1] * y_axis[0],
        direction[0] * x_axis[1] + direction[1] * y_axis[1],
        direction[0] * x_axis[2] + direction[1] * y_axis[2],
    )


def _refresh_committed(ps, state, frame, viz):
    skip_curves, skip_points = _selection_sets(state)
    nodes, edges, points, build_nodes, build_edges = _sketch_polylines(
        state, frame, viz["lift"], viz["point_lift"], skip_curves, skip_points)
    viz["geom_nodes"] = nodes
    viz["geom_edges"] = edges
    viz["geom_points"] = points
    viz["build_nodes"] = build_nodes
    viz["build_edges"] = build_edges
    viz["cam"] = None


def _refresh_preview(ps, state, frame, viz, hover=None):
    lift = viz["lift"]
    point_lift = viz["point_lift"]
    pending = list(state["pending"])
    tool = state["tool"]
    drawing = tool in _TOOLS
    if drawing and hover is not None:
        hover = _snap(hover, state) if state["snap_grid"] or pending else hover
        pending = pending + [hover]
    nodes, edges = _preview_polylines_from(pending, tool, frame, lift)
    points = [_uv_to_world(p, frame, point_lift) for p in pending]
    hit = state.get("hover_hit")
    if hit is not None and not drawing:
        hover_nodes, hover_edges, hover_points = _hit_overlay(state, hit, frame, lift, point_lift)
        nodes.extend(hover_nodes)
        edges.extend(hover_edges)
        points.extend(hover_points)
    viz["preview_points"] = points
    viz["preview_nodes"] = nodes
    viz["preview_edges"] = edges
    viz["cam"] = None


def _hit_overlay(state, hit, frame, lift, point_lift):
    nodes = []
    edges = []
    points = []
    entry = _catalog_entry(state, hit)
    if entry is None:
        return nodes, edges, points
    ref = entry.get("ref")
    if ref is None:
        return nodes, edges, points
    if ref[0] == "point":
        points.append(_uv_to_world(
            _point_position(state, (ref[1], ref[2])), frame, point_lift))
    elif ref[0] == "xy":
        if entry.get("kind") == _pick.KIND_POINT and entry.get("world") is not None:
            if _is_sketch_origin_name(hit, state.get("sketch_name") or ""):
                points.append(_uv_to_world((ref[1], ref[2]), frame, point_lift))
    elif ref[0] == "curve":
        curves = _geometry_actions(state)
        index = ref[1]
        if 0 <= index < len(curves):
            world_pts = _action_world_pts(curves[index], frame, lift)
            if len(world_pts) >= 2:
                nodes.extend(world_pts)
                edges.extend((i, i + 1) for i in range(len(world_pts) - 1))
                if curves[index]["kind"] == "circle":
                    edges.append((len(world_pts) - 1, 0))
    return nodes, edges, points


def _is_sketch_origin_name(name, sketch_name):
    return _naming.is_origin_name(name)


def _preview_polylines(state, frame, lift):
    return _preview_polylines_from(state["pending"], state["tool"], frame, lift)


def _preview_polylines_from(pending, tool, frame, lift):
    if tool == "circle" and len(pending) >= 2:
        r = _dist(pending[0], pending[1])
        if r > 1e-9:
            pts = _circle_pts(pending[0], r, frame, lift)
            return pts, [(i, (i + 1) % len(pts)) for i in range(len(pts))]
    if tool == "arc" and len(pending) >= 3:
        start, through, end = _arc_click_points(pending)
        pts = _arc_pts(start, through, end, frame, lift)
        return pts, [(i, i + 1) for i in range(len(pts) - 1)]
    if tool == "rectangle" and len(pending) >= 2:
        x0, y0 = pending[0]
        x1, y1 = pending[1]
        corners = [
            _uv_to_world((x0, y0), frame, lift),
            _uv_to_world((x1, y0), frame, lift),
            _uv_to_world((x1, y1), frame, lift),
            _uv_to_world((x0, y1), frame, lift),
        ]
        return corners, [(0, 1), (1, 2), (2, 3), (3, 0)]
    pts = [_uv_to_world(p, frame, lift) for p in pending]
    return pts, [(i, i + 1) for i in range(len(pts) - 1)]


def _draw_construction_toggle(imgui, state):
    indices = _selected_curve_indices(state)
    if not indices:
        return
    imgui.Separator()
    try:
        imgui.TextUnformatted("Selection")
    except Exception:
        pass
    on = _curves_are_construction(state, indices)
    try:
        changed, value = imgui.Checkbox("Construction", on)
    except Exception:
        changed = False
        value = on
        if imgui.Button("Construction: " + ("on" if on else "off")):
            changed = True
            value = not on
    if changed and bool(value) != on:
        _set_curves_construction(state, indices, bool(value))


def _selected_curve_indices(state):
    curves, points = _selection_sets(state)
    indices = set(curves)
    for curve_index, _role in points:
        indices.add(curve_index)
    geoms = _geometry_actions(state)
    return [index for index in sorted(indices) if 0 <= index < len(geoms)]


def _curves_are_construction(state, indices):
    geoms = _geometry_actions(state)
    return all(bool(geoms[index].get("construction")) for index in indices)


def _set_curves_construction(state, indices, construction):
    stores = [_recorded_geometry(state["actions"])]
    if state.get("solved"):
        stores.append(state["solved"])
    live = state.get("sketch")
    for index in indices:
        for store in stores:
            if 0 <= index < len(store):
                if construction:
                    store[index]["construction"] = True
                else:
                    store[index].pop("construction", None)
        if live is not None:
            setter = getattr(live, "set_construction", None)
            if setter is not None:
                try:
                    setter(index, construction)
                except Exception:
                    pass
    state["selection_dirty"] = True
    state["status"] = "Construction" if construction else "Sketch geometry"


def _selection_sets(state):
    curves = set()
    points = set()
    for selected in state["selected"]:
        ref = _catalog_ref(state, selected)
        if ref is None:
            continue
        if ref[0] == "curve":
            curves.add(ref[1])
        elif ref[0] == "point":
            points.add((ref[1], ref[2]))
    return curves, points


def _refresh_selection(ps, state, frame, viz):
    lift = viz["lift"]
    point_lift = viz["point_lift"]
    nodes = []
    edges = []
    points = []
    curves = _geometry_actions(state)
    drawn = set()
    sketch_name = state.get("sketch_name") or ""
    for selected in state["selected"]:
        ref = _catalog_ref(state, selected)
        if ref is None:
            continue
        if ref[0] == "point":
            points.append(_uv_to_world(_point_position(state, (ref[1], ref[2])), frame, point_lift))
            continue
        if ref[0] == "xy":
            if _is_sketch_origin_name(selected, sketch_name):
                points.append(_uv_to_world((ref[1], ref[2]), frame, point_lift))
            continue
        index = ref[1]
        if index in drawn or index < 0 or index >= len(curves):
            continue
        drawn.add(index)
        action = curves[index]
        world_pts = _action_world_pts(action, frame, lift)
        if len(world_pts) < 2:
            continue
        base = len(nodes)
        nodes.extend(world_pts)
        edges.extend((base + i, base + i + 1) for i in range(len(world_pts) - 1))
        if action["kind"] == "circle":
            edges.append((base + len(world_pts) - 1, base))
        for uv in _all_anchor_uvs(action):
            points.append(_uv_to_world(uv, frame, point_lift))
    viz["sel_nodes"] = nodes
    viz["sel_edges"] = edges
    viz["sel_points"] = points
    viz["cam"] = None


def _sketch_polylines(state, frame, lift, point_lift, skip_curves=None, skip_points=None):
    skip_curves = skip_curves or set()
    skip_points = skip_points or set()
    nodes = []
    edges = []
    build_nodes = []
    build_edges = []
    points = []
    for index, action in enumerate(_geometry_actions(state)):
        selected_curve = index in skip_curves
        if not selected_curve:
            world_pts = _action_world_pts(action, frame, lift)
            if len(world_pts) >= 2:
                if action.get("construction"):
                    _append_dashed_polyline(build_nodes, build_edges, world_pts, action["kind"] == "circle")
                else:
                    base = len(nodes)
                    nodes.extend(world_pts)
                    edges.extend((base + i, base + i + 1) for i in range(len(world_pts) - 1))
                    if action["kind"] == "circle":
                        edges.append((base + len(world_pts) - 1, base))
        for role, uv in _pickable_points(action):
            if (index, role) in skip_points or selected_curve:
                continue
            points.append(_uv_to_world(uv, frame, point_lift))
    return nodes, edges, points, build_nodes, build_edges


def _append_dashed_polyline(nodes, edges, world_pts, closed):
    segs = list(zip(world_pts, world_pts[1:]))
    if closed and len(world_pts) > 2:
        segs.append((world_pts[-1], world_pts[0]))
    for a, b in segs:
        for k in (0.0, 0.5):
            t0 = k
            t1 = k + 0.22
            i = len(nodes)
            nodes.append(_lerp3(a, b, t0))
            nodes.append(_lerp3(a, b, t1))
            edges.append((i, i + 1))


def _lerp3(a, b, t):
    return (
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    )


def _action_world_pts(action, frame, lift):
    return _pick.sketch_curve_world_pts(action, frame, lift)


def _circle_pts(center, radius, frame, lift=0.0, n=48):
    return _pick.circle_polyline(center, radius, frame, lift, n)


def _arc_pts(start, mid, end, frame, lift=0.0, n=32):
    return _pick.arc_polyline(start, mid, end, frame, lift, n)


def _tick_sketch_draw(
        ps, imgui, viz, cad_frame, is_ortho, world_per_pixel,
        curve_pick_verts, diamond_batch_verts, rect_batch_verts, quad_batch_faces):
    wpp = world_per_pixel(ps, imgui)
    if wpp is None or wpp <= 0.0:
        return
    pos, center, look, up, right = cad_frame(ps)
    if look is None or up is None or right is None:
        return
    from .view import (
        _EDGE_PX_ORTHO, _EDGE_PX_PERSPECTIVE, _POINT_PX_ORTHO, _POINT_PX_PERSPECTIVE, _vmul)
    edge_r = wpp * (_EDGE_PX_ORTHO if is_ortho(ps) else _EDGE_PX_PERSPECTIVE)
    point_r = wpp * (_POINT_PX_ORTHO if is_ortho(ps) else _POINT_PX_PERSPECTIVE)
    # Body overlays already pull ~3px toward the camera. Sketch used a tiny fixed
    # plane-normal epsilon, so at typical zoom the transparent plane and body
    # edge quads occupied the same depth slab and z-fought through the stroke.
    geom_pull = _vmul(look, -max(8.0 * wpp, 1e-9))
    preview_pull = _vmul(look, -max(9.0 * wpp, 1e-9))
    sel_pull = _vmul(look, -max(12.0 * wpp, 1e-9))
    cons_pull = _vmul(look, -max(11.0 * wpp, 1e-9))
    axis_pull = _vmul(look, -max(6.0 * wpp, 1e-9))
    point_pull = _vmul(look, -max(10.0 * wpp, 1e-9))
    sel_point_pull = _vmul(look, -max(14.0 * wpp, 1e-9))
    sel_edge = edge_r * 1.8
    cons_edge = edge_r * 1.15
    dim_edge = edge_r * 0.95
    sel_point = point_r * 1.25
    geom_point = point_r * 1.15
    cam = (look, up, right, edge_r, point_r, sel_edge, wpp)
    if viz.get("cam") != cam or not viz.get("meshes_ready"):
        viz["cam"] = cam
        layers = (
            ("x", viz.get("x_nodes") or [], viz.get("x_edges") or [], (0.90, 0.12, 0.10), edge_r, axis_pull),
            ("y", viz.get("y_nodes") or [], viz.get("y_edges") or [], (0.10, 0.72, 0.22), edge_r, axis_pull),
            ("geom", viz.get("geom_nodes") or [], viz.get("geom_edges") or [], _CURVE_BLUE, edge_r, geom_pull),
            ("build", viz.get("build_nodes") or [], viz.get("build_edges") or [], _CONSTRUCTION_GRAY, edge_r, geom_pull),
            ("preview", viz.get("preview_nodes") or [], viz.get("preview_edges") or [], _PREVIEW_BLUE, edge_r, preview_pull),
            ("cons", viz.get("cons_nodes") or [], viz.get("cons_edges") or [], _CONSTRAINT_PURPLE, cons_edge, cons_pull),
            ("dim", viz.get("dim_nodes") or [], viz.get("dim_edges") or [], _DIMENSION_ORANGE, dim_edge, cons_pull),
            ("sel", viz.get("sel_nodes") or [], viz.get("sel_edges") or [], _SELECT_ORANGE, sel_edge, sel_pull),
        )
        for key, nodes, edges, color, half, pull in layers:
            _sync_edge_mesh(
                ps, viz, key, nodes, edges, color, look, up, half, pull,
                curve_pick_verts, quad_batch_faces,
            )
        _sync_point_mesh(
            ps, viz, "origin", viz.get("origin") or [], _CURVE_BLUE,
            right, up, point_r, point_pull, rect_batch_verts, quad_batch_faces,
        )
        _sync_point_mesh(
            ps, viz, "geom_pts", viz.get("geom_points") or [], _CURVE_BLUE,
            right, up, geom_point, point_pull, diamond_batch_verts, quad_batch_faces,
        )
        _sync_point_mesh(
            ps, viz, "preview_pts", viz.get("preview_points") or [], _PREVIEW_BLUE,
            right, up, point_r, point_pull, diamond_batch_verts, quad_batch_faces,
        )
        _sync_point_mesh(
            ps, viz, "sel_pts", viz.get("sel_points") or [], _SELECT_ORANGE,
            right, up, sel_point, sel_point_pull, diamond_batch_verts, quad_batch_faces,
        )
        _sync_arrow_mesh(
            ps, viz, viz.get("dim_arrows") or [], look, wpp * _ARROW_PX_LEN,
            wpp * _ARROW_PX_WIDTH, cons_pull,
        )
        viz["meshes_ready"] = True
    _draw_constraint_labels(ps, imgui, viz)


def _sync_arrow_mesh(ps, viz, arrows, look, length, width, pull):
    name = "__sk_tri_dim_arrows"
    mesh_key = name + "_mesh"
    count_key = name + "_n"
    verts, faces = _arrow_batch_verts(arrows, look, length, width, pull)
    if not verts:
        _remove_mesh(ps, name)
        viz[mesh_key] = None
        viz[count_key] = 0
        return
    mesh = viz.get(mesh_key)
    if mesh is None or viz.get(count_key) != len(faces):
        _remove_mesh(ps, name)
        viz[mesh_key] = _register_flat_mesh(ps, name, verts, faces, _DIMENSION_ORANGE)
        viz[count_key] = len(faces)
        return
    try:
        mesh.update_vertex_positions(verts)
    except Exception:
        _remove_mesh(ps, name)
        viz[mesh_key] = _register_flat_mesh(ps, name, verts, faces, _DIMENSION_ORANGE)


def _arrow_batch_verts(arrows, look, length, width, pull):
    from .view import _vadd, _vmul, _vsub, _cross, _vlen
    verts = []
    faces = []
    half_w = width * 0.5
    for tip, inward in arrows:
        tip = _vadd(tip, pull)
        inward_len = _vlen(inward)
        if inward_len < 1e-12:
            continue
        inward = _vmul(inward, 1.0 / inward_len)
        perp = _cross(inward, look)
        perp_len = _vlen(perp)
        if perp_len < 1e-12:
            perp = _cross(inward, (0.0, 0.0, 1.0))
            perp_len = _vlen(perp)
        if perp_len < 1e-12:
            continue
        perp = _vmul(perp, half_w / perp_len)
        base = _vadd(tip, _vmul(inward, length))
        origin = len(verts)
        verts.append(tip)
        verts.append(_vadd(base, perp))
        verts.append(_vsub(base, perp))
        faces.append((origin, origin + 1, origin + 2))
    return verts, faces


def _label_font_size(imgui):
    size = 13.0
    getter = getattr(imgui, "GetFontSize", None) or getattr(imgui, "get_font_size", None)
    if getter is not None:
        try:
            size = float(getter())
        except Exception:
            pass
    return size * _LABEL_SCALE


def _label_text_size(imgui, text):
    font_size = _label_font_size(imgui)
    font = getattr(imgui, "GetFont", None) or getattr(imgui, "get_font", None)
    font = font() if font is not None else None
    if font is not None:
        try:
            measured = font.CalcTextSizeA(font_size, 1.0e9, 0.0, text)
            if hasattr(measured, "__len__"):
                return float(measured[0]), float(measured[1])
            return float(measured.x), float(measured.y)
        except Exception:
            pass
    tw, th = _icons._text_size(imgui, text)
    return tw * _LABEL_SCALE, th * _LABEL_SCALE


def _draw_constraint_labels(ps, imgui, viz):
    labels = viz.get("cons_labels") or []
    if not labels:
        return
    colors = {
        _cons_viz.KIND_CONS: _CONSTRAINT_PURPLE,
        _cons_viz.KIND_DIM: _DIMENSION_ORANGE,
    }
    dl = _foreground_draw_list(imgui)
    font = None
    getter = getattr(imgui, "GetFont", None) or getattr(imgui, "get_font", None)
    if getter is not None:
        try:
            font = getter()
        except Exception:
            font = None
    font_size = _label_font_size(imgui)
    for i, (text, world, kind, _action_index) in enumerate(labels):
        screen = _world_to_screen(ps, imgui, world)
        if screen is None:
            continue
        tw, th = _label_text_size(imgui, text)
        pos = (screen[0] - tw * 0.5, screen[1] - th * 0.5)
        rgb = colors.get(kind, _CONSTRAINT_PURPLE)
        col = _icons._u32(imgui, rgb)
        if _draw_scaled_label(dl, font, font_size, pos, col, text):
            continue
        _draw_constraint_label_window(imgui, i, text, pos, rgb)


def _draw_scaled_label(dl, font, font_size, pos, col, text):
    if dl is None:
        return False
    if font is not None:
        try:
            dl.AddText(font, float(font_size), pos, col, text)
            return True
        except TypeError:
            pass
        except Exception:
            pass
    try:
        dl.AddText(pos, col, text)
        return True
    except Exception:
        return False


def _draw_constraint_label_window(imgui, index, text, pos, rgb):
    flags = _constraint_label_flags(imgui)
    try:
        imgui.SetNextWindowPos(pos)
    except Exception:
        return
    try:
        imgui.SetNextWindowBgAlpha(0.0)
    except Exception:
        pass
    opened = False
    try:
        opened = imgui.Begin("##sk_clbl_{0}".format(index), flags)
    except TypeError:
        try:
            opened = imgui.Begin("##sk_clbl_{0}".format(index))
        except Exception:
            return
    if isinstance(opened, tuple):
        opened = opened[0]
    try:
        if not opened:
            return
        try:
            imgui.SetWindowFontScale(_LABEL_SCALE)
        except Exception:
            pass
        pushed = 0
        try:
            imgui.PushStyleColor(imgui.ImGuiCol_Text, (rgb[0], rgb[1], rgb[2], 1.0))
            pushed = 1
        except Exception:
            try:
                imgui.PushStyleColor(0, (rgb[0], rgb[1], rgb[2], 1.0))
                pushed = 1
            except Exception:
                pass
        try:
            imgui.TextUnformatted(text)
        except Exception:
            imgui.Text(text)
        if pushed:
            try:
                imgui.PopStyleColor()
            except Exception:
                pass
        try:
            imgui.SetWindowFontScale(1.0)
        except Exception:
            pass
    finally:
        try:
            imgui.End()
        except Exception:
            pass


def _constraint_label_flags(imgui):
    names = (
        "ImGuiWindowFlags_NoTitleBar",
        "ImGuiWindowFlags_NoResize",
        "ImGuiWindowFlags_NoMove",
        "ImGuiWindowFlags_NoSavedSettings",
        "ImGuiWindowFlags_NoFocusOnAppearing",
        "ImGuiWindowFlags_NoNav",
        "ImGuiWindowFlags_NoDecoration",
        "ImGuiWindowFlags_NoBackground",
        "ImGuiWindowFlags_AlwaysAutoResize",
        "ImGuiWindowFlags_NoInputs",
        "ImGuiWindowFlags_NoMouseInputs",
        "ImGuiWindowFlags_NoScrollbar",
    )
    flags = 0
    for name in names:
        flags |= int(getattr(imgui, name, 0) or 0)
    return flags


def _foreground_draw_list(imgui):
    getter = (
        getattr(imgui, "GetForegroundDrawList", None)
        or getattr(imgui, "get_foreground_draw_list", None)
        or getattr(imgui, "GetBackgroundDrawList", None)
        or getattr(imgui, "get_background_draw_list", None)
    )
    if getter is None:
        return None
    try:
        return getter()
    except Exception:
        return None


def _world_to_screen(ps, imgui, world):
    from .view import (
        _cad_frame, _is_ortho, _pick_viewport, _camera_fov_aspect,
        _vsub, _vlen, _dot,
    )
    pos, center, look, up, right = _cad_frame(ps)
    if pos is None or center is None or look is None or up is None or right is None:
        return None
    width, height = _pick_viewport(ps, imgui)
    if width < 2.0 or height < 2.0:
        return None
    fov, aspect = _camera_fov_aspect(ps, width, height)
    tan_half = math.tan(0.5 * math.radians(max(fov, 1.0)))
    dist = max(_vlen(_vsub(pos, center)), 1e-6)
    if _is_ortho(ps):
        try:
            half_h = 2.0 * tan_half * float(ps.get_length_scale())
        except Exception:
            half_h = dist * tan_half
        half_w = half_h * aspect
        rel = _vsub(world, pos)
        ndc_x = _dot(rel, right) / half_w
        ndc_y = _dot(rel, up) / half_h
    else:
        to_point = _vsub(world, pos)
        denom = _dot(to_point, look)
        if abs(denom) < 1e-12:
            return None
        t = _dot(_vsub(center, pos), look) / denom
        if t <= 1e-9:
            return None
        hit = (
            pos[0] + to_point[0] * t,
            pos[1] + to_point[1] * t,
            pos[2] + to_point[2] * t,
        )
        rel = _vsub(hit, center)
        half_h = dist * tan_half
        half_w = half_h * aspect
        ndc_x = _dot(rel, right) / half_w
        ndc_y = _dot(rel, up) / half_h
    if abs(ndc_x) > 4.0 or abs(ndc_y) > 4.0:
        return None
    return ((ndc_x + 1.0) * 0.5 * width, (1.0 - ndc_y) * 0.5 * height)


def _sync_edge_mesh(
        ps, viz, key, nodes, edges, color, look, up, half, pull,
        curve_pick_verts, quad_batch_faces):
    name = "__sk_edge_" + key
    mesh_key = name + "_mesh"
    count_key = name + "_n"
    if not nodes or not edges:
        _remove_mesh(ps, name)
        viz[mesh_key] = None
        viz[count_key] = 0
        return
    verts = curve_pick_verts(nodes, edges, look, up, half, pull)
    faces = quad_batch_faces(len(edges))
    mesh = viz.get(mesh_key)
    if mesh is None or viz.get(count_key) != len(edges):
        _remove_mesh(ps, name)
        mesh = _register_flat_mesh(ps, name, verts, faces, color)
        viz[mesh_key] = mesh
        viz[count_key] = len(edges)
        return
    try:
        mesh.update_vertex_positions(verts)
    except Exception:
        _remove_mesh(ps, name)
        viz[mesh_key] = _register_flat_mesh(ps, name, verts, faces, color)


def _sync_point_mesh(ps, viz, key, points, color, right, up, half, pull, diamond_batch_verts, quad_batch_faces):
    name = "__sk_pt_" + key
    mesh_key = name + "_mesh"
    count_key = name + "_n"
    if not points:
        _remove_mesh(ps, name)
        viz[mesh_key] = None
        viz[count_key] = 0
        return
    verts = diamond_batch_verts(points, right, up, half, pull)
    faces = quad_batch_faces(len(points))
    mesh = viz.get(mesh_key)
    if mesh is None or viz.get(count_key) != len(points):
        _remove_mesh(ps, name)
        viz[mesh_key] = _register_flat_mesh(ps, name, verts, faces, color)
        viz[count_key] = len(points)
        return
    try:
        mesh.update_vertex_positions(verts)
    except Exception:
        _remove_mesh(ps, name)
        viz[mesh_key] = _register_flat_mesh(ps, name, verts, faces, color)


def _register_flat_mesh(ps, name, verts, faces, color):
    mesh = ps.register_surface_mesh(
        name, verts, faces, color=color, material="flat",
        smooth_shade=False, back_face_policy="identical",
    )
    try:
        mesh.set_smooth_shade(False)
        mesh.set_selection_mode("faces_only")
    except Exception:
        pass
    return mesh


def _remove_mesh(ps, name):
    try:
        ps.remove_surface_mesh(name, error_if_absent=False)
    except Exception:
        try:
            ps.remove_surface_mesh(name)
        except Exception:
            pass


def _remove_network(ps, name):
    remover = getattr(ps, "remove_curve_network", None)
    if remover is None:
        return
    try:
        remover(name, error_if_absent=False)
    except TypeError:
        try:
            remover(name)
        except Exception:
            pass
    except Exception:
        pass


def _principal_frame(plane):
    from .api import Frame

    if plane == "yz":
        return Frame(origin=(0, 0, 0), x=(0, 1, 0), y=(0, 0, 1), z=(1, 0, 0))
    if plane == "zx":
        return Frame(origin=(0, 0, 0), x=(0, 0, 1), y=(1, 0, 0), z=(0, 1, 0))
    return Frame()


def _xyz(v):
    try:
        return (float(v.x), float(v.y), float(v.z))
    except AttributeError:
        return (float(v[0]), float(v[1]), float(v[2]))


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _plane_bounds(scene, plane, frame, default=20.0):
    if scene is not None:
        for patch in scene.patches:
            if patch.get("name") != plane:
                continue
            points = [_world_to_uv(v, frame) for v in patch.get("vertices", ())]
            if points:
                u0 = min(p[0] for p in points)
                u1 = max(p[0] for p in points)
                v0 = min(p[1] for p in points)
                v1 = max(p[1] for p in points)
                width = max(u1 - u0, 1e-6)
                height = max(v1 - v0, 1e-6)
                margin = 0.08 * max(width, height)
                return u0 - margin, u1 + margin, v0 - margin, v1 + margin
    e = max(float(default), 1.0)
    return -e, e, -e, e


def _actions_bounds(actions, fallback):
    pts = []
    for action in actions or []:
        kind = action.get("kind")
        if kind == "line":
            pts.extend((action.get("p0"), action.get("p1")))
        elif kind == "circle":
            center = action.get("center")
            radius = float(action.get("radius") or 0.0)
            if center is not None:
                pts.append((center[0] - radius, center[1] - radius))
                pts.append((center[0] + radius, center[1] + radius))
        elif kind == "arc":
            pts.extend((action.get("start"), action.get("mid"), action.get("end")))
        elif kind == "rectangle":
            pts.extend((action.get("p0"), action.get("p1")))
    pts = [p for p in pts if p is not None and len(p) >= 2]
    if not pts:
        return fallback
    u0 = min(float(p[0]) for p in pts)
    u1 = max(float(p[0]) for p in pts)
    v0 = min(float(p[1]) for p in pts)
    v1 = max(float(p[1]) for p in pts)
    span = max(u1 - u0, v1 - v0, 1.0)
    margin = 0.15 * span
    return u0 - margin, u1 + margin, v0 - margin, v1 + margin


def _extent_from_scene(scene, default=20.0):
    if scene is None or not scene.patches:
        return default
    m = 0.0
    for patch in scene.patches:
        for v in patch["vertices"]:
            m = max(m, abs(v[0]), abs(v[1]), abs(v[2]))
    return m if m > 1e-6 else default


def _nice_grid(extent):
    e = max(extent, 1.0)
    g = e / 20.0
    mag = 10 ** round(__import__("math").log10(g)) if g > 0 else 1.0
    return mag


def _dist(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return (dx * dx + dy * dy) ** 0.5
