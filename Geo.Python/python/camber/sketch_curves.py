"""Structured editors for authored sketch curves and engineering profiles.

Display samples never replace a curve's analytical source definition.
"""

from copy import deepcopy
import math


# Field kinds describe user input, rather than Python syntax.
CURVES = {
    "Ellipse": ("add_ellipse", {"center": ("point", (0, 0)), "radii": ("point", (20, 10)), "rotation": ("angle", 0)}),
    "Interpolating spline": ("add_spline", {"points": ("points", [(0, 0), (10, 8), (20, 0)])}),
    "Bezier": ("add_bezier", {"control_points": ("points", [(0, 0), (5, 15), (15, 15), (20, 0)])}),
    "B-spline / NURBS": ("add_bspline", {"control_points": ("points", [(0, 0), (5, 15), (15, 15), (20, 0)]), "degree": ("integer", 3), "weights": ("numbers", None), "knots": ("numbers", None)}),
    "Sampled curve": ("add_sampled_curve", {"points": ("points", [(0, 0), (10, 8), (20, 0)])}),
    "Involute": ("add_involute", {"center": ("point", (0, 0)), "base_radius": ("positive", 15), "t_start": ("angle", 0), "t_end": ("angle", 1), "rotation": ("angle", 0)}),
    "Spur gear": ("add_involute_gear", {"center": ("point", (0, 0)), "module": ("positive", 2), "teeth": ("integer", 20), "pressure_angle": ("angle", math.radians(20)), "addendum": ("positive", 1), "dedendum": ("positive", 1.25)}),
    "Internal gear": ("add_involute_internal_gear", {"center": ("point", (0, 0)), "module": ("positive", 2), "teeth": ("integer", 40), "pressure_angle": ("angle", math.radians(20)), "addendum": ("positive", 1), "dedendum": ("positive", 1.25)}),
    "Airfoil": ("add_naca4", {"code": ("text", "2412"), "leading_edge": ("point", (0, 0)), "chord_length": ("positive", 100), "chord_angle": ("angle", 0), "samples_per_side": ("integer", 36)}),
    "Text outline": ("add_text", {"text": ("text", "CAMBER"), "origin": ("point", (0, 0)), "family": ("text", "Arial"), "em_size": ("positive", 10)}),
}
METHODS = {method for method, _ in CURVES.values()}


def curve_size(action):
    return int(action.get("curve_count", 1)) if action.get("kind") == "curve_call" else 4 if action.get("kind") == "rectangle" else 1


def apply_curve(sketch, action):
    before = sketch.curve_count
    getattr(sketch, action["method"])(**action["arguments"])
    count = sketch.curve_count - before
    if count <= 0:
        raise ValueError("This profile produced no curves")
    action["curve_count"] = count
    action["display"] = sketch._solved_actions()[before:before + count]


def emit_curve(sketch_var, action):
    arguments = ", ".join(key + "=" + repr(value) for key, value in action["arguments"].items())
    return f"{sketch_var}.{action['method']}({arguments})  # curves={curve_size(action)}"


def new_editor(action=None):
    label = next((label for label, (method, _) in CURVES.items()
                  if action and method == action["method"]), "Ellipse")
    return {"label": label, "values": {}, "action": deepcopy(action), "pick": None}


def _number(text):
    value = float(text)
    if not math.isfinite(value):
        raise ValueError("Enter finite numbers")
    return value


def validate(arguments, method):
    """Validate coupled dimensions before calling the native sketch API."""
    if method == "add_ellipse" and min(arguments["radii"]) <= 0:
        raise ValueError("Both ellipse radii must be positive")
    points = arguments.get("points", arguments.get("control_points"))
    if points is not None and (len(points) < 2 or any(a == b for a, b in zip(points, points[1:]))):
        raise ValueError("Use at least two distinct consecutive points")
    if method == "add_bspline":
        n, degree = len(points), arguments["degree"]
        if not 1 <= degree < n:
            raise ValueError("Degree must be between 1 and the number of control points minus 1")
        weights, knots = arguments.get("weights"), arguments.get("knots")
        if weights is not None and (len(weights) != n or min(weights) <= 0):
            raise ValueError("Provide one positive weight per control point")
        if knots is not None and (len(knots) != n + degree + 1 or any(a > b for a, b in zip(knots, knots[1:])) or knots[degree] >= knots[n]):
            raise ValueError("Knots must be increasing, with control point count + degree + 1 entries")
    if method == "add_involute" and not 0 <= arguments["t_start"] < arguments["t_end"]:
        raise ValueError("End angle must exceed the nonnegative start angle")
    if "teeth" in arguments and arguments["teeth"] < 3:
        raise ValueError("Use at least three teeth")
    if "pressure_angle" in arguments and not 0 < arguments["pressure_angle"] < math.pi / 2:
        raise ValueError("Pressure angle must be between 0 and 90 degrees")
    if method == "add_involute_internal_gear" and arguments["teeth"] * (1 - math.cos(arguments["pressure_angle"])) < 2 * arguments["addendum"]:
        raise ValueError("Internal tip is inside the base circle; increase teeth or reduce addendum")
    if method == "add_naca4":
        code = arguments["code"]
        if len(code) != 4 or not code.isdigit() or int(code[2:]) == 0 or (int(code[0]) and int(code[1]) == 0):
            raise ValueError("Enter a valid four-digit airfoil code, e.g. 2412")
        if arguments["samples_per_side"] < 4:
            raise ValueError("Use at least four samples per side")
    if method == "add_text" and not arguments["text"].strip():
        raise ValueError("Enter text for the outline")


def draw_editor(ui, editor):
    """Draw explicit coordinates, point rows and dimensions; return an Apply action."""
    ui.Heading("Curve profile")
    _, label = ui.ComboBox("Curve type##profile", editor["label"], tuple(CURVES), width=max(1, ui.GetContentRegionAvail()[0]))
    if label != editor["label"]:
        editor.update(label=label, values={}, action=None, pick=None)
    method, fields = CURVES[label]
    seed = editor.get("action") or {}
    defaults = seed.get("arguments", {})
    values = editor["values"]
    # Keep existing names, flags and optional API arguments when editing dimensions.
    arguments, errors = deepcopy(defaults), []
    for key, (kind, default) in fields.items():
        default = defaults.get(key, default)
        caption = key.replace("_", " ").capitalize()
        ui.TextUnformatted(caption + (" (degrees)" if kind == "angle" else ""))
        if kind in ("point", "points"):
            rows = values.setdefault(key, [[str(v) for v in point] for point in (default if kind == "points" else [default])])
            parsed = []
            for index, row in enumerate(rows):
                width = max(30, (ui.GetContentRegionAvail()[0] - 8) / 2)
                for axis in range(2):
                    if axis:
                        ui.SameLine()
                    _, row[axis] = ui.InputText(f"{caption} {'XY'[axis]} {index+1}##profile_{key}_{index}_{axis}", row[axis], width=width)
                try:
                    parsed.append(tuple(_number(v) for v in row))
                except ValueError as exc:
                    errors.append(caption + ": " + str(exc))
                if ui.Button(f"Pick {caption.lower()} {index+1}##profile_pick_{key}_{index}"):
                    editor["pick"] = (key, index)
            if kind == "points":
                if ui.Button("Add point##profile_" + key):
                    rows.append(["0", "0"])
                ui.SameLine()
                if ui.Button("Remove last##profile_" + key, enabled=len(rows) > 2):
                    rows.pop()
            arguments[key] = parsed if kind == "points" else (parsed[0] if parsed else (0, 0))
        else:
            initial = "" if default is None else ", ".join(str(v) for v in default) if kind == "numbers" else str(math.degrees(default)) if kind == "angle" else str(default)
            _, values[key] = ui.InputText(caption + "##profile_" + key, values.get(key, initial), width=max(1, ui.GetContentRegionAvail()[0]), hint="Optional, comma-separated numbers" if kind == "numbers" else "")
            try:
                text = values[key]
                value = text if kind == "text" else [_number(v.strip()) for v in text.split(",")] if kind == "numbers" and text.strip() else None if kind == "numbers" else _number(text)
                if kind == "positive" and value <= 0:
                    raise ValueError("Enter a positive value")
                if kind == "integer":
                    if value != int(value):
                        raise ValueError("Enter an integer")
                    value = int(value)
                arguments[key] = math.radians(value) if kind == "angle" else value
            except ValueError as exc:
                errors.append(caption + ": " + str(exc))
    if not errors:
        try:
            validate(arguments, method)
        except ValueError as exc:
            errors.append(str(exc))
    ui.TextMuted("Fixed profile geometry; edit its dimensions here. Solver dimensions apply to lines, arcs and circles.", wrap=True)
    if editor.get("pick"):
        ui.TextMuted("Click a location in the sketch plane.", wrap=True)
    if errors:
        ui.ValidationMessage(errors[0])
    if ui.Button("Apply profile", enabled=not errors, primary=True):
        return {"kind": "curve_call", "method": method, "arguments": arguments}
    return None


def accept_point(editor, uv):
    target = editor.get("pick")
    if target is None:
        return False
    key, index = target
    editor["values"][key][index] = [str(v) for v in uv]
    editor["pick"] = None
    return True
