"""Structured value editors for the source-backed modeling forms."""

import ast
import math
from weakref import WeakKeyDictionary

from .api import Frame
from .modeling_commands import field_value, validate_frame
from .vec import _xyz


def reference_sequence(ui, identity, source, references, *, optional=False):
    """Edit ordered existing-feature references without entering an expression."""
    node = ast.parse(source or "[]", mode="eval").body
    if isinstance(node, ast.Constant) and node.value is None:
        selected = []
    elif isinstance(node, (ast.List, ast.Tuple)) and all(
            isinstance(item, (ast.Name, ast.Subscript, ast.Starred)) and ast.unparse(item) in references for item in node.elts):
        selected = [ast.unparse(item) for item in node.elts]
    else:
        raise ValueError("Select existing features")
    if not references:
        ui.TextMuted("Create compatible features first.", wrap=True)
    elif not selected:
        ui.TextMuted("Select features below, in the required order.", wrap=True)
    for index, name in enumerate(tuple(selected)):
        ui.TextUnformatted(str(index + 1) + ". " + name)
        if ui.Button("Up##" + identity + "/" + name, height=24, enabled=index > 0):
            selected[index - 1], selected[index] = selected[index], selected[index - 1]
        ui.SameLine()
        if ui.Button("Down##" + identity + "/" + name, height=24, enabled=index < len(selected) - 1):
            selected[index], selected[index + 1] = selected[index + 1], selected[index]
    for name in references:
        caption = name[1:] + " · all instances" if name.startswith("*") else name
        changed, checked = ui.Checkbox(caption + "##" + identity + ("/" + name if name.startswith("*") else ""), name in selected)
        if changed:
            if checked:
                selected.append(name)
            else:
                selected.remove(name)
    return "[" + ", ".join(selected) + "]" if selected or not optional else "None"


def loft_settings(ui, identity, source):
    """Present existing loft options as choices, retaining API defaults."""
    node = ast.parse(source or "None", mode="eval").body
    values = {}
    if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == "LoftOptions" and not node.args:
        values = {item.arg: ast.literal_eval(item.value) for item in node.keywords}
    elif not isinstance(node, ast.Constant) or node.value is not None:
        raise ValueError("Select loft settings")
    _, custom = ui.Checkbox("Customize loft##" + identity, isinstance(node, ast.Call))
    if not custom:
        return "None"
    choices = {
        "alignment": {"Default": None, "As authored": "as_authored", "Minimize twist": "minimum_twist", "Origin foot": "origin_foot"},
        "style": {"Default": None, "Ruled": "ruled", "Smooth": "smooth_catmull_rom", "Hermite": "hermite"},
        "correspondence": {"Default": None, "Arc length": "arc_length", "Uniform": "uniform", "Features": "features", "Vertices": "vertices"},
        "crease_policy": {"Default": None, "All profiles": "all_profiles", "First profile": "first_profile", "None": "none"}}
    if any(key not in choices or value not in choices[key].values() for key, value in values.items()):
        raise ValueError("Unsupported loft setting")
    for key, options in choices.items():
        ui.TextUnformatted(key.replace("_", " ").capitalize())
        current = next((label for label, value in options.items() if value == values.get(key)), "Default")
        _, selected = ui.ComboBox(identity + "/" + key, current, tuple(options), width=max(1, ui.GetContentRegionAvail()[0]))
        values[key] = options[selected]
    return "LoftOptions(" + ", ".join(key + "=" + repr(value) for key, value in values.items() if value is not None) + ")"


_LOFT_CURVE_NAMES = WeakKeyDictionary()


def loft_starting_curves(ui, identity, source, sections, *, required=False):
    """Choose one named contour start per selected profile, in loft order."""
    selected = ast.literal_eval(source or "None")
    if selected is not None and (not isinstance(selected, (tuple, list)) or
                                 not all(isinstance(name, str) for name in selected)):
        raise ValueError("Choose a starting curve for each profile")
    if not required:
        _, matched = ui.Checkbox("Match starting curves##"+identity, selected is not None)
        if not matched:
            return "None"
    ui.TextMuted("Choose the corresponding contour start on each profile.", wrap=True)
    result = []
    for index, (label, sketch) in enumerate(sections):
        count = sketch.curve_count
        cached = _LOFT_CURVE_NAMES.get(sketch)
        if cached is None or cached[0] != count:
            names = list(dict.fromkeys(action.get("name") for action in sketch._solved_actions() if action.get("name")))
            _LOFT_CURVE_NAMES[sketch] = count, names
        else:
            names = cached[1]
        if not names:
            ui.ValidationMessage(label + " has no named curves")
            return "[]"
        current = selected[index] if selected is not None and index < len(selected) and selected[index] in names else names[0]
        ui.TextUnformatted(label.replace("_", " ").capitalize())
        _, current = ui.ComboBox("Starting curve##"+identity+"/"+str(index), current, names,
                                  width=max(1, ui.GetContentRegionAvail()[0]))
        result.append(current)
    return repr(result)


class AngleField:
    """Display degrees while keeping the API's radians in the saved model."""

    def __init__(self, source, available):
        self.source = source
        value = field_value(ast.parse(source, mode="eval").body, available) if source.strip() else 0.0
        if type(value) not in (int, float) or not math.isfinite(float(value)):
            raise ValueError("Enter a finite angle")
        self.radians = float(value)

    def draw(self, ui, identity, error=""):
        changed, reported = ui.InputDouble(identity, self.radians, fmt="%.12g", error=error,
                                            multiplier=math.pi / 180)
        if changed:
            self.radians = reported
            self.source = repr(reported)
        return self.source


class VectorFields:
    """Three coordinates, keeping incomplete text separate from committed values."""

    def __init__(self, source, available):
        self.source = source
        self.values = ([repr(v) for v in _xyz(field_value(ast.parse(source,mode="eval").body,available))]
                       if source.strip() else ["", "", ""])

    def draw(self, ui, identity, error=""):
        changed,self.values = ui.InputVector3(identity,self.values,error=error)
        if changed:
            self.source = "("+", ".join(self.values)+")"
        return self.source


class OrientationFields:
    """Assembly quaternion presented as fixed-axis X, then Y, then Z rotations."""

    def __init__(self, source, available):
        self.source = source
        q = field_value(ast.parse(source, mode="eval").body, available)
        if not isinstance(q, (tuple, list)) or len(q) != 4 or any(
                type(v) not in (int, float) or not math.isfinite(v) for v in q):
            raise ValueError("Enter four finite quaternion components")
        scale = max(abs(v) for v in q)
        if scale <= 1e-12:
            raise ValueError("Orientation quaternion cannot be zero")
        length = math.sqrt(sum((v/scale)**2 for v in q))
        x, y, z, w = (v/scale/length for v in q)
        sin_pitch = max(-1, min(1, 2*(w*y-z*x)))
        angles = ((0, math.copysign(math.pi/2, sin_pitch), 2*math.atan2(z,w))
                  if abs(sin_pitch) >= 1-1e-15 else
                  (math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
                   math.asin(sin_pitch), math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))))
        self.values = [format(math.degrees(v), ".12g") for v in angles]

    def error(self):
        try:
            if not all(math.isfinite(float(v)) for v in self.values):
                return "Enter finite rotation angles"
        except (ValueError, OverflowError):
            return "Enter rotation angles in degrees"
        return ""

    def draw(self, ui, identity):
        ui.TextMuted("Rotation · degrees · X, then Y, then Z")
        changed, self.values = ui.InputVector3(identity, self.values, error=self.error())
        error = self.error()
        if error:
            ui.ValidationMessage(error)
        elif changed:
            hx, hy, hz = (math.radians(float(v))/2 for v in self.values)
            sx, sy, sz = math.sin(hx), math.sin(hy), math.sin(hz)
            cx, cy, cz = math.cos(hx), math.cos(hy), math.cos(hz)
            self.source = repr((sx*cy*cz-cx*sy*sz, cx*sy*cz+sx*cy*sz,
                                cx*cy*sz-sx*sy*cz, cx*cy*cz+sx*sy*sz))
        return self.source


class PointListFields:
    """Ordered coordinate rows for profiles and paths, without Python syntax."""

    def __init__(self, source, dimensions):
        self.dimensions = dimensions
        values = ast.literal_eval(source or "[]")
        if values is None:
            values = []
        if not isinstance(values, (list, tuple)) or any(
                not isinstance(row, (list, tuple)) or len(row) != dimensions for row in values):
            raise ValueError("Enter coordinate rows")
        self.rows = [[repr(v) for v in row] for row in values]
        self.source = source

    def draw(self, ui, identity, error=""):
        changed = False
        remove = None
        for index, row in enumerate(self.rows):
            ui.TextMuted("Point " + str(index + 1) + " · " + " / ".join("XYZ"[:self.dimensions]))
            width = max(1, (ui.GetContentRegionAvail()[0] - 8 * (self.dimensions - 1)) / self.dimensions)
            for component, axis in enumerate("XYZ"[:self.dimensions]):
                if component:
                    ui.SameLine()
                edited, row[component] = ui.InputText(identity + "/" + str(index) + "##" + axis,
                                                      row[component], width=width, hint=axis, error=error)
                changed |= edited
            if ui.Button("Remove point##" + identity + "/" + str(index), height=24):
                remove = index
        if remove is not None:
            self.rows.pop(remove)
            changed = True
        if ui.Button("Add point##" + identity, height=26):
            self.rows.append(["0.0"] * self.dimensions)
            changed = True
        if changed:
            self.source = "[" + ", ".join("(" + ", ".join(row) + ")" for row in self.rows) + "]"
        return self.source


class FrameFields:
    """Coordinate drafts retain incomplete input without losing the original pose."""

    def __init__(self, source, available):
        node = ast.parse(source, mode="eval").body
        self.reference = None
        self.reference_name = ""
        base = None
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == "to_global" and len(node.args) == 1 and not node.keywords):
            reference = field_value(node.func.value, available)
            if not isinstance(reference, Frame):
                raise ValueError("Select a reference frame")
            self.reference = ast.unparse(node.func.value)
            base = node.func.value
            node = node.args[0]
        elif (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
              and node.func.attr == "face_frame"):
            if not isinstance(field_value(node, available), Frame):
                raise ValueError("Select a face frame")
            base = node
            self.reference = ast.unparse(node)
            node = ast.parse("Frame()", mode="eval").body
        if (isinstance(base, ast.Call) and isinstance(base.func, ast.Attribute)
                and base.func.attr == "face_frame" and base.args
                and isinstance(base.args[0], ast.Constant) and isinstance(base.args[0].value, str)):
            self.reference_name = base.args[0].value
        frame = field_value(node, available)
        if not isinstance(frame, Frame):
            raise ValueError("Select a frame")
        self.source = source
        self.rows = {key: [repr(v) for v in _xyz(getattr(frame, key))]
                     for key in ("origin", "x", "y", "z")}

    def serialize(self):
        frame = "Frame(" + ", ".join(key+"=("+", ".join(row)+")"
                                     for key, row in self.rows.items()) + ")"
        self.source = self.reference + ".to_global(" + frame + ")" if self.reference else frame
        return self.source

    def errors(self):
        errors, vectors = {}, {}
        for key, row in self.rows.items():
            try:
                # literal_eval rejects expressions, names, and partially typed numbers.
                vectors[key] = tuple(ast.literal_eval(v) for v in row)
                if any(type(v) not in (int, float) for v in vectors[key]):
                    raise ValueError("Enter three numbers")
                if not all(math.isfinite(v) for v in vectors[key]):
                    errors[key] = "Enter three finite numbers"
                elif key != "origin" and abs(sum(v*v for v in vectors[key])-1) > 1e-6:
                    errors[key] = "Direction must have unit length"
            except (SyntaxError, ValueError, TypeError):
                errors[key] = "Enter three numbers"
        if not errors:
            try:
                validate_frame(Frame(**vectors))
            except ValueError as exc:
                errors.update({key: str(exc) for key in ("x", "y", "z")})
        return errors

    def draw(self, ui, identity):
        orientations = {"XY plane · +Z": ((1,0,0),(0,1,0),(0,0,1)),
                        "XY plane · −Z": ((1,0,0),(0,-1,0),(0,0,-1)),
                        "XZ plane · +Y": ((1,0,0),(0,0,-1),(0,1,0)),
                        "XZ plane · −Y": ((1,0,0),(0,0,1),(0,-1,0)),
                        "YZ plane · +X": ((0,1,0),(0,0,1),(1,0,0)),
                        "YZ plane · −X": ((0,1,0),(0,0,-1),(-1,0,0))}
        preset = "Custom"
        for name, axes in orientations.items():
            try:
                if all(tuple(ast.literal_eval(v) for v in self.rows[key]) == axis
                       for key, axis in zip(("x", "y", "z"), axes)):
                    preset = name
            except (SyntaxError, ValueError):
                pass
        errors = self.errors()
        if self.reference:
            ui.TextMuted("Attached to " + (self.reference_name or "reference frame"), wrap=True)
        ui.TextMuted("Offset on attachment · X / Y / Z" if self.reference else "Origin · X / Y / Z")
        changed, self.rows["origin"] = ui.InputVector3(identity+"/origin",self.rows["origin"],error=errors.get("origin", ""))
        ui.TextMuted("Orientation")
        picked, selected = ui.ComboBox(identity+"/orientation", preset, ["Custom"]+list(orientations),
                                         width=max(1, ui.GetContentRegionAvail()[0]))
        if picked and selected in orientations:
            for key, axis in zip(("x", "y", "z"), orientations[selected]):
                self.rows[key] = [str(v) for v in axis]
            changed = True
        _, expanded = ui.TreeNode("Direction vectors##"+identity, force_open=bool(errors))
        if expanded:
            for key, caption in (("x", "X direction"), ("y", "Y direction"), ("z", "Z direction (normal)")):
                ui.TextMuted(caption)
                edited, self.rows[key] = ui.InputVector3(identity+"/"+key, self.rows[key],error=errors.get(key, ""))
                changed |= edited
            ui.TextMuted("Unit directions, perpendicular to each other. Z is the normal.",wrap=True)
        if changed:
            self.serialize()
        return self.source
