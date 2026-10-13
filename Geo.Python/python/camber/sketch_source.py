"""Read the sketch calls emitted by the interactive editor without losing constraints."""

import ast
import inspect
import textwrap
import re

from .api import Sketch
from . import sketch_curves as _curves
from .sketch_emit import EXTRA_CONSTRAINTS


# Parameter names come from the public API; the values use the editor's action format.
_CONSTRAINTS = {
    "horizontal": (("curve",), (), None),
    "vertical": (("curve",), (), None),
    "parallel": (("curve_a", "curve_b"), (), None),
    "perpendicular": (("curve_a", "curve_b"), (), None),
    "equal": (("curve_a", "curve_b"), (), None),
    "concentric": (("curve_a", "curve_b"), (), None),
    "tangent": (("line_curve", "circular_curve"), (), None),
    "coincident": ((), ("point_a", "point_b"), None),
    "midpoint": (("line_curve",), ("point",), None),
    "fix": ((), ("point",), None),
    "length": (("curve",), (), "value"),
    "radius": (("curve",), (), "value"),
    "distance": ((), ("point_a", "point_b"), "value"),
    "angle": (("curve_a", "curve_b"), (), "degrees"),
}
_CONSTRAINTS.update(EXTRA_CONSTRAINTS)
_GEOMETRY = {
    "add_line": (("start", "p0_ref"), ("end", "p1_ref")),
    "add_circle": (("center", "center_ref"),),
    "add_arc": (("start", "start_ref"), ("mid", "mid_ref"), ("end", "end_ref")),
}


def read_sketch_actions(source, variable, sketch):
    """Combine solved geometry with supported source calls, rejecting lossy edits.

    This intentionally accepts the editor's ordinary literal calls, not arbitrary
    Python control flow. Unsupported source stays available in the Python editor.
    """
    geometry = [dict(a) for a in sketch._solved_actions()]
    actions, cursor = [], 0

    def literal(node):
        if (isinstance(node, ast.BinOp) and isinstance(node.op, ast.MatMult)
                and isinstance(node.left, ast.Name) and node.left.id == variable
                and isinstance(node.right, ast.Constant) and node.right.value == "origin"):
            return "Origin"
        if (isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name)
                and node.value.id == variable and node.attr == "origin"):
            return "Origin"
        return ast.literal_eval(node)

    try:
        for statement in ast.parse(textwrap.dedent(source)).body[1:]:
            if not isinstance(statement, ast.Expr) or not isinstance(statement.value, ast.Call):
                raise ValueError("Only direct sketch calls can be edited graphically")
            call = statement.value
            if not (isinstance(call.func, ast.Attribute) and isinstance(call.func.value, ast.Name)
                    and call.func.value.id == variable):
                raise ValueError("Only calls on this sketch can be edited graphically")
            method = call.func.attr
            if method == "solve":
                if call.args or call.keywords:
                    raise ValueError("Unsupported solve arguments")
                continue
            if method not in _GEOMETRY and method not in _CONSTRAINTS and method != "add_rectangle" and method not in _curves.METHODS:
                raise ValueError("Unsupported sketch call: " + method)
            bound = inspect.signature(getattr(Sketch, method)).bind(
                sketch, *(literal(n) for n in call.args),
                **{kw.arg: literal(kw.value) for kw in call.keywords})
            bound.apply_defaults()
            values = bound.arguments
            if method in _curves.METHODS:
                row = textwrap.dedent(source).splitlines()[statement.end_lineno - 1]
                count_match = re.search(r"# curves=(\d+)\s*$", row)
                count = int(count_match[1]) if count_match else 1
                displayed = geometry[cursor:cursor + count]
                if count <= 0 or len(displayed) != count:
                    raise ValueError("Profile geometry does not match the source")
                parameters = {key: value for key, value in values.items() if key != "self"}
                actions.append({"kind": "curve_call", "method": method, "arguments": parameters,
                                "curve_count": count, "display": displayed})
                cursor += count
            elif method in _GEOMETRY:
                action = dict(geometry[cursor])
                if action["kind"] != method.removeprefix("add_"):
                    raise ValueError("Sketch geometry does not match the source")
                cursor += 1
                if values.get("construction"):
                    action["construction"] = True
                for argument, key in _GEOMETRY[method]:
                    if isinstance(values[argument], str):
                        action[key] = values[argument]
                actions.append(action)
            elif method == "add_rectangle":
                sides = geometry[cursor:cursor+4]
                if len(sides) != 4 or any(a["kind"] != "line" for a in sides):
                    raise ValueError("Rectangle geometry does not match the source")
                actions.extend(sides)
                names = [a["name"] for a in sides]
                actions.extend({"kind": "coincident", "points": [names[i]+"@1.000", names[(i+1)%4]+"@0.000"]}
                               for i in range(4))
                actions.extend(({"kind": "parallel", "curves": [names[0], names[2]]},
                                {"kind": "parallel", "curves": [names[1], names[3]]},
                                {"kind": "perpendicular", "curves": [names[0], names[1]]}))
                cursor += 4
            else:
                curves, points, scalar = _CONSTRAINTS[method]
                action = {"kind": method, "curves": [values[k] for k in curves],
                          "points": [values[k] for k in points]}
                if scalar:
                    action["value"] = values[scalar]
                actions.append(action)
        if cursor != len(geometry) or cursor != sketch.curve_count:
            raise ValueError("The sketch contains geometry the palette cannot edit")
    except (ValueError, TypeError, SyntaxError, IndexError, KeyError) as exc:
        raise ValueError(str(exc) + "; edit its Python feature to preserve it") from exc
    return actions
