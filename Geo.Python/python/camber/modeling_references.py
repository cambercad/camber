"""Named viewport references to earlier modeling results."""

from .api import Solid, Surface


def _bodies(available):
    def walk(expression, value):
        if isinstance(value, (Solid, Surface)):
            yield expression, value
        elif isinstance(value, (list, tuple)):
            for index, item in enumerate(value):
                yield from walk(f"{expression}[{index}]", item)

    for variable, value in available.items():
        if variable.isidentifier():
            yield from walk(variable, value)


def selection_expression(available, name, kind, *, inward=False, ray=None):
    """Return a validated expression retaining the selected entity's name.

    ``plane`` produces the named plane argument for Part.sketch; ``frame``
    and ``point`` produce calls on the owning body. Only earlier feature
    results in ``available`` can supply references, including pattern items.
    """
    if kind not in ("plane", "frame", "point"):
        raise ValueError("Reference kind must be plane, frame, or point")
    for expression, body in _bodies(available):
        names = body.point_names if kind == "point" else body.patch_names
        if name not in names:
            continue
        if kind == "point":
            body.point(name)  # Reject obsolete or otherwise unresolvable anchors.
            return f"{expression}.point({name!r})"
        frame = body.face_frame(name, inward=inward)  # Kernel rejects nonplanar faces.
        if kind == "plane":
            return repr(name)
        suffix = ", inward=True" if inward else ""
        result = f"{expression}.face_frame({name!r}{suffix})"
        if ray is not None:
            import math
            from .api import vec3
            origin, direction = map(vec3, ray)
            dot = lambda a, b: sum(x*y for x, y in zip(a, b))
            denominator = dot(direction, frame.z)
            if not math.isfinite(denominator) or abs(denominator) < 1e-12:
                raise ValueError("Choose a face facing the viewport")
            distance = dot(frame.origin-origin, frame.z)/denominator
            if not math.isfinite(distance) or distance < 0:
                raise ValueError("Choose a face in front of the viewport")
            delta = origin + direction*distance - frame.origin
            local = (dot(delta, frame.x), dot(delta, frame.y), 0.0)
            if not all(math.isfinite(v) for v in local):
                raise ValueError("Invalid face placement")
            result += f".to_global(Frame(origin={local!r}))"
        return result
    noun = "point" if kind == "point" else "planar face"
    raise ValueError(f"Select a {noun} belonging to an earlier feature")


def is_reference(source):
    """Identify named body references for their compact parameter editor."""
    import ast

    try:
        node = ast.parse(source, mode="eval").body
    except (SyntaxError, TypeError):
        return False
    if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
            and node.func.attr == "to_global" and len(node.args) == 1):
        node = node.func.value
    return (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
            and node.func.attr in ("face_frame", "point")
            and len(node.args) == 1 and isinstance(node.args[0], ast.Constant)
            and isinstance(node.args[0].value, str))
