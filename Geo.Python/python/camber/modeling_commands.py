"""Modeling commands and parameter forms derived from the public Python API."""

import ast
from dataclasses import dataclass
from functools import lru_cache
import inspect
import keyword
import math
import re
from typing import Literal, get_args, get_origin

from . import api


_OWNERS = (api.Part, api.Sketch, api.Solid, api.Surface, api.ProjectedSketch,
           api.Curve3D, api.Frame, api.Assembly, api.AssemblyPart, api.AssemblyOccurrence)
_RESULTS = {c.__name__ for c in _OWNERS} | {"SketchCurve"}
_DATUMS = {"point": api.AssemblyPointDatum, "axis": api.AssemblyAxisDatum,
           "plane": api.AssemblyPlaneDatum}
_RESULTS.update(t.__name__ for t in _DATUMS.values())
_ASSEMBLY_MATES = {"fix", "coincident", "parallel", "concentric", "perpendicular",
                   "angle", "distance", "on_plane", "contact"}
CONSTRUCTORS = {"Frame": api.Frame, "LoftOptions": api.LoftOptions}
_POSITIVE_DIMENSIONS = {
    "Solid.hole": {"diameter", "depth"},
    "Solid.counterbore_hole": {"diameter", "counterbore_diameter", "counterbore_depth", "depth"},
    "Solid.countersink_hole": {"diameter", "countersink_diameter", "depth"},
    "Solid.shell": {"thickness"},
    "Solid.fillet": {"radius"},
    "Solid.chamfer": {"distance"},
}


def is_angle_parameter(parameter):
    return (parameter.name.endswith("angle") or parameter.name in ("radians", "twist")) and annotation_names(parameter.annotation) - {"None"} == {"float"}


def validate_frame(frame):
    """Reject invalid poses before handing them to the geometry kernel."""
    from .vec import _xyz
    vectors = [_xyz(getattr(frame, key)) for key in ("origin", "x", "y", "z")]
    if not all(math.isfinite(v) for row in vectors for v in row):
        raise ValueError("Frame coordinates must be finite numbers")
    x, y, z = vectors[1:]
    dot = lambda a, b: sum(u*v for u, v in zip(a, b))
    if any(abs(dot(a, a)-1) > 1e-6 for a in (x, y, z)):
        raise ValueError("Frame directions must have unit length")
    if any(abs(dot(a, b)) > 1e-6 for a, b in ((x,y), (y,z), (z,x))):
        raise ValueError("Frame directions must be perpendicular")
    cross = (x[1]*y[2]-x[2]*y[1], x[2]*y[0]-x[0]*y[2], x[0]*y[1]-x[1]*y[0])
    if dot(cross, z) < 1-1e-6:
        raise ValueError("Frame directions must be right-handed (X cross Y = Z)")


def field_value(node, available):
    """Evaluate only the restricted value grammar used by parameter forms."""
    if not simple_value(node):
        raise ValueError("Use a literal or an existing feature reference")
    namespace = dict(available) if hasattr(available, "items") else {}
    namespace.update(CONSTRUCTORS)
    return eval(compile(ast.fix_missing_locations(ast.Expression(node)), "<model parameter>", "eval"), {"__builtins__": {}}, namespace)


def annotation_names(annotation):
    if isinstance(annotation, type):
        return {annotation.__name__}
    return set(re.findall(r"\b[A-Za-z_]\w*\b", str(annotation)))


def simple_value(node):
    """Only literals, earlier results, indexed results, and known value constructors."""
    if isinstance(node, ast.Name):
        return True
    if isinstance(node, (ast.Tuple, ast.List, ast.Set)):
        return all(isinstance(n.value, ast.Name) if isinstance(n, ast.Starred) and isinstance(node, (ast.Tuple, ast.List))
                   else simple_value(n) for n in node.elts)
    if isinstance(node, ast.Dict):
        return all(k is not None and simple_value(k) and simple_value(v)
                   for k, v in zip(node.keys, node.values))
    if isinstance(node, ast.Subscript):
        return (simple_value(node.value) and isinstance(node.slice, ast.Constant)
                and type(node.slice.value) is int)
    if isinstance(node, ast.Attribute):
        return node.attr in ("parts", "subassemblies") and simple_value(node.value)
    if isinstance(node, ast.Call):
        function = node.func
        allowed = (isinstance(function, ast.Name) and function.id in CONSTRUCTORS or
                   isinstance(function, ast.Attribute) and function.attr in {*_DATUMS,"face_frame","to_global"}
                   and simple_value(function.value))
        return (allowed
                and all(simple_value(n) for n in node.args)
                and all(k.arg and simple_value(k.value) for k in node.keywords))
    if isinstance(node, ast.BinOp) and isinstance(node.op, (ast.MatMult, ast.Add, ast.Sub, ast.Mult, ast.Div, ast.FloorDiv, ast.Mod)):
        return simple_value(node.left) and simple_value(node.right)
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.UAdd, ast.USub)):
        return simple_value(node.operand)
    try:
        ast.literal_eval(node)
        return True
    except (ValueError, TypeError):
        return False


@dataclass(frozen=True)
class Operation:
    owner: type
    method: str
    function: object
    signature: inspect.Signature
    static: bool = False
    call_name: str = ""

    @property
    def key(self):
        return self.call_name or self.owner.__name__ + ("." + self.method if self.method else "")

    @property
    def mutation(self):
        return self.owner is api.Assembly and self.method in _ASSEMBLY_MATES

    @property
    def label(self):
        return (self.call_name.replace("_", " ").capitalize() if self.call_name else
                self.method.replace("_", " ").capitalize() if self.method else self.owner.__name__)

    @property
    def description(self):
        text = (inspect.getdoc(self.function) or self.label).split("\n\n", 1)[0].replace("\n", " ").replace("`", "")
        if any(is_angle_parameter(p) for p in self.parameters):
            text = text.replace("radians", "degrees")
        return text

    @property
    def group(self):
        return getattr(self.function, "__api_group__", self.owner.__name__)

    @property
    def parameters(self):
        return tuple(p for p in self.signature.parameters.values() if p.name != "self")

    @property
    def result_types(self):
        return annotation_names(self.signature.return_annotation) & _RESULTS

    def defaults(self):
        return {p.name: "" if p.default is inspect.Parameter.empty else
                str(p.default) if isinstance(p.default, str) and (self.is_text(p) or self.choices(p)) else
                repr(p.default) for p in self.parameters}

    @staticmethod
    def is_text(parameter):
        names = annotation_names(parameter.annotation) - {"None", "Literal"}
        return bool(names) and names <= {"str", "EntityName"}

    @staticmethod
    def choices(parameter):
        # Literal values are the public API's enum vocabulary.
        if parameter.annotation is inspect.Parameter.empty:
            return ()
        if get_origin(parameter.annotation) is Literal:
            return get_args(parameter.annotation)
        annotation = parameter.annotation if isinstance(parameter.annotation, str) else str(parameter.annotation)
        try:
            tree = ast.parse(annotation, mode="eval")
        except SyntaxError:
            return ()
        for node in ast.walk(tree):
            if isinstance(node, ast.Subscript) and isinstance(node.value, ast.Name) and node.value.id == "Literal":
                items = node.slice.elts if isinstance(node.slice, ast.Tuple) else [node.slice]
                return tuple(ast.literal_eval(item) for item in items)
        return ()

    def parse_field(self, parameter, value, available):
        value = value.strip()
        if not value:
            if parameter.default is inspect.Parameter.empty:
                raise ValueError(parameter.name.replace("_", " ").capitalize() + " is required")
            return None  # Omit optional arguments to keep API defaults.
        if self.is_text(parameter) and not (value == "None" and "None" in annotation_names(parameter.annotation)):
            return ast.Constant(value=value)
        choices = self.choices(parameter)
        if choices and value in [str(v) for v in choices]:
            return ast.Constant(value=next(v for v in choices if str(v) == value))
        node = ast.parse(value, mode="eval").body
        if not simple_value(node):
            raise ValueError(parameter.name + " must be a literal or an existing feature reference")
        for name in (n.id for n in ast.walk(node) if isinstance(n, ast.Name)):
            if name not in available and name not in CONSTRUCTORS:
                raise ValueError("Unknown feature reference: " + name)
        if choices and isinstance(node, ast.Constant) and node.value not in choices and not (
                node.value is None and "None" in annotation_names(parameter.annotation)):
            raise ValueError(parameter.name + " must be one of: " + ", ".join(map(str, choices)))
        if isinstance(node, ast.Constant) and node.value is not None:
            names = annotation_names(parameter.annotation)
            if names == {"bool"} and type(node.value) is not bool:
                raise ValueError(parameter.name + " must be True or False")
            if names == {"int"} and type(node.value) is not int:
                raise ValueError(parameter.name + " must be an integer")
            if names == {"float"} and type(node.value) not in (float, int):
                raise ValueError(parameter.name + " must be a number")
        return node

    def source(self, receiver, result, values, available=()):
        if not self.mutation and (not result or not result.isidentifier() or keyword.iskeyword(result)):
            raise ValueError("Result variable must be a Python identifier")
        target = self.call_name or (self.owner.__name__ if self.static else receiver)
        if not target:
            raise ValueError("Select an input " + self.owner.__name__)
        if self.method and not self.call_name:
            target += "." + self.method
        args = []
        for p in self.parameters:
            node = self.parse_field(p, values.get(p.name, ""), available)
            if node is not None:
                args.append(("" if p.kind is p.POSITIONAL_ONLY else p.name + "=") + ast.unparse(node))
        return "    " + ("" if self.mutation else result + " = ") + target + "(" + ", ".join(args) + ")\n"

    def field_errors(self, values, available):
        errors = {}
        datum_kinds = {}
        for parameter in self.parameters:
            try:
                node = self.parse_field(parameter, values.get(parameter.name, ""), available)
                if node is None:
                    continue
                names = annotation_names(parameter.annotation)
                if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
                    component = field_value(node.func.value, available)
                    if isinstance(component,(api.Solid,api.Surface)):
                        if node.func.attr not in ("face_frame","point"):
                            raise ValueError("Select a face or point reference on the body")
                        try:
                            value = field_value(node,available)
                        except Exception as exc:
                            raise ValueError(str(exc)) from exc
                    elif isinstance(component, api.Frame) and node.func.attr == "to_global":
                        value = field_value(node, available)
                    else:
                        if not isinstance(component, api.AssemblyPart):
                            raise ValueError("Select a part occurrence for this reference")
                        datum_type = _DATUMS.get(node.func.attr)
                        if datum_type is None or datum_type.__name__ not in names:
                            raise ValueError("Select the reference type required by this mate")
                        if len(node.args) != 1 or node.keywords or not isinstance(node.args[0], ast.Constant) or not isinstance(node.args[0].value, str):
                            raise ValueError("Select a named reference on the component")
                        datum_kinds[parameter.name] = node.func.attr
                        continue  # Resolve the datum once, in the disposable model rebuild.
                else:
                    value = field_value(node, available)
                if value is None:
                    if "None" not in names and parameter.default is not None:
                        raise ValueError("A value is required")
                    continue
                expected = tuple(getattr(api, name) for name in names if name in _RESULTS)
                if expected and names & {"Sequence", "Iterable", "list", "tuple"}:
                    if not isinstance(value, (list, tuple)) or not all(isinstance(item, expected) for item in value):
                        raise ValueError("Select " + " or ".join(t.__name__.lower() for t in expected) + " features")
                    minimum = (2 if self.key in ("Part.loft", "Part.loft_surface")
                               and parameter.name in ("sketches", "sections") else
                               0 if parameter.name == "guides" and "None" in names else 1)
                    if len(value) < minimum:
                        raise ValueError("Select at least " + str(minimum) + " features")
                if expected and "Point3Like" not in names and not names & {"Sequence", "Iterable", "list", "tuple"} and not isinstance(value, expected):
                    raise ValueError("Select a " + " or ".join(t.__name__ for t in expected))
                if names - {"None"} in ({"float"}, {"int"}):
                    if type(value) not in (int, float) or not math.isfinite(value):
                        raise ValueError("Enter a finite number")
                    if "int" in names and type(value) is not int:
                        raise ValueError("Enter an integer")
                    if parameter.name in _POSITIVE_DIMENSIONS.get(self.key, ()) and value <= 0:
                        raise ValueError("Must be greater than zero")
                    if self.key == "Solid.countersink_hole" and parameter.name == "included_angle" and not 0 < value < math.pi:
                        raise ValueError("Included angle must be between 0 and 180 degrees")
                if "EntitySelection" in names:
                    entities = (value,) if isinstance(value, str) else value
                    if not isinstance(entities, (tuple, list)) or not all(isinstance(n, str) and n for n in entities):
                        raise ValueError("Select named entities")
                    if not entities and "None" not in names:
                        raise ValueError("Select at least one entity")
                if parameter.name == "orientation" and self.key in ("Assembly.add_part", "Assembly.add_subassembly"):
                    if not isinstance(value, (tuple, list)) or len(value) != 4 or any(
                            type(v) not in (int, float) or not math.isfinite(v) for v in value):
                        raise ValueError("Enter four finite quaternion components")
                    if sum(v*v for v in value) <= 1e-24:
                        raise ValueError("Orientation quaternion cannot be zero")
                if names & {"Point2Like", "Point3Like"} and names & {"Sequence", "Iterable", "list", "tuple"}:
                    dimensions = 3 if "Point3Like" in names else 2
                    if not isinstance(value, (list, tuple)) or any(
                            not isinstance(row, (list, tuple)) or len(row) != dimensions or
                            any(type(v) not in (int, float) or not math.isfinite(v) for v in row) for row in value):
                        raise ValueError("Enter " + str(dimensions) + " finite coordinates per point")
                    if len(value) < 2:
                        raise ValueError("Add at least two points")
                elif "Point3Like" in names and not isinstance(value, expected):
                    from .vec import _xyz
                    if hasattr(value,"__len__") and len(value) != 3 or not all(math.isfinite(v) for v in _xyz(value)):
                        raise ValueError("Enter three finite coordinates")
                    if parameter.name in ("start_tangent", "end_tangent") and not any(_xyz(value)):
                        raise ValueError("Tangent direction must be nonzero")
                elif "Point2Like" in names and not names & {"Sequence", "Iterable", "list", "tuple", "str"}:
                    try:
                        coords = tuple(value)
                    except TypeError:
                        coords = ()
                    if len(coords) != 2 or any(type(v) not in (int, float) or not math.isfinite(v) for v in coords):
                        raise ValueError("Enter two finite sketch-plane coordinates")
                if isinstance(value, api.Frame):
                    validate_frame(value)
            except (ValueError, TypeError, SyntaxError, NameError, IndexError, AttributeError, ArithmeticError) as exc:
                errors[parameter.name] = "Enter a valid value" if isinstance(exc, SyntaxError) else str(exc)
        if self.key == "Part.loft":
            for end in ("start", "end"):
                closure, point = end + "_closure", end + "_point"
                if values.get(closure, "None").strip() not in ("", "None") and values.get(point, "None").strip() in ("", "None"):
                    errors[closure] = "Choose a tip point before its closure style"
        if self.key in {"Assembly.coincident", "Assembly.parallel", "Assembly.perpendicular", "Assembly.distance"}:
            if "a" in datum_kinds and "b" in datum_kinds and datum_kinds["a"] != datum_kinds["b"]:
                errors["b"] = "Select the same reference type as the first reference"
        if self.key == "Assembly.coincident" and datum_kinds.get("a") in ("axis", "point"):
            if values.get("opposite_normals", "None").strip() not in ("", "None"):
                errors["opposite_normals"] = "Face orientation applies only to plane references"
        if self.key == "Frame" and not errors:
            vectors = {}
            for parameter in self.parameters:
                node = self.parse_field(parameter, values.get(parameter.name, ""), available)
                if node is not None:
                    vectors[parameter.name] = field_value(node, available)
            try:
                validate_frame(api.Frame(**vectors))
            except ValueError as exc:
                errors.update({key: str(exc) for key in ("x", "y", "z")})
        if not errors and self.call_name:
            from .modeling_parameters import GENERATORS, named_parameter
            function = GENERATORS.get(self.call_name)
            if function or self.call_name == "named_parameter":
                arguments = {}
                for parameter in self.parameters:
                    node = self.parse_field(parameter, values.get(parameter.name, ""), available)
                    if node is not None:
                        arguments[parameter.name] = field_value(node, available)
                try:
                    if function:
                        arguments.pop("part", None)
                        arguments.pop("name", None)
                        function.parameter_type(**arguments)
                    else:
                        named_parameter(**arguments)
                except (TypeError, ValueError) as exc:
                    errors["__parameters"] = str(exc)
        return errors


def discover_operations():
    """Public methods returning CAD objects automatically get parameter editors."""
    operations = {}
    for owner in _OWNERS:
        for name, function in inspect.getmembers(owner, inspect.isfunction):
            if name.startswith("_") or name in ("show", "sketch_interactive"):
                continue
            signature = inspect.signature(function)
            if not (annotation_names(signature.return_annotation) & _RESULTS) and (owner, name) != (api.Part, "add_plane") and not (owner is api.Assembly and name in _ASSEMBLY_MATES):
                continue
            static = isinstance(inspect.getattr_static(owner, name), staticmethod)
            command = Operation(owner, name, function, signature, static)
            operations[command.key] = command
    for owner in (api.Part, api.Frame, api.LoftOptions):
        command = Operation(owner, "", owner, inspect.signature(owner), True)
        operations[command.key] = command
    from .modeling import load_component
    command = Operation(api.Part, "", load_component, inspect.signature(load_component), True, "load_component")
    operations[command.key] = command
    from .modeling_parameters import named_parameter, GENERATORS
    for function in (named_parameter, *GENERATORS.values()):
        command = Operation(api.Part, "", function, inspect.signature(function), True, function.__name__)
        operations[command.key] = command
    return operations


@lru_cache(maxsize=1)
def operations():
    return discover_operations()


def operation_for_call(receiver, method, types):
    if receiver is None:
        return operations().get(method)
    owner = receiver if receiver in {c.__name__ for c in _OWNERS} else types.get(receiver)
    return operations().get(str(owner) + "." + method)
