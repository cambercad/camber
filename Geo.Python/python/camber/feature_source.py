"""Recognize and edit the small, explicit Python blocks owned by modeling mode."""

import ast
from dataclasses import dataclass
import io
import re
import tokenize
import textwrap

from .modeling_commands import operation_for_call, simple_value


_MARKER = re.compile(r"^# camber:(begin|end) ([A-Za-z_][A-Za-z_0-9]*)$")


@dataclass(frozen=True)
class FeatureBlock:
    id: str
    start: int
    end: int
    source: str
    kind: str
    result: str | None
    dependencies: tuple[str, ...]
    editable: bool
    reason: str = ""
    operation: str = ""


def _call(statement):
    if isinstance(statement, ast.Assign) and len(statement.targets) == 1 and isinstance(statement.targets[0], ast.Name):
        result, call = statement.targets[0].id, statement.value
    elif isinstance(statement, ast.Expr):
        result, call = None, statement.value
    else:
        return None
    if not isinstance(call, ast.Call):
        return None
    method = call.func
    if isinstance(method, ast.Attribute) and isinstance(method.value, (ast.Name, ast.Subscript)):
        return result, ast.unparse(method.value), method.attr, call
    if isinstance(method, ast.Name):
        return result, None, method.id, call
    return None


def _literal(node):
    return simple_value(node)


def _classify(statements, types):
    if not statements:
        return "Code", None, False, "Empty block", None
    head = _call(statements[0])
    if head is None:
        return "Code", None, False, "Block must start with an assignment", None
    result, receiver, method, call = head
    operation = operation_for_call(receiver, method, types)
    if operation is None:
        return "Code", result, False, "Operation has no parameter editor", None
    if result is None and not operation.mutation:
        return "Code", None, False, "Operation must assign its result", None
    if result is not None and operation.mutation:
        return "Code", result, False, "Assembly mates do not return a feature", None
    kind = "Setup" if operation.key == "Part" else "Sketch" if operation.key == "Part.sketch" else operation.label
    if any(not _literal(arg) for arg in call.args) or any(
            kw.arg is None or not _literal(kw.value) for kw in call.keywords):
        return kind, result, False, "Computed argument; edit in Python", operation
    if kind == "Sketch":
        from .sketch_curves import METHODS
        from .sketch_emit import EXTRA_CONSTRAINTS
        allowed = {"add_line", "add_arc", "add_circle", "add_rectangle", "solve",
                   "coincident", "horizontal", "vertical", "parallel", "perpendicular",
                   "tangent", "equal", "midpoint", "concentric", "fix",
                   "length", "radius", "distance", "angle"}
        allowed.update(METHODS)
        allowed.update(EXTRA_CONSTRAINTS)
        for stmt in statements[1:]:
            expr = stmt.value if isinstance(stmt, ast.Expr) else None
            fn = expr.func if isinstance(expr, ast.Call) else None
            if not (isinstance(fn, ast.Attribute) and isinstance(fn.value, ast.Name)
                    and fn.value.id == result and fn.attr in allowed
                    and all(_literal(a) for a in expr.args)
                    and all(k.arg and _literal(k.value) for k in expr.keywords)):
                return kind, result, False, "Custom sketch code; edit in Python", operation
    elif len(statements) != 1:
        return kind, result, False, "Custom statements; edit in Python", operation
    try:
        operation.signature.bind(*([None] if not operation.static else []),
                                 *([None] * len(call.args)),
                                 **{k.arg: None for k in call.keywords})
    except TypeError as exc:
        return kind, result, False, str(exc), operation
    return kind, result, True, "", operation


class ModelDocument:
    """A source-preserving view of marked script or build_model features."""

    def __init__(self, source, types=None, *, path=None):
        self.source = source
        self.tree = ast.parse(source)
        # Component imports register their editors before feature classification.
        import importlib
        import sys
        from pathlib import Path
        parent = str(Path(path).resolve().parent) if path else None
        if parent:
            sys.path.insert(0, parent)
        try:
            for node in self.tree.body:
                if isinstance(node, ast.ImportFrom) and node.module and any(
                        alias.name.endswith("_component") for alias in node.names):
                    importlib.import_module(node.module)
        except ImportError as exc:
            raise ValueError("Cannot import component editor: " + str(exc)) from exc
        finally:
            if parent:
                sys.path.remove(parent)
        functions = [n for n in self.tree.body if isinstance(n, ast.FunctionDef)
                     and n.name == "build_model"]
        if len(functions) > 1:
            raise ValueError("Expected at most one build_model() function")
        self.module_level = not functions
        self.function = functions[0] if functions else self.tree
        if self.module_level:
            self.function.lineno = 0
            self.function.end_lineno = len(source.splitlines()) + 1
            self.function.decorator_list = []
        lines = source.splitlines(keepends=True)
        marks = []
        for token in tokenize.generate_tokens(io.StringIO(source).readline):
            if token.type != tokenize.COMMENT:
                continue
            match = _MARKER.fullmatch(token.string)
            if token.string.startswith("# camber:") and match is None:
                raise ValueError("Malformed feature marker at line " + str(token.start[0]))
            if match:
                if not (self.function.lineno < token.start[0] < self.function.end_lineno):
                    raise ValueError("Feature marker outside document body")
                marks.append((match.group(1), match.group(2), token.start[0]))
        opened = None
        ids = set()
        blocks = []
        types = dict(types or {})
        for action, identifier, line in marks:
            if action == "begin":
                if opened is not None or identifier in ids:
                    raise ValueError("Nested or duplicate feature marker: " + identifier)
                opened = identifier, line
            else:
                if opened is None or opened[0] != identifier:
                    raise ValueError("Mismatched feature marker: " + identifier)
                begin = opened[1]
                statements = [n for n in self.function.body
                              if begin < n.lineno and n.end_lineno < line]
                if any(begin < n.lineno <= line and n.end_lineno >= line
                       for n in self.function.body):
                    raise ValueError("Feature marker crosses a Python statement")
                kind, result, editable, reason, operation = _classify(statements, types)
                body = "".join(lines[begin:line-1])
                # Editor arity metadata belongs to the authored profile call. Other
                # comments remain protected from edits that would discard them.
                from .sketch_curves import METHODS
                profile_rows = {stmt.end_lineno-begin for stmt in statements
                                if isinstance(stmt, ast.Expr) and isinstance(stmt.value, ast.Call)
                                and isinstance(stmt.value.func, ast.Attribute)
                                and stmt.value.func.attr in METHODS}
                if any(t.type == tokenize.COMMENT and not (
                        kind == "Sketch" and t.start[0] in profile_rows
                        and re.fullmatch(r"# curves=[1-9]\d*", t.string))
                       for t in tokenize.generate_tokens(io.StringIO(body).readline)):
                    editable, reason = False, "Comments inside feature; edit in Python"
                dependencies = []
                for stmt in statements:
                    for node in ast.walk(stmt):
                        if (isinstance(node, ast.Name) and isinstance(node.ctx, ast.Load)
                                and node.id != result and node.id != "Part"):
                            if node.id not in dependencies:
                                dependencies.append(node.id)
                blocks.append(FeatureBlock(identifier, begin, line, body, kind, result,
                                           tuple(dependencies), editable, reason,
                                           operation.key if operation else ""))
                if operation and result and result not in types:
                    result_types = operation.result_types
                    types[result] = (operation.owner.__name__ if not operation.method and not operation.call_name else
                                     next(iter(result_types)) if len(result_types) == 1 else None)
                ids.add(identifier)
                opened = None
        if opened is not None:
            raise ValueError("Unclosed feature marker: " + opened[0])
        outputs = [b.result for b in blocks if b.result]
        if len(outputs) != len(set(outputs)):
            raise ValueError("Duplicate feature result variable")
        self.blocks = tuple(blocks)

    def with_imports(self, names):
        """Add only missing public value types needed by a generated operation."""
        imported = {alias.asname or alias.name for node in self.tree.body
                    if isinstance(node, ast.ImportFrom)
                    for alias in node.names}
        missing = sorted(set(names) - imported)
        if not missing:
            return self.source
        lines = self.source.splitlines(keepends=True)
        line = 0 if self.module_level else min([self.function.lineno] + [d.lineno for d in self.function.decorator_list]) - 1
        from .modeling_parameters import GENERATORS
        groups = {}
        for name in missing:
            module = (GENERATORS[name].__module__ if name in GENERATORS else
                      "camber.modeling_parameters" if name == "named_parameter" else "camber")
            groups.setdefault(module, []).append(name)
        additions = ["from " + module + " import " + ", ".join(items) + "\n"
                     for module, items in groups.items()]
        return "".join(lines[:line] + additions + ["\n"] + lines[line:])

    def block(self, identifier):
        for block in self.blocks:
            if block.id == identifier:
                return block
        raise KeyError(identifier)

    def dependents(self, identifier):
        result = self.block(identifier).result
        return tuple(b.id for b in self.blocks if b.id != identifier
                     and result and result in b.dependencies)

    def replace(self, identifier, body, *, require_editable=True):
        block = self.block(identifier)
        if require_editable and not block.editable:
            raise ValueError(block.reason)
        lines = self.source.splitlines(keepends=True)
        if self.module_level:
            body = textwrap.dedent(body)
        candidate = "".join(lines[:block.start] + [body.rstrip("\r\n") + "\n"]
                            + lines[block.end-1:])
        ModelDocument(candidate)
        return candidate

    def delete(self, identifier):
        users = self.dependents(identifier)
        if users:
            raise ValueError("Used by: " + ", ".join(users))
        block = self.block(identifier)
        lines = self.source.splitlines(keepends=True)
        candidate = "".join(lines[:block.start-1] + lines[block.end:])
        ModelDocument(candidate)
        return candidate

    def parameter_references(self, identifier):
        block = self.block(identifier)
        return tuple(node for statement in self.function.body
                     if not block.start < statement.lineno < block.end
                     for node in ast.walk(statement)
                     if isinstance(node, ast.Name) and node.id == block.result and isinstance(node.ctx, ast.Load))

    def delete_parameter(self, identifier, value):
        """Freeze references at their current numeric value, preserving source formatting."""
        from .modeling_parameters import named_parameter
        named_parameter(value)
        block = self.block(identifier)
        if block.operation != "named_parameter":
            raise ValueError("Select a named parameter")
        uses = []
        for statement in self.function.body:
            if block.start < statement.lineno < block.end:
                continue
            for node in ast.walk(statement):
                if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef, ast.Lambda,
                                     ast.ListComp, ast.SetComp, ast.DictComp, ast.GeneratorExp)):
                    if any(isinstance(n, ast.Name) and n.id == block.result for n in ast.walk(node)):
                        raise ValueError("Parameter is used in custom Python scopes; edit those references in Python first")
                if isinstance(node, ast.Name) and node.id == block.result:
                    if not isinstance(node.ctx, ast.Load):
                        raise ValueError("Parameter is reassigned in Python; edit that code first")
                    uses.append(node)
        lines = self.source.encode('utf-8').splitlines(keepends=True)
        offsets = [0]
        for line in lines:
            offsets.append(offsets[-1] + len(line))
        source = self.source.encode('utf-8')
        for node in sorted(uses, key=lambda n: (n.lineno, n.col_offset), reverse=True):
            start = offsets[node.lineno-1] + node.col_offset
            end = offsets[node.end_lineno-1] + node.end_col_offset
            source = source[:start] + repr(value).encode('ascii') + source[end:]
        return ModelDocument(source.decode('utf-8')).delete(identifier)

    def insert(self, identifier, body):
        if not re.fullmatch(r"[A-Za-z_][A-Za-z_0-9]*", identifier):
            raise ValueError("Invalid feature ID")
        if any(b.id == identifier for b in self.blocks):
            raise ValueError("Feature ID already exists")
        parsed = ast.parse(textwrap.dedent(body)).body
        call = _call(parsed[0]) if parsed else None
        parameter_line = (max((b.end for b in self.blocks if b.kind == "Setup" or b.operation == "named_parameter"),
                              default=0) + 1) if call and call[2] == "named_parameter" else None
        if self.module_level:
            displays = [n for n in self.tree.body if isinstance(n, ast.Assign)
                        and any(isinstance(t, ast.Name) and t.id == "displayed" for t in n.targets)]
            if len(displays) != 1:
                raise ValueError("Module documents must assign displayed once")
            lines = self.source.splitlines(keepends=True)
            addition = ("# camber:begin " + identifier + "\n" +
                        textwrap.dedent(body).rstrip("\r\n") +
                        "\n# camber:end " + identifier + "\n\n")
            line = (parameter_line or displays[0].lineno) - 1
            candidate = "".join(lines[:line] + [addition] + lines[line:])
            ModelDocument(candidate)
            return candidate
        returns = [n for n in self.function.body if isinstance(n, ast.Return)]
        if len(returns) != 1:
            raise ValueError("Expected one return in build_model()")
        line = parameter_line or returns[0].lineno
        lines = self.source.splitlines(keepends=True)
        addition = ("    # camber:begin " + identifier + "\n" + body.rstrip("\r\n")
                    + "\n    # camber:end " + identifier + "\n\n")
        candidate = "".join(lines[:line-1] + [addition] + lines[line-1:])
        ModelDocument(candidate)
        return candidate

    def display_result(self, result):
        """Change only the simple generated return statement."""
        if self.module_level:
            displays = [n for n in self.tree.body if isinstance(n, ast.Assign)
                        and len(n.targets) == 1 and isinstance(n.targets[0], ast.Name)
                        and n.targets[0].id == "displayed"]
            if len(displays) != 1 or displays[0].lineno != displays[0].end_lineno:
                raise ValueError("Expected a simple displayed assignment")
            lines = self.source.splitlines(keepends=True)
            lines[displays[0].lineno-1] = "displayed = " + result + "\n"
            candidate = "".join(lines)
            ModelDocument(candidate)
            return candidate
        returns = [n for n in self.function.body if isinstance(n, ast.Return)]
        if len(returns) != 1 or returns[0].lineno != returns[0].end_lineno:
            raise ValueError("Edit the return statement in Python")
        value = returns[0].value
        if not (isinstance(value, ast.Tuple) and len(value.elts) == 2
                and isinstance(value.elts[0], ast.Name) and value.elts[0].id == "part"):
            raise ValueError("Expected return part, displayed_result")
        lines = self.source.splitlines(keepends=True)
        line = returns[0].lineno - 1
        lines[line] = "    return part, " + result + "\n"
        candidate = "".join(lines)
        ModelDocument(candidate)
        return candidate
