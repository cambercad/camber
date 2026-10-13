"""Recognize and edit the small, explicit Python blocks owned by modeling mode."""

import ast
from dataclasses import dataclass
import io
import re
import tokenize

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
    """A source-preserving view of marked features inside ``build_model``."""

    def __init__(self, source, types=None):
        self.source = source
        self.tree = ast.parse(source)
        functions = [n for n in self.tree.body if isinstance(n, ast.FunctionDef)
                     and n.name == "build_model"]
        if len(functions) != 1:
            raise ValueError("Expected exactly one build_model() function")
        self.function = functions[0]
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
                    raise ValueError("Feature marker outside build_model()")
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
                    types[result] = (operation.owner.__name__ if not operation.method else
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
                    if isinstance(node, ast.ImportFrom) and node.module == "camber"
                    for alias in node.names}
        missing = sorted(set(names) - imported)
        if not missing:
            return self.source
        lines = self.source.splitlines(keepends=True)
        line = min([self.function.lineno] + [d.lineno for d in self.function.decorator_list]) - 1
        return "".join(lines[:line] + ["from camber import " + ", ".join(missing) + "\n\n"] + lines[line:])

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

    def insert(self, identifier, body):
        if not re.fullmatch(r"[A-Za-z_][A-Za-z_0-9]*", identifier):
            raise ValueError("Invalid feature ID")
        if any(b.id == identifier for b in self.blocks):
            raise ValueError("Feature ID already exists")
        returns = [n for n in self.function.body if isinstance(n, ast.Return)]
        if len(returns) != 1:
            raise ValueError("Expected one return in build_model()")
        line = returns[0].lineno
        lines = self.source.splitlines(keepends=True)
        addition = ("    # camber:begin " + identifier + "\n" + body.rstrip("\r\n")
                    + "\n    # camber:end " + identifier + "\n\n")
        candidate = "".join(lines[:line-1] + [addition] + lines[line-1:])
        ModelDocument(candidate)
        return candidate

    def display_result(self, result):
        """Change only the simple generated return statement."""
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
