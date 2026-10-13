"""Session-local reuse of immutable definitions from managed component sources."""
import ast
from contextlib import contextmanager
from contextvars import ContextVar


_CACHE = ContextVar("model_component_cache", default=None)
_DEPENDENCIES = ContextVar("model_component_dependencies", default=None)


def _managed_geometry(document):
    from .api import Part, Solid, Surface, Sketch, Curve3D, Frame, LoftOptions
    if not document.blocks or any(not b.editable for b in document.blocks):
        return False
    imports = {"Part", "Frame", "Curve3D", "LoftOptions", "load_component"}
    methods = {name for cls in (Part,Solid,Surface,Sketch,Curve3D,Frame,LoftOptions)
               for name in dir(cls) if not name.startswith("_")}
    for node in document.tree.body:
        if isinstance(node,ast.ImportFrom):
            if node.module not in {"camber","camber.modeling"} or any(n.name not in imports or n.asname for n in node.names):
                return False
        elif isinstance(node,ast.FunctionDef) and node.name == "build_model" and not node.decorator_list:
            if node.args.args or node.args.posonlyargs or node.args.kwonlyargs or node.args.vararg or node.args.kwarg:
                return False
            if any(not isinstance(statement,ast.Return) and not any(
                    b.start <= statement.lineno and statement.end_lineno <= b.end for b in document.blocks)
                   for statement in node.body):
                return False
        else:
            return False
    for node in ast.walk(document.tree):
        if isinstance(node,(ast.Lambda,ast.For,ast.While,ast.With,ast.Try,ast.ListComp,ast.GeneratorExp)):
            return False
        if isinstance(node,ast.Call):
            fn = node.func
            if isinstance(fn,ast.Name):
                if fn.id not in imports:
                    return False
            elif not isinstance(fn,ast.Attribute) or fn.attr not in methods or fn.attr.startswith(("load_","import_","read_","write_","export_","save")):
                return False
    return True


class ComponentCache:
    def __init__(self):
        self.entries = {}
        self.used = set()

    @contextmanager
    def scope(self, *, prune=True):
        self.used = set()
        token = _CACHE.set(self)
        try:
            yield
        finally:
            _CACHE.reset(token)
            if prune:
                self.entries = {p: entry for p,entry in self.entries.items() if p in self.used}

    def load(self, target, source, document, build):
        parent = _DEPENDENCIES.get()
        self.used.add(target)
        entry = self.entries.get(target)
        if entry:
            result, sources = entry
            try:
                valid = all(p.read_text(encoding="utf-8") == text for p,text in sources.items())
            except OSError:
                valid = False
            if valid:
                self.used.update(sources)
                if parent:
                    parent[0].update(sources)
                return result
            self.entries.pop(target,None)
        # A mutable flag also propagates unmanaged/external or assembly children.
        state = [{target:source}, _managed_geometry(document)]
        token = _DEPENDENCIES.set(state)
        try:
            result = build()
        finally:
            _DEPENDENCIES.reset(token)
        from .api import Solid
        state[1] = state[1] and isinstance(result,Solid)
        if parent:
            parent[0].update(state[0])
            parent[1] = parent[1] and state[1]
        if state[1]:
            self.entries[target] = (result,state[0])
        return result


def component_cache():
    return _CACHE.get()
