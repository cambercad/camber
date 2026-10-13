"""Occurrence-scoped display and reference choices for the modeling workspace."""

from dataclasses import dataclass

from .api import Assembly
from .display import DisplayScene
from . import glview


def assembly_scene(assembly):
    """Give every displayed entity its stable occurrence path, preserving colors."""
    scene = DisplayScene()
    for leaf in assembly.leaves():
        placed = glview._as_scene(leaf.solid)
        for collection in (placed.patches, placed.curves, placed.points):
            for item in collection:
                local = item["name"]
                item["name"] = leaf.path + ":" + local
                item["color_name"] = local
        scene.extend(placed)
    return scene


@dataclass(frozen=True)
class Component:
    expression: str
    path: str
    part: object
    references: dict


class AssemblyReferences:
    """Build once per model revision; indices distinguish repeated definitions."""

    def __init__(self, assembly, expression, sources=None):
        self.assembly = assembly
        leaves = {leaf.path: leaf for leaf in assembly.leaves()}
        self.components = {}
        self.children = {}
        self.containers = {}
        self.constraints = {}
        self.entities = {}
        self._fields = {}
        sources = sources or {}

        def visit(container, source, path):
            self.children[source] = []
            self.containers[source] = (path, container)
            self.constraints[source] = (container if isinstance(container, Assembly) else container.assembly).constraints
            for index, part in enumerate(container.parts):
                address = sources.get(source+f".parts[{index}]",source+f".parts[{index}]")
                part_path = path + f"/{part.name}[{index+1}]"
                leaf = leaves[part_path]
                placed = glview._as_scene(leaf.solid)
                refs = {"plane": [], "axis": [], "point": []}
                for patch in placed.patches:
                    tag = glview._SURFACE_TYPE_TAGS.get(patch.get("surface_type"))
                    if tag == "pln":
                        refs["plane"].append(patch["name"])
                    if tag in ("pln", "cyl"):
                        refs["axis"].append(patch["name"])
                refs["point"] = [point["name"] for point in placed.points]
                component = Component(address, part_path, part, refs)
                self.components[address] = component
                self.children[source].append(address)
                for kind, names in refs.items():
                    for name in names:
                        self.entities[(part_path+":"+name, kind)] = address + "." + kind + "(" + repr(name) + ")"
            for index, child in enumerate(container.subassemblies):
                address = source+f".subassemblies[{index}]"
                address = sources.get(address,address)
                self.children[source].append(address)
                visit(child,address, path+f"/{child.name}[{index+1}]")

        if not isinstance(assembly, Assembly):
            raise TypeError("Select an assembly")
        visit(assembly, expression, assembly.name)

    def from_pick(self, name, kinds):
        return next((self.entities[(name, kind)] for kind in kinds if (name,kind) in self.entities), None)

    def draw_datum(self, ui, identity, value, kinds, *, picked=()):
        """Choose a component and a geometric reference, never enter an expression."""
        import ast
        source, kind, reference = "", kinds[0], ""
        state = self._fields.get(identity)
        if state and state[0] == value:
            _,source,kind,reference = state
        else:
            try:
                call = ast.parse(value,mode="eval").body
                if isinstance(call,ast.Call) and isinstance(call.func,ast.Attribute):
                    source, kind = ast.unparse(call.func.value), call.func.attr
                    reference = ast.literal_eval(call.args[0])
            except (SyntaxError,ValueError,IndexError):
                pass
        labels = {component.path: expression for expression,component in self.components.items()}
        current = self.components[source].path if source in self.components else ""
        width = max(1,ui.GetContentRegionAvail()[0])
        changed, selected = ui.ComboBox(identity+"/component",current,tuple(labels),width=width)
        if changed:
            source, reference = labels[selected], ""
        if len(kinds)>1:
            changed, selected = ui.ComboBox(identity+"/kind",kind,kinds,width=width)
            if changed:
                kind, reference = selected, ""
        if source in self.components and kind in kinds:
            choices = self.components[source].references[kind]
            changed, selected = ui.ComboBox(identity+"/entity",reference,choices,width=width)
            if changed:
                reference = selected
            if reference in choices:
                value = source+"."+kind+"("+repr(reference)+")"
            else:
                value = ""
        if picked and ui.Button("Use selected reference##"+identity,height=26):
            selected = self.from_pick(picked[-1],kinds)
            if selected:
                value = selected
        self._fields[identity] = (value,source,kind,reference)
        return value
