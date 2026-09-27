"""Small CadQuery-style assembly facade over Camber's native mate solver.

Components may come from independent ``Part`` instances. The native assembly
re-expresses exact mesh coordinates in its own lattice for assembly operations.
"""

import math

from .api import Solid, Surface
from .vec import vec3


class Color:
    _named = {
        "black": (0, 0, 0), "blue": (0, 0, 1), "gray": (0.5, 0.5, 0.5),
        "green": (0, 0.7, 0), "lightgray": (0.83, 0.83, 0.83),
        "orange": (1, 0.55, 0), "red": (1, 0, 0), "yellow": (1, 1, 0),
    }

    def __init__(self, red, green=None, blue=None, alpha=1):
        if isinstance(red, str) and green is None and blue is None:
            try:
                red, green, blue = self._named[red.lower()]
            except KeyError:
                raise ValueError("unsupported assembly color {!r}".format(red))
        if green is None or blue is None:
            raise TypeError("Color needs a known name or red, green, blue values")
        self.rgba = tuple(float(v) for v in (red, green, blue, alpha))
        if any(not math.isfinite(v) or not 0 <= v <= 1 for v in self.rgba):
            raise ValueError("Color channels must be in [0, 1]")


class Location:
    """Translation and optional axis-angle rotation, in CadQuery's degrees."""

    def __init__(self, position=(0, 0, 0), axis=None, angle=0):
        self.position = tuple(float(v) for v in position)
        if len(self.position) != 3:
            raise ValueError("Location position must be 3D")
        if axis is None:
            self.orientation = (0, 0, 0, 1)
        else:
            axis = vec3(axis)
            length = math.sqrt(axis.x ** 2 + axis.y ** 2 + axis.z ** 2)
            if length == 0:
                raise ValueError("Location rotation axis cannot be zero")
            half = math.radians(float(angle)) / 2
            scale = math.sin(half) / length
            self.orientation = (axis.x * scale, axis.y * scale,
                                axis.z * scale, math.cos(half))


class Assembly:
    """CadQuery-shaped add/constrain/solve for supported native mate types.

    The first component determines the assembly's coordinate frame unless ``part`` is supplied.
    """

    def __init__(self, name=None, part=None):
        self._part = part
        self._name = name or "cadquery_assembly"
        self._assembly = None
        self._objects = {}
        self._parts = {}
        self._colors = {}
        self._fixed = set()
        self._source_geometry = {}
        self._geometry = {}

    def add(self, object, name=None, loc=None, color=None):
        from .cqcompat import Workplane

        if isinstance(object, Workplane):
            solid = object._solid
            if solid is None:
                raise ValueError("assembly component has no solid")
        elif isinstance(object, (Solid, Surface)):
            solid = object
            object = Workplane(obj=solid)
        else:
            raise TypeError("assembly component must be a Solid, Surface, or Workplane")
        if self._part is None:
            self._part = solid._part
        if self._assembly is None:
            self._assembly = self._part.assembly(self._name)
        name = name or solid.name
        if name in self._parts:
            raise ValueError("duplicate assembly component {!r}".format(name))
        loc = loc or Location()
        source_key = id(solid._n)
        if source_key not in self._source_geometry:
            points, _ = solid.mesh()
            self._source_geometry[source_key] = (
                tuple(points), tuple(object.faces()._selected_faces), tuple(object._edges))
        self._geometry[name] = self._source_geometry[source_key]
        occurrence = self._assembly.add_part(solid, loc.position, loc.orientation)
        self._parts[name] = occurrence
        self._objects[name] = object
        if color is not None:
            self._colors[name] = color if isinstance(color, Color) else Color(color)
        return self

    def _reference(self, reference):
        from .cqcompat import _select_edges, _select_faces
        if "@faces@" in reference:
            name, selector = reference.split("@faces@", 1)
            selected = _select_faces(self._geometry[name][1], selector)
            if len(selected) != 1:
                raise ValueError("assembly selector {!r} found {} faces, expected one".format(
                    reference, len(selected)))
            center, normal = selected[0]["center"], selected[0]["normal"]
        elif "@edges@" in reference:
            name, selector = reference.split("@edges@", 1)
            selected = _select_edges(self._geometry[name][2], selector)
            if len(selected) != 1:
                raise ValueError("assembly selector {!r} must identify one edge".format(reference))
            center, normal = selected[0]["center"], selected[0]["direction"]
        elif "@vertices@" in reference:
            name, selector = reference.split("@vertices@", 1)
            points = self._geometry[name][0]
            for clause in selector.split(" and "):
                clause = clause.strip()
                if len(clause) != 2 or clause[0] not in "<>" or clause[1] not in "XYZ":
                    raise ValueError("unsupported vertex selector {!r}".format(selector))
                index = "XYZ".index(clause[1])
                extreme = (min if clause[0] == "<" else max)(p[index] for p in points)
                points = [p for p in points if abs(p[index] - extreme) < 1e-7]
            if len(points) != 1:
                # Tessellated corners may be duplicated; coordinates must agree.
                if not points or any(tuple(p) != tuple(points[0]) for p in points):
                    raise ValueError("vertex selector {!r} is not unique".format(reference))
            center, normal = points[0], (0, 0, 1)
        elif "?" in reference:
            raise NotImplementedError("CadQuery tagged assembly references are not yet mapped")
        elif "@" in reference:
            raise NotImplementedError("assembly reference {!r} is unsupported".format(reference))
        else:
            name = reference
            faces = self._geometry[name][1]
            if isinstance(self._objects[name]._solid, Surface) and len(faces) == 1:
                center, normal = faces[0]["center"], faces[0]["normal"]
            else:
                points = self._geometry[name][0]
                center = tuple((min(p[i] for p in points) + max(p[i] for p in points)) / 2
                               for i in range(3))
                normal = (0, 0, 1)
        return name, vec3(center), vec3(normal)

    def _datum(self, reference, kind):
        name, center, normal = self._reference(reference)
        part = self._parts[name]
        if kind == "point":
            return part.point_at(center)
        if kind == "axis":
            return part.axis_at(center, normal)
        return part.plane_at(center, normal)

    def constrain(self, first, second, kind=None, param=None):
        if self._assembly is None:
            raise ValueError("assembly has no components")
        if isinstance(second, str) and second.startswith("Fixed") and kind is not None:
            second, kind = kind, second
        if kind is None:
            kind, second = second, None
        if kind == "Fixed":
            if second is not None or param is not None:
                raise TypeError("Fixed takes only a component reference")
            self._assembly.fix(self._parts[first])
            self._fixed.add(first)
            return self
        if kind == "FixedPoint":
            return self._fixed_point(first, second)
        if kind == "FixedAxis":
            return self._fixed_axis(first, second)
        if kind == "FixedRotation":
            return self._fixed_rotation(first, second)
        if kind == "Point":
            a, b = self._datum(first, "point"), self._datum(second, "point")
            if param is None or float(param) == 0:
                self._assembly.coincident(a, b)
            else:
                self._assembly.distance(a, b, float(param))
        elif kind == "Axis":
            a, b = self._datum(first, "axis"), self._datum(second, "axis")
            self._assembly.angle(a, b, math.radians(180 if param is None else float(param)))
        elif kind == "Plane":
            a, b = self._datum(first, "plane"), self._datum(second, "plane")
            angle = 180 if param is None else float(param)
            if angle not in (0, 180):
                raise NotImplementedError("Plane supports only param=0 or 180")
            self._assembly.coincident(a, b, opposite_normals=(angle == 180))
        elif kind == "PointInPlane":
            if param in (None, 0):
                plane = self._datum(second, "plane")
            else:
                name, center, normal = self._reference(second)
                normal = normal / math.sqrt(sum(v * v for v in normal))
                plane = self._parts[name].plane_at(center + normal * float(param), normal)
            self._assembly.on_plane(self._datum(first, "point"), plane)
        elif kind == "PointOnLine":
            if param not in (None, 0):
                raise NotImplementedError("nonzero PointOnLine distance is not yet supported")
            name, center, direction = self._reference(second)
            direction = direction / math.sqrt(sum(v * v for v in direction))
            seed = vec3(1, 0, 0) if abs(direction.x) < 0.8 else vec3(0, 1, 0)
            normal_a = direction.cross(seed)
            normal_b = direction.cross(normal_a)
            point = self._datum(first, "point")
            self._assembly.on_plane(point, self._parts[name].plane_at(center, normal_a))
            self._assembly.on_plane(point, self._parts[name].plane_at(center, normal_b))
        else:
            raise NotImplementedError("CadQuery assembly constraint {!r} has no native mapping".format(kind))
        return self

    def _ground_anchor(self, world_point, direction=None):
        if not self._fixed:
            raise NotImplementedError("world-fixed mates require a Fixed reference component")
        name = next(name for name in self._parts if name in self._fixed)
        leaf = self._assembly.leaves()[list(self._parts).index(name)]
        frame = leaf.frame
        delta = vec3(world_point) - vec3(frame.origin)
        local = vec3(sum(delta[i] * frame.x[i] for i in range(3)),
                     sum(delta[i] * frame.y[i] for i in range(3)),
                     sum(delta[i] * frame.z[i] for i in range(3)))
        if direction is None:
            return self._parts[name].point_at(local)
        direction = vec3(direction)
        local_direction = vec3(sum(direction[i] * frame.x[i] for i in range(3)),
                               sum(direction[i] * frame.y[i] for i in range(3)),
                               sum(direction[i] * frame.z[i] for i in range(3)))
        return self._parts[name].axis_at(local, local_direction)

    def _fixed_point(self, reference, target):
        self._assembly.coincident(self._datum(reference, "point"), self._ground_anchor(target))
        return self

    def _fixed_axis(self, reference, target):
        source = self._datum(reference, "axis")
        ground = self._ground_anchor((0, 0, 0), target)
        self._assembly.angle(source, ground, 0)
        return self

    def _fixed_rotation(self, reference, angles):
        # Two independent directed axes fix all rotational degrees of freedom.
        if "@" in reference or "?" in reference:
            raise NotImplementedError("FixedRotation expects a whole component")
        from .cqcompat import _rotate_frame
        from .api import Frame
        frame = _rotate_frame(Frame(), angles)
        for local_axis, world_axis in (((1, 0, 0), frame.x), ((0, 1, 0), frame.y)):
            source = self._parts[reference].axis_at((0, 0, 0), local_axis)
            ground = self._ground_anchor((0, 0, 0), world_axis)
            self._assembly.angle(source, ground, 0)
        return self

    def solve(self):
        if self._assembly is None:
            raise ValueError("assembly has no components")
        result = self._assembly.solve()
        if not result.converged or result.unsatisfied:
            raise RuntimeError("assembly did not solve: {!r}".format(result))
        return result

    def leaves(self):
        return () if self._assembly is None else self._assembly.leaves()

    def display_scene(self):
        """Posed geometry with per-occurrence names for independent colors/picking."""
        if self._assembly is None:
            raise ValueError("assembly has no components")
        from .display import decode
        scene = decode(self._assembly._n.dump_display())
        offset = 0
        for name, obj in self._objects.items():
            count = len(obj._solid.patch_names)
            for patch in scene.patches[offset:offset + count]:
                patch["name"] = name + "/" + patch["name"]
            offset += count
        if offset != len(scene.patches):
            raise RuntimeError("assembly display patch count does not match components")
        return scene

    def show(self, title="Camber CadQuery assembly"):
        if self._assembly is None:
            raise ValueError("assembly has no components")
        from . import show
        colors = {name + "/*": color.rgba[:3] for name, color in self._colors.items()}
        show(self.display_scene(), title=title, colors=colors, checker=False)
        return self
