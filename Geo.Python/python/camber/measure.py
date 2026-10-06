"""Thin native geometry-query bridge; Python only packs viewer buffers and names."""
from dataclasses import dataclass
import base64
import struct
import math

@dataclass(frozen=True)
class Distance:
    first: tuple
    second: tuple
    length: float
    first_name: str
    second_name: str

class Geometry:
    """Immutable C# query snapshot with a lazily built native BVH."""
    def __init__(self, primitives):
        from _camber_native.__dotwrap_generated.main import NativeGeometryQuery
        records = []
        self.count = 0
        for primitive in primitives:
            n = len(primitive)
            if n not in (1, 2, 3) or any(len(p) != 3 or not all(math.isfinite(c) for c in p) for p in primitive):
                raise ValueError("expected 3D points, segments or triangles")
            records.append(struct.pack("<B" + "d" * (3*n), n,
                                       *(float(c) for p in primitive for c in p)))
            self.count += 1
        payload = struct.pack("<i", self.count) + b"".join(records)
        self.native = NativeGeometryQuery(base64.b64encode(payload).decode("ascii"))

    def ray(self, origin, direction, count=3, radius=0, clip_plane=None):
        ray = " ".join(format(float(v), ".17g") for v in (*origin, *direction))
        plane = "" if clip_plane is None else " ".join(
            format(float(v), ".17g") for v in (*clip_plane[0], clip_plane[1]))
        index, depth, distance = map(float, self.native.ray(ray, count, radius, plane).split())
        return int(index), depth, distance

def closest_entities(left, right, first_name="", second_name=""):
    left = left if isinstance(left, Geometry) else Geometry(left)
    right = right if isinstance(right, Geometry) else Geometry(right)
    if not left.count or not right.count:
        raise ValueError("entities must have nonempty geometry")
    values = tuple(map(float, left.native.closest(right.native).split()))
    return Distance(values[:3], values[3:6], values[6], first_name, second_name)

def named_geometry(packeds, name):
    """Resolve a selected viewer name to points, sampled curve segments or face triangles."""
    out = []
    for packed in packeds:
        for item in packed.get("catalog", ()):
            if item.get("name") != name:
                continue
            if item.get("kind") == "point" and item.get("world") is not None:
                out.append((tuple(item["world"]),))
            elif item.get("kind") == "curve":
                pts = item.get("points") or ()
                out.extend((tuple(a), tuple(b)) for a, b in zip(pts, pts[1:]))
                if item.get("closed") and len(pts) > 2:
                    out.append((tuple(pts[-1]), tuple(pts[0])))
        vertices = packed.get("mesh_pos") or ()
        for tri, face_name in zip(packed.get("mesh_idx") or (), packed.get("face_names") or ()):
            if face_name == name:
                out.append(tuple(tuple(vertices[i]) for i in tri))
    return out


