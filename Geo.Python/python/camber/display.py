"""Decode the native display blob (named patches, curves, anchor points)."""

import base64
import struct


class DisplayScene(object):
    def __init__(self):
        self.patches = []
        self.curves = []
        self.points = []

    def extend(self, other):
        self.patches.extend(other.patches)
        self.curves.extend(other.curves)
        self.points.extend(other.points)


def decode(blob):
    if blob is None or blob == "":
        return DisplayScene()
    if isinstance(blob, str):
        data = base64.b64decode(blob)
    else:
        data = bytes(blob)
    return _decode_bytes(data)


def decode_native_solid(native_solid):
    return decode(native_solid.dump_display())


def decode_native_part(native_part):
    return decode(native_part.dump_display())


def _decode_bytes(data):
    if len(data) < 12 or data[0:4] != b"CMBR":
        raise ValueError("not a camber display blob")
    version, n_meshes = struct.unpack_from("<II", data, 4)
    if version not in (1, 2, 3):
        raise ValueError("unsupported display blob version {0}".format(version))
    off = 12
    scene = DisplayScene()
    for _ in range(n_meshes):
        off = _read_mesh(data, off, scene, version)
    return scene


def _read_mesh(data, off, scene, version=1):
    n_verts, = struct.unpack_from("<i", data, off)
    off += 4
    n_pos = n_verts * 3
    positions = struct.unpack_from("<" + "d" * n_pos, data, off)
    off += n_pos * 8
    verts = [(positions[i], positions[i + 1], positions[i + 2]) for i in range(0, n_pos, 3)]

    n_norm, = struct.unpack_from("<i", data, off)
    off += 4
    n_ncomp = n_norm * 3
    normals = []
    if n_ncomp > 0:
        raw_n = struct.unpack_from("<" + "d" * n_ncomp, data, off)
        normals = [(raw_n[i], raw_n[i + 1], raw_n[i + 2]) for i in range(0, n_ncomp, 3)]
    off += n_ncomp * 8

    uvs = []
    if version >= 2:
        n_uv, = struct.unpack_from("<i", data, off)
        off += 4
        n_uvcomp = n_uv * 2
        if n_uvcomp > 0:
            raw_uv = struct.unpack_from("<" + "d" * n_uvcomp, data, off)
            uvs = [(raw_uv[i], raw_uv[i + 1]) for i in range(0, n_uvcomp, 2)]
        off += n_uvcomp * 8

    n_tris, = struct.unpack_from("<i", data, off)
    off += 4
    faces_by_group = {}
    for _ in range(n_tris):
        a, b, c, gid = struct.unpack_from("<iiii", data, off)
        off += 16
        faces_by_group.setdefault(gid, []).append((a, b, c))

    n_names, = struct.unpack_from("<i", data, off)
    off += 4
    names = {}
    surface_types = {}
    for _ in range(n_names):
        gid, = struct.unpack_from("<i", data, off)
        off += 4
        name, off = _read_str(data, off)
        names[gid] = name
        if version >= 3:
            surface_type, = struct.unpack_from("<i", data, off)
            off += 4
            surface_types[gid] = int(surface_type)

    for gid, faces in faces_by_group.items():
        name = names.get(gid, "group_{0}".format(gid))
        cv, cf, cn, cu = _compact(verts, faces, normals, uvs)
        patch = {"name": name, "vertices": cv, "faces": cf}
        if cn:
            patch["normals"] = cn
        if cu:
            patch["uvs"] = cu
        if version >= 3:
            patch["surface_type"] = surface_types.get(gid, 0)
        scene.patches.append(patch)

    n_curves, = struct.unpack_from("<i", data, off)
    off += 4
    for _ in range(n_curves):
        cname, off = _read_str(data, off)
        edge_type = 0
        if version >= 3:
            edge_type, = struct.unpack_from("<i", data, off)
            off += 4
        closed = data[off] != 0
        off += 1
        n_pts, = struct.unpack_from("<i", data, off)
        off += 4
        pts = []
        for _p in range(n_pts):
            x, y, z = struct.unpack_from("<ddd", data, off)
            off += 24
            pts.append((x, y, z))
        curve = {"name": cname, "points": pts, "closed": closed}
        if version >= 3:
            curve["edge_type"] = int(edge_type)
        scene.curves.append(curve)
        n_anchors, = struct.unpack_from("<i", data, off)
        off += 4
        for _a in range(n_anchors):
            pname, off = _read_str(data, off)
            x, y, z = struct.unpack_from("<ddd", data, off)
            off += 24
            scene.points.append({"name": pname, "position": (x, y, z)})
    return off


def _read_str(data, off):
    n, = struct.unpack_from("<i", data, off)
    off += 4
    s = data[off:off + n].decode("utf-8")
    return s, off + n


def _compact(verts, faces, normals=None, uvs=None):
    """Index like the C# GPU mesh: share verts with the same position, normal, and UV.

    Decompose keys on UV, so a smooth cylinder can arrive as a triangle soup.
    Without UVs, UV-only splits are welded so connectivity-based viewers stay smooth.
    When UVs are present they are kept, including seam splits, so the GL checkerboard
    can interpolate the real parameterization.
    Sharp edges keep separate verts — C# already stored different normals there.
    """
    remap = {}
    key_to = {}
    cv = []
    cn = []
    cu = []
    have_normals = bool(normals) and len(normals) == len(verts)
    have_uvs = bool(uvs) and len(uvs) == len(verts)
    for a, b, c in faces:
        for i in (a, b, c):
            if i in remap:
                continue
            v = verts[i]
            key = (v[0], v[1], v[2])
            if have_normals:
                n = normals[i]
                key += (n[0], n[1], n[2])
            if have_uvs:
                uv = uvs[i]
                key += (uv[0], uv[1])
            existing = key_to.get(key)
            if existing is None:
                existing = len(cv)
                cv.append(v)
                if have_normals:
                    cn.append(normals[i])
                if have_uvs:
                    cu.append(uvs[i])
                key_to[key] = existing
            remap[i] = existing
    cf = [(remap[a], remap[b], remap[c]) for a, b, c in faces]
    return cv, cf, cn, cu
