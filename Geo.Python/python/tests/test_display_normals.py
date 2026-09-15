import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from camber.display import _compact, decode
from camber.view import _concat_patches


def _pack_mesh(verts, normals, faces, names, uvs=None, version=1):
    buf = bytearray(b"CMBR")
    buf += struct.pack("<II", version, 1)
    buf += struct.pack("<i", len(verts))
    for x, y, z in verts:
        buf += struct.pack("<ddd", x, y, z)
    buf += struct.pack("<i", len(normals))
    for x, y, z in normals:
        buf += struct.pack("<ddd", x, y, z)
    if version >= 2:
        uv_list = uvs or []
        buf += struct.pack("<i", len(uv_list))
        for u, v in uv_list:
            buf += struct.pack("<dd", u, v)
    buf += struct.pack("<i", len(faces))
    for a, b, c, gid in faces:
        buf += struct.pack("<iiii", a, b, c, gid)
    buf += struct.pack("<i", len(names))
    for gid, name in names:
        raw = name.encode("utf-8")
        buf += struct.pack("<ii", gid, len(raw))
        buf += raw
    buf += struct.pack("<i", 0)
    return buf


class DisplayNormalTests(unittest.TestCase):
    def test_compact_keeps_split_vertices_at_the_same_point(self):
        verts = [
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 1.0),
            (0.0, 1.0, 0.0),
        ]
        normals = [
            (0.0, 0.0, 1.0),
            (0.0, 0.0, 1.0),
            (0.0, 0.0, 1.0),
            (1.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
        ]
        cv, cf, cn, cu = _compact(verts, [(0, 1, 2), (3, 4, 5)], normals)
        self.assertEqual(6, len(cv))
        self.assertEqual(6, len(cn))
        self.assertEqual([], cu)
        self.assertEqual((0.0, 0.0, 1.0), cn[0])
        self.assertEqual((1.0, 0.0, 0.0), cn[3])
        self.assertEqual(cv[0], cv[3])
        self.assertNotEqual(cf[0][0], cf[1][0])

    def test_compact_welds_uv_only_splits_with_the_same_normal(self):
        verts = [
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (1.0, 1.0, 0.0),
        ]
        up = (0.0, 0.0, 1.0)
        cv, cf, cn, cu = _compact(verts, [(0, 1, 2), (3, 4, 5)], [up] * 6)
        self.assertEqual(4, len(cv))
        self.assertEqual(4, len(cn))
        self.assertEqual([], cu)
        self.assertEqual(cf[0][0], cf[1][0])
        self.assertEqual(cf[0][1], cf[1][1])

    def test_compact_keeps_uv_only_splits_when_uvs_are_present(self):
        verts = [
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (1.0, 1.0, 0.0),
        ]
        up = (0.0, 0.0, 1.0)
        uvs = [
            (0.0, 0.0),
            (1.0, 0.0),
            (0.0, 1.0),
            (1.0, 0.0),
            (0.0, 0.0),
            (0.0, 1.0),
        ]
        cv, cf, cn, cu = _compact(verts, [(0, 1, 2), (3, 4, 5)], [up] * 6, uvs)
        self.assertEqual(6, len(cv))
        self.assertEqual(6, len(cu))
        self.assertEqual((1.0, 0.0), cu[3])
        self.assertNotEqual(cf[0][0], cf[1][0])

    def test_decode_reads_csharp_normals_and_keeps_sharp_splits(self):
        blob = _pack_mesh(
            [
                (0.0, 0.0, 0.0),
                (1.0, 0.0, 0.0),
                (0.0, 1.0, 0.0),
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 1.0),
                (0.0, 1.0, 0.0),
            ],
            [
                (0.0, 0.0, 1.0),
                (0.0, 0.0, 1.0),
                (0.0, 0.0, 1.0),
                (1.0, 0.0, 0.0),
                (1.0, 0.0, 0.0),
                (1.0, 0.0, 0.0),
            ],
            [(0, 1, 2, 0), (3, 4, 5, 0)],
            [(0, "box:corner")],
        )
        scene = decode(blob)
        self.assertEqual(1, len(scene.patches))
        patch = scene.patches[0]
        self.assertEqual(6, len(patch["vertices"]))
        self.assertEqual(6, len(patch["normals"]))
        self.assertEqual((1.0, 0.0, 0.0), patch["normals"][3])
        self.assertNotIn("uvs", patch)

    def test_decode_reads_version2_uvs(self):
        blob = _pack_mesh(
            [
                (0.0, 0.0, 0.0),
                (1.0, 0.0, 0.0),
                (0.0, 1.0, 0.0),
            ],
            [
                (0.0, 0.0, 1.0),
                (0.0, 0.0, 1.0),
                (0.0, 0.0, 1.0),
            ],
            [(0, 1, 2, 0)],
            [(0, "box:top")],
            uvs=[(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)],
            version=2,
        )
        scene = decode(blob)
        self.assertEqual(1, len(scene.patches))
        patch = scene.patches[0]
        self.assertEqual([(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)], patch["uvs"])

    def test_concat_preserves_split_normals_across_patches(self):
        patches = [
            {
                "name": "a",
                "vertices": [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)],
                "normals": [(0.0, 0.0, 1.0), (0.0, 0.0, 1.0), (0.0, 0.0, 1.0)],
                "faces": [(0, 1, 2)],
            },
            {
                "name": "b",
                "vertices": [(0.0, 0.0, 0.0), (0.0, 0.0, 1.0), (0.0, 1.0, 0.0)],
                "normals": [(1.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 0.0, 0.0)],
                "faces": [(0, 1, 2)],
            },
        ]
        verts, faces, names, normals = _concat_patches(patches, set())
        self.assertEqual(6, len(verts))
        self.assertEqual(6, len(normals))
        self.assertEqual(verts[0], verts[3])
        self.assertNotEqual(normals[0], normals[3])
        self.assertEqual(["a", "b"], names)


if __name__ == "__main__":
    unittest.main()
