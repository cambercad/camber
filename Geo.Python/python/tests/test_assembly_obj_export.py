import math
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from camber import Part, set_progress_log

set_progress_log(False)


class AssemblyObjExportTests(unittest.TestCase):
    def test_nested_rotated_instances_keep_world_positions_and_names(self):
        part = Part((-20, -20, -20), (20, 20, 20), tolerance=.01)
        block = part.cuboid((0, 0, 0), (1, 2, 3), name="block")
        child = part.assembly("child")
        child.add_part(block, (1, 0, 0))
        top = part.assembly("top")
        top.add_part(block, (-5, 0, 0))
        half = math.sqrt(.5)
        top.add_subassembly(child, (0, 10, 0), (0, 0, half, half))

        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "assembly.obj")
            top.save_obj(path)
            with open(path, encoding="utf-8") as source:
                lines = source.read().splitlines()

        names = [line[2:] for line in lines if line.startswith("o ")]
        self.assertEqual(["top/block[1]", "top/child[1]/block[1]"], names)
        vertices = [tuple(map(float, line.split()[1:])) for line in lines if line.startswith("v ")]
        uvs = [tuple(map(float, line.split()[1:])) for line in lines if line.startswith("vt ")]
        normals = [tuple(map(float, line.split()[1:])) for line in lines if line.startswith("vn ")]
        faces = [line.split()[1:] for line in lines if line.startswith("f ")]
        self.assertEqual(len(vertices), len(uvs))
        self.assertEqual(len(vertices), len(normals))
        self.assertEqual(2 * len(block.mesh()[1]), len(faces))
        for face in faces:
            for corner in face:
                position, uv, normal = map(int, corner.split("/"))
                self.assertEqual(position, uv)
                self.assertEqual(position, normal)
                self.assertTrue(1 <= position <= len(vertices))
        self.assertTrue(any(a == b and normals[i] != normals[j]
                            for i, a in enumerate(vertices)
                            for j, b in enumerate(vertices[:i])))
        split = len(vertices) // 2
        direct, nested = vertices[:split], vertices[split:]
        self.assertAlmostEqual(-5, min(x for x, _, _ in direct), delta=.01)
        self.assertAlmostEqual(-4, max(x for x, _, _ in direct), delta=.01)
        self.assertAlmostEqual(-2, min(x for x, _, _ in nested), delta=.01)
        self.assertAlmostEqual(0, max(x for x, _, _ in nested), delta=.01)
        self.assertAlmostEqual(11, min(y for _, y, _ in nested), delta=.01)
        self.assertAlmostEqual(12, max(y for _, y, _ in nested), delta=.01)


if __name__ == "__main__":
    unittest.main()
