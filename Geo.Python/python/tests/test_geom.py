import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from camber import (
    Part,
    convex_hull,
    is_ccw,
    point_in_polygon,
    signed_area,
    triangulate,
    vec2,
    vec3,
    set_progress_log,
)

set_progress_log(False)


class GeomTests(unittest.TestCase):
    def test_unit_square_area_and_fill(self):
        square = [(0, 0), (1, 0), (1, 1), (0, 1)]
        self.assertAlmostEqual(1.0, signed_area(square))
        self.assertTrue(is_ccw(square))
        pts, tris = triangulate(square)
        self.assertGreaterEqual(len(tris), 1)
        area = 0.0
        for i, j, k in tris:
            a, b, c = pts[i], pts[j], pts[k]
            area += 0.5 * ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y))
        self.assertAlmostEqual(1.0, abs(area), places=6)
        self.assertTrue(point_in_polygon(square, (0.5, 0.5)))
        self.assertFalse(point_in_polygon(square, (2, 2)))
        self.assertTrue(point_in_polygon(square, (0, 0)))
        self.assertFalse(point_in_polygon(square, (0, 0), on_edge=False))

    def test_hole_subtracts_area(self):
        outer = [(0, 0), (3, 0), (3, 3), (0, 3)]
        hole = [(1, 1), (1, 2), (2, 2), (2, 1)]
        pts, tris = triangulate(outer, hole)
        area = 0.0
        for i, j, k in tris:
            a, b, c = pts[i], pts[j], pts[k]
            area += 0.5 * ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y))
        self.assertAlmostEqual(8.0, abs(area), places=5)

    def test_convex_hull(self):
        hull = convex_hull([(0, 0), (1, 0), (0.5, 0.1), (1, 1), (0, 1)])
        self.assertEqual(4, len(hull))

    def test_working_volume_part_and_box(self):
        square = [(0, 0), (1, 0), (1, 1), (0, 1)]
        part = Part(vec3(-5), vec3(5), tolerance=0.01)
        self.assertAlmostEqual(1.0, signed_area(square, working_volume=part), places=4)
        self.assertAlmostEqual(
            1.0, signed_area(square, working_volume=((0, 0), (1, 1))), places=6)
        pts, tris = triangulate(square, working_volume=part)
        self.assertGreaterEqual(len(tris), 1)

    def test_cube_volume_mesh_raycast(self):
        part = Part(vec3(-5), vec3(5), tolerance=0.01)
        sk = part.sketch("xy")
        sk.add_rectangle((0, 0), (1, 1))
        cube = part.extrude(sk, 1.0)
        self.assertTrue(cube.is_watertight())
        self.assertAlmostEqual(1.0, cube.volume(), places=4)
        pts, tris = cube.mesh()
        self.assertGreaterEqual(len(pts), 4)
        self.assertGreaterEqual(len(tris), 4)
        hit = part.raycast(cube, (0.5, 0.5, -1), (0, 0, 1))
        self.assertIsNotNone(hit)
        self.assertAlmostEqual(0.0, hit.point.z, places=4)
        rebuilt = part.solid_from_mesh(pts, tris, name="copy")
        self.assertAlmostEqual(cube.volume(), rebuilt.volume(), places=3)

    def test_sketch_polylines(self):
        part = Part(vec3(-5), vec3(5), tolerance=0.01)
        sk = part.sketch("xy")
        sk.add_rectangle((0, 0), (1, 1))
        loops = sk.polylines()
        self.assertGreaterEqual(len(loops), 1)
        self.assertGreaterEqual(len(loops[0]), 4)

    def test_sketch_triangulate_nested(self):
        part = Part(vec3(-10), vec3(10), tolerance=0.01)
        sk = part.sketch("xy")
        sk.add_rectangle((0, 0), (3, 3))
        sk.add_rectangle((1, 1), (2, 2))
        sk.add_rectangle((1.25, 1.25), (1.75, 1.75))
        pts, tris = sk.triangulate()
        self.assertGreaterEqual(len(tris), 2)
        area = 0.0
        for i, j, k in tris:
            a, b, c = pts[i], pts[j], pts[k]
            area += 0.5 * ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y))
        self.assertAlmostEqual(8.25, abs(area), places=4)

    def test_sketch_surface_is_open_and_displayable(self):
        from camber.display import decode
        from camber.glview import pack_scene

        part = Part(vec3(-10), vec3(10), tolerance=0.01)
        sk = part.sketch("xy", name="washer")
        sk.add_rectangle((0, 0), (3, 3))
        sk.add_rectangle((1, 1), (2, 2))
        sheet = sk.surface(name="washer")
        self.assertFalse(sheet.is_volume)
        self.assertFalse(sheet.is_watertight())
        self.assertGreater(sheet.triangle_count, 0)
        scene = decode(sheet._n.dump_display())
        self.assertGreaterEqual(len(scene.patches), 1)
        packed = pack_scene(scene)
        self.assertGreaterEqual(len(packed["mesh_idx"]), 1)
        self.assertEqual(len(packed["mesh_pos"]), len(packed["mesh_uv"]))

    def test_vec_dot(self):
        self.assertEqual(11.0, vec2(1, 2).dot((3, 4)))
        self.assertEqual(32.0, vec3(1, 2, 3).dot((4, 5, 6)))


if __name__ == "__main__":
    unittest.main()
