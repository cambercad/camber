import unittest

from camber import glview, pick


class NearestPickBvhTests(unittest.TestCase):
    def layers(self, count=50):
        vertices, faces = [], []
        for z in range(count, 0, -1):
            base = len(vertices)
            vertices.extend(((-1, -1, z), (1, -1, z), (0, 1, z)))
            faces.append((base, base+1, base+2))
        return vertices, faces, glview.build_triangle_cache(vertices, faces)

    def test_finds_frontmost_hit_from_both_sides(self):
        vertices, faces, cache = self.layers()
        self.assertEqual(0, glview.closest_triangle((0, 0, 100), (0, 0, -1), vertices, faces, cache))
        self.assertEqual(49, glview.closest_triangle((0, 0, 0), (0, 0, 1), vertices, faces, cache))

    def test_prunes_layers_behind_the_first_hit(self):
        vertices, faces, cache = self.layers(4096)
        self.assertEqual(4095, glview.closest_triangle((0, 0, 0), (0, 0, 1), vertices, faces, cache))
        from camber.measure import Geometry
        self.assertIsInstance(cache, Geometry)

    def test_pick_respects_the_live_clip_plane(self):
        vertices, faces, cache = self.layers()
        self.assertEqual(25, glview.closest_triangle((0, 0, 100), (0, 0, -1), vertices, faces, cache,
                                                    ((0, 0, 1), 25)))
        catalog = [{"kind": "point", "name": "hidden", "world": (0, 0, 3)},
                   {"kind": "curve", "name": "edge", "points": [(0, 0, -1), (0, 0, 2)]}]
        name, _, depth = pick.nearest_hit(catalog, (0, 0, 5), (0, 0, -1), .1, .1,
                                         clip_plane=((0, 0, 1), 0))
        self.assertEqual("edge", name)
        self.assertEqual(5, depth)
