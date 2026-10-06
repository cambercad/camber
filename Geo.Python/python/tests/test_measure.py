"""Closest distance regression cases; uses the same geometry as viewer picks."""
import math
import random
import unittest
from unittest.mock import patch

from camber import measure
from camber.measure import Geometry, closest_entities, named_geometry


class ClosestDistanceTests(unittest.TestCase):
    def test_native_queries_do_not_require_numpy(self):
        with patch.dict("sys.modules", {"numpy": None}):
            query = Geometry([((0, 0, 0), (1, 0, 0), (0, 1, 0))])
            self.assertEqual(0, query.ray((.2, .2, 1), (0, 0, -1))[0])
            self.assertEqual(2, closest_entities(query, [((.2, .2, 2),)]).length)

    def assert_distance(self, first, second, expected):
        for left, right in ((first, second), (second, first)):
            hit = closest_entities(left, right)
            self.assertAlmostEqual(expected, hit.length, delta=max(1e-14, expected*1e-12))
            self.assertAlmostEqual(hit.length, math.dist(hit.first, hit.second))
        return closest_entities(first, second)

    def test_all_six_entity_combinations(self):
        p = [((.25, .25, 3),)]
        e = [((0, 0, 2), (1, 0, 2))]
        f = [((0, 0, 0), (1, 0, 0), (0, 1, 0))]
        self.assert_distance(p, [((.25, .25, 7),)], 4)
        self.assert_distance(p, e, math.hypot(.25, 1))
        hit = self.assert_distance(p, f, 3)
        self.assertEqual((.25, .25, 0), hit.second)
        self.assert_distance(e, [((.5, -1, 4), (.5, 1, 4))], 2)
        self.assert_distance(e, f, 2)
        self.assert_distance(f, [((0, 0, 5), (1, 0, 5), (0, 1, 5))], 5)

    def test_segment_and_triangle_cross_in_their_interiors(self):
        self.assert_distance([((.2, .2, -1), (.2, .2, 1))],
                             [((0, 0, 0), (1, 0, 0), (0, 1, 0))], 0)

    def test_crossing_and_coplanar_overlapping_triangles(self):
        face = [((-1, -1, 0), (1, -1, 0), (0, 1, 0))]
        self.assert_distance(face, [((0, -1, -1), (0, 1, -1), (0, 0, 1))], 0)
        self.assert_distance(face, [((-.1, -.1, 0), (.1, -.1, 0), (0, .1, 0))], 0)

    def test_degenerate_triangles_and_zero_length_segments(self):
        self.assert_distance([((1, 1, 0),)], [((0, 0, 0), (0, 0, 0), (2, 0, 0))], 1)
        self.assert_distance([((1, 0, 0), (1, 0, 0))], [((0, 0, 0), (0, 0, 0))], 1)

    def test_small_nonzero_gap_is_not_reported_as_an_intersection(self):
        # The segment crosses the supporting plane, but misses the triangle.
        gap = 1e-12
        hit = closest_entities([((1+gap, 0, -1), (1+gap, 0, 1))],
                               [((0, 0, 0), (1, 0, 0), (0, 1, 0))])
        self.assertGreater(hit.length, 0)
        self.assertAlmostEqual(gap, hit.length, delta=1e-15)

    def test_distance_scales_and_translates_with_the_geometry(self):
        for scale, shift in ((1e-6, 0), (1, 1e9), (1e6, -1e9)):
            point = [(shift+.25*scale, .25*scale, 2*scale)]
            face = [(shift, 0, 0), (shift+scale, 0, 0), (shift, scale, 0)]
            self.assert_distance([tuple(point)], [tuple(face)], 2*scale)

    def test_bvh_result_matches_exhaustive_query_and_reuses_tree(self):
        rng = random.Random(103)
        segments = [((rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(-5, 5)),
                     (rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(-5, 5))) for _ in range(120)]
        point = ((1, 2, 3),)
        expected = min(closest_entities([segment], [point]).length for segment in segments)
        entity = Geometry(segments)
        hit = closest_entities(entity, [point])
        self.assertAlmostEqual(expected, hit.length)
        tree = entity.native
        closest_entities(entity, [point])
        self.assertIs(tree, entity.native)

    def test_bvh_prunes_distant_geometry(self):
        segments = Geometry([((i, 0, 0), (i, 1, 0)) for i in range(1000)])
        hit = closest_entities(segments, [((650.25, .5, 3),)])
        self.assertAlmostEqual(math.hypot(.25, 3), hit.length)

    def test_resolves_named_points_edges_faces_and_closed_curves(self):
        packed = {"catalog": [{"name": "p", "kind": "point", "world": (1, 2, 3)},
                              {"name": "e", "kind": "curve", "closed": True,
                               "points": [(0, 0, 0), (1, 0, 0), (0, 1, 0)]}],
                  "mesh_pos": [(0, 0, 0), (1, 0, 0), (0, 1, 0)],
                  "mesh_idx": [(0, 1, 2)], "face_names": ["f"]}
        self.assertEqual(1, len(named_geometry([packed], "p")))
        self.assertEqual(3, len(named_geometry([packed], "e")))
        self.assertEqual(1, len(named_geometry([packed], "f")))
        self.assertEqual([], named_geometry([packed], "missing"))

    def test_invalid_geometry_is_rejected(self):
        for geometry in ([((math.nan, 0, 0),)], [((0, 0),)], [tuple((0, 0, 0) for _ in range(4))]):
            with self.subTest(geometry=geometry), self.assertRaises(ValueError):
                Geometry(geometry)
        with self.assertRaises(ValueError):
            closest_entities([], [((0, 0, 0),)])
