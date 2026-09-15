import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from camber import pick
from camber.display import DisplayScene
from camber import sketch_ui
from camber.sketch_ui import build_sketch_pick_catalog


class _Frame(object):
    def __init__(self):
        self.origin = (0.0, 0.0, 0.0)
        self.x = (1.0, 0.0, 0.0)
        self.y = (0.0, 1.0, 0.0)
        self.z = (0.0, 0.0, 1.0)


class RayPickTests(unittest.TestCase):
    def test_ray_hits_named_point(self):
        targets = [
            {"name": "box:[A,B]@0.000", "kind": pick.KIND_POINT, "world": (0.0, 0.0, 0.0)},
            {"name": "box:[A,B]@1.000", "kind": pick.KIND_POINT, "world": (10.0, 0.0, 0.0)},
        ]
        hit = pick.closest((0.0, 0.0, 10.0), (0.0, 0.0, -1.0), targets, 0.25, 0.25)
        self.assertEqual("box:[A,B]@0.000", hit)

    def test_nearer_point_wins(self):
        targets = [
            {"name": "far", "kind": pick.KIND_POINT, "world": (2.0, 0.0, 0.0)},
            {"name": "near", "kind": pick.KIND_POINT, "world": (0.2, 0.0, 0.0)},
        ]
        hit = pick.closest((0.0, 0.0, 10.0), (0.0, 0.0, -1.0), targets, 3.0, 3.0)
        self.assertEqual("near", hit)

    def test_point_wins_over_nearby_curve(self):
        targets = [
            {
                "name": "Sketch1:Line1",
                "kind": pick.KIND_CURVE,
                "points": [(0.0, 0.0, 0.0), (4.0, 0.0, 0.0)],
            },
            {"name": "Sketch1:Line1@1.000", "kind": pick.KIND_POINT, "world": (4.0, 0.0, 0.0)},
        ]
        hit = pick.closest((4.0, 0.0, 10.0), (0.0, 0.0, -1.0), targets, 0.35, 0.35)
        self.assertEqual("Sketch1:Line1@1.000", hit)

    def test_curve_hit_when_ray_misses_points(self):
        targets = [
            {"name": "Sketch1:Line1@0.000", "kind": pick.KIND_POINT, "world": (0.0, 0.0, 0.0)},
            {"name": "Sketch1:Line1@1.000", "kind": pick.KIND_POINT, "world": (4.0, 0.0, 0.0)},
            {
                "name": "Sketch1:Line1",
                "kind": pick.KIND_CURVE,
                "points": [(0.0, 0.0, 0.0), (4.0, 0.0, 0.0)],
            },
        ]
        hit = pick.closest((2.0, 0.0, 10.0), (0.0, 0.0, -1.0), targets, 0.2, 0.2)
        self.assertEqual("Sketch1:Line1", hit)

    def test_miss_and_behind_camera(self):
        targets = [
            {"name": "behind", "kind": pick.KIND_POINT, "world": (0.0, 0.0, 20.0)},
            {"name": "ahead", "kind": pick.KIND_POINT, "world": (0.0, 0.0, 0.0)},
        ]
        self.assertEqual(
            "ahead",
            pick.closest((0.0, 0.0, 10.0), (0.0, 0.0, -1.0), targets, 0.5, 0.5),
        )
        self.assertIsNone(
            pick.closest((0.0, 0.0, 10.0), (0.0, 0.0, -1.0), [
                {"name": "only", "kind": pick.KIND_POINT, "world": (10.0, 10.0, 0.0)},
            ], 0.2, 0.2)
        )

    def test_catalog_assigns_unique_names_and_pick_returns_them(self):
        actions = [
            {"kind": "line", "name": "Line1", "p0": (0.0, 0.0), "p1": (4.0, 0.0)},
            {"kind": "circle", "name": "Circle1", "center": (2.0, 2.0), "radius": 1.0},
        ]
        part_points = [((1.0, 0.0), (1.0, 0.0, 0.0), "box:[A,B]@0.500")]
        catalog = build_sketch_pick_catalog(actions, part_points, _Frame(), 0.0, 0.0, "Sketch1")
        names = [item["name"] for item in catalog]
        self.assertIn("Sketch1:Line1", names)
        self.assertIn("Sketch1:Line1@0.000", names)
        self.assertIn("Sketch1:Line1@1.000", names)
        self.assertIn("Sketch1:Circle1@center", names)
        self.assertIn("Sketch1:Circle1@0.000", names)
        self.assertIn("Sketch1:origin", names)
        self.assertIn("box:[A,B]@0.500", names)

        end = pick.closest(
            (4.0, 0.0, 10.0), (0.0, 0.0, -1.0), catalog, 0.25, 0.25)
        self.assertEqual("Sketch1:Line1@1.000", end)
        body = pick.closest(
            (1.0, 0.0, 10.0), (0.0, 0.0, -1.0), catalog, 0.25, 0.25)
        self.assertEqual("box:[A,B]@0.500", body)
        east = pick.closest(
            (3.0, 2.0, 10.0), (0.0, 0.0, -1.0), catalog, 0.25, 0.25)
        self.assertEqual("Sketch1:Circle1@0.000", east)


class PreferSelectionTests(unittest.TestCase):
    def test_catalog_curve_wins_over_nearby_surface(self):
        self.assertEqual(
            "box:[A,B]",
            pick.prefer_selection("box:[A,B]", pick.KIND_CURVE, "box:top", pick.KIND_SURFACE),
        )

    def test_point_still_wins_over_surface(self):
        self.assertEqual(
            "box:[A,B]@0.500",
            pick.prefer_selection(
                "box:[A,B]@0.500", pick.KIND_POINT, "box:top", pick.KIND_SURFACE),
        )

    def test_catalog_curve_used_when_mesh_is_not_a_surface(self):
        self.assertEqual(
            "box:[A,B]",
            pick.prefer_selection("box:[A,B]", pick.KIND_CURVE, None, None),
        )

    def test_surface_used_when_catalog_misses(self):
        self.assertEqual(
            "box:top",
            pick.prefer_selection(None, None, "box:top", pick.KIND_SURFACE),
        )


class SharedCatalogTests(unittest.TestCase):
    def test_scene_includes_off_plane_named_points(self):
        scene = DisplayScene()
        scene.points = [
            {"name": "box:[A,B]@0.000", "position": (0.0, 0.0, 0.0)},
            {"name": "box:[C,D]@1.000", "position": (4.0, 2.0, 7.5)},
        ]
        catalog = pick.catalog_from_scene(scene, _Frame())
        names = [item["name"] for item in catalog]
        self.assertIn("box:[A,B]@0.000", names)
        self.assertIn("box:[C,D]@1.000", names)
        off = pick.entry_by_name(catalog, "box:[C,D]@1.000")
        self.assertEqual(pick.KIND_POINT, off["kind"])
        self.assertEqual(("xy", 4.0, 2.0), off["ref"])
        self.assertAlmostEqual(7.5, off["world"][2])

    def test_ray_hits_off_plane_model_point_and_sketch_keeps_the_name(self):
        scene = DisplayScene()
        scene.points = [{"name": "box:[A,B]@0.500", "position": (1.5, -0.25, 9.0)}]
        catalog = pick.build_catalog(
            [{"kind": "line", "name": "Line1", "p0": (0.0, 0.0), "p1": (4.0, 0.0)}],
            _Frame(), 0.0, 0.0, "Sketch1", scene=scene,
        )
        name = pick.hit(
            catalog, (1.5, -0.25, 20.0), (0.0, 0.0, -1.0), 0.3, 0.3, mode=pick.MODE_POINTS,
        )
        self.assertEqual("box:[A,B]@0.500", name)
        state = {"pick_catalog": catalog, "grid": 1.0, "sketch_name": "Sketch1"}
        picked = sketch_ui._hit_at(state, ((1.5, -0.25, 20.0), (0.0, 0.0, -1.0)), 0.3, sketch_ui._PICK_POINTS)
        self.assertEqual("box:[A,B]@0.500", picked)
        self.assertEqual("box:[A,B]@0.500", sketch_ui._hit_as_point(state, picked))
        self.assertEqual(("xy", 1.5, -0.25), pick.entry_by_name(catalog, picked)["ref"])

    def test_sketch_handle_wins_name_collision_with_model_point(self):
        scene = DisplayScene()
        scene.points = [{"name": "Sketch1:Origin", "position": (9.0, 8.0, 7.0)}]
        catalog = pick.build_catalog([], _Frame(), 0.0, 0.0, "Sketch1", scene=scene)
        origin = pick.entry_by_name(catalog, "Sketch1:origin")
        self.assertEqual(pick.SOURCE_SKETCH, origin["source"])
        self.assertEqual((0.0, 0.0, 0.0), origin["world"])
        self.assertEqual(("xy", 0.0, 0.0), origin["ref"])
        hit = pick.hit(catalog, (0.0, 0.0, 10.0), (0.0, 0.0, -1.0), 0.25, 0.25)
        self.assertEqual("Sketch1:origin", hit)

    def test_viewer_and_sketch_catalogs_hit_the_same_model_names(self):
        scene = DisplayScene()
        scene.curves = [{
            "name": "box:[A,B]",
            "points": [(2.0, 1.0, 1.0), (8.0, 1.0, 1.0)],
            "closed": False,
        }]
        scene.points = [
            {"name": "box:[A,B]@0.000", "position": (2.0, 1.0, 1.0)},
            {"name": "box:[A,B]@1.000", "position": (8.0, 1.0, 1.0)},
        ]
        viewer = pick.catalog_from_scene(scene)
        sketch = pick.build_catalog([], _Frame(), 0.0, 0.0, "Sketch1", scene=scene)
        ray_pt = ((2.0, 1.0, 10.0), (0.0, 0.0, -1.0))
        ray_curve = ((5.0, 1.0, 10.0), (0.0, 0.0, -1.0))
        self.assertEqual(
            "box:[A,B]@0.000",
            pick.hit(viewer, ray_pt[0], ray_pt[1], 0.25, 0.25),
        )
        self.assertEqual(
            "box:[A,B]@0.000",
            pick.hit(sketch, ray_pt[0], ray_pt[1], 0.25, 0.25),
        )
        self.assertEqual(
            "box:[A,B]",
            pick.hit(viewer, ray_curve[0], ray_curve[1], 0.2, 0.2),
        )
        self.assertEqual(
            "box:[A,B]",
            pick.hit(sketch, ray_curve[0], ray_curve[1], 0.2, 0.2),
        )

    def test_model_curve_is_not_a_sketch_line_slot(self):
        scene = DisplayScene()
        scene.curves = [{
            "name": "box:[A,B]",
            "points": [(0.0, 0.0, 0.0), (4.0, 0.0, 0.0)],
            "closed": False,
        }]
        catalog = pick.catalog_from_scene(scene)
        self.assertEqual(
            "box:[A,B]",
            pick.hit(catalog, (2.0, 0.0, 10.0), (0.0, 0.0, -1.0), 0.25, 0.25, mode=pick.MODE_ANY),
        )
        self.assertIsNone(
            pick.hit(catalog, (2.0, 0.0, 10.0), (0.0, 0.0, -1.0), 0.25, 0.25, mode=pick.MODE_LINES),
        )

    def test_viewer_uniquify_matches_reserved_patch_names(self):
        scene = DisplayScene()
        scene.points = [{"name": "Face1", "position": (0.0, 0.0, 0.0)}]
        catalog = pick.catalog_from_scene(scene, reserved_names=set(["Face1"]))
        self.assertEqual("Face1_2", catalog[0]["name"])
        raw = pick.catalog_from_scene(scene)
        self.assertEqual("Face1", raw[0]["name"])

    def test_project_to_uv_and_exclude(self):
        frame = _Frame()
        uv = pick.project_to_uv((3.0, 4.0, 9.0), frame)
        self.assertEqual((3.0, 4.0), uv)
        world = pick.uv_to_world((3.0, 4.0), frame, lift=2.0)
        self.assertEqual((3.0, 4.0, 2.0), world)
        catalog = [
            pick.point_entry("keep", (0.0, 0.0, 0.0)),
            pick.point_entry("skip", (0.1, 0.0, 0.0)),
        ]
        self.assertEqual(
            "keep",
            pick.hit(catalog, (0.1, 0.0, 10.0), (0.0, 0.0, -1.0), 0.5, 0.5, exclude=("skip",)),
        )


if __name__ == "__main__":
    unittest.main()
