import unittest

from camber import Curve3D, Part, Solid, Surface, set_progress_log


class SurfaceTests(unittest.TestCase):
    def setUp(self):
        set_progress_log(False)
        self.part = Part((-4, -4, -4), (4, 4, 4), tolerance=.01)

    def test_open_mesh_is_surface_and_trims_solid_on_selected_side(self):
        surface = self.part.solid_from_mesh(
            [(-3, -3, 0), (3, -3, 0), (3, 3, 0), (-3, 3, 0)],
            [(0, 1, 2), (0, 2, 3)], name="splitter")
        body = self.part.cuboid((-1, -1, -1), (1, 1, 1), name="body")

        self.assertIsInstance(surface, Surface)
        self.assertNotIsInstance(surface, Solid)
        self.assertFalse(surface.is_volume)
        self.assertIsInstance(self.part.solid("splitter"), Surface)

        upper = body.trim(surface, side="normal", name="upper")
        lower = body.trim(surface, side="opposite", name="lower")
        self.assertIsInstance(upper, Solid)
        self.assertTrue(upper.is_volume)
        self.assertAlmostEqual(4.0, upper.volume(), places=3)
        self.assertAlmostEqual(4.0, lower.volume(), places=3)
        self.assertEqual("Trim", self.part.operations[-1].kind)
        self.assertEqual(("body", "splitter"), self.part.operations[-1].inputs)
        self.assertEqual("side=opposite", self.part.operations[-1].details)

    def test_trim_requires_surface_from_the_same_part(self):
        body = self.part.cube((0, 0, 0), 2, name="body")
        with self.assertRaises(TypeError):
            body.trim(body)

    def test_surface_thicken_returns_watertight_solid(self):
        surface = self.part.solid_from_mesh(
            [(-2, -1, 0), (2, -1, 0), (2, 1, 0), (-2, 1, 0)],
            [(0, 1, 2), (0, 2, 3)], name="sheet")

        result = surface.thicken(0.5, name="plate")

        self.assertIsInstance(result, Solid)
        self.assertTrue(result.is_volume)
        self.assertAlmostEqual(4.0, result.volume(), places=2)
        self.assertEqual("Thicken", self.part.operations[-1].kind)
        self.assertIn("bothSides=False", self.part.operations[-1].details)

    def test_surface_split_keeps_both_resolver_trimmed_sides(self):
        surface = self.part.solid_from_mesh(
            [(-2, -2, 0), (2, -2, 0), (2, 2, 0), (-2, 2, 0)],
            [(0, 1, 2), (0, 2, 3)], name="sheet")
        cutter = self.part.solid_from_mesh(
            [(0, -3, -1), (0, 3, -1), (0, 3, 1), (0, -3, 1)],
            [(0, 1, 2), (0, 2, 3)], name="cutter")

        halves = surface.split(cutter, name="halves")

        self.assertEqual(2, len(halves))
        self.assertTrue(all(isinstance(half, Surface) and half.triangle_count > 0 for half in halves))
        self.assertEqual("Surface split", self.part.operations[-1].kind)

    def test_exactly_coincident_faces_can_be_sewn_into_a_solid(self):
        source = self.part.cube((0, 0, 0), 2, name="source")
        faces = [source.face_surface(name) for name in source.patch_names]

        result = self.part.sew(faces, make_solid=True, name="sewn")

        self.assertIsInstance(result, Solid)
        self.assertTrue(result.is_volume)
        self.assertAlmostEqual(8.0, result.volume(), places=3)

    def test_planar_boundary_cap_returns_solid(self):
        sketch = self.part.sketch("xy", name="profile")
        sketch.add_rectangle((0, 0), (3, 2))
        sides = sketch.extrude_surface(5, name="sides")

        result = sides.cap_planar_boundaries(name="capped")

        self.assertIsInstance(result, Solid)
        self.assertTrue(result.is_volume)
        self.assertAlmostEqual(30.0, result.volume(), places=2)

    def test_sweep_and_surface_sweep_accept_connected_guide_curves(self):
        profile = self.part.sketch("xy", name="sweep_profile")
        profile.add_rectangle((-.5, -.5), (.5, .5))
        guides = [
            Curve3D.line((0, 0, 0), (0, 0, 3), name="rise"),
            Curve3D.line((0, 0, 3), (3, 0, 3), name="turn"),
        ]

        solid = profile.extrude_along_curve_strip(
            guides, name="sweep", reference_direction=(0, 1, 0))
        sheet = profile.sweep_surface(guides, name="sweep_sheet")

        self.assertIsInstance(solid, Solid)
        self.assertTrue(solid.is_volume)
        self.assertGreater(solid.volume(), 0)
        self.assertIsInstance(sheet, Surface)
        self.assertFalse(sheet.is_volume)

    def test_surface_solid_intersection_returns_generic_3d_curve(self):
        surface = self.part.solid_from_mesh(
            [(-3, -3, 0), (3, -3, 0), (3, 3, 0), (-3, 3, 0)],
            [(0, 1, 2), (0, 2, 3)], name="splitter")
        body = self.part.cube((0, 0, 0), 2, name="body")

        curves = surface.intersection_curves(body, name="section")

        self.assertEqual(1, len(curves))
        self.assertIsInstance(curves[0], Curve3D)
        points = curves[0].tessellate()
        self.assertGreaterEqual(len(points), 5)
        self.assertEqual("section", curves[0].name)
        self.assertLess((points[0] - points[-1]).norm(), self.part.max_deviation * 2)

    def test_surface_surface_intersection_returns_multiple_generic_curves(self):
        horizontal = self.part.solid_from_mesh(
            [(-3, -3, 0), (3, -3, 0), (3, 3, 0), (-3, 3, 0)],
            [(0, 1, 2), (0, 2, 3)], name="horizontal")
        vertical = self.part.solid_from_mesh(
            [(0, -3, -1), (0, 3, -1), (0, 3, 1), (0, -3, 1)],
            [(0, 1, 2), (0, 2, 3)], name="vertical")

        curves = horizontal.intersection_curves(vertical, name="crossing")

        self.assertEqual(1, len(curves))
        self.assertIsInstance(curves[0], Curve3D)
        points = curves[0].tessellate()
        self.assertGreaterEqual(len(points), 2)
        self.assertAlmostEqual(0, points[0].x, places=5)
        self.assertAlmostEqual(0, points[-1].x, places=5)

    def test_curve3d_tessellate_validates_tolerance(self):
        curve = Curve3D.circle((0, 0, 0), (1, 0, 0), (0, 1, 0), 1)
        points = curve.tessellate(.02)

        self.assertGreaterEqual(len(points), 8)
        with self.assertRaises(ValueError):
            curve.tessellate(0)

    def test_curve3d_wraps_sampled_and_torus_knot_implementations(self):
        sampled = Curve3D.sampled([(0, 0, 0), (1, 0, 0), (1, 1, 0)])
        knot = Curve3D.torus_knot(2, 3, 1)
        framed_line = Curve3D.line((0, 0, 0), (2, 0, 0), up=(0, 0, 1))

        self.assertIsInstance(sampled, Curve3D)
        self.assertEqual(3, len(sampled.tessellate()))
        self.assertIsInstance(knot, Curve3D)
        self.assertGreater(len(knot.tessellate(.01)), 100)
        self.assertIsInstance(framed_line, Curve3D)
        with self.assertRaises(Exception):
            Curve3D.line((0, 0, 0), (2, 0, 0), up=(1, 0, 0))

    def test_intersection_curves_returns_one_curve_per_disconnected_strip(self):
        horizontal = self.part.solid_from_mesh(
            [(-3, -3, 0), (3, -3, 0), (3, 3, 0), (-3, 3, 0)],
            [(0, 1, 2), (0, 2, 3)], name="horizontal")
        points = [
            (0, -3, -1), (0, -1, -1), (0, -1, 1), (0, -3, 1),
            (0, 1, -1), (0, 3, -1), (0, 3, 1), (0, 1, 1),
        ]
        vertical = self.part.solid_from_mesh(
            points,
            [(0, 1, 2), (0, 2, 3), (4, 5, 6), (4, 6, 7)], name="vertical")

        curves = horizontal.intersection_curves(vertical, name="crossing")

        self.assertEqual(2, len(curves))
        self.assertTrue(all(isinstance(curve, Curve3D) for curve in curves))
        self.assertEqual(["crossing_1", "crossing_2"], [curve.name for curve in curves])


if __name__ == "__main__":
    unittest.main()
