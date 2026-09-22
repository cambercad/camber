import unittest

from camber import Part, Solid, Surface, set_progress_log


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


if __name__ == "__main__":
    unittest.main()
