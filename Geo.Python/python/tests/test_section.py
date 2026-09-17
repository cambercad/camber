import math
import os
import tempfile
import unittest
from dataclasses import FrozenInstanceError
from pathlib import Path

from camber import Frame, Part, Section, section, set_progress_log
from camber.inspection import _scale_bar


class MeasurementTests(unittest.TestCase):
    def test_world_local_points_and_immutable_results(self):
        cut = Section(None, Frame((10, 20, 30), x=(0, 1, 0), y=(0, 0, 1), z=(1, 0, 0)))
        m = cut.measure((0, 0), (3, 4), label="Diagonal")
        self.assertEqual((10, 20, 30), m.start)
        self.assertEqual((10, 23, 34), m.end)
        self.assertEqual(5, m.length)
        self.assertEqual((m,), cut.measurements)
        with self.assertRaises(FrozenInstanceError):
            m.length = 6
        with self.assertRaises(ValueError):
            cut.measure((math.nan, 0), (0, 0))
        cut.plane.origin.x = 100
        self.assertEqual(10, cut.plane.origin.x)

    def test_scale_bars_track_visible_model_span(self):
        for span in (.0003, .8, 15, 1300, 1e10):
            bar = _scale_bar(span)
            self.assertLessEqual(bar, span / 5)
            self.assertGreater(bar, span / 15)


class SectionTests(unittest.TestCase):
    def setUp(self):
        from camber.api import _native_mod
        _native_mod()["NativePart"].clear()
        set_progress_log(False)
        self.part = Part((-20, -20, -20), (20, 20, 20), tolerance=.01)
        self.solid = self.part.cuboid((-2, -2, -2), (2, 2, 2), name="block")

    def tearDown(self):
        from camber.api import _native_mod
        _native_mod()["NativePart"].clear()

    def test_named_planes_and_actual_ray_measurements(self):
        before = self.solid.mesh()
        for name, direction in (("XY", (0, 0, 1)), ("XZ", (0, -1, 0)), ("YZ", (1, 0, 0))):
            with self.subTest(plane=name):
                cut = section(self.solid, name, offset=1)
                a = cut.raycast(tuple(5*v for v in direction), tuple(-v for v in direction))
                b = cut.raycast(tuple(-5*v for v in direction), direction)
                self.assertIsNotNone(a)
                self.assertIsNotNone(b)
                self.assertAlmostEqual(3, cut.measure(a, b).length, places=4)
        self.assertEqual(before, self.solid.mesh())

    def test_invalid_inputs_and_empty_section(self):
        for kwargs in ({"plane": "bad"}, {"offset": math.inf}, {"plane": Frame(x=(math.nan, 0, 0))}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                section(self.solid, **kwargs)
        cut = section(self.solid, offset=-3)
        self.assertIsNone(cut.raycast((0, 0, 10), (0, 0, -1)))
        with self.assertRaises(ValueError):
            cut.raycast((0, 0, 0), (0, 0, 0))

    @unittest.skipUnless(os.environ.get("CAMBER_TEST_GL") == "1", "requires existing GL renderer")
    def test_annotated_section_capture(self):
        from camber import render_views
        cut = section(self.solid)
        cut.measure((-2, 0), (2, 0), label="Width")
        with tempfile.TemporaryDirectory() as folder:
            path = render_views(cut, Path(folder)/"section.png", tile_size=(640, 480))
            self.assertEqual(b"\x89PNG\r\n\x1a\n", path.read_bytes()[:8])
