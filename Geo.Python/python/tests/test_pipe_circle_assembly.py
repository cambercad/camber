import contextlib
import io
import os
import runpy
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from camber import Assembly, set_progress_log


class PipeCircleAssemblyTests(unittest.TestCase):
    def test_named_flange_faces_form_a_closed_ring(self):
        examples = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "examples", "standalone"))
        sys.path.insert(0, examples)
        set_progress_log(False)
        try:
            with patch.object(Assembly, "show"), contextlib.redirect_stdout(io.StringIO()):
                scene = runpy.run_path(os.path.join(examples, "pipe_circle_assembly.py"))
        finally:
            sys.path.remove(examples)

        self.assertTrue(scene["result"].converged)
        positions = [body.pose for body in scene["bodies"]]
        for actual, expected in zip(positions, ((0, 0), (-410, 410), (0, 820), (410, 410))):
            self.assertAlmostEqual(expected[0], actual[0], delta=0.05)
            self.assertAlmostEqual(expected[1], actual[1], delta=0.05)

        records = scene["assembly"].constraints[1:]
        self.assertEqual(12, len(records))
        for i in range(0, len(records), 3):
            self.assertEqual((-1, 1, 1), tuple(record.value for record in records[i:i + 3]))
            self.assertIn("-ExtrudeTop", records[i].entities[0])
            self.assertIn("-ExtrudeBottom", records[i].entities[1])
            for record in records[i:i + 3]:
                a, b = record.first, record.second
                direction_dot = sum(x * y for x, y in zip(a.world_direction, b.world_direction))
                plane_gap = sum(n * (y - x) for n, x, y in zip(a.world_direction, a.world_point, b.world_point))
                self.assertAlmostEqual(record.value, direction_dot, delta=1e-5)
                self.assertAlmostEqual(0, plane_gap, delta=0.01)
