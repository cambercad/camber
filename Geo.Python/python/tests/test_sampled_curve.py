import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from camber import Part, vec3, set_progress_log

set_progress_log(False)


class SampledCurveTests(unittest.TestCase):
    def test_sketch_add_sampled_curve_stores_one_native_curve(self):
        part = Part(vec3(-10, -10, -10), vec3(10, 10, 10))
        sketch = part.sketch(name="sampled")

        curve = sketch.add_sampled_curve([(0, 0), (1, 1), (2, 0), (3, 1)])

        self.assertEqual(1, sketch.curve_count)
        self.assertEqual("sampled", curve.kind)
        self.assertTrue(curve.name)
        self.assertEqual(curve.name, sketch._n.last_curve_name())


if __name__ == "__main__":
    unittest.main()
