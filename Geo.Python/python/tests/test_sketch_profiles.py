"""Analytical profile source and native display indices stay independent."""
import math
import unittest

from camber import Part, set_progress_log
from camber.sketch_curves import CURVES, apply_curve, validate
from camber.sketch_emit import emit_sketch_python, apply_actions
from camber.sketch_source import read_sketch_actions
from camber.feature_source import ModelDocument


class SketchProfileTests(unittest.TestCase):
    def setUp(self):
        set_progress_log(False)

    def test_every_profile_roundtrips_without_turning_it_into_lines(self):
        part = Part((-200, -200, -200), (200, 200, 200))
        for label, (method, fields) in CURVES.items():
            with self.subTest(profile=label):
                sketch = part.sketch("xy", constrained=True)
                action = {"kind": "curve_call", "method": method,
                          "arguments": {key: field[1] for key, field in fields.items()}}
                apply_curve(sketch, action)
                self.assertEqual(sketch.curve_count, len(sketch._solved_actions()))
                source = emit_sketch_python([action])
                restored = read_sketch_actions(source, "sk", sketch)
                self.assertEqual(method, restored[0]["method"])
                self.assertEqual(action["curve_count"], restored[0]["curve_count"])
                self.assertNotIn("add_sampled_curve", source if method != "add_sampled_curve" else "")
                copy = part.sketch("xy", constrained=True)
                apply_actions(copy, restored)
                self.assertEqual(sketch.curve_count, copy.curve_count)

    def test_analytical_control_profile_reopens_and_extrudes_to_same_volume(self):
        part = Part((-50, -50, -50), (50, 50, 50), tolerance=.01)
        sketch = part.sketch("xy", constrained=True)
        actions = [
            {"kind": "curve_call", "method": "add_bezier", "arguments": {"control_points": [(-20, 0), (-15, 15), (15, 15), (20, 0)]}},
            {"kind": "curve_call", "method": "add_bspline", "arguments": {"control_points": [(20, 0), (15, -5), (-15, -5), (-20, 0)], "degree": 3}},
        ]
        apply_actions(sketch, actions)
        solid = sketch.extrude(5)
        self.assertTrue(solid.is_watertight())
        restored = read_sketch_actions(emit_sketch_python(actions), "sk", sketch)
        another = part.sketch("xy", constrained=True)
        apply_actions(another, restored)
        self.assertAlmostEqual(solid.volume(), another.extrude(5).volume(), delta=.001)

    def test_invalid_coupled_parameters_are_rejected_before_mutation(self):
        with self.assertRaisesRegex(ValueError, "base circle"):
            validate(dict(teeth=30, pressure_angle=math.radians(20), addendum=1), "add_involute_internal_gear")
        with self.assertRaisesRegex(ValueError, "weight"):
            validate(dict(control_points=[(0, 0), (1, 1), (2, 0)], degree=2, weights=[1, 0, 1]), "add_bspline")
        with self.assertRaisesRegex(ValueError, "Degree"):
            validate(dict(control_points=[(0, 0), (1, 1)], degree=3), "add_bspline")

    def test_all_authored_profiles_remain_graphically_editable_after_loading(self):
        for label, (method, fields) in CURVES.items():
            with self.subTest(profile=label):
                action = dict(kind="curve_call", method=method,
                              arguments={key: field[1] for key, field in fields.items()})
                calls = emit_sketch_python([action], sketch_var="sk")
                source = "from camber import Part\n\ndef build_model():\n"
                source += "    # camber:begin setup\n    part = Part((-50,-50,-50),(50,50,50))\n    # camber:end setup\n"
                source += "    # camber:begin sketch\n" + "\n".join("    " + line for line in calls.splitlines())
                source += "\n    # camber:end sketch\n    return part, sk\n"
                block = next(block for block in ModelDocument(source).blocks if block.id == "sketch")
                self.assertTrue(block.editable, block.reason)


if __name__ == "__main__":
    unittest.main()
