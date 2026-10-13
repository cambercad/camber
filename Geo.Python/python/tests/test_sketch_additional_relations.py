"""Additional relation source, signed values and editable glyphs."""
import unittest

from camber import Part, set_progress_log
from camber.sketch_emit import apply_actions, emit_sketch_python
from camber.sketch_source import read_sketch_actions
from camber.sketch_icons import dimension_error
from camber.sketch_constraints_viz import build_constraint_overlay, DIMENSION_KINDS


class AdditionalSketchRelationTests(unittest.TestCase):
    def test_signed_offset_and_zero_parameter_input_validation(self):
        for kind in ("horizontal_distance", "vertical_distance"):
            for value in (-10, 0, 10):
                self.assertEqual("", dimension_error(kind, value))
        self.assertEqual("", dimension_error("coincident_on_curve", 0))
        self.assertEqual("", dimension_error("coincident_on_curve", 1))
        self.assertTrue(dimension_error("coincident_on_curve", 1.01))
        self.assertTrue(dimension_error("distance_to_line", -1))
        self.assertEqual("", dimension_error("distance_to_line", 0))
        curve = dict(kind="line", name="Line1", p0=(0, 0), p1=(0, 10))
        action = dict(kind="horizontal_distance", points=["Line1@0.000", "Line1@1.000"], value=0)
        overlay = build_constraint_overlay([curve, action], [curve])
        self.assertTrue(overlay.labels)
        self.assertEqual(0, float(overlay.labels[0][0]))

    def test_every_additional_relation_preserves_source_and_emits_a_pickable_glyph(self):
        set_progress_log(False)
        part = Part((-50, -50, -50), (50, 50, 50))
        geometry = [
            dict(kind="line", name="Guide", p0=(-20, -10), p1=(20, -10)),
            dict(kind="line", name="Slider", p0=(-5, -10), p1=(5, 0)),
            dict(kind="line", name="Follower", p0=(-5, 0), p1=(5, 0)),
            dict(kind="circle", name="Cam", center=(-10, 0), radius=5),
            dict(kind="circle", name="Roller", center=(0, 0), radius=5),
        ]
        relations = [
            dict(kind="point_on_line", points=["Slider@0.000"], curves=["Guide"]),
            dict(kind="point_on_circle", points=["Follower@0.000"], curves=["Cam"]),
            dict(kind="tangent_circles", curves=["Cam", "Roller"]),
            dict(kind="horizontal_distance", points=["Slider@0.000", "Slider@1.000"], curves=[], value=10),
            dict(kind="vertical_distance", points=["Slider@1.000", "Slider@0.000"], curves=[], value=-10),
            dict(kind="distance_to_line", points=["Slider@1.000"], curves=["Guide"], value=10),
            dict(kind="coincident_on_curve", points=["Follower@0.000"], curves=["Cam"], value=0),
        ]
        for relation in relations:
            with self.subTest(kind=relation["kind"]):
                sketch = part.sketch("xy", constrained=True)
                actions = geometry + [relation]
                apply_actions(sketch, actions)
                self.assertEqual(2 if relation["kind"] == "coincident_on_curve" else 1, sketch.constraint_count)
                restored = read_sketch_actions(emit_sketch_python(actions), "sk", sketch)
                self.assertEqual(relation["kind"], restored[-1]["kind"])
                self.assertEqual(relation.get("value"), restored[-1].get("value"))
                overlay = build_constraint_overlay(restored, sketch._solved_actions())
                self.assertTrue(overlay.labels or overlay.hit_segs)
                if relation["kind"] in DIMENSION_KINDS:
                    self.assertTrue(any(item[2] == "dim" for item in overlay.labels))


if __name__ == "__main__":
    unittest.main()
