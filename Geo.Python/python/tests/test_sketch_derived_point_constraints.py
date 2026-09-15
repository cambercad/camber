import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from camber import naming
from camber.sketch_emit import apply_actions, derived_target, emit_sketch_python
from camber import sketch_ui
from camber.sketch_ui import _action_from_slots, build_sketch_pick_catalog

try:
    from camber import Part, vec3
    from _camber_native.__dotwrap_generated.main import NativePart
    _NATIVE_AVAILABLE = NativePart is not None
except ImportError:
    _NATIVE_AVAILABLE = False


class _Frame(object):
    origin = (0.0, 0.0, 0.0)
    x = (1.0, 0.0, 0.0)
    y = (0.0, 1.0, 0.0)
    z = (0.0, 0.0, 1.0)


class _SketchSpy(object):
    def __init__(self):
        self.calls = []
        self.solved = False

    def add_line(self, start, end):
        self.calls.append(("line", start, end))

    def add_circle(self, center, radius):
        self.calls.append(("circle", center, radius))

    def add_arc(self, start, mid, end):
        self.calls.append(("arc", start, mid, end))

    def coincident(self, first, second):
        self.calls.append(("coincident", first, second))

    def coincident_on_curve(self, point, curve, uniform):
        self.calls.append(("coincident_on_curve", point, curve, uniform))

    def midpoint(self, point, curve):
        self.calls.append(("midpoint", point, curve))

    def fix(self, point):
        self.calls.append(("fix", point))

    def radius(self, curve, value):
        self.calls.append(("radius", curve, value))

    def solve(self):
        self.solved = True


def _finish(state, first, second):
    state.update({
        "pending_constraint": "coincident",
        "pending_dimension": None,
        "slots": [first, second],
        "selected": [first, second],
        "selection_dirty": False,
        "on_selection": None,
        "status": "",
    })
    captured = []
    original = sketch_ui._append_and_solve
    sketch_ui._append_and_solve = lambda ignored, added: captured.extend(added) or True
    try:
        sketch_ui._finish_constraint_slots(state)
    finally:
        sketch_ui._append_and_solve = original
    return captured


class DerivedPointCoincidentTests(unittest.TestCase):
    def _two_lines(self):
        actions = [
            {"kind": "line", "p0": (0.0, 0.0), "p1": (1.0, 1.0)},
            {"kind": "line", "p0": (10.0, 0.0), "p1": (10.0, 4.0)},
        ]
        catalog = build_sketch_pick_catalog(actions, [], _Frame(), 0.0, 0.0, "Sketch1")
        return actions, {
            "actions": actions,
            "solved": list(actions),
            "pick_catalog": catalog,
            "grid": 1.0,
            "sketch_name": "Sketch1",
        }

    def _line_and_circle(self):
        actions = [
            {"kind": "line", "p0": (0.0, 0.0), "p1": (1.0, 1.0)},
            {"kind": "circle", "center": (5.0, 5.0), "radius": 2.0},
        ]
        catalog = build_sketch_pick_catalog(actions, [], _Frame(), 0.0, 0.0, "Sketch1")
        return actions, {
            "actions": actions,
            "solved": list(actions),
            "pick_catalog": catalog,
            "grid": 1.0,
            "sketch_name": "Sketch1",
        }

    def _line_and_arc(self):
        actions = [
            {"kind": "line", "p0": (0.0, 0.0), "p1": (1.0, 1.0)},
            {"kind": "arc", "start": (4.0, 0.0), "mid": (5.0, 1.0), "end": (6.0, 0.0)},
        ]
        catalog = build_sketch_pick_catalog(actions, [], _Frame(), 0.0, 0.0, "Sketch1")
        return actions, {
            "actions": actions,
            "solved": list(actions),
            "pick_catalog": catalog,
            "grid": 1.0,
            "sketch_name": "Sketch1",
        }

    def test_line_mid_is_a_derived_target(self):
        actions, _state = self._two_lines()
        self.assertEqual("line", derived_target(actions, (1, 3))["kind"])
        self.assertEqual(0.5, derived_target(actions, (1, 3))["uniform"])
        self.assertIsNone(derived_target(actions, (0, 1)))
        self.assertIsNone(derived_target(actions, (1, 0)))

    def test_slots_keep_line_mid_as_named_coincident(self):
        _actions, state = self._two_lines()
        line_end = naming.qualify("Sketch1", "Line1@1.000")
        other_mid = naming.qualify("Sketch1", "Line2@0.500")
        action = _action_from_slots(state, "coincident", [line_end, other_mid])
        self.assertEqual(
            {"kind": "coincident", "points": ["Line1@1.000", "Line2@0.500"]},
            action,
        )

    def test_interactive_line_end_on_other_line_mid_locks_that_line(self):
        _actions, state = self._two_lines()
        captured = _finish(
            state,
            naming.qualify("Sketch1", "Line1@1.000"),
            naming.qualify("Sketch1", "Line2@0.500"),
        )
        self.assertEqual(
            [
                {"kind": "coincident", "points": ["Line1@1.000", "Line2@0.500"]},
            ],
            captured,
        )
        self.assertIsNone(state["pending_constraint"])

    def test_replay_line_mid_calls_coincident(self):
        actions = [
            {"kind": "line", "p0": (0.0, 0.0), "p1": (1.0, 1.0)},
            {"kind": "line", "p0": (10.0, 0.0), "p1": (10.0, 4.0)},
            {"kind": "fix", "points": [(1, 0)]},
            {"kind": "fix", "points": [(1, 1)]},
            {"kind": "coincident", "points": [(0, 1), (1, 3)]},
        ]
        sketch = _SketchSpy()
        apply_actions(sketch, actions)
        self.assertIn(("coincident", "Line1@1.000", "Line2@0.500"), sketch.calls)
        self.assertNotIn(("midpoint", (0, 1), 1), sketch.calls)
        self.assertTrue(sketch.solved)

    def test_emit_line_mid_uses_named_coincident(self):
        code = emit_sketch_python([
            {"kind": "line", "p0": (0.0, 0.0), "p1": (1.0, 1.0)},
            {"kind": "line", "p0": (10.0, 0.0), "p1": (10.0, 4.0)},
            {"kind": "fix", "points": [(1, 0)]},
            {"kind": "fix", "points": [(1, 1)]},
            {"kind": "coincident", "points": [(0, 1), (1, 3)]},
        ])
        self.assertIn('coincident("Line1@1.000", "Line2@0.500")', code)
        self.assertNotIn("midpoint(", code)
        self.assertNotIn("coincident_on_curve", code)

    def test_interactive_circle_cardinal_locks_center_and_radius(self):
        _actions, state = self._line_and_circle()
        captured = _finish(
            state,
            naming.qualify("Sketch1", "Line1@1.000"),
            naming.qualify("Sketch1", "Circle2@0.500"),
        )
        self.assertEqual(
            [
                {"kind": "coincident", "points": ["Line1@1.000", "Circle2@0.500"]},
            ],
            captured,
        )

    def test_replay_circle_cardinal_calls_coincident(self):
        actions = [
            {"kind": "line", "p0": (0.0, 0.0), "p1": (1.0, 1.0)},
            {"kind": "circle", "center": (5.0, 5.0), "radius": 2.0},
            {"kind": "fix", "points": [(1, 2)]},
            {"kind": "radius", "curves": [1], "value": 2.0},
            {"kind": "coincident", "points": [(0, 1), (1, 5)]},
        ]
        sketch = _SketchSpy()
        apply_actions(sketch, actions)
        self.assertIn(("coincident", "Line1@1.000", "Circle2@0.500"), sketch.calls)
        self.assertNotIn(("coincident_on_curve", (0, 1), 1, 0.5), sketch.calls)

    def test_emit_circle_cardinal_uses_named_coincident(self):
        code = emit_sketch_python([
            {"kind": "line", "p0": (0.0, 0.0), "p1": (1.0, 1.0)},
            {"kind": "circle", "center": (5.0, 5.0), "radius": 2.0},
            {"kind": "coincident", "points": [(0, 1), (1, 5)]},
        ])
        self.assertIn('coincident("Line1@1.000", "Circle2@0.500")', code)
        self.assertNotIn("coincident_on_curve", code)

    def test_interactive_arc_mid_locks_arc(self):
        _actions, state = self._line_and_arc()
        captured = _finish(
            state,
            naming.qualify("Sketch1", "Line1@1.000"),
            naming.qualify("Sketch1", "Arc2@0.500"),
        )
        self.assertEqual(
            [{"kind": "coincident", "points": ["Line1@1.000", "Arc2@0.500"]}],
            captured,
        )

    def test_replay_arc_mid_calls_coincident(self):
        actions = [
            {"kind": "line", "p0": (0.0, 0.0), "p1": (1.0, 1.0)},
            {"kind": "arc", "start": (4.0, 0.0), "mid": (5.0, 1.0), "end": (6.0, 0.0)},
            {"kind": "coincident", "points": [(0, 1), (1, 3)]},
        ]
        sketch = _SketchSpy()
        apply_actions(sketch, actions)
        self.assertIn(("coincident", "Line1@1.000", "Arc2@0.500"), sketch.calls)

    @unittest.skipUnless(_NATIVE_AVAILABLE, "requires the installed camber native module")
    def test_native_line_end_moves_to_other_line_midpoint(self):
        part = Part(vec3(-20.0), vec3(20.0), tolerance=0.01)
        sketch = part.sketch("xy", constrained=True, name="line_mid")
        apply_actions(sketch, [
            {"kind": "line", "p0": (0.0, 0.0), "p1": (1.0, 1.0)},
            {"kind": "line", "p0": (10.0, 0.0), "p1": (10.0, 4.0)},
            {"kind": "fix", "points": [(1, 0)]},
            {"kind": "fix", "points": [(1, 1)]},
            {"kind": "coincident", "points": [(0, 1), (1, 3)]},
        ])
        line = sketch._solved_actions()[0]
        self.assertAlmostEqual(10.0, line["p1"][0], places=5)
        self.assertAlmostEqual(2.0, line["p1"][1], places=5)

    @unittest.skipUnless(_NATIVE_AVAILABLE, "requires the installed camber native module")
    def test_native_line_end_moves_to_each_circle_cardinal(self):
        expected = (
            (3, (1.0, 0.0)),
            (4, (0.0, 1.0)),
            (5, (-1.0, 0.0)),
            (6, (0.0, -1.0)),
        )
        for role, direction in expected:
            part = Part(vec3(-20.0), vec3(20.0), tolerance=0.01)
            sketch = part.sketch("xy", constrained=True, name="card_{0}".format(role))
            apply_actions(sketch, [
                {"kind": "line", "p0": (0.0, 0.0), "p1": (1.0, 1.0)},
                {"kind": "circle", "center": (5.0, 5.0), "radius": 2.0},
                {"kind": "fix", "points": [(1, 2)]},
                {"kind": "radius", "curves": [1], "value": 2.0},
                {"kind": "coincident", "points": [(0, 1), (1, role)]},
            ])
            line, circle = sketch._solved_actions()[:2]
            self.assertAlmostEqual(
                circle["center"][0] + circle["radius"] * direction[0],
                line["p1"][0],
                places=5,
            )
            self.assertAlmostEqual(
                circle["center"][1] + circle["radius"] * direction[1],
                line["p1"][1],
                places=5,
            )
            self.assertGreater(circle["radius"], 1.0)


if __name__ == "__main__":
    unittest.main()
