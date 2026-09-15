import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from camber import naming
from camber.sketch_emit import apply_actions, emit_sketch_python, _materialize_point_ref
from camber import sketch_ui
from camber.sketch_ui import _PICK_POINTS, _action_from_slots, _hit_at, build_sketch_pick_catalog

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


class CircleCardinalCoincidentTests(unittest.TestCase):
    def _catalog_and_state(self):
        actions = [
            {"kind": "line", "p0": (0.0, 0.0), "p1": (1.0, 1.0)},
            {"kind": "circle", "center": (5.0, 5.0), "radius": 2.0},
        ]
        catalog = build_sketch_pick_catalog(actions, [], _Frame(), 0.0, 0.0, "Sketch1")
        return actions, catalog, {
            "actions": actions,
            "solved": list(actions),
            "pick_catalog": catalog,
            "grid": 1.0,
            "sketch_name": "Sketch1",
        }

    def test_named_circle_anchor_becomes_native_cardinal_reference(self):
        actions, catalog, state = self._catalog_and_state()
        line_end = naming.qualify("Sketch1", "Line1@1.000")
        circle_east = naming.qualify("Sketch1", "Circle2@0.000")

        action = _action_from_slots(state, "coincident", [line_end, circle_east])

        self.assertEqual(
            {"kind": "coincident", "points": ["Line1@1.000", "Circle2@0.000"]},
            action,
        )

    def test_each_ray_picks_its_circle_cardinal_not_the_center(self):
        _actions, _catalog, state = self._catalog_and_state()
        expected = (
            ((7.0, 5.0), 3, "0.000"),
            ((5.0, 7.0), 4, "0.250"),
            ((3.0, 5.0), 5, "0.500"),
            ((5.0, 3.0), 6, "0.750"),
        )

        for position, role, suffix in expected:
            hit = _hit_at(
                state,
                ((position[0], position[1], 10.0), (0.0, 0.0, -1.0)),
                0.25,
                _PICK_POINTS,
            )
            self.assertEqual("Sketch1:Circle2@" + suffix, hit)
            self.assertEqual(("point", 1, role), next(
                item["ref"] for item in state["pick_catalog"] if item["name"] == hit))

    def test_interactive_completion_keeps_selected_cardinal_reference(self):
        _actions, _catalog, state = self._catalog_and_state()
        state.update({
            "pending_constraint": "coincident",
            "pending_dimension": None,
            "slots": ["Sketch1:Line1@1.000", "Sketch1:Circle2@0.000"],
            "selected": ["Sketch1:Line1@1.000", "Sketch1:Circle2@0.000"],
            "selection_dirty": False,
            "on_selection": None,
            "status": "",
        })
        captured = []
        original = sketch_ui._append_and_solve
        sketch_ui._append_and_solve = lambda ignored_state, added: captured.extend(added) or True
        try:
            sketch_ui._finish_constraint_slots(state)
        finally:
            sketch_ui._append_and_solve = original

        self.assertEqual(
            [
                {"kind": "coincident", "points": ["Line1@1.000", "Circle2@0.000"]},
            ],
            captured,
        )
        self.assertIsNone(state["pending_constraint"])

    def test_interactive_completion_locks_radius_for_west_cardinal(self):
        _actions, _catalog, state = self._catalog_and_state()
        state.update({
            "pending_constraint": "coincident",
            "pending_dimension": None,
            "slots": ["Sketch1:Line1@1.000", "Sketch1:Circle2@0.500"],
            "selected": ["Sketch1:Line1@1.000", "Sketch1:Circle2@0.500"],
            "selection_dirty": False,
            "on_selection": None,
            "status": "",
        })
        captured = []
        original = sketch_ui._append_and_solve
        sketch_ui._append_and_solve = lambda ignored_state, added: captured.extend(added) or True
        try:
            sketch_ui._finish_constraint_slots(state)
        finally:
            sketch_ui._append_and_solve = original

        self.assertEqual(
            [
                {"kind": "coincident", "points": ["Line1@1.000", "Circle2@0.500"]},
            ],
            captured,
        )

    def test_west_cardinal_emits_evaluate_parameter_not_center(self):
        actions = [
            {"kind": "line", "p0": (0.0, 0.0), "p1": (1.0, 1.0)},
            {"kind": "circle", "center": (5.0, 5.0), "radius": 2.0},
        ]
        self.assertEqual(("xy", 3.0, 5.0), _materialize_point_ref(actions, (1, 5)))
        self.assertEqual((1, 2), _materialize_point_ref(actions, (1, 2)))
        code = emit_sketch_python(
            actions + [
                {"kind": "fix", "points": [(1, 2)]},
                {"kind": "radius", "curves": [1], "value": 2.0},
                {"kind": "coincident", "points": [(0, 1), (1, 5)]},
            ]
        )
        self.assertIn('coincident("Line1@1.000", "Circle2@0.500")', code)
        self.assertNotIn("coincident_on_curve", code)

    def test_live_replay_passes_cardinal_to_evaluate_parameter(self):
        actions = [
            {"kind": "line", "p0": (0.0, 0.0), "p1": (1.0, 1.0)},
            {"kind": "circle", "center": (5.0, 5.0), "radius": 2.0},
            {"kind": "fix", "points": [(1, 2)]},
            {"kind": "radius", "curves": [1], "value": 2.0},
            {"kind": "coincident", "points": [(0, 1), (1, 3)]},
        ]
        sketch = _SketchSpy()

        apply_actions(sketch, actions)

        self.assertIn(("coincident", "Line1@1.000", "Circle2@0.000"), sketch.calls)
        self.assertIn(("fix", "Circle2@center"), sketch.calls)
        self.assertIn(("radius", 1, 2.0), sketch.calls)
        self.assertTrue(sketch.solved)

    @unittest.skipUnless(_NATIVE_AVAILABLE, "requires the installed camber native module")
    def test_native_replay_keeps_all_cardinals_off_the_circle_center(self):
        expected = (
            (3, (1.0, 0.0)),
            (4, (0.0, 1.0)),
            (5, (-1.0, 0.0)),
            (6, (0.0, -1.0)),
        )
        for role, direction in expected:
            part = Part(vec3(-20.0), vec3(20.0), tolerance=0.01)
            sketch = part.sketch("xy", constrained=True, name="cardinal_{0}".format(role))
            actions = [
                {"kind": "line", "p0": (0.0, 0.0), "p1": (1.0, 1.0)},
                {"kind": "circle", "center": (5.0, 5.0), "radius": 2.0},
                {"kind": "fix", "points": [(1, 2)]},
                {"kind": "radius", "curves": [1], "value": 2.0},
                {"kind": "coincident", "points": [(0, 1), (1, role)]},
            ]

            apply_actions(sketch, actions)
            solved = sketch._solved_actions()
            line = solved[0]
            circle = solved[1]
            expected_end = (
                circle["center"][0] + circle["radius"] * direction[0],
                circle["center"][1] + circle["radius"] * direction[1],
            )
            self.assertAlmostEqual(expected_end[0], line["p1"][0], places=6)
            self.assertAlmostEqual(expected_end[1], line["p1"][1], places=6)
            self.assertAlmostEqual(2.0, circle["radius"], places=6)
            self.assertGreater(
                abs(line["p1"][0] - circle["center"][0])
                + abs(line["p1"][1] - circle["center"][1]),
                0.5,
            )

    @unittest.skipUnless(_NATIVE_AVAILABLE, "requires the installed camber native module")
    def test_interactive_finish_solves_west_cardinal_not_center(self):
        part = Part(vec3(-20.0), vec3(20.0), tolerance=0.01)
        actions = [
            {"kind": "line", "p0": (0.0, 0.0), "p1": (1.0, 1.0)},
            {"kind": "circle", "center": (5.0, 5.0), "radius": 2.0},
        ]
        catalog = build_sketch_pick_catalog(actions, [], _Frame(), 0.0, 0.0, "ui_west_cardinal")
        state = {
            "actions": list(actions),
            "solved": list(actions),
            "pick_catalog": catalog,
            "pending_constraint": "coincident",
            "pending_dimension": None,
            "slots": ["ui_west_cardinal:Line1@1.000", "ui_west_cardinal:Circle2@0.500"],
            "selected": ["ui_west_cardinal:Line1@1.000", "ui_west_cardinal:Circle2@0.500"],
            "selection_dirty": False,
            "on_selection": None,
            "status": "",
            "part": part,
            "plane": "xy",
            "frame": None,
            "emit_frame": False,
            "sketch": None,
            "sketch_name": "ui_west_cardinal",
            "grid": 1.0,
        }

        sketch_ui._finish_constraint_slots(state)

        self.assertIsNone(state["pending_constraint"])
        self.assertTrue(
            any(a.get("kind") == "coincident" for a in state["actions"]),
            state["actions"],
        )
        solved = state.get("solved") or []
        self.assertGreaterEqual(len(solved), 2)
        line, circle = solved[0], solved[1]
        self.assertAlmostEqual(2.0, circle["radius"], places=5)
        self.assertAlmostEqual(circle["center"][0] - circle["radius"], line["p1"][0], places=5)
        self.assertAlmostEqual(circle["center"][1], line["p1"][1], places=5)
        self.assertGreater(
            abs(line["p1"][0] - circle["center"][0])
            + abs(line["p1"][1] - circle["center"][1]),
            0.5,
        )


if __name__ == "__main__":
    unittest.main()
