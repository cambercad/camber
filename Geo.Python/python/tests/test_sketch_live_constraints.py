import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from camber import naming
from camber.sketch_emit import apply_actions
from camber import sketch_ui
from camber.sketch_ui import build_sketch_pick_catalog

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
        self.solve_count = 0

    def add_line(self, start, end):
        self.calls.append(("line", start, end))

    def add_circle(self, center, radius):
        self.calls.append(("circle", center, radius))

    def horizontal(self, curve):
        self.calls.append(("horizontal", curve))

    def vertical(self, curve):
        self.calls.append(("vertical", curve))

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
        self.solve_count += 1

    def _solved_actions(self):
        return [
            {"kind": "line", "name": "Line1", "p0": (0.0, 0.0), "p1": (2.0, 0.0)},
            {"kind": "line", "name": "Line2", "p0": (3.0, 0.0), "p1": (3.0, 2.0)},
        ]


def _horizontal_error(line):
    return abs(line["p0"][1] - line["p1"][1])


def _vertical_error(line):
    return abs(line["p0"][0] - line["p1"][0])


def _end_error(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


class ApplyActionsIncrementalTests(unittest.TestCase):
    def test_start_applies_only_new_constraint(self):
        sketch = _SketchSpy()
        actions = [
            {"kind": "line", "p0": (0.0, 0.0), "p1": (2.0, 1.0)},
            {"kind": "line", "p0": (3.0, 0.0), "p1": (4.0, 2.0)},
            {"kind": "horizontal", "curves": [0]},
        ]
        apply_actions(sketch, actions, start=2)
        self.assertEqual([("horizontal", 0)], sketch.calls)
        self.assertEqual(1, sketch.solve_count)

    def test_start_rewrites_cardinal_using_full_action_list(self):
        sketch = _SketchSpy()
        actions = [
            {"kind": "line", "p0": (0.0, 0.0), "p1": (1.0, 1.0)},
            {"kind": "circle", "center": (5.0, 5.0), "radius": 2.0},
            {"kind": "fix", "points": [(1, 2)]},
            {"kind": "radius", "curves": [1], "value": 2.0},
            {"kind": "coincident", "points": [(0, 1), (1, 5)]},
        ]
        apply_actions(sketch, actions, start=2)
        self.assertNotIn(("line", (0.0, 0.0), (1.0, 1.0)), sketch.calls)
        self.assertNotIn(("circle", (5.0, 5.0), 2.0), sketch.calls)
        self.assertIn(("coincident", "Line1@1.000", "Circle2@0.500"), sketch.calls)
        self.assertNotIn(("coincident_on_curve", (0, 1), 1, 0.5), sketch.calls)


class LiveSketchKeepsConstraintsTests(unittest.TestCase):
    def test_second_constraint_stays_on_the_same_live_sketch(self):
        live = _SketchSpy()
        unregistered = []
        original = sketch_ui._unregister_session_sketch
        sketch_ui._unregister_session_sketch = lambda part, sketch: unregistered.append(sketch)
        try:
            actions = [
                {"kind": "line", "p0": (0.0, 0.0), "p1": (2.0, 1.0)},
                {"kind": "line", "p0": (3.0, 0.0), "p1": (4.0, 2.0)},
            ]
            state = {
                "actions": list(actions),
                "solved": list(actions),
                "sketch": live,
                "part": None,
                "plane": "xy",
                "frame": _Frame(),
                "emit_frame": False,
                "sketch_name": "Sketch1",
            }
            self.assertTrue(sketch_ui._append_and_solve(
                state, [{"kind": "horizontal", "curves": [0]}]))
            self.assertTrue(sketch_ui._append_and_solve(
                state, [{"kind": "vertical", "curves": [1]}]))
            self.assertIs(live, state["sketch"])
            self.assertEqual([], unregistered)
            self.assertEqual(
                [("horizontal", 0), ("vertical", 1)],
                [c for c in live.calls if c[0] in ("horizontal", "vertical")],
            )
            self.assertEqual(2, live.solve_count)
            self.assertEqual(1, len([c for c in live.calls if c[0] == "horizontal"]))
        finally:
            sketch_ui._unregister_session_sketch = original

    def test_append_does_not_recreate_when_live_exists(self):
        created = []
        original = sketch_ui._create_constraint_sketch

        def boom(*args, **kwargs):
            created.append(1)
            raise AssertionError("must not create a second sketch")

        sketch_ui._create_constraint_sketch = boom
        try:
            live = _SketchSpy()
            state = {
                "actions": [
                    {"kind": "line", "p0": (0.0, 0.0), "p1": (2.0, 1.0)},
                ],
                "solved": [],
                "sketch": live,
                "part": None,
                "plane": "xy",
                "frame": _Frame(),
                "emit_frame": False,
                "sketch_name": "Sketch1",
                "batches": [],
            }
            self.assertTrue(sketch_ui._append_and_solve(
                state, [{"kind": "horizontal", "curves": [0]}]))
            self.assertEqual([], created)
            self.assertIs(live, state["sketch"])
        finally:
            sketch_ui._create_constraint_sketch = original


@unittest.skipUnless(_NATIVE_AVAILABLE, "requires the installed camber native module")
class NativeSequentialConstraintTests(unittest.TestCase):
    def _state(self, part, actions, name):
        catalog = build_sketch_pick_catalog(actions, [], _Frame(), 0.0, 0.0, "Sketch1")
        state = {
            "actions": list(actions),
            "solved": list(actions),
            "pick_catalog": catalog,
            "pending_constraint": None,
            "pending_dimension": None,
            "slots": [],
            "selected": [],
            "selection_dirty": False,
            "on_selection": None,
            "status": "",
            "part": part,
            "plane": "xy",
            "frame": _Frame(),
            "emit_frame": False,
            "sketch": None,
            "sketch_name": name,
            "grid": 1.0,
            "part_points": [],
            "scene": None,
        }
        sketch_ui._seed_live_from_actions(state)
        return state

    def _name(self, state, local):
        return naming.qualify(state.get("sketch_name") or "Sketch1", local)

    def _finish(self, state, kind, locals_):
        sketch_ui._rebuild_pick_catalog(state)
        slots = [self._name(state, local) for local in locals_]
        state["pending_constraint"] = kind
        state["slots"] = list(slots)
        state["selected"] = list(slots)
        sketch_ui._finish_constraint_slots(state)
        self.assertIsNone(state["pending_constraint"], state.get("status"))

    def test_session_creates_only_one_sketch(self):
        creates = []
        original = sketch_ui._create_constraint_sketch

        def wrapped(*args, **kwargs):
            sketch = original(*args, **kwargs)
            creates.append(sketch)
            return sketch

        sketch_ui._create_constraint_sketch = wrapped
        try:
            part = Part(vec3(-20.0), vec3(20.0), tolerance=0.01)
            state = self._state(part, [
                {"kind": "line", "p0": (0.0, 0.0), "p1": (2.0, 1.0)},
                {"kind": "line", "p0": (3.0, 0.0), "p1": (4.0, 2.0)},
            ], "once")
            live = state["sketch"]
            self._finish(state, "horizontal", ["Line1"])
            self._finish(state, "vertical", ["Line2"])
            self.assertEqual(1, len(creates))
            self.assertIs(live, state["sketch"])
            self.assertIs(creates[0], live)
        finally:
            sketch_ui._create_constraint_sketch = original

    def test_horizontal_then_vertical_keeps_both(self):
        part = Part(vec3(-20.0), vec3(20.0), tolerance=0.01)
        state = self._state(part, [
            {"kind": "line", "p0": (0.0, 0.0), "p1": (2.0, 1.0)},
            {"kind": "line", "p0": (3.0, 0.0), "p1": (4.0, 2.0)},
        ], "keep_hv")
        live = state["sketch"]
        self._finish(state, "horizontal", ["Line1"])
        self._finish(state, "vertical", ["Line2"])
        self.assertIs(live, state["sketch"])
        solved = state["solved"]
        self.assertAlmostEqual(0.0, _horizontal_error(solved[0]), places=6)
        self.assertAlmostEqual(0.0, _vertical_error(solved[1]), places=6)

    def test_coincident_then_horizontal_keeps_both(self):
        part = Part(vec3(-20.0), vec3(20.0), tolerance=0.01)
        state = self._state(part, [
            {"kind": "line", "p0": (0.0, 0.0), "p1": (2.0, 1.0)},
            {"kind": "line", "p0": (2.2, 1.1), "p1": (4.0, 3.0)},
        ], "keep_coin_h")
        live = state["sketch"]
        self._finish(state, "coincident", ["Line1@1.000", "Line2@0.000"])
        self._finish(state, "horizontal", ["Line1"])
        self.assertIs(live, state["sketch"])
        solved = state["solved"]
        self.assertAlmostEqual(0.0, _end_error(solved[0]["p1"], solved[1]["p0"]), places=6)
        self.assertAlmostEqual(0.0, _horizontal_error(solved[0]), places=6)

    def test_cardinal_then_horizontal_stays_on_circle(self):
        part = Part(vec3(-20.0), vec3(20.0), tolerance=0.01)
        state = self._state(part, [
            {"kind": "line", "p0": (0.0, 0.0), "p1": (1.0, 1.0)},
            {"kind": "circle", "center": (5.0, 5.0), "radius": 2.0},
        ], "keep_cardinal_h")
        live = state["sketch"]
        self._finish(state, "coincident", ["Line1@1.000", "Circle1@0.500"])
        self._finish(state, "horizontal", ["Line1"])
        self.assertIs(live, state["sketch"])
        line, circle = state["solved"][:2]
        self.assertAlmostEqual(2.0, circle["radius"], places=5)
        self.assertAlmostEqual(circle["center"][0] - circle["radius"], line["p1"][0], places=5)
        self.assertAlmostEqual(circle["center"][1], line["p1"][1], places=5)
        self.assertAlmostEqual(0.0, _horizontal_error(line), places=5)

    def test_second_cardinal_does_not_drop_the_first(self):
        part = Part(vec3(-20.0), vec3(20.0), tolerance=0.01)
        state = self._state(part, [
            {"kind": "line", "p0": (0.0, 0.0), "p1": (1.0, 1.0)},
            {"kind": "line", "p0": (8.0, 5.0), "p1": (9.0, 6.0)},
            {"kind": "circle", "center": (5.0, 5.0), "radius": 2.0},
        ], "two_cardinals")
        live = state["sketch"]
        self._finish(state, "coincident", ["Line1@1.000", "Circle1@0.500"])
        self._finish(state, "coincident", ["Line2@0.000", "Circle1@0.000"])
        self.assertIs(live, state["sketch"])
        kinds = [a.get("kind") for a in state["actions"]]
        self.assertNotIn("fix", kinds)
        self.assertNotIn("radius", kinds)
        line_a, line_b, circle = state["solved"][:3]
        self.assertAlmostEqual(2.0, circle["radius"], places=5)
        self.assertAlmostEqual(circle["center"][0] - circle["radius"], line_a["p1"][0], places=5)
        self.assertAlmostEqual(circle["center"][1], line_a["p1"][1], places=5)
        self.assertAlmostEqual(circle["center"][0] + circle["radius"], line_b["p0"][0], places=5)
        self.assertAlmostEqual(circle["center"][1], line_b["p0"][1], places=5)


if __name__ == "__main__":
    unittest.main()
