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
    def test_named_drag_after_fixed_bezier_moves_only_the_requested_line(self):
        part = Part(vec3(-50), vec3(50), tolerance=.01)
        actions = [
            {"kind": "curve_call", "method": "add_bezier",
             "arguments": {"control_points": [(0, 0), (3, 4), (6, 0)]}},
            {"kind": "line", "name": "guide", "p0": (12, 0), "p1": (20, 0)},
        ]
        state = self._state(part, actions, "mixed_drag")
        before = state["sketch"]._solved_actions()[0]
        self.assertEqual((True, True), sketch_ui._apply_handle_drag(state, 1, 0, (12, 3)))
        curves = state["sketch"]._solved_actions()
        self.assertEqual(before, curves[0])
        self.assertAlmostEqual(12, curves[1]["p0"][0], places=6)
        self.assertAlmostEqual(3, curves[1]["p0"][1], places=6)
        self.assertEqual("guide", curves[1]["name"])

    def test_translated_semicircle_midpoint_snap_keeps_its_radius(self):
        part = Part(vec3(-50), vec3(50), tolerance=.01)
        sketch = part.sketch("xy", constrained=True)
        sketch.solve_after_every_constraint = False
        axis = sketch.add_line((18, 0), (30, 0), construction=True)
        side = sketch.add_line((4, -2), (16, -2))
        arc = sketch.add_arc(side @ 1, axis @ 0, (16, 2))
        sketch.tangent(side, arc).solve()
        solved = sketch._solved_actions()[-1]
        self.assertAlmostEqual(2, sketch_ui._arc_radius(solved), places=7)
        self.assertAlmostEqual(18, sketch.eval_xy(arc @ .5)[0], places=7)
        self.assertAlmostEqual(0, sketch.eval_xy(arc @ .5)[1], places=7)

    def test_dimension_edit_and_delete_undo_preserve_seed_geometry(self):
        from copy import deepcopy
        part = Part(vec3(-20), vec3(20), tolerance=.01)
        state = self._state(part, [
            {"kind": "line", "name": "Line1", "p0": (0, 0), "p1": (10, 0)},
            {"kind": "horizontal", "curves": ["Line1"]},
            {"kind": "length", "curves": ["Line1"], "value": 10},
        ], "editable_seed")
        state["pending"] = []
        state["batches"] = [{"actions": deepcopy(state["actions"]), "curves": 1,
                             "constraints": state["sketch"].constraint_count}]
        self.assertTrue(sketch_ui._update_dimension(state, 2, 12), state.get("edit_error"))
        sketch_ui._undo_pending_or_action(state)
        self.assertEqual(1, state["sketch"].curve_count)
        self.assertAlmostEqual(10, sketch_ui._dist(state["sketch"].eval_xy("Line1@1.000"), state["sketch"].eval_xy("Line1@0.000")))
        count = state["sketch"].constraint_count
        self.assertTrue(sketch_ui._delete_constraint(state, 2))
        self.assertEqual(count-1, state["sketch"].constraint_count)
        sketch_ui._undo_pending_or_action(state)
        self.assertEqual(count, state["sketch"].constraint_count)
        self.assertEqual(10, state["actions"][2]["value"])

    def test_failed_solve_restores_coordinates_even_if_solver_moved_them(self):
        from copy import deepcopy
        from unittest.mock import patch
        part = Part(vec3(-20), vec3(20), tolerance=.01)
        state = self._state(part, [{"kind": "line", "name": "Line1", "p0": (0, 0), "p1": (10, 0)}], "recovery")
        state["batches"] = [{"actions": deepcopy(state["actions"]), "curves": 1, "constraints": 0}]
        live = state["sketch"]
        solve = live.solve
        def moved_then_failed():
            live._try_drag_point(0, 0, (4, 7))
            raise ValueError("Simulated solver failure after geometry movement")
        calls = 0
        def fail_once():
            nonlocal calls
            calls += 1
            return moved_then_failed() if calls == 1 else solve()
        with patch.object(live, "solve", side_effect=fail_once):
            self.assertFalse(sketch_ui._append_and_solve(state, [{"kind": "horizontal", "curves": ["Line1"]}]))
        self.assertFalse(state.get("live_invalid"))
        self.assertEqual(0, live.constraint_count)
        self.assertAlmostEqual(0, live.eval_xy("Line1@0.000")[0])
        self.assertAlmostEqual(0, live.eval_xy("Line1@0.000")[1])
        self.assertAlmostEqual(10, live.eval_xy("Line1@1.000")[0])

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

    def test_clickable_origin_is_a_named_fixed_constraint_anchor(self):
        part = Part(vec3(-20.0), vec3(20.0), tolerance=0.01)
        state = self._state(part, [
            {"kind": "line", "p0": (2.0, 3.0), "p1": (8.0, 3.0)},
        ], "origin_anchor")
        origin = next(e for e in state["pick_catalog"] if naming.is_origin_name(e["name"]))
        hit = sketch_ui._hit_at(state, ((0, 0, 10), (0, 0, -1)), .1, sketch_ui._PICK_POINTS)
        self.assertEqual(origin["name"], hit)
        self._finish(state, "coincident", ["Line1@0.000", "origin"])
        self.assertAlmostEqual(0, state["solved"][0]["p0"][0], places=5)
        self.assertAlmostEqual(0, state["solved"][0]["p0"][1], places=5)
        self.assertIn("origin", state["actions"][-1]["points"])

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
