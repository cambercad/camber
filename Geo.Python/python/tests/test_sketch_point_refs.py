import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from camber.api import Sketch, SketchCurve, _classify_point, _eval_named_point
from camber.sketch_emit import emit_sketch_python
from camber.sketch_ui import _arc_click_points, _on_point, _preview_polylines_from


class _Native(object):
    def __init__(self):
        self.calls = []
        self.name = "Sketch1"

    def set_coincident(self, *args):
        self.calls.append(("set_coincident",) + args)

    def set_coincident_xy(self, *args):
        self.calls.append(("set_coincident_xy",) + args)

    def set_coincident_named(self, *args):
        self.calls.append(("set_coincident_named",) + args)

    def set_coincident_names(self, *args):
        self.calls.append(("set_coincident_names",) + args)

    def set_coincident_handle_name(self, *args):
        self.calls.append(("set_coincident_handle_name",) + args)

    def set_coincident_name_xy(self, *args):
        self.calls.append(("set_coincident_name_xy",) + args)

    def set_distance(self, *args):
        self.calls.append(("set_distance",) + args)

    def set_distance_xy(self, *args):
        self.calls.append(("set_distance_xy",) + args)

    def set_distance_named(self, *args):
        self.calls.append(("set_distance_named",) + args)

    def set_distance_names(self, *args):
        self.calls.append(("set_distance_names",) + args)

    def set_distance_handle_name(self, *args):
        self.calls.append(("set_distance_handle_name",) + args)

    def set_distance_name_xy(self, *args):
        self.calls.append(("set_distance_name_xy",) + args)

    def fix_point(self, *args):
        self.calls.append(("fix_point",) + args)

    def fix_point_named(self, *args):
        self.calls.append(("fix_point_named",) + args)

    def set_midpoint(self, *args):
        self.calls.append(("set_midpoint",) + args)

    def set_midpoint_named(self, *args):
        self.calls.append(("set_midpoint_named",) + args)

    def set_coincident_on_curve(self, *args):
        self.calls.append(("set_coincident_on_curve",) + args)

    def set_coincident_on_curve_named(self, *args):
        self.calls.append(("set_coincident_on_curve_named",) + args)


class PointRefDispatchTests(unittest.TestCase):
    def setUp(self):
        self.native = _Native()
        self.sk = Sketch(self.native, None)

    def test_classify(self):
        self.assertEqual(("xy", (2.0, 3.0)), _classify_point(("xy", 2, 3)))
        self.assertEqual(("name", "box:[A,B]@0.500"), _classify_point("box:[A,B]@0.500"))
        self.assertEqual(("name", "Line1@1.000"), _classify_point(" Line1@1.000 "))
        with self.assertRaises(ValueError):
            _classify_point((0, 1))

    def test_curve_at_operator(self):
        top = SketchCurve("top", "line")
        rim = SketchCurve("rim", "arc")
        self.assertEqual("top@1.000", top @ 1.000)
        self.assertEqual("top@0.000", top @ 0)
        self.assertEqual("rim@0.500", rim @ 0.5)
        self.assertEqual("rim@center", rim @ "center")
        self.assertEqual("rim@center", rim @ "Center")
        self.assertEqual("Sketch1:origin", self.sk @ "origin")
        self.assertEqual(("name", "top@1.000"), _classify_point(top @ 1.000))
        self.assertEqual(("name", "Sketch1:origin"), _classify_point(self.sk @ "origin"))
        self.sk.coincident(top @ 1.000, rim @ 0.000)
        self.assertEqual(
            [("set_coincident_names", "top@1.000", "rim@0.000")],
            self.native.calls,
        )

    def test_coincident_all_mixes(self):
        self.sk.coincident("Line1@1.000", "box:[A,B]@0.500")
        self.sk.coincident("Line1@1.000", ("xy", 1, 2))
        self.sk.coincident(("xy", 1, 2), "Line1@1.000")
        self.assertEqual(
            [
                ("set_coincident_names", "Line1@1.000", "box:[A,B]@0.500"),
                ("set_coincident_name_xy", "Line1@1.000", 1.0, 2.0),
                ("set_coincident_name_xy", "Line1@1.000", 1.0, 2.0),
            ],
            self.native.calls,
        )
        with self.assertRaises(ValueError):
            self.sk.coincident(("xy", 0, 0), ("xy", 1, 1))
        with self.assertRaises(ValueError):
            self.sk.coincident((0, 1), (1, 0))

    def test_distance_all_mixes(self):
        self.sk.distance("A@0.000", "origin", 2.5)
        self.sk.distance("A@0.000", "B@1.000", 3)
        self.sk.distance("A@0.000", ("xy", 1, 0), 4)
        self.assertEqual(
            [
                ("set_distance_names", "A@0.000", "Origin", 2.5),
                ("set_distance_names", "A@0.000", "B@1.000", 3.0),
                ("set_distance_name_xy", "A@0.000", 1.0, 0.0, 4.0),
            ],
            self.native.calls,
        )

    def test_fix_and_midpoint_and_on_curve(self):
        self.sk.fix("Line1@0.000")
        self.sk.fix(("xy", 1, 2))
        self.sk.midpoint("box:[A,B]@0.500", 2)
        self.sk.coincident_on_curve("Line1@1.000", 1, 0.25)
        self.assertEqual(
            [
                ("fix_point_named", "Line1@0.000"),
                ("set_midpoint_named", "box:[A,B]@0.500", 2),
                ("set_coincident_on_curve_named", "Line1@1.000", 1, 0.25),
            ],
            self.native.calls,
        )
        with self.assertRaises(ValueError):
            self.sk.fix((0, 0))

    def test_emit_keeps_quoted_names(self):
        code = emit_sketch_python([
            {"kind": "line", "p0": (0.0, 0.0), "p1": (1.0, 1.0)},
            {"kind": "coincident", "points": [(0, 1), "box:[A,B]@0.500"]},
            {"kind": "distance", "points": [(0, 0), "origin"], "value": 2.0},
        ])
        self.assertIn('add_line(sk @ "origin", (1, 1), name="Line1")', code)
        self.assertIn('coincident("Line1@1.000", "box:[A,B]@0.500")', code)
        self.assertIn('distance("Line1@0.000", sk @ "origin", 2)', code)

    def test_emit_uses_names_not_indices(self):
        code = emit_sketch_python([
            {
                "kind": "line",
                "p0": (-150.5435, 24.348),
                "p1": (-24.6872, 136.547),
                "name": "Line1",
            },
            {
                "kind": "circle",
                "center": (21.3081, 136.547),
                "radius": 45.9953,
                "name": "Circle1",
            },
            {
                "kind": "line",
                "p0": (-87.6153, 80.4475),
                "p1": (36.13, 35.6455),
                "name": "Line2",
            },
            {"kind": "coincident", "points": ["Line1@1.000", "Circle1@0.500"]},
            {"kind": "coincident", "points": ["Line2@0.000", "Line1@0.500"]},
            {"kind": "angle", "curves": ["Line1", "Line2"], "value": 61.6193},
        ])
        self.assertIn('add_line((-150.5435, 24.348), (-24.6872, 136.547), name="Line1")', code)
        self.assertIn('add_circle((21.3081, 136.547), 45.9953, name="Circle1")', code)
        self.assertIn('add_line((-87.6153, 80.4475), (36.13, 35.6455), name="Line2")', code)
        self.assertIn('coincident("Line1@1.000", "Circle1@0.500")', code)
        self.assertIn('coincident("Line2@0.000", "Line1@0.500")', code)
        self.assertIn('angle("Line1", "Line2", 61.6193)', code)
        self.assertNotIn("coincident_on_curve", code)
        self.assertNotIn("midpoint(", code)

    def test_eval_named_circle_center_from_dump(self):
        class _DumpSketch(object):
            _n = None

            def _solved_actions(self):
                return [
                    {"kind": "circle", "name": "Circle1", "center": (5.0, 7.0), "radius": 2.0},
                ]

        sketch = _DumpSketch()
        self.assertEqual((5.0, 7.0), _eval_named_point(sketch, "Circle1@center"))
        self.assertEqual((5.0, 7.0), _eval_named_point(sketch, "Sketch1:Circle1@center"))
        self.assertEqual((0.0, 0.0), _eval_named_point(sketch, "Sketch1:origin"))
        self.assertEqual((0.0, 0.0), _eval_named_point(sketch, "Origin"))

    def test_emit_line_from_circle_center(self):
        code = emit_sketch_python([
            {"kind": "circle", "center": (5.0, 5.0), "radius": 2.0, "name": "Circle1"},
            {
                "kind": "line",
                "p0": (5.0, 5.0),
                "p1": (10.0, 5.0),
                "p0_ref": "Sketch1:Circle1@center",
                "name": "Line1",
            },
        ])
        self.assertIn('add_circle((5, 5), 2, name="Circle1")', code)
        self.assertIn('add_line("Circle1@center", (10, 5), name="Line1")', code)

    def test_emit_construction_geometry(self):
        code = emit_sketch_python([
            {
                "kind": "line",
                "p0": (0.0, 0.0),
                "p1": (1.0, 0.0),
                "name": "Line1",
                "construction": True,
            },
            {"kind": "circle", "center": (0.0, 0.0), "radius": 2.0, "name": "Circle1"},
        ])
        self.assertIn('add_line(sk @ "origin", (1, 0), name="Line1", construction=True)', code)
        self.assertIn('add_circle(sk @ "origin", 2, name="Circle1")', code)
        self.assertNotIn("add_circle(sk @ \"origin\", 2, name=\"Circle1\", construction=True)", code)

    def test_toggle_construction_updates_recorded_curve(self):
        from camber.sketch_ui import _set_curves_construction
        state = {
            "actions": [
                {"kind": "line", "p0": (0.0, 0.0), "p1": (1.0, 0.0), "name": "Line1"},
            ],
            "solved": [
                {"kind": "line", "p0": (0.0, 0.0), "p1": (1.0, 0.0), "name": "Line1"},
            ],
            "selected": [],
            "sketch": None,
        }
        _set_curves_construction(state, [0], True)
        self.assertTrue(state["actions"][0]["construction"])
        self.assertTrue(state["solved"][0]["construction"])
        self.assertIn("construction=True", emit_sketch_python(state["actions"]))
        _set_curves_construction(state, [0], False)
        self.assertNotIn("construction", state["actions"][0])
        self.assertNotIn("construction=True", emit_sketch_python(state["actions"]))

    def test_draw_line_stores_curve_name_and_snap_refs(self):
        state = {
            "actions": [],
            "tool": "line",
            "pending": [],
            "pending_refs": [],
            "sketch_name": "Sketch1",
        }
        self.assertEqual([], _on_point(state, (0.0, 0.0), "origin"))
        added = _on_point(state, (1.0, 0.0), "box:[A,B]@0.500")
        self.assertEqual("Line1", added[0]["name"])
        self.assertEqual("origin", added[0]["p0_ref"])
        self.assertEqual("box:[A,B]@0.500", added[0]["p1_ref"])
        code = emit_sketch_python(added)
        self.assertIn('add_line(sk @ "origin", "box:[A,B]@0.500", name="Line1")', code)

    def test_arc_third_click_is_through_point_ending_at_second(self):
        self.assertEqual(((0.0, 0.0), (1.0, 1.0), (2.0, 0.0)), _arc_click_points([
            (0.0, 0.0), (2.0, 0.0), (1.0, 1.0),
        ]))
        state = {
            "actions": [],
            "tool": "arc",
            "pending": [],
            "pending_refs": [],
            "sketch_name": "Sketch1",
        }
        self.assertEqual([], _on_point(state, (0.0, 0.0), "origin"))
        self.assertEqual([], _on_point(state, (2.0, 0.0), None))
        added = _on_point(state, (1.0, 1.0), None)
        self.assertEqual((0.0, 0.0), added[0]["start"])
        self.assertEqual((1.0, 1.0), added[0]["mid"])
        self.assertEqual((2.0, 0.0), added[0]["end"])
        self.assertEqual("origin", added[0]["start_ref"])
        nodes, edges = _preview_polylines_from(
            [(0.0, 0.0), (2.0, 0.0), (1.0, 1.0)],
            "arc",
            _PreviewFrame(),
            0.0,
        )
        self.assertGreaterEqual(len(nodes), 3)
        self.assertAlmostEqual(0.0, nodes[0][0], places=5)
        self.assertAlmostEqual(0.0, nodes[0][1], places=5)
        self.assertAlmostEqual(2.0, nodes[-1][0], places=5)
        self.assertAlmostEqual(0.0, nodes[-1][1], places=5)


class _PreviewFrame(object):
    origin = (0.0, 0.0, 0.0)
    x = (1.0, 0.0, 0.0)
    y = (0.0, 1.0, 0.0)
    z = (0.0, 0.0, 1.0)


try:
    from camber import Part, vec3
    from _camber_native.__dotwrap_generated.main import NativePart
    _NATIVE_AVAILABLE = NativePart is not None
except ImportError:
    _NATIVE_AVAILABLE = False


@unittest.skipUnless(_NATIVE_AVAILABLE, "native camber not available")
class CurveAtOperatorNativeTests(unittest.TestCase):
    def test_add_line_returns_curve_usable_with_at(self):
        part = Part(vec3(-20.0), vec3(20.0), tolerance=0.01)
        sk = part.sketch("xy", constrained=True, name="at")
        sk.solve_after_every_constraint = False
        a = sk.add_line((0.0, 0.0), (2.0, 1.0), name="a")
        b = sk.add_line((2.2, 1.1), (4.0, 0.0), name="b")
        self.assertIsInstance(a, SketchCurve)
        self.assertEqual("a", a.name)
        self.assertEqual("a@1.000", a @ 1.000)
        sk.coincident(a @ 1.000, b @ 0.000)
        sk.horizontal(a)
        sk.solve()
        solved = sk._solved_actions()
        self.assertAlmostEqual(solved[0]["p1"][0], solved[1]["p0"][0], places=5)
        self.assertAlmostEqual(solved[0]["p1"][1], solved[1]["p0"][1], places=5)
        self.assertAlmostEqual(solved[0]["p0"][1], solved[0]["p1"][1], places=5)

    def test_sketch_at_origin_is_qualified(self):
        part = Part(vec3(-20.0), vec3(20.0), tolerance=0.01)
        sk = part.sketch("xy", constrained=True, name="profile")
        sk.solve_after_every_constraint = False
        left = sk.add_line((1.0, -2.0), (1.0, 3.0), name="left")
        self.assertEqual("profile:origin", sk @ "origin")
        sk.vertical(left)
        sk.point_on_line(sk @ "origin", left)
        sk.solve()
        solved = sk._solved_actions()
        self.assertAlmostEqual(0.0, solved[0]["p0"][0], places=5)
        self.assertAlmostEqual(0.0, solved[0]["p1"][0], places=5)


if __name__ == "__main__":
    unittest.main()
