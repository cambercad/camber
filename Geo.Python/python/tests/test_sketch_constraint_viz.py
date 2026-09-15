import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from camber.sketch_constraints_viz import (
    LABEL_SCALE,
    build_constraint_overlay,
    format_angle_deg,
    format_length,
    format_radius,
    label_offset,
    nearest_constraint,
    nearest_dimension,
)
from camber import sketch_icons
from camber import sketch_ui


def _near(a, b, tol=1e-9):
    return abs(a - b) <= tol


def _xy_near(a, b, tol=1e-9):
    return _near(a[0], b[0], tol) and _near(a[1], b[1], tol)


def _has_seg(segs, a, b, tol=1e-8):
    for p, q in segs:
        if (_xy_near(p, a, tol) and _xy_near(q, b, tol)) or (
                _xy_near(p, b, tol) and _xy_near(q, a, tol)):
            return True
    return False


def _labels_of(overlay, kind=None):
    out = []
    for item in overlay.labels:
        text, xy, label_kind = item[0], item[1], item[2]
        if kind is None or label_kind == kind:
            out.append((text, xy))
    return out


class LabelOffsetTests(unittest.TestCase):
    def test_index_zero_points_along_x(self):
        offset = label_offset(0, 10.0)
        self.assertTrue(_xy_near(offset, (0.5, 0.0)))

    def test_uses_minimum_for_points(self):
        offset = label_offset(0, 0.0)
        self.assertTrue(_xy_near(offset, (0.01, 0.0)))


class GeometricGlyphTests(unittest.TestCase):
    def test_horizontal_h_at_mid_plus_offset(self):
        curves = [{"kind": "line", "p0": (0.0, 0.0), "p1": (10.0, 0.0)}]
        actions = [
            curves[0],
            {"kind": "horizontal", "curves": [0]},
        ]
        overlay = build_constraint_overlay(actions, curves)
        labels = _labels_of(overlay, "cons")
        self.assertEqual(1, len(labels))
        self.assertEqual("H", labels[0][0])
        self.assertTrue(_xy_near(labels[0][1], (5.5, 0.0)))
        self.assertTrue(_has_seg(overlay.cons_segs, (5.0, 0.0), (5.5, 0.0)))

    def test_vertical_v_at_mid(self):
        curves = [{"kind": "line", "p0": (2.0, 0.0), "p1": (2.0, 4.0)}]
        actions = [curves[0], {"kind": "vertical", "curves": [0]}]
        overlay = build_constraint_overlay(actions, curves)
        text, xy = _labels_of(overlay, "cons")[0]
        self.assertEqual("V", text)
        self.assertTrue(_xy_near(xy, (2.0 + 0.2, 2.0)))

    def test_parallel_two_slashes_per_line(self):
        curves = [
            {"kind": "line", "p0": (0.0, 0.0), "p1": (10.0, 0.0)},
            {"kind": "line", "p0": (0.0, 3.0), "p1": (10.0, 3.0)},
        ]
        actions = curves + [{"kind": "parallel", "curves": [0, 1]}]
        overlay = build_constraint_overlay(actions, curves)
        self.assertEqual(4, len(overlay.cons_segs))
        self.assertEqual(0, len(overlay.labels))

    def test_perpendicular_square_at_intersection(self):
        curves = [
            {"kind": "line", "p0": (0.0, 0.0), "p1": (10.0, 0.0)},
            {"kind": "line", "p0": (4.0, -2.0), "p1": (4.0, 6.0)},
        ]
        actions = curves + [{"kind": "perpendicular", "curves": [0, 1]}]
        overlay = build_constraint_overlay(actions, curves)
        size = max(0.01 * 6.0, min(10.0, 8.0) * 0.14)
        # Interior corner is toward the line midpoints: +X and +Y from (4, 0).
        a = (4.0 + size, 0.0)
        corner = (4.0 + size, size)
        c = (4.0, size)
        self.assertTrue(_has_seg(overlay.cons_segs, a, corner))
        self.assertTrue(_has_seg(overlay.cons_segs, corner, c))

    def test_midpoint_m(self):
        curves = [{"kind": "line", "p0": (0.0, 0.0), "p1": (8.0, 0.0)}]
        actions = [
            curves[0],
            {"kind": "midpoint", "points": [(0, 3)], "curves": [0]},
        ]
        overlay = build_constraint_overlay(actions, curves)
        text, xy = _labels_of(overlay, "cons")[0]
        self.assertEqual("M", text)
        expected = (4.0 + label_offset(0, 8.0)[0], label_offset(0, 8.0)[1])
        self.assertTrue(_xy_near(xy, expected))

    def test_coincident_pop_at_shared_point(self):
        curves = [
            {"kind": "line", "p0": (0.0, 0.0), "p1": (2.0, 0.0)},
            {"kind": "line", "p0": (2.0, 0.0), "p1": (2.0, 3.0)},
        ]
        actions = curves + [{"kind": "coincident", "points": [(0, 1), (1, 0)]}]
        overlay = build_constraint_overlay(actions, curves)
        text, xy = _labels_of(overlay, "cons")[0]
        self.assertEqual("PoP", text)
        expected = (2.0 + 0.01, 0.0)
        self.assertTrue(_xy_near(xy, expected))

    def test_named_coincident_to_line_mid_is_pol(self):
        curves = [
            {"kind": "line", "name": "Line1", "p0": (0.0, 0.0), "p1": (8.0, 0.0)},
            {"kind": "line", "name": "Line2", "p0": (4.0, 2.0), "p1": (4.0, -1.0)},
        ]
        actions = curves + [{
            "kind": "coincident",
            "points": ["Line2@0.000", "Line1@0.500"],
        }]
        overlay = build_constraint_overlay(actions, curves)
        text, xy = _labels_of(overlay, "cons")[0]
        self.assertEqual("PoL", text)
        self.assertTrue(_xy_near(xy, (4.0, 0.0), tol=0.2))

    def test_coincident_to_circle_cardinal_is_poc(self):
        curves = [
            {"kind": "line", "p0": (3.0, 0.0), "p1": (5.0, 0.0)},
            {"kind": "circle", "center": (0.0, 0.0), "radius": 3.0},
        ]
        actions = curves + [{"kind": "coincident", "points": [(0, 0), (1, 3)]}]
        overlay = build_constraint_overlay(actions, curves)
        text, xy = _labels_of(overlay, "cons")[0]
        self.assertEqual("PoC", text)
        offset = label_offset(0, 2.0 * math.pi * 3.0)
        self.assertTrue(_xy_near(xy, (3.0 + offset[0], offset[1])))

    def test_fix_is_pop(self):
        curves = [{"kind": "line", "p0": (1.0, 2.0), "p1": (4.0, 2.0)}]
        actions = [curves[0], {"kind": "fix", "points": [(0, 0)]}]
        overlay = build_constraint_overlay(actions, curves)
        text, xy = _labels_of(overlay, "cons")[0]
        self.assertEqual("PoP", text)
        self.assertTrue(_xy_near(xy, (1.01, 2.0)))

    def test_equal_length_marks_both_mids(self):
        curves = [
            {"kind": "line", "p0": (0.0, 0.0), "p1": (4.0, 0.0)},
            {"kind": "line", "p0": (0.0, 2.0), "p1": (4.0, 2.0)},
        ]
        actions = curves + [{"kind": "equal", "curves": [0, 1]}]
        overlay = build_constraint_overlay(actions, curves)
        texts = [t for t, _xy in _labels_of(overlay, "cons")]
        self.assertEqual(["=", "="], texts)

    def test_concentric_two_rings_and_cross(self):
        curves = [
            {"kind": "circle", "center": (0.0, 0.0), "radius": 2.0},
            {"kind": "circle", "center": (0.0, 0.0), "radius": 4.0},
        ]
        actions = curves + [{"kind": "concentric", "curves": [0, 1]}]
        overlay = build_constraint_overlay(actions, curves)
        r_mark = max(0.05, 2.0 * 0.12)
        self.assertGreaterEqual(len(overlay.cons_segs), 24 * 2 + 2)
        self.assertTrue(_has_seg(
            overlay.cons_segs,
            (-r_mark * 0.55, 0.0),
            (r_mark * 0.55, 0.0),
        ))
        self.assertTrue(_has_seg(
            overlay.cons_segs,
            (0.0, -r_mark * 0.55),
            (0.0, r_mark * 0.55),
        ))


class DimensionGlyphTests(unittest.TestCase):
    def test_length_offset_extensions_and_inward_arrows(self):
        curves = [{"kind": "line", "p0": (0.0, 0.0), "p1": (10.0, 0.0)}]
        actions = [curves[0], {"kind": "length", "curves": [0], "value": 10.0}]
        overlay = build_constraint_overlay(actions, curves)
        self.assertTrue(_has_seg(overlay.dim_segs, (0.0, 0.0), (0.0, 0.5)))
        self.assertTrue(_has_seg(overlay.dim_segs, (10.0, 0.0), (10.0, 0.5)))
        self.assertTrue(_has_seg(overlay.dim_segs, (0.0, 0.5), (10.0, 0.5)))
        labels = _labels_of(overlay, "dim")
        self.assertEqual([(format_length(10.0), (5.0, 0.5))], labels)
        self.assertEqual(2, len(overlay.arrows))
        self.assertTrue(_xy_near(overlay.arrows[0][0], (0.0, 0.5)))
        self.assertTrue(_xy_near(overlay.arrows[0][1], (1.0, 0.0)))
        self.assertTrue(_xy_near(overlay.arrows[1][0], (10.0, 0.5)))
        self.assertTrue(_xy_near(overlay.arrows[1][1], (-1.0, 0.0)))

    def test_distance_uses_handle_positions(self):
        curves = [
            {"kind": "line", "p0": (0.0, 0.0), "p1": (3.0, 0.0)},
            {"kind": "line", "p0": (0.0, 4.0), "p1": (3.0, 4.0)},
        ]
        actions = curves + [{
            "kind": "distance",
            "points": [(0, 0), (1, 0)],
            "value": 4.0,
        }]
        overlay = build_constraint_overlay(actions, curves)
        labels = _labels_of(overlay, "dim")
        self.assertEqual(format_length(4.0), labels[0][0])
        self.assertTrue(_has_seg(overlay.dim_segs, (0.0, 0.0), (-0.2, 0.0)))

    def test_radius_line_and_g4_label(self):
        curves = [{"kind": "circle", "center": (1.0, 2.0), "radius": 3.0}]
        actions = [curves[0], {"kind": "radius", "curves": [0], "value": 3.0}]
        overlay = build_constraint_overlay(actions, curves)
        self.assertTrue(_has_seg(overlay.dim_segs, (1.0, 2.0), (-2.0, 2.0)))
        text, xy = _labels_of(overlay, "dim")[0]
        self.assertEqual(format_radius(3.0), text)
        self.assertTrue(_xy_near(xy, (-0.5, 2.0)))

    def test_angle_label_is_degrees(self):
        curves = [
            {"kind": "line", "p0": (0.0, 0.0), "p1": (4.0, 0.0)},
            {"kind": "line", "p0": (0.0, 0.0), "p1": (0.0, 4.0)},
        ]
        actions = curves + [{
            "kind": "angle",
            "curves": [0, 1],
            "value": 90.0,
        }]
        overlay = build_constraint_overlay(actions, curves)
        texts = [t for t, _xy in _labels_of(overlay, "dim")]
        self.assertEqual([format_angle_deg(90.0)], texts)
        self.assertGreaterEqual(len(overlay.dim_segs), 10)

    def test_uses_solved_geometry_not_seed(self):
        recorded = [
            {"kind": "line", "p0": (0.0, 0.0), "p1": (4.0, 1.0)},
            {"kind": "horizontal", "curves": [0]},
        ]
        solved = [{"kind": "line", "p0": (0.0, 2.0), "p1": (4.0, 2.0)}]
        overlay = build_constraint_overlay(recorded, solved)
        text, xy = _labels_of(overlay, "cons")[0]
        self.assertEqual("H", text)
        self.assertTrue(_xy_near(xy, (2.2, 2.0)))


class ConstraintPickAndEditTests(unittest.TestCase):
    def test_label_scale_is_four_and_a_half(self):
        self.assertEqual(4.5, LABEL_SCALE)

    def test_length_label_picks_the_dimension_action(self):
        curves = [{"kind": "line", "p0": (0.0, 0.0), "p1": (10.0, 0.0)}]
        actions = [
            curves[0],
            {"kind": "horizontal", "curves": [0]},
            {"kind": "length", "curves": [0], "value": 10.0},
        ]
        overlay = build_constraint_overlay(actions, curves)
        self.assertEqual(2, overlay.labels[-1][3])
        self.assertEqual(2, nearest_constraint(overlay, (5.0, 0.5), label_radius=0.3))
        self.assertEqual(2, nearest_dimension(overlay, (5.0, 0.5)))
        self.assertEqual(2, nearest_dimension(overlay, (2.0, 0.5)))

    def test_named_length_and_angle_are_editable_dimensions(self):
        curves = [
            {"kind": "line", "name": "Line1", "p0": (0.0, 0.0), "p1": (8.0, 0.0)},
            {"kind": "line", "name": "Line2", "p0": (0.0, 0.0), "p1": (0.0, 6.0)},
        ]
        actions = curves + [
            {"kind": "length", "curves": ["Line1"], "value": 8.0},
            {"kind": "angle", "curves": ["Line1", "Line2"], "value": 90.0},
            {"kind": "distance", "points": ["Line1@0.000", "Line2@1.000"], "value": 6.0},
        ]
        overlay = build_constraint_overlay(actions, curves)
        self.assertEqual(2, nearest_dimension(overlay, (4.0, 0.4)))
        for index, kind in ((2, "length"), (3, "angle"), (4, "distance")):
            state = {
                "actions": actions,
                "tool": "select",
                "pending": [],
                "pending_constraint": None,
                "pending_dimension": None,
                "slots": [],
                "dimension": None,
            }
            self.assertTrue(sketch_ui._begin_dimension_edit(state, index), kind)
            self.assertEqual(kind, state["dimension"]["kind"])

    def test_begin_edit_opens_the_construction_value_box(self):
        action = {"kind": "length", "curves": [0], "points": [], "value": 10.0}
        state = {
            "actions": [{"kind": "line", "p0": (0.0, 0.0), "p1": (10.0, 0.0)}, action],
            "tool": "select",
            "pending": [],
            "pending_constraint": None,
            "pending_dimension": None,
            "slots": [],
            "dimension": None,
        }
        self.assertTrue(sketch_ui._begin_dimension_edit(state, 1))
        self.assertEqual(10.0, state["dimension"]["value"])
        self.assertEqual(1, state["dimension"]["edit_index"])
        self.assertTrue(state["dimension_focus"])

    def test_dimension_editor_stays_inside_palette_width(self):
        pair = sketch_icons.dimension_button_width() * 2 + sketch_icons.ITEM_GAP
        self.assertAlmostEqual(sketch_icons.CONTENT_W, pair)
        self.assertLessEqual(sketch_icons.CONTENT_W, sketch_icons.PANEL_W)
        imgui = _DimensionEditorSpy()
        changed, value, apply_clicked, cancel_clicked = sketch_icons.dimension_editor(
            imgui, "radius", 12.5, focus=True)
        self.assertFalse(changed)
        self.assertAlmostEqual(12.5, value)
        self.assertFalse(apply_clicked)
        self.assertFalse(cancel_clicked)
        self.assertEqual([sketch_icons.CONTENT_W], imgui.item_widths)
        self.assertEqual(1, imgui.focus_count)
        self.assertTrue(imgui.labels[0].startswith("##"))
        self.assertLessEqual(max(imgui.button_widths), sketch_icons.CONTENT_W)

    def test_commit_edit_updates_the_existing_action(self):
        live = _DimensionSpy()
        geom = {"kind": "line", "p0": (0.0, 0.0), "p1": (10.0, 0.0)}
        dim = {"kind": "length", "curves": [0], "points": [], "value": 10.0}
        state = {
            "actions": [geom, dim],
            "batches": [
                {"actions": [geom], "curves": 1, "constraints": 0},
                {"actions": [dim], "curves": 0, "constraints": 1},
            ],
            "sketch": live,
            "dimension": {
                "kind": "length",
                "curves": [0],
                "points": [],
                "value": 7.5,
                "edit_index": 1,
            },
            "selected": [],
            "solved": [geom],
        }
        sketch_ui._commit_dimension(state)
        self.assertIsNone(state["dimension"])
        self.assertEqual(2, len(state["actions"]))
        self.assertEqual(7.5, state["actions"][1]["value"])
        self.assertIn(("remove_constraint",), live.calls)
        self.assertIn(("length", 0, 7.5), live.calls)
        self.assertNotIn(("length", 0, 10.0), live.calls)


class _FakeDrawList(object):
    def AddRectFilled(self, *args, **kwargs):
        pass

    def AddText(self, *args, **kwargs):
        pass


class _DimensionEditorSpy(object):
    def __init__(self):
        self.item_widths = []
        self.focus_count = 0
        self.labels = []
        self.button_widths = []

    def GetContentRegionAvail(self):
        return (sketch_icons.CONTENT_W, 100.0)

    def CalcTextSize(self, text):
        return (len(text) * 7.0, 14.0)

    def GetCursorPosX(self):
        return 0.0

    def GetCursorScreenPos(self):
        return (0.0, 0.0)

    def PushTextWrapPos(self, *args):
        pass

    def PopTextWrapPos(self):
        pass

    def TextUnformatted(self, text):
        pass

    def Text(self, text):
        pass

    def PushItemWidth(self, width):
        self.item_widths.append(float(width))

    def PopItemWidth(self):
        pass

    def SetKeyboardFocusHere(self, *args):
        self.focus_count += 1

    def InputDouble(self, label, value, *args):
        self.labels.append(label)
        return False, value

    def InvisibleButton(self, button_id, size, *args):
        width = size[0] if hasattr(size, "__len__") else size.x
        self.button_widths.append(float(width))
        return False

    def IsItemHovered(self):
        return False

    def GetWindowDrawList(self):
        return _FakeDrawList()

    def GetColorU32(self, color):
        return 0

    def SameLine(self):
        pass


class _DimensionSpy(object):
    def __init__(self):
        self.calls = []
        self.curve_count = 1
        self.constraint_count = 1

    def length(self, curve, value):
        self.calls.append(("length", curve, value))
        self.constraint_count += 1

    def remove_last_constraint(self):
        self.calls.append(("remove_constraint",))
        self.constraint_count = max(0, self.constraint_count - 1)

    def remove_last_curve(self):
        self.calls.append(("remove_curve",))
        self.curve_count = max(0, self.curve_count - 1)

    def solve(self):
        self.calls.append(("solve",))

    def _solved_actions(self):
        return [{"kind": "line", "p0": (0.0, 0.0), "p1": (7.5, 0.0)}]


class ActionsBoundsTests(unittest.TestCase):
    def test_fits_lines_and_arc(self):
        fallback = (-20.0, 20.0, -20.0, 20.0)
        bounds = sketch_ui._actions_bounds(
            [
                {"kind": "line", "p0": (0.0, 25.0), "p1": (31.0, 25.0)},
                {"kind": "arc", "start": (31.0, 25.0), "mid": (40.0, 0.0), "end": (19.0, -35.0)},
                {"kind": "line", "p0": (19.0, -35.0), "p1": (0.0, -35.0)},
            ],
            fallback,
        )
        self.assertLess(bounds[0], 0.0)
        self.assertGreater(bounds[1], 31.0)
        self.assertLess(bounds[2], -35.0)
        self.assertGreater(bounds[3], 25.0)

    def test_empty_keeps_fallback(self):
        fallback = (-8.0, 8.0, -8.0, 8.0)
        self.assertEqual(fallback, sketch_ui._actions_bounds([], fallback))


if __name__ == "__main__":
    unittest.main()
