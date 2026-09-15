import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from camber import pick
from camber.view import (
    _SELECT_SURFACE,
    _SURFACE_BLUE,
    _CONSTRAINT_REF_COLORS,
    _constraint_ref_color,
    _constraint_row_text,
    _load_constraints,
    _mate_triangle_indices,
    _name_is_selected,
    _parse_constraint_dump,
    _csharp_phong_shade,
    _selected_plane_context,
    _surface_face_colors,
    _vertex_ambient_occlusion,
)


class SurfaceSelectionHighlightTests(unittest.TestCase):
    def test_selected_surface_paints_matching_faces_orange(self):
        names = ["box:top", "box:top", "box:side", "box:side"]
        colors = _surface_face_colors(
            names, _SURFACE_BLUE, {"box:top"}, _SELECT_SURFACE, [],
        )
        self.assertEqual(_SELECT_SURFACE, colors[0])
        self.assertEqual(_SELECT_SURFACE, colors[1])
        self.assertEqual(_SURFACE_BLUE, colors[2])
        self.assertEqual(_SURFACE_BLUE, colors[3])

    def test_clicking_near_an_edge_selects_the_curve(self):
        selected = pick.prefer_selection(
            "box:[top,side]", pick.KIND_CURVE, "box:top", pick.KIND_SURFACE)
        self.assertEqual("box:[top,side]", selected)

    def test_clicking_a_face_interior_selects_the_surface(self):
        selected = pick.prefer_selection(None, None, "box:top", pick.KIND_SURFACE)
        self.assertEqual("box:top", selected)
        names = ["box:top", "box:top", "box:side"]
        colors = _surface_face_colors(
            names, _SURFACE_BLUE, {selected}, _SELECT_SURFACE, [],
        )
        self.assertTrue(any(color == _SELECT_SURFACE for color in colors))
        self.assertEqual(_SELECT_SURFACE, colors[0])
        self.assertEqual(_SURFACE_BLUE, colors[2])

    def test_qualified_surface_name_still_matches_display_faces(self):
        self.assertTrue(_name_is_selected("box:top", {"box:top"}))
        names = ["pipe:pipe-Line1", "pipe:pipe-Line1", "pipe:pipe-ExtrudeTop"]
        colors = _surface_face_colors(
            names, _SURFACE_BLUE, {"pipe:pipe-Line1"}, _SELECT_SURFACE, [],
        )
        self.assertEqual(_SELECT_SURFACE, colors[0])
        self.assertEqual(_SURFACE_BLUE, colors[2])

    def test_unselected_faces_keep_ao_tint(self):
        names = ["box:top", "box:side"]
        colors = _surface_face_colors(
            names, _SURFACE_BLUE, {"box:top"}, _SELECT_SURFACE, [], [0.5, 0.5],
        )
        self.assertEqual(_SELECT_SURFACE, colors[0])
        self.assertEqual((0.5 * _SURFACE_BLUE[0], 0.5 * _SURFACE_BLUE[1], 0.5 * _SURFACE_BLUE[2]), colors[1])


class StudioLightingTests(unittest.TestCase):
    def test_key_light_is_brighter_than_underside(self):
        lit = _csharp_phong_shade(-0.35, 0.45, 0.82)
        down = _csharp_phong_shade(0.0, -0.85, 0.53)
        self.assertGreater(lit, down)

    def test_phong_matches_csharp_preview_constants(self):
        x, y, z = 0.0, 0.0, 1.0
        lx, ly, lz = 1.0 / 3.0, 2.0 / 3.0, 2.0 / 3.0
        ndotl = abs(x * lx + y * ly + z * lz)
        hx, hy, hz = lx, ly, lz + 1.0
        hl = (hx * hx + hy * hy + hz * hz) ** 0.5
        ndoth = abs((x * hx + y * hy + z * hz) / hl)
        expected = 0.6 + 0.3 * ndotl + 0.1 * (ndoth ** 8)
        self.assertAlmostEqual(expected, _csharp_phong_shade(x, y, z))

    def test_facing_plates_are_more_occluded_than_a_lone_quad(self):
        lone = [
            (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0), (0.0, 1.0, 0.0),
        ]
        lone_faces = [(0, 1, 2), (0, 2, 3)]
        gap = 0.04
        facing = list(lone)
        facing.extend((x, y, gap) for x, y, _z in lone)
        facing_faces = lone_faces + [(4, 6, 5), (4, 7, 6)]
        open_ao = _vertex_ambient_occlusion(lone, lone_faces)
        closed_ao = _vertex_ambient_occlusion(facing, facing_faces)
        if open_ao is None or closed_ao is None:
            self.skipTest("numpy is required for mesh AO")
        self.assertTrue(all(0.35 <= a <= 1.0 for a in open_ao + closed_ao))
        self.assertLess(sum(closed_ao) / len(closed_ao), sum(open_ao) / len(open_ao) - 0.04)


class _FakeFrame(object):
    def __init__(self):
        self.origin = (0.0, 0.0, 1.0)
        self.x = (1.0, 0.0, 0.0)
        self.y = (0.0, 1.0, 0.0)
        self.z = (0.0, 0.0, 1.0)


class _FakePart(object):
    def sketch_interactive(self, *args, **kwargs):
        return None

    def plane_frame(self, name):
        if name != "box:top":
            return None
        return _FakeFrame()


class _FakeSolid(object):
    def __init__(self, part):
        self._part = part


class SelectedPlaneSketchTests(unittest.TestCase):
    def test_solid_uses_owning_part_plane(self):
        part = _FakePart()
        ctx = _selected_plane_context(_FakeSolid(part), ["box:top"])
        self.assertIsNotNone(ctx)
        self.assertIs(part, ctx["part"])
        self.assertEqual("box:top", ctx["name"])
        self.assertFalse(ctx["emit_frame"])

    def test_requires_exactly_one_planar_name(self):
        solid = _FakeSolid(_FakePart())
        self.assertIsNone(_selected_plane_context(solid, []))
        self.assertIsNone(_selected_plane_context(solid, ["box:top", "box:side"]))
        self.assertIsNone(_selected_plane_context(solid, ["not-a-face"]))

    def test_cuboid_face_resolves_to_part_frame(self):
        try:
            from camber import Part
        except ImportError:
            self.skipTest("native camber is required")
        part = Part(-10, 10)
        solid = part.cuboid((-1, -1, -1), (1, 1, 1), name="box")
        name = "box:box-ExtrudeTop"
        ctx = _selected_plane_context(solid, [name])
        self.assertIsNotNone(ctx)
        self.assertIs(part, ctx["part"])
        self.assertEqual(name, ctx["name"])
        self.assertFalse(ctx["emit_frame"])
        self.assertAlmostEqual(1.0, ctx["frame"].origin[2])


class AssemblyConstraintPanelTests(unittest.TestCase):
    def test_tab_item_uses_selected_not_truthy_struct(self):
        class _Tab(object):
            def __init__(self, selected):
                self.selected = selected

            def __bool__(self):
                return True

        from camber.view import _imgui_flag
        self.assertTrue(_imgui_flag(_Tab(True), "selected"))
        self.assertFalse(_imgui_flag(_Tab(False), "selected"))
        self.assertTrue(_imgui_flag((True, True), "selected"))

    def test_dump_parses_kind_label_and_entities(self):
        dump = (
            "C\tCoincidentPlanes\tCoincident: FaceA <> FaceB\n"
            "E\tpipe1:pipe1-Flange\n"
            "E\tpipe2:pipe2-Flange\n"
            "G\tplane\t0\t0\t1\t0\t0\t1\n"
            "G\tplane\t0\t0\t0\t0\t0\t-1\n"
            "C\tFixPart\tFix pipe1\n"
            "E\tpipe1:\n"
            "G\tpoint\t0\t0\t0\t0\t0\t1\n"
        )
        parsed = _parse_constraint_dump(dump)
        self.assertEqual(2, len(parsed))
        self.assertEqual("CoincidentPlanes", parsed[0]["kind"])
        self.assertEqual(["pipe1:pipe1-Flange", "pipe2:pipe2-Flange"], parsed[0]["entities"])
        self.assertEqual("plane", parsed[0]["glyphs"][0]["kind"])
        self.assertEqual((0.0, 0.0, 1.0), parsed[0]["glyphs"][0]["origin"])
        self.assertIn("Coincident", _constraint_row_text(parsed[0]))
        self.assertIn("Flange", _constraint_row_text(parsed[0]))
        self.assertTrue(_constraint_row_text(parsed[1]).startswith("Fix"))

    def test_load_constraints_uses_python_records(self):
        class _Asm(object):
            def __init__(self):
                self._n = object()
                self.constraints = [{
                    "kind": "ParallelAxes",
                    "label": "Parallel: A <> B",
                    "entities": ["p1:axis", "p2:axis"],
                }]

        loaded = _load_constraints(_Asm())
        self.assertEqual(1, len(loaded))
        self.assertEqual("ParallelAxes", loaded[0]["kind"])

    def test_mate_refs_paint_the_same_named_patches_as_picking(self):
        names = ["pipe1:flange", "pipe1:flange", "pipe2:flange", "pipe2:body"]
        refs = ["pipe1:flange", "pipe2:flange"]
        picked = _surface_face_colors(
            names, _SURFACE_BLUE, {"pipe1:flange"}, _SELECT_SURFACE, [])
        mated = _surface_face_colors(names, _SURFACE_BLUE, set(), _SELECT_SURFACE, refs)
        self.assertEqual(_SELECT_SURFACE, picked[0])
        self.assertEqual(_SURFACE_BLUE, picked[2])
        self.assertEqual(_CONSTRAINT_REF_COLORS[0], mated[0])
        self.assertEqual(_CONSTRAINT_REF_COLORS[0], mated[1])
        self.assertEqual(_CONSTRAINT_REF_COLORS[1], mated[2])
        self.assertEqual(_SURFACE_BLUE, mated[3])
        self.assertEqual(_CONSTRAINT_REF_COLORS[0], _constraint_ref_color("pipe1:flange", refs))
        self.assertIsNone(_constraint_ref_color("pipe2:body", refs))
        self.assertIsNone(_constraint_ref_color("housing_od:housing_od-Circle1", ["housing_od:"]))

    def test_mate_triangles_are_the_opaque_subset(self):
        names = [
            "pipe1:flange", "pipe1:flange", "pipe1:flange",
            "pipe1:body", "pipe1:body", "pipe1:body",
            "pipe2:flange", "pipe2:flange", "pipe2:flange",
        ]
        idx = [(0, 1, 2), (3, 4, 5), (6, 7, 8)]
        refs = ["pipe1:flange", "pipe2:flange"]
        self.assertEqual(
            [(0, 1, 2), (6, 7, 8)],
            _mate_triangle_indices(idx, names, refs))
        self.assertEqual([], _mate_triangle_indices(idx, names, []))
        self.assertEqual([], _mate_triangle_indices(idx, names, ["pipe1:"]))

    def test_mate_highlight_includes_split_patch_islands(self):
        names = [
            "sun:sun_shaft-Circle1",
            "sun:sun_shaft-Circle1_1",
            "sun:sun_shaft-ExtrudeTop",
        ]
        refs = ["sun:sun_shaft-Circle1"]
        colors = _surface_face_colors(names, _SURFACE_BLUE, set(), _SELECT_SURFACE, refs)
        self.assertEqual(_CONSTRAINT_REF_COLORS[0], colors[0])
        self.assertEqual(_CONSTRAINT_REF_COLORS[0], colors[1])
        self.assertEqual(_SURFACE_BLUE, colors[2])


if __name__ == "__main__":
    unittest.main()
