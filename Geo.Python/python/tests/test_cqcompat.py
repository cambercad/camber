import os
import sys
import unittest
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from camber.cqcompat import (
    Workplane,
    named_plane,
    _parse_selector as parse_selector,
    _radius_arc_mid as radius_arc_mid,
    _select_edges as select_edges,
    _select_faces as select_faces,
    _sample_parametric_curve,
)
from camber.cqcompat import _center_shifts, _prism_faces
from camber.vec import vec3

try:
    from _camber_native.__dotwrap_generated.main import NativePart
    _NATIVE_AVAILABLE = NativePart is not None
except ImportError:
    _NATIVE_AVAILABLE = False


def _face(name, center, normal, kind="plane"):
    return {"name": name, "center": center, "normal": normal, "kind": kind}


def _edge(name, center, direction, kind="line"):
    return {"name": name, "center": center, "direction": direction, "kind": kind}


class PlaneAndSelectorTests(unittest.TestCase):
    def test_named_xy_matches_world_axes(self):
        frame = named_plane("XY")
        self.assertEqual(vec3(frame.x), vec3(1, 0, 0))
        self.assertEqual(vec3(frame.y), vec3(0, 1, 0))
        self.assertEqual(vec3(frame.z), vec3(0, 0, 1))

    def test_aliases_and_origin(self):
        front = named_plane("front", origin=(1, 2, 3))
        self.assertEqual(vec3(front.origin), vec3(1, 2, 3))
        self.assertEqual(vec3(front.z), vec3(named_plane("XY").z))
        top = named_plane("top")
        self.assertAlmostEqual(top.z.y, 1.0)
        self.assertAlmostEqual(_dot_z(named_plane("YZ")), 0.0)

    def test_unknown_plane_raises(self):
        with self.assertRaises(ValueError):
            named_plane("UV")

    def test_parse_selector_tokens(self):
        self.assertEqual([(">", "Z")], parse_selector(">Z"))
        self.assertEqual([("|", "Z"), (">", "X")], parse_selector("|Z and >X"))
        self.assertEqual([("type", "CIRCLE")], parse_selector("%CIRCLE"))
        self.assertEqual([("name", "box-ExtrudeTop")], parse_selector("box-ExtrudeTop"))
        self.assertEqual([], parse_selector(""))

    def test_center_shifts(self):
        self.assertEqual((-1.0, -2.0, -3.0), _center_shifts((2, 4, 6), True))
        self.assertEqual((0.0, 0.0, 0.0), _center_shifts((2, 4, 6), False))
        self.assertEqual((-1.0, 0.0, -3.0), _center_shifts((2, 4, 6), (True, False, True)))

    def test_face_extreme_and_parallel(self):
        faces = [
            _face("top", (0, 0, 1), (0, 0, 1)),
            _face("bottom", (0, 0, -1), (0, 0, -1)),
            _face("south", (0, -1, 0), (0, -1, 0)),
            _face("east", (1, 0, 0), (1, 0, 0)),
        ]
        self.assertEqual(["top"], [f["name"] for f in select_faces(faces, ">Z")])
        self.assertEqual(["bottom"], [f["name"] for f in select_faces(faces, "<Z")])
        self.assertEqual(["top"], [f["name"] for f in select_faces(faces, "+Z")])
        walls = sorted(f["name"] for f in select_faces(faces, "|Z"))
        self.assertEqual(["east", "south"], walls)
        caps = sorted(f["name"] for f in select_faces(faces, "#Z"))
        self.assertEqual(["bottom", "top"], caps)

    def test_face_selector_or_combines_branches(self):
        faces = [
            _face("top", (0, 0, 1), (0, 0, 1)),
            _face("west", (-1, 0, 0), (-1, 0, 0)),
            _face("east", (1, 0, 0), (1, 0, 0)),
            _face("south", (0, -1, 0), (0, -1, 0)),
        ]
        self.assertEqual(["top", "west", "east"],
                         [face["name"] for face in select_faces(faces, "+Z or -X or +X")])

    def test_edge_parallel_to_z(self):
        edges = [
            _edge("vert", (0, 0, 0), (0, 0, 1)),
            _edge("horiz", (0, 0, 1), (1, 0, 0)),
        ]
        self.assertEqual(["vert"], [e["name"] for e in select_edges(edges, "|Z")])
        self.assertEqual(["horiz"], [e["name"] for e in select_edges(edges, "#Z")])

    def test_radius_arc_mid_semicircle(self):
        mid = radius_arc_mid((0.0, 0.0), (2.0, 0.0), 1.0)
        self.assertAlmostEqual(mid[0], 1.0)
        self.assertAlmostEqual(mid[1], -1.0)

    def test_prism_registry_has_caps_and_verticals(self):
        from camber.api import Frame
        frame = Frame()
        faces, edges = _prism_faces("box", frame, -1, 1, -2, 2, -3, 3)
        names = [f["name"] for f in faces]
        self.assertIn("box-ExtrudeTop", names)
        self.assertIn("box-south", names)
        vertical = [e["name"] for e in select_edges(edges, "|Z")]
        self.assertEqual(4, len(vertical))
        top = select_faces(faces, ">Z")[0]
        self.assertEqual("box-ExtrudeTop", top["name"])
        self.assertAlmostEqual(top["center"].z, 3.0)


def _dot_z(frame):
    return frame.z.x * 0 + frame.z.y * 0 + frame.z.z


class ParametricCurveSamplingTests(unittest.TestCase):
    def test_adaptive_circle_sampling_respects_chord_tolerance(self):
        points = _sample_parametric_curve(
            lambda t: (math.cos(t), math.sin(t)), N=8,
            start=0, stop=2 * math.pi, tolerance=0.005,
        )
        self.assertGreater(len(points), 9)
        self.assertAlmostEqual(points[0][0], points[-1][0], places=12)
        self.assertAlmostEqual(points[0][1], points[-1][1], places=12)
        angles = [math.atan2(y, x) for x, y in points]
        for first, second in zip(angles, angles[1:]):
            delta = abs(second - first)
            if delta > math.pi:
                delta = 2 * math.pi - delta
            self.assertLessEqual(1 - math.cos(delta / 2), 0.005 + 1e-9)

    def test_tighter_tolerance_adds_samples(self):
        curve = lambda t: (math.cos(t), math.sin(t))
        coarse = _sample_parametric_curve(curve, 8, 0, 2 * math.pi, 0.05)
        fine = _sample_parametric_curve(curve, 8, 0, 2 * math.pi, 0.001)
        self.assertGreater(len(fine), len(coarse))

    def test_rejects_invalid_callback_results_and_parameters(self):
        with self.assertRaises(ValueError):
            _sample_parametric_curve(lambda t: (float("nan"), 0), 8, 0, 1, 0.01)
        with self.assertRaises(ValueError):
            _sample_parametric_curve(lambda t: (t, 0), 1, 0, 1, 0.01)
        with self.assertRaises(ValueError):
            _sample_parametric_curve(lambda t: (t, 0), 100000, 0, 1, 0.01)
        with self.assertRaises(NotImplementedError):
            _sample_parametric_curve(lambda t: (t, 0, 1), 8, 0, 1, 0.01)


class PendingGeometryTests(unittest.TestCase):
    def test_rect_and_circle_are_pending(self):
        wp = Workplane("XY").rect(2, 4).circle(0.5)
        self.assertEqual(2, len(wp._pending))
        self.assertEqual("rect", wp._pending[0].kind)
        self.assertEqual("circle", wp._pending[1].kind)

    def test_line_to_close(self):
        wp = Workplane("XY").moveTo(0, 0).lineTo(2, 0).lineTo(2, 1).close()
        self.assertEqual(1, len(wp._pending))
        self.assertGreaterEqual(len(wp._pending[0].segments), 3)

    def test_rarray_centered_grid(self):
        locs = Workplane("XY").rarray(2, 4, 2, 2)._locations
        self.assertEqual(
            [(-1.0, -2.0), (1.0, -2.0), (-1.0, 2.0), (1.0, 2.0)],
            locs,
        )

    def test_polar_array_cardinals(self):
        locs = Workplane("XY").polarArray(2, 0, 360, 4)._locations
        self.assertEqual(4, len(locs))
        self.assertAlmostEqual(locs[0][0], 2.0)
        self.assertAlmostEqual(locs[1][1], 2.0)

    def test_circle_at_each_location(self):
        wp = Workplane("XY").rarray(2, 2, 2, 1).circle(0.25)
        self.assertEqual(2, len(wp._pending))

    def test_sketch_finalize_returns_parent_with_wires(self):
        wp = Workplane("XY").sketch().rect(1, 1).circle(0.2).finalize()
        self.assertEqual(2, len(wp._pending))
        self.assertIsInstance(wp, Workplane)

    def test_workplane_offset_records_loft_section(self):
        wp = Workplane("XY").circle(1).workplane(offset=5).circle(2)
        self.assertEqual(1, len(wp._sections))
        self.assertAlmostEqual(wp._frame.origin.z, 5.0)
        self.assertEqual(1, len(wp._wires()))

    def test_selected_face_becomes_active_plane(self):
        wp = Workplane("XY")._spawn(_selected_faces=[{
            "name": "box-ExtrudeTop",
            "center": vec3(0, 0, 1),
            "normal": vec3(0, 0, 1),
            "kind": "plane",
        }])
        frame = wp._active_frame()
        self.assertAlmostEqual(frame.origin.z, 1.0)
        self.assertAlmostEqual(frame.z.z, 1.0)

    def test_missing_features_raise(self):
        wp = Workplane("XY")
        with self.assertRaises(ValueError):
            wp.shell(0.2)
        with self.assertRaises(ValueError):
            wp.translate((1, 0, 0))
        with self.assertRaises(NotImplementedError):
            wp.vertices()

    def test_center_keeps_earlier_wires_in_place(self):
        wp = Workplane("XY").circle(3).center(1.5, 0).rect(0.5, 0.5)
        self.assertEqual((0.0, 0.0), wp._pending[0].segments[0][1])
        self.assertEqual((1.25, -0.25), wp._pending[1].segments[0][1])

    def test_center_shifts_cursor_and_absolute_path_points(self):
        wire = (Workplane("XY").center(-10, 0).vLine(3)
                .threePointArc((10, 9), (20, 3)).vLine(-3).mirrorX()._pending[0])
        self.assertEqual(("line", (-10.0, 0.0), (-10.0, 3.0)), wire.segments[0])
        self.assertEqual(("arc", (-10.0, 3.0), (0.0, 9.0), (10.0, 3.0)), wire.segments[1])
        xs = [point[0] for segment in wire.segments for point in segment[1:]]
        self.assertAlmostEqual(-10.0, min(xs))
        self.assertAlmostEqual(10.0, max(xs))

    def test_arc_stays_in_path_order(self):
        wp = Workplane("XY").lineTo(2, 0).threePointArc((3, 1), (2, 2)).close()
        self.assertEqual(["line", "arc", "line"], [s[0] for s in wp._pending[0].segments])

    def test_construction_rectangle_yields_four_locations(self):
        wp = Workplane("XY").rect(2, 4, forConstruction=True).vertices()
        self.assertEqual({(-1.0, -2.0), (1.0, -2.0), (1.0, 2.0), (-1.0, 2.0)},
                         set(wp._locations))

    def test_2d_mirror_closes_outline(self):
        wp = Workplane("XY").polyline([(0, 1), (2, 1), (2, -1), (0, -1)]).mirrorY()
        self.assertEqual(1, len(wp._pending))
        self.assertIsNone(wp._open)
        self.assertGreater(len(wp._pending[0].segments), 4)


@unittest.skipUnless(_NATIVE_AVAILABLE, "requires the installed camber native module")
class NativeWorkplaneTests(unittest.TestCase):
    def test_classic_bottle_profile_is_centered_and_extrudes_one_sided(self):
        body = (Workplane("XY", size=50, tolerance=0.01).center(-10, 0).vLine(3)
                .threePointArc((10, 9), (20, 3)).vLine(-3).mirrorX()
                .extrude(30.0, True))
        points, _ = body.val().mesh()
        self.assertAlmostEqual(-10.0, min(point.x for point in points), delta=0.01)
        self.assertAlmostEqual(10.0, max(point.x for point in points), delta=0.01)
        self.assertAlmostEqual(0.0, min(point.z for point in points), delta=0.01)
        self.assertAlmostEqual(30.0, max(point.z for point in points), delta=0.01)
        bottle = (body.faces(">Z").workplane(centerOption="CenterOfMass")
                  .circle(3.0).extrude(2.0, True))
        bottle_points, _ = bottle.val().mesh()
        self.assertAlmostEqual(32.0, max(point.z for point in bottle_points), delta=0.01)

    def test_closed_inward_shell_bridge(self):
        block = Workplane("XY", size=20, tolerance=0.01).box(2, 2, 2)
        result = block.shell(-0.1)
        self.assertTrue(result.val().is_watertight())
        self.assertAlmostEqual(result.val().volume(), 8 - 1.8**3, delta=0.01)
        self.assertTrue(any("ShellInner_" in name for name in result.val().patch_names))
        self.assertFalse(any("ShellRim_" in name for name in result.val().patch_names))

    def test_outward_shell_bridge(self):
        block = Workplane("XY", size=20, tolerance=0.01).box(2, 2, 2)
        closed = block.shell(0.1)
        opened = block.faces(">Z").shell(0.1)
        self.assertTrue(closed.val().is_watertight())
        self.assertTrue(opened.val().is_watertight())
        self.assertAlmostEqual(closed.val().volume(), 2.2**3 - 8, delta=0.01)
        self.assertAlmostEqual(opened.val().volume(), 2.2**3 - 2*2*2.1, delta=0.01)
        self.assertTrue(any("ShellRim_" in name for name in opened.val().patch_names))
        self.assertAlmostEqual(block.part.shell(block.val(), 0.1, outward=True).volume(),
                               closed.val().volume(), delta=1e-6)

    def test_part_shell_optional_faces(self):
        block = Workplane("XY", size=20, tolerance=0.01).box(2, 2, 2)
        closed = block.part.shell(block.val(), 0.1)
        explicit_empty = block.part.shell(block.val(), 0.1, faces=[])
        self.assertAlmostEqual(closed.volume(), explicit_empty.volume(), delta=1e-6)

    def test_closed_spherical_shell(self):
        result = Workplane("XY", size=20, tolerance=0.01).sphere(2).shell(-0.2)
        self.assertTrue(result.val().is_watertight())
        self.assertAlmostEqual(result.val().volume(), 4 * 3.141592653589793 / 3 *
                               (2**3 - 1.8**3), delta=0.3)

    def test_open_inward_shell_bridge(self):
        result = Workplane("XY", size=20, tolerance=0.05).box(2, 2, 2).faces(">Z").shell(-0.1)
        self.assertTrue(result.val().is_watertight())
        self.assertLess(result.val().volume(), 8.0)

    def test_offset2d_profiles(self):
        source = Workplane("XY", size=20, tolerance=0.05).polygon(5, 10).extrude(0.1)
        outward = Workplane("XY", size=20, tolerance=0.05).polygon(5, 10).offset2D(1).extrude(0.1)
        inward = Workplane("XY", size=20, tolerance=0.05).polygon(5, 10).offset2D(-0.5, "intersection").extrude(0.1)
        self.assertGreater(outward.val().volume(), source.val().volume())
        self.assertLess(inward.val().volume(), source.val().volume())
        self.assertTrue(outward.val().is_watertight())
        self.assertTrue(inward.val().is_watertight())

    def test_to_pending_offsets_selected_face_edges(self):
        block = Workplane("XY", size=20, tolerance=0.05).box(4, 2, 0.5)
        selected = block.faces(">Z").edges().toPending()
        self.assertEqual(1, len(selected._pending))
        self.assertEqual(4, len(selected._pending[0].segments))
        holes = selected.offset2D(-0.25, forConstruction=True).vertices().cboreHole(
            0.125, 0.25, 0.125)
        self.assertTrue(holes.val().is_watertight())
        self.assertLess(holes.val().volume(), block.val().volume())

    def test_tagged_face_selection_survives_later_solid_features(self):
        prism = Workplane("XY", size=20, tolerance=0.05).polygon(3, 5).extrude(4).tag("prism")
        result = prism.sphere(10).faces("<X", tag="prism").workplane().circle(1).cutThruAll()
        self.assertTrue(result.val().is_watertight())
        self.assertLess(result.val().volume(), prism.sphere(10).val().volume())

    def test_split_keeps_requested_halves_on_tilted_workplane(self):
        body = Workplane("XY", size=20, tolerance=0.05).box(2, 2, 2)
        with self.assertRaises(ValueError):
            body.split()
        split = body.transformed(rotate=(0, 30, 0)).split(keepTop=True, keepBottom=True)
        top, bottom = split.vals()
        self.assertEqual(2, len(split.all()))
        self.assertIs(split.val(), top)
        self.assertIs(split._solid, body.val())
        self.assertTrue(top.is_watertight())
        self.assertTrue(bottom.is_watertight())
        self.assertAlmostEqual(top.volume() + bottom.volume(), body.val().volume(), delta=0.01)
        self.assertAlmostEqual(top.volume(), bottom.volume(), delta=0.01)
        self.assertIs(split.all()[1].val(), bottom)
        bottom_only = body.split(keepBottom=True)
        self.assertEqual(1, len(bottom_only.vals()))
        self.assertTrue(bottom_only.val().is_watertight())

    def test_split_uses_face_normal_and_live_mesh_bounds(self):
        body = Workplane("XY", size=20, tolerance=0.05).box(1, 1, 1).faces(">Z").workplane().hole(0.5)
        plane = body.faces(">Y").workplane(-0.5)
        self.assertAlmostEqual(plane._frame.origin.y, 0.0, delta=0.01)
        self.assertGreater(plane._frame.z.y, 0.0)
        halves = plane.split(keepTop=True, keepBottom=True).vals()
        self.assertEqual(2, len(halves))
        self.assertTrue(all(half.is_watertight() for half in halves))
        self.assertAlmostEqual(sum(half.volume() for half in halves), body.val().volume(), delta=0.01)

    def test_box_is_volume(self):
        result = Workplane("XY", size=20, tolerance=0.05).box(2, 2, 2)
        solid = result.val()
        self.assertTrue(solid.is_volume)
        self.assertGreater(solid.triangle_count, 0)
        top = result.part.plane_frame(solid.name + "-ExtrudeTop")
        self.assertIsNotNone(top)
        self.assertAlmostEqual(abs(top.z.z), 1.0, places=5)

    def test_circle_extrude_is_volume(self):
        result = Workplane("XY", size=20, tolerance=0.05).circle(1).extrude(2)
        self.assertTrue(result.val().is_volume)
        self.assertGreater(result.val().triangle_count, 0)

    def test_boss_on_top_face(self):
        box = Workplane("XY", size=20, tolerance=0.05).box(2, 2, 1)
        n0 = box.val().triangle_count
        boss = box.faces(">Z").circle(0.3).extrude(0.4)
        self.assertTrue(boss.val().is_volume)
        self.assertGreater(boss.val().triangle_count, n0)

    def test_through_hole_stays_a_volume(self):
        box = Workplane("XY", size=20, tolerance=0.05).box(3, 3, 1)
        n0 = box.val().triangle_count
        holed = box.faces(">Z").workplane().hole(0.6)
        self.assertTrue(holed.val().is_volume)
        self.assertGreater(holed.val().triangle_count, n0)

    def test_rect_cut_matches_explicit_boolean(self):
        wp = Workplane("XY", size=20, tolerance=0.05)
        block = wp.box(4, 4, 2)
        cutter = Workplane("XY", part=block.part, size=20, tolerance=0.05).circle(0.5).extrude(3, combine=False)
        cut = block - cutter
        self.assertTrue(cut.val().is_volume)
        self.assertGreater(cut.val().triangle_count, 0)

    def test_vertical_edge_fillet(self):
        box = Workplane("XY", size=20, tolerance=0.05).box(2, 2, 2)
        n0 = box.val().triangle_count
        blended = box.edges("|Z").fillet(0.15)
        self.assertTrue(blended.val().is_volume)
        self.assertGreater(blended.val().triangle_count, n0)

    def test_revolve_disk_is_volume(self):
        # Profile in XY; default revolve is around workplane Y.
        result = (
            Workplane("XY", size=20, tolerance=0.05)
            .moveTo(0.2, -0.5)
            .lineTo(1.0, -0.5)
            .lineTo(1.0, 0.5)
            .lineTo(0.2, 0.5)
            .close()
            .revolve(360)
        )
        self.assertTrue(result.val().is_volume)
        self.assertGreater(result.val().triangle_count, 0)

    def test_revolve_custom_axis_uses_workplane_coordinates(self):
        # Gallery #24: local axis (-20, 0) passes through world origin.
        result = (Workplane(origin=(20, 0, 0), tolerance=0.05).circle(2)
                  .revolve(180, (-20, 0, 0), (-20, -1, 0)))
        solid = result.val()
        points, _ = solid.mesh()
        self.assertTrue(solid.is_watertight())
        self.assertGreater(solid.volume(), 700)
        self.assertAlmostEqual(min(p.x for p in points), -22, delta=0.1)
        self.assertAlmostEqual(max(p.x for p in points), 22, delta=0.1)

    def test_loft_two_circles(self):
        result = (
            Workplane("XY", size=20, tolerance=0.05)
            .circle(1.0)
            .workplane(offset=3)
            .circle(0.4)
            .loft()
        )
        self.assertTrue(result.val().is_volume)
        self.assertGreater(result.val().triangle_count, 0)

    def test_loft_starts_from_selected_face_profile(self):
        result = (Workplane("XY", size=20, tolerance=0.05).box(4, 4, 0.25)
                  .faces(">Z").circle(1.5).workplane(offset=3)
                  .rect(0.75, 0.5).loft())
        self.assertTrue(result.val().is_watertight())
        self.assertGreater(result.val().volume(), 4.0)


if __name__ == "__main__":
    unittest.main()
