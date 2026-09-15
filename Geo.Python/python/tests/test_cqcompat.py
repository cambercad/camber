import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from camber.cqcompat import (
    Workplane,
    named_plane,
    _parse_selector as parse_selector,
    _radius_arc_mid as radius_arc_mid,
    _select_edges as select_edges,
    _select_faces as select_faces,
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
        with self.assertRaises(NotImplementedError):
            wp.shell(0.2)
        with self.assertRaises(NotImplementedError):
            wp.translate((1, 0, 0))
        with self.assertRaises(NotImplementedError):
            wp.vertices()


@unittest.skipUnless(_NATIVE_AVAILABLE, "requires the installed camber native module")
class NativeWorkplaneTests(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
