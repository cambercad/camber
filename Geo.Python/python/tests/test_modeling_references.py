"""Named face and point picks keep their modeling references across rebuilds."""

import ast
import unittest
from unittest.mock import patch

from camber import Part, Solid, Surface, set_progress_log
from camber.modeling_references import is_reference, selection_expression


class ReferenceBody(Solid):
    """Selection fixture, independent of a native library or renderer."""
    patch_names = ("body:top", "body:curved")
    point_names = ("body:edge@0.500",)

    def __init__(self):
        self.height = 4

    def face_frame(self, name, *, inward=False):
        if name == "body:curved":
            raise ValueError("Selected face does not define a plane")
        from camber import Frame
        return Frame(origin=(0, 0, self.height))

    def point(self, name):
        return (0, 0, self.height)


class SelectionExpressionTests(unittest.TestCase):
    def test_frame_and_point_retain_names_and_follow_changed_feature(self):
        body = ReferenceBody()
        namespace = {"body": body}
        point = selection_expression(namespace, body.point_names[0], "point")
        frame = selection_expression(namespace, body.patch_names[0], "frame", inward=True)
        self.assertEqual("body.point('body:edge@0.500')", point)
        self.assertEqual("body.face_frame('body:top', inward=True)", frame)
        self.assertTrue(is_reference(point))
        self.assertTrue(is_reference(frame))
        body.height = 11
        self.assertEqual((0, 0, 11), eval(point, {"__builtins__": {}}, namespace))
        self.assertEqual(11, eval(frame, {"__builtins__": {}}, namespace).origin.z)

    def test_named_plane_and_pattern_result_are_supported(self):
        body = ReferenceBody()
        available = {"pattern": [body]}
        self.assertEqual("pattern[0].point('body:edge@0.500')",
                         selection_expression(available, body.point_names[0], "point"))
        self.assertEqual("body:top", ast.literal_eval(
            selection_expression(available, body.patch_names[0], "plane")))

    def test_wrong_entity_type_and_later_feature_are_rejected(self):
        body = ReferenceBody()
        for available, name, kind in (({}, "body:top", "plane"),
                                      ({"body": body}, "body:top", "point"),
                                      ({"body": body}, "body:edge@0.500", "frame"),
                                      ({"body": body}, "body:curved", "frame")):
            with self.subTest(kind=kind, name=name), self.assertRaises(ValueError):
                selection_expression(available, name, kind)


class NativeReferenceTests(unittest.TestCase):
    def setUp(self):
        set_progress_log(False)
        self.part = Part((-20, -20, -20), (20, 20, 20), tolerance=.01)

    def test_frame_pickers_place_at_mouse_hit_and_internal_thread_tools_point_inward(self):
        from camber.modeling import _Workspace
        from camber.modeling_commands import field_value
        body = self.part.cuboid((0,0,0),(12,10,7),name="pose_block")
        top = next(n for n in body.patch_names if n.endswith("-ExtrudeTop"))
        for key,inward in (("Part.create_metric_thread_for_hole_negative",True),
                           ("Part.create_metric_thread_for_nut_negative",True),
                           ("Part.create_metric_thread_for_bolt_negative",False),
                           ("Part.cylinder_revolve",False)):
            with self.subTest(operation=key):
                workspace = _Workspace.__new__(_Workspace)
                workspace.picked = [top]
                workspace.fields = {"pose":""}
                workspace._pick_reference = (key,"pose","frame")
                self.assertTrue(workspace._reference_from_selection("pose","frame",{"body":body},
                                                                     ray=((5,4,20),(0,0,-1))))
                frame = field_value(ast.parse(workspace.fields["pose"],mode="eval").body,{"body":body})
                for actual,expected in zip(frame.origin,(5,4,7)):
                    self.assertAlmostEqual(actual,expected,delta=self.part.smallest_unit())
                self.assertAlmostEqual(frame.z.z,-1 if inward else 1)

    def test_named_face_sketch_and_inward_frame_use_current_geometry(self):
        body = self.part.cuboid((0, 0, 0), (2, 3, 7), name="reference_block")
        name = next(name for name in body.patch_names if name.endswith("-ExtrudeTop"))
        frame = body.face_frame(name)
        inward = body.face_frame(name, inward=True)
        self.assertAlmostEqual(7, frame.origin.z, delta=self.part.smallest_unit())
        self.assertEqual(frame.x, inward.x)
        self.assertEqual(-frame.y, inward.y)
        self.assertEqual(-frame.z, inward.z)
        sketch = self.part.sketch(plane=name, constrained=True, name="on_face")
        self.assertAlmostEqual(7, sketch.frame.origin.z, delta=self.part.smallest_unit())
        self.assertAlmostEqual(1, sketch.frame.z.z)

    def test_clicked_hole_placement_retains_face_and_local_coordinates(self):
        from camber import Frame
        from camber.modeling_commands import operations, field_value
        body = self.part.cuboid((0, 0, 0), (12, 10, 7), name="hole_block")
        name = next(n for n in body.patch_names if n.endswith("-ExtrudeTop"))
        source = selection_expression({"body": body}, name, "frame", inward=True,
                                      ray=((5, 4, 20), (0, 0, -1)))
        self.assertTrue(is_reference(source))
        frame = field_value(ast.parse(source, mode="eval").body, {"body": body})
        for actual, expected in zip(frame.origin, (5, 4, 7)):
            self.assertAlmostEqual(expected, actual, delta=self.part.smallest_unit())
        self.assertAlmostEqual(-1, frame.z.z)
        self.assertEqual({}, operations()["Solid.hole"].field_errors(
            {"mouth": source, "diameter": "2", "depth": "None",
             "name": "None", "max_deviation": "-1"}, {"body": body}))
        # Changing the supporting face preserves its local placement.
        raised = Frame(origin=(0, 0, 11), x=frame.x, y=frame.y, z=frame.z)
        with patch.object(body, "face_frame", return_value=raised):
            updated = field_value(ast.parse(source, mode="eval").body, {"body": body})
        self.assertAlmostEqual(11, updated.origin.z)
        for direction in ((1, 0, 0), (0, 0, 1)):
            with self.subTest(direction=direction), self.assertRaises(ValueError):
                selection_expression({"body": body}, name, "frame",
                                     ray=((5, 4, 20), direction))

    def test_named_points_match_the_renderer_and_missing_points_fail(self):
        from camber.display import decode_native_solid
        body = self.part.cuboid((0, 0, 0), (2, 3, 4), name="anchor_block")
        displayed = decode_native_solid(body._n).points
        self.assertTrue(displayed)
        for point in displayed:
            actual = body.point(point["name"])
            for value, expected in zip(actual, point["position"]):
                self.assertAlmostEqual(expected, value)
        with self.assertRaises(Exception):
            body.point("missing@0.500")

    def test_open_surfaces_share_references_and_curved_faces_are_rejected(self):
        sheet = self.part.solid_from_mesh([(0, 0, 0), (4, 0, 0), (0, 4, 0)],
                                          [(0, 1, 2)], name="reference_sheet")
        self.assertIsInstance(sheet, Surface)
        self.assertAlmostEqual(1, sheet.face_frame(sheet.patch_names[0]).z.z)
        cylinder = self.part.cylinder((0, 0, 0), 2, 3, name="reference_round")
        lateral = next(name for name in cylinder.patch_names if name.endswith("-Circle1"))
        with self.assertRaises(Exception):
            cylinder.face_frame(lateral)


if __name__ == "__main__":
    unittest.main()
