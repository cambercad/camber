"""Source round-trips and rebuild behavior for the visual modeling workspace."""

import os
import sys
import tempfile
import unittest
import textwrap
from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from camber.feature_source import ModelDocument
from camber.modeling import (ModelSession, _TEMPLATE, _parameter_form,
                             _parameter_source, _sketch_for_edit, _feature_groups)
from camber.modeling_commands import discover_operations, operations


class ViewportFitTests(unittest.TestCase):
    def test_f_over_the_viewport_fits_the_model_once(self):
        from pyglet.window import key
        from camber.modeling import _Workspace

        workspace = _Workspace.__new__(_Workspace)
        workspace._fit_key_down = False
        workspace.sketch_session = None
        fitted = []
        workspace.viewer = SimpleNamespace(
            _solid={"packed": {"bounds": ((0, 0, 0), (2, 2, 2))}},
            camera=SimpleNamespace(fit=lambda *bounds: fitted.append(bounds)))
        io = SimpleNamespace(MousePos=(50, 40), KeysDown={key.F: True}, KeyCtrl=False, KeyAlt=False)
        workspace.ui = SimpleNamespace(GetIO=lambda: io, viewport_rect=(10, 10, 200, 150))
        workspace._maybe_fit_view()
        workspace._maybe_fit_view()
        self.assertEqual([((0, 0, 0), (2, 2, 2))], fitted)
        io.MousePos = (1, 1)
        io.KeysDown = {}
        workspace._maybe_fit_view()
        io.KeysDown = {key.F: True}
        workspace._maybe_fit_view()
        self.assertEqual([((0, 0, 0), (2, 2, 2))], fitted)

    def test_f_over_the_viewport_fits_the_open_sketch(self):
        from pyglet.window import key
        from camber.modeling import _Workspace

        workspace = _Workspace.__new__(_Workspace)
        workspace._fit_key_down = False
        fitted = []
        workspace.sketch_session = {"fit": lambda: fitted.append("sketch")}
        workspace.viewer = SimpleNamespace(_solid=None, camera=SimpleNamespace(fit=lambda *bounds: fitted.append("model")))
        io = SimpleNamespace(MousePos=(20, 20), KeysDown={key.F: True}, KeyCtrl=False, KeyAlt=False)
        workspace.ui = SimpleNamespace(GetIO=lambda: io, viewport_rect=(0, 0, 100, 80))
        workspace._maybe_fit_view()
        self.assertEqual(["sketch"], fitted)


class SeparateBodyTests(unittest.TestCase):
    def test_uncombined_bodies_are_shown_together_and_clicks_fill_subtract(self):
        from camber.modeling import (
            _Workspace, _combined_body_scene, _visible_body_entries,
        )
        from camber.modeling_commands import operations
        from camber import glview

        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as folder:
            session = ModelSession(Path(folder) / "two_bodies.py")
            source = session.source.replace(
                "    return part, part",
                "    # camber:begin block\n"
                "    block = part.cuboid((-20, -10, 0), (0, 10, 12), name=\"block\")\n"
                "    # camber:end block\n\n"
                "    # camber:begin peg\n"
                "    peg = part.cylinder((8, 0, 0), 4, 18, name=\"peg\")\n"
                "    # camber:end peg\n\n"
                "    return part, peg\n")
            self.assertTrue(session.rebuild(source), session.error)
            self.assertEqual(["block", "peg"], [name for name, _ in _visible_body_entries(session)])
            scene, owners = _combined_body_scene(session)
            packed = glview.pack_scene(scene)
            shown = set(packed["face_names"])
            self.assertTrue(any(name.startswith("block:") for name in shown))
            self.assertTrue(any(name.startswith("peg:") for name in shown))
            self.assertIn("block", {owners[name] for name in shown})
            self.assertIn("peg", {owners[name] for name in shown})

            workspace = _Workspace.__new__(_Workspace)
            workspace.pending = operations()["Part.subtract"]
            workspace.fields = workspace.pending.defaults()
            workspace._pick_reference = None
            workspace._pick_datum = None
            workspace.model = session
            self.assertTrue(workspace._assign_clicked_body("block"))
            self.assertEqual("block", workspace.fields["a"])
            self.assertTrue(workspace._assign_clicked_body("peg"))
            self.assertEqual("peg", workspace.fields["b"])

            combined = source.replace("    return part, peg\n",
                                      "    # camber:begin cut\n"
                                      "    cut = part.subtract(block, peg, name=\"cut\")\n"
                                      "    # camber:end cut\n\n"
                                      "    return part, cut\n")
            self.assertTrue(session.rebuild(combined), session.error)
            self.assertEqual(["cut"], [name for name, _ in _visible_body_entries(session)])
            self.assertIsNone(_combined_body_scene(session))


class OperationFormTests(unittest.TestCase):
    def test_edge_picker_accepts_points_on_its_input_edge_without_accepting_other_bodies(self):
        from camber.modeling import _Workspace
        workspace = _Workspace.__new__(_Workspace)
        workspace.receiver = "body"
        workspace.fields = {"edges":"()"}
        workspace.picked = ["body:[top,side]@0.500"]
        available = {"body":SimpleNamespace(curve_names=("body:[top,side]",),patch_names=("body:top",))}
        self.assertTrue(workspace._reference_from_selection("edges","edges",available))
        self.assertEqual("('body:[top,side]',)",workspace.fields["edges"])
        workspace.picked = ["other:[top,side]@0.500"]
        self.assertFalse(workspace._reference_from_selection("edges","edges",available))
        self.assertEqual("('body:[top,side]',)",workspace.fields["edges"])

    def test_section_inspection_retains_solid_final_result(self):
        from camber.api import Part, Solid
        from camber.modeling import _Workspace
        source = ModelDocument(_TEMPLATE).insert("body", "    body = part.cylinder((0,0,0),10,5)\n")
        source = ModelDocument(source).display_result("body")
        workspace = _Workspace.__new__(_Workspace)
        workspace.model = SimpleNamespace(document=ModelDocument(source),selected="body",displayed=object.__new__(Solid))
        workspace.pending = operations()["Part.section_sketch"]
        workspace.result_name = "inspection"
        workspace.receiver = "part"
        workspace.fields = workspace.pending.defaults()
        workspace.fields["solid"] = "body"
        workspace._available_values = lambda before=None: dict(part=object.__new__(Part),body=workspace.model.displayed)
        workspace._form_errors = lambda *args: {}
        candidate = workspace._candidate_source()
        self.assertIn("return part, body",candidate)
        self.assertIn("inspection = part.section_sketch",candidate)
        workspace.model.displayed = None
        self.assertIn("return part, inspection",workspace._candidate_source())

    def test_point_path_editor_retains_incomplete_coordinates_and_validation_blocks_build(self):
        from camber.modeling_fields import PointListFields
        draft = PointListFields("[(0,0),(20,0)]",2)
        ui = Mock()
        ui.GetContentRegionAvail.return_value = (300,500)
        ui.Button.return_value = False
        ui.InputText.side_effect = lambda label,value,**kwargs: (label.endswith("/1##X"), "-" if label.endswith("/1##X") else value)
        source = draft.draw(ui,"path")
        self.assertEqual("[(0, 0), (-, 0)]",source)
        operation = operations()["Solid.rib"]
        fields = operation.defaults()
        fields.update(path=source,thickness="2",to="10")
        self.assertIn("path",operation.field_errors(fields,{}))
        for path in ("[(0,0)]","[(0,0),(1,2,3)]","[(0,0),('x',1)]"):
            fields["path"] = path
            self.assertIn("path",operation.field_errors(fields,{}))
        fields["path"] = "[(0,0),(20,0)]"
        self.assertNotIn("path",operation.field_errors(fields,{}))

    def test_countersink_angle_and_all_named_angles_use_degree_editors(self):
        import math
        from camber.modeling_commands import is_angle_parameter
        operation = operations()["Solid.countersink_hole"]
        fields = operation.defaults()
        for angle in ("0",repr(math.pi),"90"):
            fields["included_angle"] = angle
            self.assertIn("included_angle",operation.field_errors(fields,{}))
        fields["included_angle"] = repr(math.pi/2)
        self.assertNotIn("included_angle",operation.field_errors(fields,{}))
        for key,names in (("Curve3D.arc",("start_angle","sweep_angle")),("Sketch.extrude",("twist","taper_angle")),("Solid.countersink_hole",("included_angle",))):
            for parameter in operations()[key].parameters:
                if parameter.name in names:
                    self.assertTrue(is_angle_parameter(parameter))

    def test_omitted_twist_center_still_calls_an_older_extrude(self):
        from camber.api import _call_optional_center

        def legacy(*args):
            if len(args) != 2:
                raise TypeError("legacy() takes 2 positional arguments but {} were given".format(len(args)))
            return args

        def current(*args):
            return args

        self.assertEqual(("sketch", 4), _call_optional_center(legacy, ("sketch", 4), None))
        self.assertEqual(("sketch", 4, float("nan"), float("nan"))[0:2],
                         _call_optional_center(current, ("sketch", 4), None)[0:2])
        with self.assertRaisesRegex(TypeError, "sketch point"):
            _call_optional_center(legacy, ("sketch", 4), (1, 0))

    def test_extrude_twist_center_is_optional_sketch_point(self):
        operation = operations()["Sketch.extrude"]
        fields = operation.defaults()
        fields["height"] = "4"
        self.assertEqual("None", fields["twist_center"])
        self.assertNotIn("twist_center", operation.field_errors(fields, {}))
        source = operation.source("profile", "body", fields, {})
        self.assertIn("twist_center=None", source)
        fields["twist_center"] = "(1, 0)"
        self.assertNotIn("twist_center", operation.field_errors(fields, {}))
        source = operation.source("profile", "body", fields, {})
        self.assertIn("twist_center=(1, 0)", source)
        fields["twist_center"] = "(1,)"
        self.assertIn("twist_center", operation.field_errors(fields, {}))

    def test_loft_alignment_uses_existing_kernel_setting_and_rejects_invalid_choice(self):
        from camber import LoftOptions
        options = LoftOptions(alignment="minimum_twist")
        self.assertEqual("minimum_twist", options.alignment)
        for value in ("as_authored", "origin_foot"):
            options.alignment = value
            self.assertEqual(value, options.alignment)
        with self.assertRaises(ValueError):
            options.alignment = "unknown"

    def test_loft_settings_editor_serializes_selected_choices(self):
        from camber.modeling_fields import loft_settings
        ui = Mock()
        ui.Checkbox.return_value = (True, True)
        ui.GetContentRegionAvail.return_value = (250, 500)
        ui.ComboBox.side_effect = lambda identity, current, choices, **kwargs: (
            True, "Minimize twist" if identity.endswith("/alignment") else current)
        self.assertEqual("LoftOptions(alignment='minimum_twist')", loft_settings(ui, "options", "None"))
        ui.Checkbox.return_value = (True, False)
        self.assertEqual("None", loft_settings(ui, "options", "LoftOptions(alignment='minimum_twist')"))

    def test_ordered_profile_editor_adds_reorders_and_removes_without_python_input(self):
        from camber.modeling_fields import reference_sequence
        ui = Mock()
        ui.Button.return_value = False
        ui.Checkbox.side_effect = lambda label, checked: (label.startswith("second##"), True if label.startswith("second##") else checked)
        self.assertEqual("[first, second]", reference_sequence(ui, "profiles", "[first]", ["first", "second"]))
        ui.Checkbox.side_effect = lambda label, checked: (False, checked)
        ui.Button.side_effect = lambda label, **kwargs: label == "Up##profiles/second"
        self.assertEqual("[second, first]", reference_sequence(ui, "profiles", "[first, second]", ["first", "second"]))
        ui.Button.side_effect = None
        ui.Button.return_value = False
        ui.Checkbox.side_effect = lambda label, checked: (label.startswith("first##"), False if label.startswith("first##") else checked)
        self.assertEqual("[second]", reference_sequence(ui, "profiles", "[second, first]", ["first", "second"]))
        with self.assertRaises(ValueError):
            reference_sequence(ui, "profiles", "[unknown]", ["first", "second"])
        ui.Checkbox.side_effect = lambda label, checked: (False, checked)
        self.assertEqual("[copies[0], copies[2]]", reference_sequence(
            ui,"solids","[copies[0], copies[2]]",["copies[0]","copies[1]","copies[2]"]))
        self.assertEqual("[first, *copies]", reference_sequence(
            ui,"solids","[first, *copies]",["first","*copies"]))

    def test_whole_pattern_references_remain_dynamic_and_restricted(self):
        import ast
        from camber.api import Solid
        from camber.modeling_commands import simple_value, field_value
        source = "[base, *copies]"
        node = ast.parse(source,mode="eval").body
        base = object.__new__(Solid)
        copies = [object.__new__(Solid),object.__new__(Solid)]
        available = dict(base=base,copies=copies)
        self.assertTrue(simple_value(node))
        self.assertEqual([base,*copies],field_value(node,available))
        operation = operations()["Part.batch_union"]
        fields = operation.defaults()
        fields["meshes"] = source
        self.assertNotIn("meshes",operation.field_errors(fields,available))
        copies.append(object.__new__(Solid))
        self.assertEqual(4,len(field_value(node,available)))
        for unsafe in ("[*function()]","[*copies[0]]","{*copies}"):
            self.assertFalse(simple_value(ast.parse(unsafe,mode="eval").body))
        fields["meshes"] = "[*unknown]"
        self.assertIn("meshes",operation.field_errors(fields,available))

    def test_loft_profiles_require_two_sketches_before_execution(self):
        from camber.api import Sketch, Solid
        operation = operations()["Part.loft"]
        fields = operation.defaults()
        available = {"first": object.__new__(Sketch), "second": object.__new__(Sketch),
                     "body": object.__new__(Solid)}
        for value in ("[]", "[first]", "[first, body]", "first"):
            fields["sketches"] = value
            self.assertIn("sketches", operation.field_errors(fields, available), value)
        fields["sketches"] = "[second, first]"
        self.assertNotIn("sketches", operation.field_errors(fields, available))

    def test_loft_guide_and_end_controls_validate_independently(self):
        from camber.api import Sketch, Curve3D
        operation = operations()["Part.loft"]
        fields = operation.defaults()
        available = {"first": object.__new__(Sketch), "second": object.__new__(Sketch),
                     "rail": object.__new__(Curve3D)}
        fields.update(sketches="[first, second]", guides="[rail]")
        self.assertEqual({}, operation.field_errors(fields, available))
        fields["guides"] = "[]"
        self.assertEqual({}, operation.field_errors(fields, available))
        fields["start_tangent"] = "(0, 0, 0)"
        self.assertIn("start_tangent", operation.field_errors(fields, available))
        fields["start_tangent"] = "(0, 0, 20)"
        fields["start_closure"] = "'rounded'"
        self.assertIn("start_closure", operation.field_errors(fields, available))
        fields["start_point"] = "(0, 0, -5)"
        self.assertEqual({}, operation.field_errors(fields, available))

    def test_edge_selections_and_dressup_sizes_are_validated_before_building(self):
        for key, dimension in (("Solid.shell", "thickness"), ("Solid.fillet", "radius"),
                               ("Solid.chamfer", "distance")):
            operation = operations()[key]
            fields = operation.defaults()
            fields[dimension] = "0"
            if "edges" in fields:
                fields["edges"] = "()"
            errors = operation.field_errors(fields, {})
            self.assertIn(dimension, errors)
            if "edges" in fields:
                self.assertIn("edges", errors)
                fields["edges"] = "(123,)"
                self.assertIn("edges", operation.field_errors(fields, {}))

    def test_angle_editor_shows_degrees_and_reports_radians(self):
        import math
        from camber.modeling_fields import AngleField
        draft = AngleField(repr(math.tau), {})
        ui = Mock()
        ui.InputDouble.return_value = (False, math.tau)
        self.assertAlmostEqual(math.tau, float(draft.draw(ui, "Angle")))
        _args, kwargs = ui.InputDouble.call_args
        self.assertAlmostEqual(math.tau, _args[1])
        self.assertAlmostEqual(math.pi / 180, kwargs["multiplier"])
        for degrees in (180, -90, 0, 360):
            ui.InputDouble.return_value = (True, math.radians(degrees))
            self.assertAlmostEqual(math.radians(degrees), float(draft.draw(ui, "Angle")))

    def test_orientation_editor_preserves_rotations_including_pitch_singularities(self):
        import ast
        import math
        from camber.modeling_fields import OrientationFields
        for quaternion in ((0, 0, 0, 1), (.2, .3, .4, .5),
                           (0, math.sqrt(.5), 0, math.sqrt(.5)), (.5, -.5, .5, .5),
                           (2e300, 3e300, 4e300, 5e300)):
            with self.subTest(quaternion=quaternion):
                draft = OrientationFields(repr(quaternion), {})
                ui = Mock()
                ui.InputVector3.return_value = (True, draft.values)
                result = ast.literal_eval(draft.draw(ui, "Orientation"))
                scale = max(abs(v) for v in quaternion)
                normalized = [v/scale for v in quaternion]
                length = math.sqrt(sum(v*v for v in normalized))
                self.assertAlmostEqual(1, abs(sum(a*b/length for a,b in zip(result, normalized))), places=10)
        for value in ("-", "", "inf"):
            ui.InputVector3.return_value = (True, [value, "0", "0"])
            previous = draft.source
            self.assertEqual(previous, draft.draw(ui, "Orientation"))
            self.assertTrue(draft.error())
        with self.assertRaisesRegex(ValueError, "cannot be zero"):
            OrientationFields("(0, 0, 0, 0)", {})

    def test_fillet_icon_has_continuous_tangent_edges(self):
        from camber.modeling_icons import fillet
        import math
        drawing = Mock(spec=["AddLine"])
        drawing.scale = 1
        fillet(drawing, 0, 0, "ink", "accent")
        segments = [(call.args[0], call.args[1]) for call in drawing.AddLine.call_args_list]
        for (start, end), (next_start, next_end) in zip(segments, segments[1:]):
            self.assertLess(math.dist(end, next_start), 1e-9)
            direction = (end[0]-start[0], end[1]-start[1])
            next_direction = (next_end[0]-next_start[0], next_end[1]-next_start[1])
            cosine = sum(a*b for a,b in zip(direction,next_direction)) / (
                math.hypot(*direction)*math.hypot(*next_direction))
            self.assertGreater(cosine, .99, "Rounded edge must not reverse at its tangent")

    def test_frame_or_point_parameters_accept_both_and_reject_bad_coordinates(self):
        operation = operations()["Part.cuboid"]
        values = operation.defaults()
        values.update(pose_or_min="(0, 0, 0)",extents_or_max="(10, 20, 30)")
        self.assertEqual({},operation.field_errors(values,{}))
        values["pose_or_min"] = "Frame()"
        self.assertEqual({},operation.field_errors(values,{}))
        values["pose_or_min"] = "(0, 0, 0, 0)"
        self.assertIn("pose_or_min",operation.field_errors(values,{}))

    def test_frame_editor_preserves_pose_and_incomplete_edits(self):
        from camber.modeling_fields import FrameFields
        from camber.modeling_commands import field_value
        import ast
        draft = FrameFields("Frame(origin=(0, 0, 32), y=(0, -1, 0), z=(0, 0, -1))", {})
        self.assertEqual({}, draft.errors())
        original = draft.source
        self.assertEqual(original, draft.source)
        draft.rows["origin"][0] = "12.5"
        frame = field_value(ast.parse(draft.serialize(), mode="eval").body, {})
        self.assertEqual(12.5, frame.origin.x)
        self.assertEqual(-1, frame.z.z)
        draft.rows["origin"][0] = "-"
        self.assertIn("origin", draft.errors())
        draft.serialize()
        self.assertEqual("-", draft.rows["origin"][0])

    def test_frame_offset_edits_keep_the_attachment_after_reference_changes(self):
        import ast
        from types import SimpleNamespace
        from camber import Frame
        from camber.modeling_fields import FrameFields
        from camber.modeling_commands import field_value
        available = {"attachment": Frame(origin=(10, 20, 30), x=(0, 1, 0), y=(-1, 0, 0))}
        available["body"] = SimpleNamespace(face_frame=lambda name, inward: available["attachment"])
        draft = FrameFields("body.face_frame('cap', inward=True).to_global(Frame(origin=(2, 3, 0)))", available)
        self.assertEqual(["2.0", "3.0", "0.0"], draft.rows["origin"])
        draft.rows["origin"][0] = "5"
        source = draft.serialize()
        self.assertIn("body.face_frame('cap', inward=True).to_global(", source)
        self.assertEqual({}, draft.errors())
        frame = field_value(ast.parse(source, mode="eval").body, available)
        self.assertEqual((7, 25, 30), (frame.origin.x, frame.origin.y, frame.origin.z))
        available["attachment"] = Frame(origin=(100, 200, 300), x=(0, 1, 0), y=(-1, 0, 0))
        frame = field_value(ast.parse(source, mode="eval").body, available)
        self.assertEqual((97, 205, 300), (frame.origin.x, frame.origin.y, frame.origin.z))

    def test_parameter_errors_identify_bad_frame_and_numeric_fields(self):
        operation = operations()["Solid.counterbore_hole"]
        values = operation.defaults()
        values.update(mouth="Frame(x=(0, 0, 0))", diameter="'large'", counter_diameter="26", counter_depth="5")
        errors = operation.field_errors(values, {})
        self.assertIn("unit length", errors["mouth"])
        self.assertIn("diameter", errors)
        self.assertNotIn("counter_diameter", errors)
        self.assertNotIn("name", errors)
        values.update(mouth="Frame(y=(0, -1, 0))", diameter="20")
        self.assertIn("right-handed", operation.field_errors(values, {})["mouth"])
        values["mouth"] = "Frame(origin=(1e309, 0, 0))"
        self.assertIn("finite", operation.field_errors(values, {})["mouth"])

    def test_invalid_new_feature_does_not_generate_or_build_source(self):
        from camber.modeling import _Workspace
        workspace = object.__new__(_Workspace)
        workspace.model = SimpleNamespace(document=ModelDocument(_TEMPLATE), values={})
        workspace.pending = operations()["Frame"]
        workspace.vector_fields = {}
        workspace.result_name = "bad name"
        workspace.fields = workspace.pending.defaults()
        workspace.receiver = ""
        with patch.object(workspace, "_available_values", return_value={}), patch.object(workspace, "_apply") as apply:
            workspace._new_feature()
        apply.assert_not_called()
        self.assertIn("Python name", workspace.status)

    def test_discovers_modeling_methods_and_excludes_viewer_and_data_queries(self):
        catalog = discover_operations()
        for key in ("Solid.hole", "Solid.pattern_circular", "Solid.trim", "Surface.split",
                    "Surface.thicken", "Part.load_obj", "Part.loft_surface",
                    "Sketch.add_bspline", "Sketch.tangent", "Sketch.sweep_surface",
                    "Curve3D.helix", "Frame.offset"):
            self.assertIn(key, catalog)
        for key in ("Solid.show", "Sketch.show", "Part.sketch_interactive", "Solid.volume", "Sketch.polylines"):
            self.assertNotIn(key, catalog)

    def test_new_annotated_method_gets_a_form_without_ui_registration(self):
        from camber.api import Solid
        def emboss(self, depth: float, *, invert: bool = False) -> "Solid":
            return self
        with patch.object(Solid, "emboss", emboss, create=True):
            operation = discover_operations()["Solid.emboss"]
            self.assertEqual({"depth": "", "invert": "False"}, operation.defaults())
            self.assertEqual("    raised = base.emboss(depth=2, invert=True)\n",
                             operation.source("base", "raised", {"depth": "2", "invert": "True"}))

    def test_optional_arguments_and_enums_round_trip(self):
        source = ModelDocument(_TEMPLATE).insert(
            "cylinder", '    cylinder = part.cylinder((0, 0, 0), 5, 10, name="cylinder")\n')
        block = ModelDocument(source).block("cylinder")
        fields = dict(_parameter_form(block))
        self.assertEqual("z", fields["axis"])
        fields["axis"] = "x"
        generated = _parameter_source(block, fields)
        self.assertIn("axis='x'", generated)
        fields["axis"] = "invalid"
        with self.assertRaises((ValueError, SyntaxError)):
            _parameter_source(block, fields)

    def test_changing_the_input_feature_updates_the_generated_receiver(self):
        source = ModelDocument(_TEMPLATE).insert("first", '    first = part.sketch("xy")\n')
        source = ModelDocument(source).insert("second", '    second = part.sketch("xz")\n')
        source = ModelDocument(source).insert("pad", '    pad = first.extrude(8)\n')
        block = ModelDocument(source).block("pad")
        generated = _parameter_source(block, dict(_parameter_form(block)), ("first", "second"), receiver="second")
        self.assertIn("second.extrude", generated)

    def test_invalid_and_required_fields_fail_before_execution(self):
        operation = operations()["Solid.pattern_linear"]
        for count in ("", "2.5", "side_effect()"):
            with self.subTest(count=count), self.assertRaises((ValueError, SyntaxError)):
                operation.source("base", "pattern", {"count": count, "step": "(10, 0, 0)"})
        with self.assertRaisesRegex(ValueError, "identifier"):
            operation.source("base", "for", {"count": "2", "step": "(10, 0, 0)"})

    def test_frame_options_and_indexed_references_are_safe_inputs(self):
        operation = operations()["Part.loft"]
        values = {"sketches": "[profiles[0], second]", "options": "LoftOptions(style='ruled')"}
        generated = operation.source("part", "loft1", values, ("profiles", "second"))
        self.assertIn("profiles[0]", generated)
        self.assertIn("LoftOptions", generated)
        values["options"] = "__import__('os')"
        with self.assertRaises(ValueError):
            operation.source("part", "loft1", values, ("profiles", "second"))


class FeatureSourceTests(unittest.TestCase):
    def test_consumed_sketch_is_nested_once_and_shared_profile_stays_available(self):
        source = ModelDocument(_TEMPLATE).insert("profile", '    profile = part.sketch("xy")\n')
        source = ModelDocument(source).insert("first", '    first = profile.extrude(5)\n')
        source = ModelDocument(source).insert("second", '    second = profile.extrude(10)\n')
        document = ModelDocument(source)
        groups = _feature_groups(document)
        self.assertEqual(["setup", "first", "second"], [b.id for b, _ in groups])
        self.assertEqual(["profile"], [b.id for b in groups[1][1]])
        self.assertEqual([], groups[2][1])
        self.assertIn("profile", document.block("second").dependencies)

    def test_round_trip_preserves_unrelated_source(self):
        source = "# a user's header\n" + _TEMPLATE + "\n# a user's footer\n"
        doc = ModelDocument(source)
        changed = doc.replace("setup", "    part = Part((-8, -8, -8), (8, 8, 8))\n")
        self.assertTrue(changed.startswith("# a user's header\n"))
        self.assertTrue(changed.endswith("\n# a user's footer\n"))
        self.assertEqual(ModelDocument(changed).block("setup").kind, "Setup")

    def test_rejects_malformed_markers(self):
        for candidate in (_TEMPLATE.replace("camber:end setup", "camber:end wrong"),
                          _TEMPLATE.replace("camber:end setup", "camber:begin other"),
                          _TEMPLATE.replace("return part, part", "# camber:begin setup\n    return part, part")):
            with self.subTest(candidate=candidate):
                with self.assertRaises(ValueError):
                    ModelDocument(candidate)

    def test_comment_and_expression_make_block_code_only(self):
        commented = _TEMPLATE.replace("    part = Part", "    # keep this comment\n    part = Part")
        self.assertFalse(ModelDocument(commented).block("setup").editable)
        expression = _TEMPLATE.replace("tolerance=0.01", "tolerance=scale * 0.01")
        self.assertFalse(ModelDocument(expression).block("setup").editable)

    def test_dependencies_block_delete(self):
        source = ModelDocument(_TEMPLATE).insert(
            "profile", '    profile = part.sketch("xy", name="profile")\n'
                       '    profile.add_rectangle((-5, -5), (5, 5))\n')
        source = ModelDocument(source).insert(
            "pad", '    pad = profile.extrude(8, name="pad")\n')
        doc = ModelDocument(source)
        self.assertEqual(doc.dependents("profile"), ("pad",))
        with self.assertRaisesRegex(ValueError, "pad"):
            doc.delete("profile")

    def test_form_generation(self):
        block = ModelDocument(_TEMPLATE).block("setup")
        fields = dict(_parameter_form(block))
        fields["tolerance"] = "0.02"
        self.assertIn("tolerance=0.02", _parameter_source(block, fields))
        fields["tolerance"] = "side_effect()"
        with self.assertRaises(ValueError):
            _parameter_source(block, fields)


class ModelSessionTests(unittest.TestCase):
    def test_untouched_new_document_is_clean_without_weakening_save_protection(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as folder:
            path = Path(folder) / "new.py"
            session = ModelSession(path)
            self.assertTrue(session.rebuild(), session.error)
            self.assertFalse(session.dirty)
            changed = session.document.replace("setup", "    part = Part((-20, -20, -20), (20, 20, 20))\n")
            self.assertTrue(session.rebuild(changed), session.error)
            self.assertTrue(session.dirty)
            self.assertTrue(session.undo(), session.error)
            self.assertFalse(session.dirty)
            path.write_text("# created elsewhere\n", encoding="utf-8")
            with self.assertRaisesRegex(FileExistsError, "created elsewhere"):
                session.save()

    def test_housing_example_edits_bearing_seat_and_rebuilds_downstream_features(self):
        path = Path(__file__).parents[1] / "examples/modeling/bearing_housing.py"
        session = ModelSession(path)
        self.assertTrue(session.rebuild(), session.error)
        self.assertTrue(all(b.editable for b in session.document.blocks))
        before = session.displayed.volume()
        block = session.document.block("bearing_seat")
        values = dict(_parameter_form(block))
        values["diameter"] = "22"
        edited = _parameter_source(block, values, session.values)
        self.assertTrue(session.rebuild(session.document.replace(block.id, edited)), session.error)
        self.assertTrue(session.displayed.is_watertight())
        self.assertLess(session.displayed.volume(), before)
        self.assertIs(session.displayed, session.values["oil_port"])
        self.assertNotEqual(session.source, path.read_text(encoding="utf-8"))

    def test_conditional_return_captures_features_without_changing_nested_returns(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as folder:
            session = ModelSession(Path(folder) / "conditional.py")
            source = session.source.replace(
                "    return part, part",
                "    def enabled():\n        return True\n"
                "    if enabled():\n        return part, part\n"
                "    return part, part")
            self.assertTrue(session.rebuild(source), session.error)
            self.assertIs(session.part, session.values["part"])

    def test_face_pick_keeps_named_reference_for_rebuild(self):
        from camber.modeling import _Workspace, _pack_result
        from camber import Part
        part = Part((-10, -10, -10), (10, 10, 10))
        body = part.cuboid((-3, -3, 0), (3, 3, 2), name="base")
        preview = _pack_result(part, body)
        face = next(name for name, kind in preview["entity_types"].items() if kind == "pln")
        workspace = _Workspace.__new__(_Workspace)
        workspace.viewer = SimpleNamespace(_solid={"packed": preview})
        workspace.model = SimpleNamespace(packed={"entity_types": {}})
        workspace.picked, workspace.fields = [face], {}
        workspace._reference_from_selection("mouth", "frame", {"base": body})
        self.assertEqual(f"base.face_frame({face!r}, inward=True)", workspace.fields["mouth"])

    def test_named_picks_validate_type_and_retain_previous_values(self):
        from camber.modeling import _Workspace
        from camber import Part
        part = Part((-10,-10,-10),(10,10,10))
        body = part.cylinder((0,0,0),2,4,name="ref")
        w = _Workspace.__new__(_Workspace)
        w.fields = {"plane":"xy", "frame":"Frame()", "origin":"(0,0,0)"}
        w._pick_reference = ("Part.sketch","plane","plane")
        w.picked = [next(n for n in body.patch_names if n.endswith("-Circle1"))]
        self.assertFalse(w._reference_from_selection("plane","plane",{"body":body}))
        self.assertEqual("xy",w.fields["plane"])
        self.assertIsNotNone(w._pick_reference)
        w.picked = [next(n for n in body.patch_names if n.endswith("-ExtrudeTop"))]
        self.assertTrue(w._reference_from_selection("plane","plane",{"body":body}))
        self.assertEqual(w.picked[0],w.fields["plane"])
        self.assertEqual("None",w.fields["frame"])
        self.assertIsNone(w._pick_reference)
        w.picked = [body.point_names[0]]
        self.assertTrue(w._reference_from_selection("origin","point",{"body":body}))
        self.assertEqual(f"body.point({w.picked[0]!r})",w.fields["origin"])
        op = operations()["Part.cylinder"]
        values = op.defaults()
        values.update(origin="body.point('missing@0.500')",radius="1",height="2")
        self.assertIn("origin",op.field_errors(values,{"body":body}))

    def _operation(self, source, key, receiver, result, **values):
        operation = operations()[key]
        fields = operation.defaults()
        fields.update(values)
        if "name" in fields:
            fields["name"] = result
        doc = ModelDocument(source)
        available = [b.result for b in doc.blocks if b.result]
        return doc.insert(result, operation.source(receiver, result, fields, available))

    def test_generated_hole_and_pattern_rebuild_and_display_results(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as folder:
            session = ModelSession(Path(folder) / "holes.py")
            source = self._operation(session.source, "Part.cuboid", "part", "base",
                                     pose_or_min="(-10, -10, -10)", extents_or_max="(10, 10, 10)")
            source = self._operation(source, "Solid.hole", "base", "drilled",
                                     mouth="Frame(origin=(0, 0, 10), y=(0, -1, 0), z=(0, 0, -1))",
                                     diameter="4", depth="8")
            source = self._operation(source, "Solid.pattern_linear", "drilled", "copies",
                                     count="2", step="(25, 0, 0)")
            source = ModelDocument(source).display_result("copies")
            self.assertTrue(session.rebuild(source), session.error)
            self.assertEqual(2, len(session.displayed))
            self.assertLess(session.values["drilled"].volume(), session.values["base"].volume())
            self.assertTrue(session.document.block("copies").editable)
            self.assertIn("copies", session.values)

    def test_generated_surface_thicken_and_sketch_preview(self):
        from camber.modeling import _pack_result
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as folder:
            session = ModelSession(Path(folder) / "sheet.py")
            source = ModelDocument(session.source).insert(
                "profile", '    profile = part.sketch("xy", name="profile", constrained=True)\n'
                           '    profile.add_rectangle((-5, -3), (5, 3))\n')
            source = self._operation(source, "Sketch.surface", "profile", "sheet")
            source = self._operation(source, "Surface.thicken", "sheet", "thick", thickness="2")
            source = ModelDocument(source).display_result("thick")
            self.assertTrue(session.rebuild(source), session.error)
            self.assertTrue(session.displayed.is_watertight())
            self.assertTrue(session.document.block("thick").editable)
            preview = _pack_result(session.part, session.values["profile"])
            self.assertTrue(any("Line" in name for name in preview["line_names"]))

    def test_curve_constructor_and_sweep_rebuild(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as folder:
            session = ModelSession(Path(folder) / "sweep.py")
            source = ModelDocument(session.source).insert(
                "profile", '    profile = part.sketch("xy", name="profile", constrained=True)\n'
                           '    profile.add_rectangle((-2, -2), (2, 2))\n')
            source = self._operation(source, "Curve3D.line", "", "guide", start="(0, 0, 0)", end="(0, 0, 15)")
            source = self._operation(source, "Sketch.extrude_along_curve", "profile", "swept", curve="guide")
            source = ModelDocument(source).display_result("swept")
            self.assertTrue(session.rebuild(source), session.error)
            self.assertTrue(session.displayed.is_watertight())
            self.assertTrue(session.document.block("swept").editable)

    def test_failed_apply_keeps_previous_model_and_source(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as folder:
            session = ModelSession(os.path.join(folder, "model.py"))
            self.assertTrue(session.rebuild(), session.error)
            initial = session.part
            bad = session.source.replace("tolerance=0.01", "tolerance=-1")
            self.assertFalse(session.rebuild(bad))
            self.assertIs(session.part, initial)
            self.assertEqual(session.source, _TEMPLATE)
            self.assertTrue(session.error)

    def test_save_undo_redo(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as folder:
            path = os.path.join(folder, "model.py")
            session = ModelSession(path)
            self.assertTrue(session.rebuild(), session.error)
            changed = session.document.replace(
                "setup", "    part = Part((-20, -20, -20), (20, 20, 20))\n")
            self.assertTrue(session.rebuild(changed), session.error)
            self.assertTrue(session.undo(), session.error)
            self.assertEqual(session.source, _TEMPLATE)
            self.assertTrue(session.redo(), session.error)
            self.assertEqual(session.source, changed)
            session.save()
            self.assertFalse(session.dirty)
            with open(path, encoding="utf-8") as stream:
                self.assertEqual(stream.read(), changed)

    def test_invalid_python_opens_for_repair(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as folder:
            path = Path(folder) / "invalid.py"
            path.write_text(_TEMPLATE.replace("return part, part", "return ("),
                            encoding="utf-8")
            session = ModelSession(path)
            self.assertIsNone(session.document)
            self.assertFalse(session.rebuild())
            self.assertTrue(session.rebuild(_TEMPLATE), session.error)
            self.assertIsNotNone(session.document)

    def test_save_refuses_external_edit(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as folder:
            path = Path(folder) / "model.py"
            path.write_text(_TEMPLATE, encoding="utf-8")
            session = ModelSession(path)
            self.assertTrue(session.rebuild(), session.error)
            path.write_text(_TEMPLATE + "\n# external change\n", encoding="utf-8")
            with self.assertRaisesRegex(OSError, "changed on disk"):
                session.save()

    def test_sketch_source_round_trip_builds(self):
        from camber.sketch_emit import emit_sketch_python
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as folder:
            session = ModelSession(Path(folder) / "sketch.py")
            source = session.document.insert(
                "profile", '    profile = part.sketch("xy", name="profile", constrained=True)\n'
                           '    profile.add_circle((0, 0), 5)\n')
            doc = ModelDocument(source)
            _, sketch, actions = _sketch_for_edit(doc, doc.block("profile"), session.path)
            emitted = emit_sketch_python(actions,
                                         sketch_var="profile", name="profile")
            source = doc.replace("profile", "".join("    " + line + "\n"
                                                       for line in emitted.splitlines()))
            self.assertTrue(session.rebuild(source), session.error)
            self.assertEqual(session.document.block("profile").kind, "Sketch")

    def test_sketch_edit_preserves_implicit_rectangle_constraints(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as folder:
            session = ModelSession(Path(folder) / "sketch.py")
            source = session.document.insert(
                "profile", '    profile = part.sketch("xy", name="profile", constrained=True)\n'
                           '    profile.add_rectangle((-5, -3), (5, 3))\n')
            self.assertTrue(session.rebuild(source), session.error)
            count = session.values["profile"].constraint_count
            self.assertGreater(count, 0)
            _, _, actions = _sketch_for_edit(session.document, session.document.block("profile"), session.path)
            from camber.sketch_emit import emit_sketch_python
            emitted = emit_sketch_python(actions, sketch_var="profile", name="profile")
            self.assertTrue(session.rebuild(session.document.replace("profile", textwrap.indent(emitted, "    "))), session.error)
            self.assertEqual(count, session.values["profile"].constraint_count)

    def test_offset_sketch_and_loft_build(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as folder:
            session = ModelSession(Path(folder) / "loft.py")
            source = session.document.insert(
                "first", '    first = part.sketch("xy", name="first", constrained=True)\n'
                         '    first.add_rectangle((-10, -10), (10, 10))\n')
            source = ModelDocument(source).insert(
                "second", '    second = part.sketch(frame=Frame(origin=(0, 0, 20)), '
                          'name="second", constrained=True)\n'
                          '    second.add_rectangle((-5, -5), (5, 5))\n')
            source = ModelDocument(source).insert(
                "loft1", '    loft1 = part.loft([first, second], name="loft1")\n')
            source = ModelDocument(source).display_result("loft1")
            self.assertTrue(session.rebuild(source), session.error)
            self.assertTrue(all(b.editable for b in session.document.blocks))


class ModelingViewerTests(unittest.TestCase):
    @unittest.skipUnless(os.environ.get("CAMBER_TEST_GL") == "1", "requires OpenGL")
    def test_assembly_commands_target_displayed_parent_after_loading_child(self):
        from camber.modeling import _Workspace
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as folder:
            session = ModelSession(Path(folder) / "model.py")
            source = session.document.insert("assembly", '    assembly = part.assembly()\n')
            source = ModelDocument(source).insert("child", '    child = part.assembly()\n')
            source = ModelDocument(source).insert("instance", '    instance = assembly.add_subassembly(child)\n')
            source = ModelDocument(source).display_result("assembly")
            self.assertTrue(session.rebuild(source), session.error)
            workspace = _Workspace(session)
            try:
                for key in ("Assembly.pattern_linear", "Assembly.pattern_circular", "Assembly.fix"):
                    workspace._choose_operation(key)
                    self.assertEqual("assembly", workspace.receiver)
                    workspace.viewer._draw_frame()
            finally:
                workspace.viewer.close()

    @contextmanager
    def cylinder_workspace(self):
        from camber.modeling import _Workspace
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as folder:
            session = ModelSession(Path(folder)/"model.py")
            source = session.document.insert("cylinder",'    cylinder = part.cylinder((0,0,0), 5, 10)\n')
            self.assertTrue(session.rebuild(source),session.error)
            session.save()
            workspace = _Workspace(session)
            workspace.viewer.window.set_visible(False)
            workspace._select("cylinder")
            workspace.viewer._draw_frame()
            try:
                yield workspace
            finally:
                workspace.viewer.close()

    @unittest.skipUnless(os.environ.get("CAMBER_TEST_GL") == "1", "requires OpenGL")
    def test_switching_features_protects_edits_and_validation_blocks_apply(self):
        with self.cylinder_workspace() as w:
            original = w.model.source
            # A toolbar action and the last text edit can arrive in one frame.
            identity,editor = next((key,value) for key,value in w.ui._editors.items() if key[1] == "radius##Part.cylinder")
            editor.document.text = "invalid"
            w.ui._focus = identity
            w._queue_action("select","setup")
            w.viewer._draw_frame()
            self.assertEqual(("select","setup"),w._pending_action)
            self.assertEqual("cylinder",w.model.selected)
            w._resolve_action("apply")
            self.assertEqual(original,w.model.source)
            self.assertIsNotNone(w._pending_action)
            w._resolve_action("stay")
            self.assertEqual("invalid",w.fields["radius"])
            w._request_action("select","setup")
            w._resolve_action("discard")
            self.assertEqual("setup",w.model.selected)
            self.assertEqual(original,w.model.source)

    @unittest.skipUnless(os.environ.get("CAMBER_TEST_GL") == "1", "requires OpenGL")
    def test_apply_save_undo_and_python_edit_protection(self):
        with self.cylinder_workspace() as w:
            from pyglet.window import key
            original = w.model.source
            w.fields["radius"] = "7"
            w._on_key_press(key.S,key.MOD_CTRL)
            w.viewer._draw_frame()
            self.assertEqual(("save",None),w._pending_action)
            w._resolve_action("apply")
            self.assertIn("radius=7",w.model.source)
            self.assertEqual(w.model.source,w.model.path.read_text(encoding="utf-8"))
            self.assertFalse(w.model.dirty)
            w.viewer._draw_frame()
            w._request_action("undo")
            self.assertEqual(original,w.model.source)
            w.viewer._draw_frame()
            w.editor = w.editor.replace("5, 10","6, 10")
            self.assertTrue(w._source_dirty())
            w._request_action("select","setup")
            self.assertIsNotNone(w._pending_action)
            w._resolve_action("stay")
            self.assertTrue(w._source_dirty())

    @unittest.skipUnless(os.environ.get("CAMBER_TEST_GL") == "1", "requires OpenGL")
    def test_close_waits_for_save_choice_and_after_frame_cleanup(self):
        with self.cylinder_workspace() as w:
            w.fields["radius"] = "6"
            w._on_close()
            w.viewer._draw_frame()
            self.assertEqual(("close",None),w._pending_action)
            self.assertFalse(w.viewer._closed)
            with patch.object(w.model,"save",side_effect=OSError("Cannot write model")):
                w._resolve_action("apply")
            self.assertIsNotNone(w._pending_action)
            self.assertFalse(w.viewer._closed)
            self.assertIn("Cannot write",w.form_build_error)
            self.assertIn("Cannot write",w.ui._menu.message)
            with patch.object(w.viewer._pyglet.clock,"schedule_once") as schedule:
                w._resolve_action("discard")
                self.assertFalse(w.viewer._closed)
                schedule.assert_called_once()
                schedule.call_args.args[0](0)
                self.assertTrue(w.viewer._closed)

    @unittest.skipUnless(os.environ.get("CAMBER_TEST_GL") == "1", "requires OpenGL")
    def test_close_confirmation_is_centered_and_escape_preserves_edits(self):
        with self.cylinder_workspace() as w:
            key = w.viewer._pyglet.window.key
            original = w.model.source
            for theme in ("light", "dark"):
                w.ui.set_theme(theme)
                w.fields["radius"] = "6"
                w._on_close()
                w.viewer._draw_frame()
                w.viewer._draw_frame()
                menu = w.ui._menu
                self.assertEqual("Unsaved changes", menu.name)
                self.assertEqual(["Save and close", "Discard and close", "Keep editing"],
                                 [item.label for item in menu.items])
                x, y, width, height = menu.rect
                self.assertAlmostEqual(w.viewer.window.width/2, x+width/2)
                self.assertAlmostEqual(w.viewer.window.height/2, y+height/2)
                self.assertTrue(w.ui.GetIO().WantCaptureMouse)
                w.ui.on_key_press(key.ESCAPE, 0)
                w.ui.on_key_release(key.ESCAPE, 0)
                w.viewer._draw_frame()
                self.assertIsNone(w._pending_action)
                self.assertIsNone(w.ui._menu)
                self.assertFalse(w.viewer._closed)
                self.assertEqual("6", w.fields["radius"])
                self.assertEqual(original, w.model.source)

    @unittest.skipUnless(os.environ.get("CAMBER_TEST_GL") == "1", "requires OpenGL")
    def test_extrude_guides_through_missing_sketch_and_selects_finished_profile(self):
        with self.cylinder_workspace() as w:
            original = w.model.source
            history = len(w.model.undo_stack)
            w._choose_operation("Sketch.extrude")
            self.assertEqual("Part.sketch",w.pending.key)
            self.assertEqual("Sketch.extrude",w._sketch_next_operation)
            with patch.object(w,"_start_sketch",side_effect=lambda block:setattr(w,"sketch_session",{})):
                self.assertTrue(w._new_feature())
            identifier = w.model.selected
            result = w.model.document.block(identifier).result
            w.sketch_session = {"state": {"done": "finish"}}
            w.sketch_block = identifier
            w.sketch_transparency = (w.viewer._solid_transparent, w.viewer._transparency)
            code = f'{result} = part.sketch("xy", constrained=True)\n{result}.add_rectangle((0, 0), (10, 10))\n'
            with patch("camber.sketch_ui.sketch_session_code",return_value=code), patch("camber.sketch_ui.finalize_sketch_session"):
                w._finish_sketch()
            self.assertEqual("Sketch.extrude",w.pending.key)
            self.assertEqual(result,w.receiver)
            self.assertIsNone(w._sketch_next_operation)
            self.assertEqual(history+1,len(w.model.undo_stack))
            self.assertEqual(original,w.model.undo_stack[-1])
            committed = w.model
            w._choose_operation("Part.sketch")
            with patch.object(w,"_start_sketch",side_effect=lambda block:setattr(w,"sketch_session",{})):
                self.assertTrue(w._new_feature())
            w.sketch_session = {"state": {"done": "cancel"}}
            w.sketch_block = w.model.selected
            w.sketch_transparency = (w.viewer._solid_transparent, w.viewer._transparency)
            with patch("camber.sketch_ui.finalize_sketch_session",return_value=(None,None)):
                w._finish_sketch()
            self.assertIs(committed,w.model)
            self.assertEqual(history+1,len(w.model.undo_stack))

    @unittest.skipUnless(os.environ.get("CAMBER_TEST_GL") == "1", "requires OpenGL")
    def test_active_sketch_must_be_resolved_before_close(self):
        with self.cylinder_workspace() as w:
            w.sketch_session = {"tick":lambda:None, "sketch_name":"Profile",
                                "state":{"done":None,"selected":[],"actions":[],"tool":"select"}}
            w._on_close()
            self.assertTrue(w._sketch_close_requested)
            self.assertFalse(w.viewer._closed)
            with patch.object(w.ui,"MessageBox",return_value="Discard and close"), patch.object(w,"_finish_sketch") as finish:
                w.viewer._draw_frame()
            finish.assert_called_once()
            self.assertEqual("cancel",w.sketch_session["state"]["done"])
            self.assertEqual(("close",None),w._queued_action)
            self.assertFalse(w.viewer._closed)

    def test_file_dialog_selection_and_cancellation_are_nonblocking(self):
        from camber.modeling import _Workspace
        workspace = _Workspace.__new__(_Workspace)
        workspace.model = SimpleNamespace(dirty=False, path=Path("model.py"))
        workspace._switch_file = unittest.mock.Mock()
        with patch("pyglet.window.dialog.FileOpenDialog") as factory:
            workspace._file_dialog()
            dialog = factory.return_value
            dialog.open.assert_called_once()
            chosen = dialog.push_handlers.call_args.kwargs["on_dialog_open"]
            chosen([])
            workspace._switch_file.assert_not_called()
            self.assertIsNone(workspace.dialog)
            chosen(["housing.py"])
            workspace._switch_file.assert_called_once_with(new=False,discard=False)
            self.assertEqual("housing.py",workspace.file_path)

    @unittest.skipUnless(os.environ.get("CAMBER_TEST_GL") == "1", "requires OpenGL")
    def test_double_click_reopens_hidden_feature_editor(self):
        from camber.modeling import _Workspace
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as folder:
            session = ModelSession(Path(folder) / "model.py")
            source = session.document.insert("pad", '    pad = part.cuboid((-4, -4, 0), (4, 4, 3))\n')
            self.assertTrue(session.rebuild(source), session.error)
            workspace = _Workspace(session)
            try:
                self.assertEqual((1920,1080), workspace.viewer.window.get_size())
                workspace.ui.set_dock_visible("Properties", False)
                workspace.viewer._draw_frame()
                args = next(args for kind, args, _ in workspace.ui._commands
                            if kind == "text_align" and args[2] == "Pad")
                workspace.ui.io.MousePos = args[0]
                workspace.ui.io.MouseDoubleClicked[0] = True
                workspace.viewer._draw_frame()
                self.assertEqual("pad", session.selected)
                self.assertTrue(workspace.ui.docks.visible["Properties"])
                workspace.viewer._draw_frame()
                self.assertEqual("pad", workspace.draft_for)
                self.assertIn("pose_or_min", workspace.fields)
            finally:
                workspace.viewer.close()

    def test_viewport_background_follows_theme(self):
        from camber.host import Viewer
        from camber import glview
        viewer = Viewer.__new__(Viewer)
        viewer._ui = SimpleNamespace(theme_name="light")
        self.assertEqual(viewer._background_color(), glview._BG)
        viewer._ui.theme_name = "dark"
        self.assertLess(max(viewer._background_color()), 0.2)

    @unittest.skipUnless(os.environ.get("CAMBER_TEST_GL") == "1", "requires OpenGL")
    def test_workspace_renders_both_themes(self):
        from camber.modeling import _Workspace
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as folder:
            session = ModelSession(Path(folder) / "model.py")
            self.assertTrue(session.rebuild(), session.error)
            workspace = _Workspace(session)
            try:
                for theme in ("light", "dark"):
                    workspace.ui.set_theme(theme)
                    workspace.viewer._draw_frame()
                self.assertIn("Features", workspace.ui.docks.rects)
                self.assertIn("Properties", workspace.ui.docks.rects)
            finally:
                workspace.viewer.close()

    @unittest.skipUnless(os.environ.get("CAMBER_TEST_GL") == "1", "requires OpenGL")
    def test_all_operation_forms_render_without_creating_features(self):
        from camber.modeling import _Workspace
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as folder:
            session = ModelSession(Path(folder) / "model.py")
            source = session.document.insert("profile",'    profile = part.sketch("xy")\n')
            self.assertTrue(session.rebuild(source), session.error)
            workspace = _Workspace(session)
            try:
                source, part = session.source, session.part
                operation_count = len(part.operations)
                for key in operations():
                    if key == "Part":
                        continue
                    with self.subTest(operation=key):
                        workspace._choose_operation(key)
                        self.assertEqual(key,workspace.pending.key)
                        workspace.viewer._draw_frame()
                        for identity in workspace.ui._tree_open:
                            if identity[1] == "More options##" + key:
                                workspace.ui._tree_open[identity] = True
                        workspace.viewer._draw_frame()
                self.assertEqual(source, session.source)
                self.assertIs(part, session.part)
                self.assertEqual(operation_count, len(part.operations))
                workspace.operation_filter = "no matching operation at all"
                workspace.ui._tabs[(id(workspace.ui._panels["Model"]), "Modeling commands")] = "Operations"
                workspace.viewer._draw_frame()
            finally:
                workspace.viewer.close()


if __name__ == "__main__":
    unittest.main()
