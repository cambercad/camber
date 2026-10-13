"""Saved-part, assembly-reference and guided UI workflow regressions."""

import os
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from camber import load_component, set_progress_log
from camber.feature_source import ModelDocument
from camber.modeling import ModelSession, _Workspace, _ASSEMBLY_TEMPLATE
from camber.modeling_commands import operations
from camber.modeling_assembly import AssemblyReferences


class _ModelFixture(unittest.TestCase):
    def setUp(self):
        set_progress_log(False)
        self.folder = tempfile.TemporaryDirectory(dir=Path(__file__).parent)
        self.root = Path(self.folder.name).resolve()
        part = ModelSession(self.root/"pad.py")
        source = part.document.insert("profile",'    profile = part.sketch("xy", name="Profile", constrained=True)\n    profile.add_rectangle((0, 0), (10, 10))\n')
        source = ModelDocument(source).insert("pad",'    pad = profile.extrude(5, name="pad")\n')
        self.assertTrue(part.rebuild(ModelDocument(source).display_result("pad")),part.error)
        part.save()

    def tearDown(self):
        self.folder.cleanup()

    def assembly(self):
        session = ModelSession(self.root/"assembly.py")
        source = ModelDocument(_ASSEMBLY_TEMPLATE).with_imports({"load_component"})
        for name,body in (("definition",'definition = load_component("pad.py")'),
                          ("base",'base = assembly.add_part(definition)'),
                          ("second",'second = assembly.add_part(definition, position=(20, 0, 0))'),
                          ("ground",'assembly.fix(base)')):
            source = ModelDocument(source).insert(name,"    "+body+"\n")
        self.assertTrue(session.rebuild(source),session.error)
        return session

class AssemblyDocumentTests(_ModelFixture):
    def test_mate_reference_types_are_validated_before_building(self):
        session = self.assembly()
        for method in ("coincident", "parallel", "perpendicular", "distance"):
            operation = operations()["Assembly."+method]
            values = operation.defaults()
            values.update(a="base.plane('pad:pad-ExtrudeTop')",
                          b="second."+("point" if method == "distance" else "axis")+"('reference')")
            self.assertIn("b", operation.field_errors(values, session.values))
            values["b"] = "second.plane('pad:pad-ExtrudeBottom')"
            self.assertNotIn("b", operation.field_errors(values, session.values))
        operation = operations()["Assembly.coincident"]
        values = operation.defaults()
        values.update(a="base.axis('reference')", b="second.axis('reference')", opposite_normals="True")
        self.assertIn("opposite_normals", operation.field_errors(values, session.values))
        values["opposite_normals"] = "None"
        self.assertEqual({}, operation.field_errors(values, session.values))

    def test_relative_components_and_plain_mate_calls_round_trip(self):
        session = self.assembly()
        body = "    assembly.coincident(base.plane('pad:pad-ExtrudeTop'), second.plane('pad:pad-ExtrudeBottom'), opposite_normals=True)\n"
        source = session.document.insert("join",body)
        self.assertTrue(session.rebuild(source),session.error)
        block = session.document.block("join")
        self.assertTrue(block.editable)
        self.assertIsNone(block.result)
        self.assertEqual("Assembly.coincident",block.operation)
        self.assertIn("second",block.dependencies)
        session.save()
        restored = ModelSession(session.path)
        self.assertTrue(restored.rebuild(),restored.error)
        constraint = restored.displayed.constraints[-1]
        a, b = constraint.first.world_direction, constraint.second.world_direction
        self.assertLess(a.x*b.x+a.y*b.y+a.z*b.z,-.999)
        self.assertAlmostEqual(5,restored.displayed.parts[1].pose[2],places=4)
        self.assertEqual(2,len(restored.displayed.constraints))
        self.assertEqual(2,len(load_component(session.path).parts))

    def test_repeated_component_picks_are_distinct_and_preserve_colors(self):
        from camber import glview
        from camber.modeling import _pack_result
        import numpy as np
        session = self.assembly()
        original = glview.pack_scene(glview._as_scene(session.displayed))
        packed = _pack_result(session.part,session.displayed)
        names = packed["entity_types"]
        first = "assembly/pad[1]:pad:pad-ExtrudeTop"
        second = "assembly/pad[2]:pad:pad-ExtrudeTop"
        self.assertIn(first,names)
        self.assertIn(second,names)
        context = AssemblyReferences(session.displayed,"assembly")
        self.assertNotEqual(context.from_pick(first,("plane",)),context.from_pick(second,("plane",)))
        np.testing.assert_allclose(original["mesh_color"],packed["mesh_color"])

    def test_nested_navigator_exposes_components_and_internal_constraints(self):
        session = self.assembly()
        parent = session.part.assembly("parent")
        parent.add_subassembly(session.displayed)
        context = AssemblyReferences(parent,"parent")
        address = "parent.subassemblies[0]"
        self.assertEqual([address],context.children["parent"])
        self.assertEqual(2,len(context.children[address]))
        self.assertEqual("FixPart",context.constraints[address][0].kind)
        self.assertEqual(2,len(context.components))

    def test_bad_mate_retains_last_successful_assembly(self):
        session = self.assembly()
        source = session.document.insert("join","    assembly.coincident(base.plane('pad:pad-ExtrudeTop'), second.plane('pad:pad-ExtrudeBottom'), opposite_normals=True)\n")
        self.assertTrue(session.rebuild(source),session.error)
        previous, packed = session.displayed, session.packed
        invalid = session.document.insert("conflict","    assembly.distance(base.plane('pad:pad-ExtrudeTop'), second.plane('pad:pad-ExtrudeBottom'), 10)\n")
        self.assertFalse(session.rebuild(invalid))
        self.assertIn("cannot be satisfied",session.error)
        self.assertEqual(source,session.source)
        self.assertIs(previous,session.displayed)
        self.assertIs(packed,session.packed)
        with self.assertRaisesRegex(ValueError,"Used by"):
            session.document.delete("second")

    def test_circular_component_loading_is_rejected(self):
        source = ModelDocument(_ASSEMBLY_TEMPLATE).with_imports({"load_component"})
        source = ModelDocument(source).insert("recursive",'    recursive = load_component("loop.py")\n')
        (self.root/"loop.py").write_text(source,encoding="utf-8")
        with self.assertRaisesRegex(ValueError,"Circular component"):
            load_component(self.root/"loop.py")


@unittest.skipUnless(os.environ.get("CAMBER_TEST_GL") == "1","requires OpenGL")
class AssemblyWorkflowTests(_ModelFixture):
    def test_component_orientation_uses_degrees_blocks_incomplete_edits_and_round_trips(self):
        from camber.gui_testing import GuiDriver
        w = _Workspace(self.assembly())
        w.viewer.window.set_visible(False)
        try:
            driver = GuiDriver(w.ui, w.viewer._draw_frame)
            driver.click("Second", panel="Features")
            driver.click("More options##Assembly.add_part", panel="Properties")
            driver.click("orientation##Assembly.add_part##X", panel="Properties")
            driver.text("-", replace=True)
            self.assertFalse(driver.control("Apply", panel="Properties").enabled)
            driver.text("20", replace=True)
            self.assertTrue(driver.control("Apply", panel="Properties").enabled)
            driver.click("Apply", panel="Properties")
            self.assertFalse(w.form_build_error, w.form_build_error)
            w.model.save()
            restored = ModelSession(w.model.path)
            self.assertTrue(restored.rebuild(), restored.error)
            context = AssemblyReferences(restored.displayed, "assembly")
            self.assertEqual(2, len(context.components))
            driver.click("Second", panel="Features")
            editor = w.vector_fields["orientation##Assembly.add_part"]
            self.assertAlmostEqual(20, float(editor.values[0]), places=8)
            self.assertAlmostEqual(0, float(editor.values[1]), places=8)
            self.assertAlmostEqual(0, float(editor.values[2]), places=8)
        finally:
            w.viewer.close()

    def test_every_mate_datum_type_can_be_created_and_removed_in_gui(self):
        from camber.gui_testing import GuiDriver
        w = _Workspace(self.assembly())
        w.viewer.window.set_visible(False)
        try:
            driver = GuiDriver(w.ui, w.viewer._draw_frame)
            def choose(label, value):
                driver.click(label, panel="Properties")
                index = w.ui._combo_options.index(value) if isinstance(value, str) else value
                driver.key("HOME")
                for _ in range(index):
                    driver.key("DOWN")
                driver.key("RETURN")
            cases = (("coincident", "plane"), ("coincident", "axis"), ("coincident", "point"),
                     ("parallel", "plane"), ("parallel", "axis"),
                     ("perpendicular", "plane"), ("perpendicular", "axis"),
                     ("concentric", "axis"), ("angle", "axis"),
                     ("distance", "plane"), ("distance", "point"),
                     ("on_plane", "point"), ("contact", "point"))
            driver.click("Assembly##tab", panel="Model")
            for method, kind in cases:
                with self.subTest(method=method, kind=kind):
                    driver.click("On plane" if method == "on_plane" else method.capitalize(), panel="Model")
                    fields = ("point", "plane") if method in ("on_plane", "contact") else ("a", "b")
                    for index, field in enumerate(fields):
                        identity = field+"##Assembly."+method
                        choose(identity+"/component", index)
                        if method in ("coincident", "parallel", "perpendicular", "distance"):
                            choose(identity+"/kind", kind)
                        choose(identity+"/entity", 0)
                    if method in ("angle", "distance"):
                        label = ("radians" if method == "angle" else "value")+"##Assembly."+method
                        driver.click(label, panel="Properties")
                        driver.text("30" if method == "angle" else "20", replace=True)
                    driver.click("Create mate", panel="Properties")
                    self.assertIsNone(w.pending, w.form_build_error or w.model.error)
                    self.assertEqual(2, len(w.model.displayed.constraints))
                    self.assertTrue(w.model.displayed.solve().converged)
                    block = next(b for b in reversed(w.model.document.blocks) if b.operation == "Assembly."+method)
                    driver.click(block.id.replace("_", " ").capitalize(), panel="Features")
                    driver.click("Delete feature", panel="Properties")
                    self.assertEqual(1, len(w.model.displayed.constraints))
        finally:
            w.viewer.close()

    def test_mate_form_blocks_mismatched_reference_kinds_and_allows_correction(self):
        from camber.gui_testing import GuiDriver
        w = _Workspace(self.assembly())
        w.viewer.window.set_visible(False)
        try:
            driver = GuiDriver(w.ui, w.viewer._draw_frame, capture=w.viewer.capture_image)
            def choose(label, index):
                driver.click(label, panel="Properties")
                driver.key("HOME")
                for _ in range(index):
                    driver.key("DOWN")
                driver.key("RETURN")
            driver.click("Assembly##tab", panel="Model")
            driver.click("Coincident", panel="Model")
            choose("a##Assembly.coincident/component", 0)
            choose("a##Assembly.coincident/entity", 0)
            choose("b##Assembly.coincident/component", 1)
            choose("b##Assembly.coincident/kind", 1)
            choose("b##Assembly.coincident/entity", 0)
            self.assertFalse(driver.control("Create mate", panel="Properties").enabled)
            choose("b##Assembly.coincident/kind", 0)
            choose("b##Assembly.coincident/entity", 0)
            self.assertTrue(driver.control("Create mate", panel="Properties").enabled)
            self.assertEqual(1, len(w.model.displayed.constraints))
        finally:
            w.viewer.close()

    def test_new_assembly_from_untouched_document_needs_no_save_prompt(self):
        from camber.gui_testing import GuiDriver
        session = ModelSession(self.root/"untouched.py")
        self.assertTrue(session.rebuild(), session.error)
        w = _Workspace(session)
        w.viewer.window.set_visible(False)
        try:
            driver = GuiDriver(w.ui, w.viewer._draw_frame, capture=w.viewer.capture_image)
            with patch("pyglet.window.dialog.FileSaveDialog") as factory:
                driver.click("File##tab", panel="Model")
                driver.click("New assembly", panel="Model")
                self.assertIsNone(w._pending_action)
                factory.return_value.open.assert_called_once()
                selected = factory.return_value.push_handlers.call_args.kwargs["on_dialog_save"]
                selected(str(self.root/"new_assembly.py"))
                driver.frame()
            self.assertEqual("Assembly", type(w.model.displayed).__name__)
            self.assertFalse(w.model.dirty)
            self.assertIsNone(w.model.saved_source)
            self.assertFalse(w.model.path.exists())
            self.assertTrue(any(args[2] == "New model  ·  Experimental modeling"
                                for kind, args, panel in w.ui._commands if kind == "text_align"))
            with patch("pyglet.window.dialog.FileSaveDialog") as factory:
                driver.click("File##tab", panel="Model")
                driver.click("New part", panel="Model")
                self.assertIsNone(w._pending_action)
                factory.return_value.open.assert_called_once()
        finally:
            w.viewer.close()

    def test_current_and_prior_assembly_references_are_cached_until_rebuild(self):
        session = self.assembly()
        w = _Workspace(session)
        w.viewer.window.set_visible(False)
        try:
            full = w._available_values()
            prior = w._available_values(session.document.block("second").start)
            with patch("camber.modeling_assembly.AssemblyReferences", wraps=AssemblyReferences) as build:
                current_context = w._assembly_context(full)
                prior_context = w._assembly_context(prior)
                count = build.call_count
                for _ in range(3):
                    self.assertIs(current_context, w._assembly_context(full))
                    self.assertIs(prior_context, w._assembly_context(prior))
                self.assertEqual(count, build.call_count)
                self.assertTrue(session.rebuild(), session.error)
                rebuilt = w._assembly_context(w._available_values())
                self.assertIsNot(current_context, rebuilt)
                rebuilt_prior = w._assembly_context(w._available_values(session.document.block("second").start))
                self.assertIsNot(prior_context, rebuilt_prior)
                self.assertGreater(build.call_count, count)
        finally:
            w.viewer.close()

    def test_bearing_component_sketch_edit_save_and_return_rebuilds_the_assembly(self):
        from camber.gui_testing import GuiDriver
        example = Path(__file__).parents[1]/"examples"/"modeling"/"ball_bearing"
        for name in ("bearing.py", "inner_race.py", "outer_race.py", "ball.py"):
            (self.root/name).write_text((example/name).read_text(encoding="utf-8"), encoding="utf-8")
        session = ModelSession(self.root/"bearing.py")
        self.assertTrue(session.rebuild(), session.error)
        w = _Workspace(session)
        w.viewer.window.set_visible(False)
        try:
            driver = GuiDriver(w.ui, w.viewer._draw_frame, capture=w.viewer.capture_image)
            self.assertEqual(11, len(w.model.displayed.parts))
            driver.click("Inner race instance", panel="Features")
            driver.click("Open component model", panel="Properties")
            item = driver.control("Raceway tool", panel="Features")
            driver.click(x=item.rect[0]+8, y=item.rect[1]+item.rect[3]/2)
            driver.click("Raceway profile", panel="Features", count=2)
            self.assertIsNotNone(w.sketch_session, w.status)
            driver.click("Raceway", panel="Features", count=2)
            driver.click("Handle##curve_editor", panel="Properties")
            driver.key("DOWN")
            driver.key("RETURN")
            driver.click("Coordinate##curve_editor_0", panel="Properties")
            driver.text("-", replace=True)
            self.assertFalse(driver.control("Apply##curve_editor", panel="Properties").enabled)
            self.assertFalse(driver.control("Finish sketch", panel="Model").enabled)
            driver.text("3.6", replace=True)
            driver.click("Apply##curve_editor", panel="Properties")
            driver.click("Finish sketch", panel="Model")
            self.assertIsNone(w.sketch_session)
            self.assertTrue(w.model.dirty)
            self.assertIn("3.6", w.model.document.block("raceway_profile").source)
            driver.key("S", modifiers=w.viewer._pyglet.window.key.MOD_CTRL)
            self.assertFalse(w.model.dirty)
            driver.click("Back to assembly", panel="Document")
            self.assertEqual(session.path, w.model.path)
            self.assertEqual(11, len(w.model.displayed.parts))
            self.assertFalse(w._return_paths)
            self.assertFalse(w.model.dirty)
            restored = ModelSession(self.root/"inner_race.py")
            self.assertTrue(restored.rebuild(), restored.error)
            self.assertIn("3.6", restored.document.block("raceway_profile").source)
            for theme in ("light", "dark"):
                w.ui.set_theme(theme)
                image = driver.screenshot()
                width, height = w.viewer.window.get_framebuffer_size()
                self.assertEqual((width, height), (image.width, image.height))
                self.assertNotEqual(image.get_bytes("RGBA", -width*4)[:width*160*4],
                                    w.viewer.capture_rgba().tobytes()[:width*160*4])
        finally:
            w.viewer.close()

    def test_initial_assembly_setup_draws_without_evaluating_a_missing_part(self):
        w = _Workspace(self.assembly())
        w.viewer.window.set_visible(False)
        try:
            self.assertEqual({}, w._available_values(w.model.document.blocks[0].start))
            w.viewer._draw_frame()
        finally:
            w.viewer.close()

    def click(self,w,label):
        w.viewer._draw_frame()
        commands = [args for kind,args,panel in w.ui._commands if kind == "text_align" and args[2] == label]
        self.assertTrue(commands,"Missing action: "+label)
        x,y = commands[-1][0]
        w.ui.on_mouse_press(x,w.viewer.window.height-y,1,0)
        w.viewer._draw_frame()
        w.ui.on_mouse_release(x,w.viewer.window.height-y,1,0)
        w.viewer._draw_frame()

    def test_insert_dialog_fix_preview_apply_save_and_reopen(self):
        session = ModelSession(self.root/"new_assembly.py")
        self.assertTrue(session.rebuild(_ASSEMBLY_TEMPLATE),session.error)
        w = _Workspace(session)
        w.viewer.window.set_visible(False)
        try:
            w._select("assembly")
            with patch("pyglet.window.dialog.FileOpenDialog") as factory:
                self.click(w,"Insert component")
                factory.return_value.open.assert_called_once()
                callback = factory.return_value.push_handlers.call_args.kwargs["on_dialog_open"]
                callback([])
                self.assertEqual(0,len(session.displayed.parts))
                self.click(w,"Insert")
                factory.return_value.push_handlers.call_args.kwargs["on_dialog_open"]([str(self.root/"pad.py")])
                w.viewer._draw_frame()
            self.assertEqual(1,len(session.displayed.parts))
            self.assertEqual(1,len(session.displayed.constraints))
            self.assertTrue(w._insert_component(self.root/"pad.py"),w.status)
            w.viewer._draw_frame()
            texts = [args[2] for kind,args,panel in w.ui._commands if kind == "text_align"]
            self.assertIn("Components",texts)
            self.assertIn("Constraints",texts)
            self.click(w,"Coincident")
            self.assertEqual("Assembly.coincident",w.pending.key)
            context = w._assembly_context(w._available_values())
            components = list(context.components)
            w.fields["a"] = components[0]+".plane('pad:pad-ExtrudeTop')"
            w.fields["b"] = components[1]+".plane('pad:pad-ExtrudeBottom')"
            source, history = session.source, len(session.undo_stack)
            self.click(w,"Preview")
            self.assertEqual(source,session.source)
            self.assertEqual(history,len(session.undo_stack))
            self.click(w,"Create mate")
            self.assertEqual(2,len(session.displayed.constraints))
            self.assertIsNone(w.pending)
            from pyglet.window import key
            w._on_key_press(key.S,key.MOD_CTRL)
            w.viewer._draw_frame()
            reopened = ModelSession(session.path)
            self.assertTrue(reopened.rebuild(),reopened.error)
            self.assertEqual(2,len(reopened.displayed.constraints))
            self.assertAlmostEqual(5,reopened.displayed.parts[1].pose[2],places=4)
        finally:
            w.viewer.close()

    def test_open_component_and_return_rebuilds_the_parent_assembly(self):
        session = self.assembly()
        session.save()
        w = _Workspace(session)
        w.viewer.window.set_visible(False)
        try:
            with patch.object(w.viewer, "highlight_solid", wraps=w.viewer.highlight_solid) as highlight:
                w._select("base")
                names = highlight.call_args.args[0]
                self.assertTrue(names)
                self.assertTrue(all(n.startswith("assembly/pad[1]:") for n in names))
            self.assertFalse(w.picked, "Feature highlighting is not a picked modeling reference")
            path = w._component_file("base")
            w._run_action("open_component",path)
            self.assertEqual(path,w.model.path)
            self.assertEqual([session.path],w._return_paths)
            block = w.model.document.block("pad")
            candidate = w.model.document.replace("pad",block.source.replace("extrude(5,", "extrude(8,"))
            self.assertTrue(w._apply(candidate),w.status)
            self.assertTrue(w._save_model(),w.status)
            w._run_action("return_parent")
            self.assertEqual(session.path,w.model.path)
            self.assertFalse(w._return_paths)
            self.assertAlmostEqual(8,w.model.displayed.leaves()[0].bounds[1].z,places=4)
        finally:
            w.viewer.close()

    def test_every_assembly_constraint_command_creates_a_solved_mate(self):
        session = self.assembly()
        original = session.source
        w = _Workspace(session)
        w.viewer.window.set_visible(False)
        try:
            for method in ("fix", "coincident", "parallel", "concentric", "perpendicular", "angle", "distance", "on_plane", "contact"):
                with self.subTest(method=method):
                    self.assertTrue(w.model.rebuild(original),w.model.error)
                    w._choose_operation("Assembly."+method)
                    context = w._assembly_context(w._available_values())
                    components = list(context.components)
                    def reference(component, kind):
                        return component+"."+kind+"("+repr(context.components[component].references[kind][0])+")"
                    if method == "fix":
                        w.fields["part"] = components[1]
                    elif method in ("on_plane", "contact"):
                        w.fields["point"] = reference(components[0],"point")
                        w.fields["plane"] = reference(components[1],"plane")
                    else:
                        kind = "axis" if method in ("concentric", "angle") else "plane"
                        w.fields["a"] = reference(components[0],kind)
                        w.fields["b"] = reference(components[1],kind)
                        if method == "angle":
                            w.fields["radians"] = "0.5"
                        if method == "distance":
                            w.fields["value"] = "3"
                    w.viewer._draw_frame()
                    self.assertFalse(w._form_errors(w.pending,w._available_values(),w.result_name))
                    self.assertTrue(w._new_feature(),w.model.error)
                    self.assertEqual(2,len(w.model.displayed.constraints))
                    self.assertIsNone(w.pending)
        finally:
            w.viewer.close()

    def test_guided_mate_picking_advances_through_plane_and_point_references(self):
        w = _Workspace(self.assembly())
        w.viewer.window.set_visible(False)
        try:
            original = w.model.source
            for method, labels, kinds in (("coincident", ("a", "b"), ("plane", "plane")),
                                          ("on_plane", ("point", "plane"), ("point", "plane")),
                                          ("contact", ("point", "plane"), ("point", "plane"))):
                with self.subTest(method=method):
                    self.assertTrue(w.model.rebuild(original), w.model.error)
                    w._assembly_references = w._prior_values = None
                    key = "Assembly."+method
                    w._choose_operation(key)
                    available = w._available_values()
                    context = w._assembly_context(available)
                    first, second = list(context.components.values())
                    def pick(component, kind):
                        return component.path+":"+component.references[kind][0]
                    self.assertEqual((key, labels[0]), w._pick_datum)
                    self.assertTrue(w._accept_datum_pick(pick(first, kinds[0]), available))
                    self.assertEqual((key, labels[1]), w._pick_datum)
                    unchanged = w.fields[labels[1]]
                    self.assertFalse(w._accept_datum_pick(pick(first, kinds[1]), available))
                    self.assertEqual(unchanged, w.fields[labels[1]])
                    self.assertIn("another component", w.status)
                    self.assertTrue(w._accept_datum_pick(pick(second, kinds[1]), available))
                    self.assertIsNone(w._pick_datum)
                    self.assertTrue(w._new_feature(), w.model.error)
                    w.model.save()
                    restored = ModelSession(w.model.path)
                    self.assertTrue(restored.rebuild(), restored.error)
                    self.assertEqual(2, len(restored.displayed.constraints))
        finally:
            w.viewer.close()

    def test_new_assembly_and_save_as_dialog_callbacks(self):
        session = self.assembly()
        session.save()
        w = _Workspace(session)
        w.viewer.window.set_visible(False)
        try:
            new_path = self.root/"empty_assembly.py"
            with patch("pyglet.window.dialog.FileSaveDialog") as factory:
                w._run_action("new_assembly")
                factory.return_value.push_handlers.call_args.kwargs["on_dialog_save"](str(new_path))
            self.assertEqual("Assembly",type(w.model.displayed).__name__)
            self.assertFalse(new_path.exists())
            self.assertTrue(w._insert_component(self.root/"pad.py"),w.status)
            history = len(w.model.undo_stack)
            selected = w.model.selected
            target = self.root/"copies"/"saved.py"
            with patch("pyglet.window.dialog.FileSaveDialog") as factory:
                w._save_as_dialog()
                callback = factory.return_value.push_handlers.call_args.kwargs["on_dialog_save"]
                callback("")
                self.assertEqual(new_path,w.model.path)
                callback(str(target))
            self.assertTrue(target.exists(),w.status)
            self.assertFalse(w.model.dirty)
            self.assertEqual(history,len(w.model.undo_stack))
            self.assertEqual(selected,w.model.selected)
            self.assertIn("../pad.py",w.model.source.replace("\\\\","/"))
            restored = ModelSession(target)
            self.assertTrue(restored.rebuild(),restored.error)
            self.assertEqual(1,len(restored.displayed.parts))
        finally:
            w.viewer.close()
