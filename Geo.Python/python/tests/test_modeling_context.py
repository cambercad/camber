"""Mode-specific navigation and lossless graphical sketch editing."""

import os
from pathlib import Path
import tempfile
import textwrap
import unittest

from camber import set_progress_log
from camber.modeling import ModelSession, _Workspace, _sketch_for_edit
from camber.sketch_emit import emit_sketch_python
from camber import sketch_ui


class _SketchFixture(unittest.TestCase):
    def setUp(self):
        set_progress_log(False)
        self.folder = tempfile.TemporaryDirectory(dir=Path(__file__).parent)
        self.model = ModelSession(Path(self.folder.name)/"sketch.py")

    def tearDown(self):
        self.folder.cleanup()

    def sketch(self, fixed=False):
        code = ('    profile = part.sketch("xy", constrained=True, name="profile")\n'
                '    profile.add_line((0, 0), (10, 0), name="Edge", construction=True)\n'
                '    profile.horizontal("Edge")\n')
        if fixed:
            code += '    profile.fix("Edge@0.000")\n    profile.fix("Edge@1.000")\n'
        code += '    profile.length("Edge", 10)\n    profile.solve()\n'
        source = self.model.document.insert("profile", code)
        self.assertTrue(self.model.rebuild(source), self.model.error)
        _, sketch, actions = _sketch_for_edit(self.model.document, self.model.document.block("profile"), self.model.path)
        state = {"actions": actions, "solved": sketch._solved_actions(), "sketch": sketch,
                 "sketch_name": "profile", "frame": sketch.frame, "selected": [], "part_points": [],
                 "batches": [{"actions": list(actions), "curves": sketch.curve_count,
                              "constraints": sketch.constraint_count}]}
        sketch_ui._rebuild_pick_catalog(state)
        return state


class SketchContextTests(_SketchFixture):
    def test_constrained_construction_sketch_round_trip_and_tree_selection(self):
        state = self.sketch()
        sketch_ui.select_tree_action(state, 1)
        self.assertIn("profile:Edge", state["selected"])
        dimension = len(state["actions"])-1
        sketch_ui.select_tree_action(state, dimension, edit=True)
        self.assertEqual(dimension, state["dimension"]["edit_index"])
        self.assertTrue(sketch_ui._update_dimension(state, dimension, 12))
        emitted = emit_sketch_python(state["actions"], sketch_var="profile", name="profile")
        self.assertTrue(self.model.rebuild(self.model.document.replace("profile", textwrap.indent(emitted, "    "))), self.model.error)
        sketch = self.model.values["profile"]
        self.assertEqual(2, sketch.constraint_count)
        self.assertTrue(sketch._n.curve_is_construction(0))
        edge = sketch._solved_actions()[0]
        self.assertAlmostEqual(12, abs(edge["p1"][0]-edge["p0"][0]), places=4)
        sketch_ui.select_tree_action(state, 0, edit=True)
        self.assertEqual(0, state["curve_edit"]["index"])

    def test_failed_dimension_edit_restores_live_geometry_and_source_actions(self):
        state = self.sketch(fixed=True)
        before = state["sketch"]._solved_actions()
        index = len(state["actions"])-1
        sketch_ui.select_tree_action(state, index, edit=True)
        state["dimension"]["value"] = 15
        sketch_ui._commit_dimension(state)
        self.assertIsNotNone(state["dimension"])
        self.assertEqual(10, state["actions"][index]["value"])
        self.assertEqual(before, state["sketch"]._solved_actions())
        self.assertEqual(4, state["sketch"].constraint_count)
        self.assertIn("previous sketch retained", state["edit_error"])

    def test_solved_updates_keep_batch_order_when_curves_follow_constraints(self):
        state = self.sketch()
        state.update(part=self.model.part, plane="xy", emit_frame=False)
        added = {"kind": "line", "name": "Second", "p0": (0, 2), "p1": (3, 2)}
        self.assertTrue(sketch_ui._append_and_solve(state, [added]))
        self.assertEqual("line", state["actions"][-1]["kind"])
        self.assertTrue(sketch_ui._update_dimension(state, 2, 14))
        self.assertEqual(2, state["sketch"].curve_count)
        self.assertEqual("Second", state["sketch"]._solved_actions()[1]["name"])

    def test_unsupported_python_sketch_is_not_replaced_by_a_lossy_dump(self):
        source = self.model.document.insert("profile", '    profile = part.sketch("xy", constrained=True)\n'
                                           '    for x in (0, 5):\n'
                                           '        profile.add_circle((x, 0), 2)\n')
        self.assertTrue(self.model.rebuild(source), self.model.error)
        with self.assertRaisesRegex(ValueError, "preserve"):
            _sketch_for_edit(self.model.document, self.model.document.block("profile"), self.model.path)
        self.assertEqual(source, self.model.source)

    def test_undo_cancels_curve_editor_without_removing_sketch(self):
        state = self.sketch()
        state["pending"] = []
        sketch_ui.select_tree_action(state, 0, edit=True)
        sketch_ui._undo_pending_or_action(state)
        self.assertIsNone(state["curve_edit"])
        self.assertEqual(1, state["sketch"].curve_count)
        self.assertEqual(2, state["sketch"].constraint_count)

    def test_constraint_edit_preserves_geometry_and_other_constraints(self):
        state = self.sketch()
        source = self.model.source.replace('    profile.horizontal("Edge")',
            '    profile.add_line((0, 2), (3, 5), name="Second")\n    profile.horizontal("Edge")')
        self.assertTrue(self.model.rebuild(source), self.model.error)
        _, live, actions = _sketch_for_edit(self.model.document, self.model.document.block("profile"), self.model.path)
        state.update(actions=actions, sketch=live, solved=live._solved_actions(),
                     batches=[{"actions": list(actions), "curves": 2, "constraints": 2}])
        sketch_ui._rebuild_pick_catalog(state)
        sketch_ui.select_tree_action(state, 2, edit=True)
        self.assertIsNotNone(state["constraint_edit"])
        self.assertTrue(sketch_ui._replace_action(state, 2, {"kind": "horizontal", "curves": ["Second"], "points": []}), state.get("edit_error"))
        first, second = state["sketch"]._solved_actions()
        self.assertAlmostEqual(second["p0"][1], second["p1"][1], places=4)
        self.assertAlmostEqual(10, abs(first["p0"][0]-first["p1"][0]), places=4)
        self.assertEqual(2, state["sketch"].constraint_count)


@unittest.skipUnless(os.environ.get("CAMBER_TEST_GL") == "1", "requires OpenGL")
class SketchNavigatorTests(_SketchFixture):
    def test_standard_views_fit_and_picking_guidance_are_available_without_python(self):
        from camber.gui_testing import GuiDriver
        source = self.model.document.insert("body", '    body = part.cuboid((-2,-3,-4), (2,3,4))\n')
        self.assertTrue(self.model.rebuild(source), self.model.error)
        w = _Workspace(self.model)
        w.viewer.window.set_visible(False)
        try:
            d = GuiDriver(w.ui, w.viewer._draw_frame)
            d.click("View##tab", panel="Model")
            center = w.viewer.camera.center
            zoom = w.viewer.camera.zoom
            for name, expected in (("Top", (0, 0, -1)), ("Front", (0, 1, 0)),
                                   ("Right", (-1, 0, 0)), ("Left", (1, 0, 0))):
                d.click(name, panel="Model")
                self.assertEqual(center, w.viewer.camera.center)
                self.assertEqual(zoom, w.viewer.camera.zoom)
                for actual, target in zip(w.viewer.camera.frame()[0], expected):
                    self.assertAlmostEqual(actual, target, places=8)
                w.viewer.camera.fit((-2, -3, -4), (2, 3, 4))
                for actual, target in zip(w.viewer.camera.frame()[0], expected):
                    self.assertAlmostEqual(actual, target, places=8)
                zoom = w.viewer.camera.zoom
            d.click("Solid##tab", panel="Model")
            d.click("Hole", panel="Model")
            d.click("Pick planar face##mouth##Solid.hole", panel="Properties")
            self.assertIn("Pick planar face", w.status)
            self.assertFalse(d.control("Create feature", panel="Properties").enabled)
        finally:
            w.viewer.close()

    def test_right_click_deletes_the_named_feature(self):
        from camber.gui_testing import GuiDriver
        source = self.model.document.insert("body", '    body = part.cuboid((-2,-3,-4), (2,3,4))\n')
        self.assertTrue(self.model.rebuild(source), self.model.error)
        w = _Workspace(self.model)
        w.viewer.window.set_visible(False)
        try:
            driver = GuiDriver(w.ui, w.viewer._draw_frame)
            item = driver.control("Body##body", panel="Features")
            x, y = item.rect[0] + 36, item.rect[1] + 12
            mouse = w.viewer._pyglet.window.mouse
            driver.move(x, y)
            w.ui.on_mouse_press(x, w.viewer.window.height - y, mouse.RIGHT, 0)
            driver.frame()
            self.assertEqual("Body", w.ui._menu.items[0].label)
            rect = dict(w.ui._menu.rows)[0]
            driver.click(x=rect[0] + 24, y=rect[1] + 10)
            self.assertNotIn("body", [block.id for block in w.model.document.blocks])
        finally:
            w.viewer.close()

    def test_create_sketch_draw_extrude_validate_and_save_through_gui_events(self):
        from camber.gui_testing import GuiDriver
        self.assertTrue(self.model.rebuild(), self.model.error)
        w = _Workspace(self.model)
        w.viewer.window.set_visible(False)
        try:
            driver = GuiDriver(w.ui, w.viewer._draw_frame, capture=w.viewer.capture_image)
            driver.click("Create sketch", panel="Properties")
            driver.click("Create sketch", panel="Properties")
            self.assertIsNotNone(w.sketch_session, w.status)
            driver.click("Circle", panel="Model")
            x, y, width, height = w.ui.viewport_rect
            driver.click(x=x+width/2, y=y+height/2)
            driver.click(x=x+width/2+80, y=y+height/2)
            self.assertEqual(1, w.sketch_session["state"]["sketch"].curve_count)
            driver.click("Finish sketch", panel="Model")
            driver.click("Extrude", panel="Properties")
            driver.click("height##Sketch.extrude", panel="Properties")
            driver.text("-", replace=True)
            self.assertFalse(driver.control("Create feature", panel="Properties").enabled)
            self.assertFalse(self.model.path.exists())
            driver.text("6", replace=True)
            driver.click("Create feature", panel="Properties")
            self.assertAlmostEqual(6, w.model.packed["bounds"][1][2])
            driver.key("S", modifiers=w.viewer._pyglet.window.key.MOD_CTRL)
            self.assertTrue(self.model.path.exists())
            self.assertFalse(w.model.dirty)
            restored = ModelSession(self.model.path)
            self.assertTrue(restored.rebuild(), restored.error)
            self.assertEqual(3, len(restored.document.blocks))
            self.assertAlmostEqual(6, restored.packed["bounds"][1][2])
        finally:
            w.viewer.close()

    def test_sketch_context_is_translucent_without_a_duplicate_body_and_restores_on_close(self):
        from camber.feature_source import ModelDocument
        source = self.model.document.insert("body", '    body = part.cuboid((0, 0, 0), (10, 10, 4), name="body")\n')
        source = ModelDocument(source).insert("profile", '    profile = part.sketch(frame=Frame(origin=(0, 0, 2)), constrained=True, name="profile")\n'
                                              '    profile.add_circle((5, 5), 2, name="Circle")\n')
        self.assertTrue(self.model.rebuild(source), self.model.error)
        w = _Workspace(self.model)
        w.viewer.window.set_visible(False)
        try:
            for transparent, done in ((False, "finish"), (True, "cancel")):
                w.viewer.set_solid_transparent(transparent)
                original_mode = w.viewer._transparency
                w._run_action("edit_sketch", "profile")
                self.assertIsNotNone(w.sketch_session, w.status)
                self.assertTrue(w.viewer._solid_transparent)
                self.assertTrue(len(w.viewer._solid["packed"]["mesh_idx"]))
                self.assertFalse(any(name.startswith("__ctx_s") for name in w.viewer._overlays))
                w.sketch_session["state"]["done"] = done
                self.assertTrue(w._finish_sketch(), w.status)
                self.assertEqual(transparent, w.viewer._solid_transparent)
                self.assertEqual(original_mode, w.viewer._transparency)
        finally:
            w.viewer.close()

    def test_editing_profile_hides_downstream_solid_and_restores_it_on_cancel(self):
        from camber.feature_source import ModelDocument
        source = self.model.document.insert("profile", '    profile = part.sketch("xy", constrained=True, name="profile")\n'
                                             '    profile.add_circle((0, 0), 5, name="Circle")\n')
        source = ModelDocument(source).insert("body", '    body = profile.extrude(4, name="body")\n')
        self.assertTrue(self.model.rebuild(source), self.model.error)
        w = _Workspace(self.model)
        w.viewer.window.set_visible(False)
        try:
            final = self.model.packed
            original_eye = w.viewer.camera.eye
            self.assertTrue(len(final["mesh_idx"]))
            w._run_action("edit_sketch", "profile")
            self.assertIsNotNone(w.sketch_session, w.status)
            self.assertFalse(len(w.viewer._solid["packed"]["mesh_idx"]))
            w.viewer._draw_frame()
            self.assertEqual(122, w.ui.docks.rects["Model"][1][3])
            state = w.sketch_session["state"]
            state["done"] = "cancel"
            w._finish_sketch()
            w.viewer._draw_frame()
            self.assertIs(final, w.viewer._solid["packed"])
            self.assertEqual(original_eye, w.viewer.camera.eye)
            self.assertEqual(122, w.ui.docks.rects["Model"][1][3])
            self.assertEqual(source, self.model.source)
        finally:
            w.viewer.close()

    def test_sketch_close_dialog_keeps_live_sketch_on_enter_and_escape(self):
        self.sketch()
        w = _Workspace(self.model)
        w.viewer.window.set_visible(False)
        try:
            w._run_action("edit_sketch", "profile")
            session = w.sketch_session
            original = self.model.source
            key = w.viewer._pyglet.window.key
            for symbol in (key.RETURN, key.ESCAPE):
                w._on_close()
                w.viewer._draw_frame()
                menu = w.ui._menu
                self.assertEqual("Close sketch?", menu.name)
                self.assertEqual("Keep editing", menu.items[menu.nav].label)
                x, y, width, height = menu.rect
                self.assertAlmostEqual(w.viewer.window.width/2, x+width/2)
                self.assertAlmostEqual(w.viewer.window.height/2, y+height/2)
                w.ui.on_key_press(symbol, 0)
                w.ui.on_key_release(symbol, 0)
                w.viewer._draw_frame()
                self.assertFalse(w._sketch_close_requested)
                self.assertIs(session, w.sketch_session)
                self.assertIsNone(session["state"]["done"])
                self.assertEqual(original, self.model.source)
                self.assertFalse(w.viewer._closed)
        finally:
            w.viewer.close()

    def test_failed_downstream_rebuild_keeps_the_live_sketch_and_all_edits(self):
        from camber.feature_source import ModelDocument
        source = self.model.document.insert("profile", '    profile = part.sketch("xy", constrained=True, name="profile")\n'
                                             '    profile.add_circle((0, 0), 5, name="Circle")\n')
        source = ModelDocument(source).insert("body", '    body = profile.extrude(4, name="body")\n')
        self.assertTrue(self.model.rebuild(source), self.model.error)
        w = _Workspace(self.model)
        w.viewer.window.set_visible(False)
        try:
            w._run_action("edit_sketch", "profile")
            session = w.sketch_session
            state = session["state"]
            history = len(self.model.undo_stack)
            # An open profile is valid sketch geometry, but cannot replace this extrusion.
            sketch_ui._undo_pending_or_action(state)
            self.assertTrue(sketch_ui._append_and_solve(state, [
                {"kind": "line", "name": "OpenEdge", "p0": (0, 0), "p1": (10, 0)}]))
            state["done"] = "finish"
            self.assertFalse(w._finish_sketch())
            self.assertIs(session, w.sketch_session)
            self.assertIsNone(state["done"])
            self.assertIsNotNone(state["sketch"])
            self.assertEqual("OpenEdge", state["actions"][0]["name"])
            self.assertEqual(source, self.model.source)
            self.assertEqual(history, len(self.model.undo_stack))
            self.assertIn("Cannot finish sketch", state["status"])
            self.assertTrue(w.viewer._overlays)
            # Repair the profile and finish without reopening or re-entering the work.
            sketch_ui._undo_pending_or_action(state)
            self.assertTrue(sketch_ui._append_and_solve(state, [
                {"kind": "circle", "name": "Circle", "center": (0, 0), "radius": 6}]))
            state["done"] = "finish"
            self.assertTrue(w._finish_sketch(), w.status)
            self.assertIsNone(w.sketch_session)
            self.assertEqual(history+1, len(self.model.undo_stack))
            self.assertIn("6", self.model.document.block("profile").source)
        finally:
            w.viewer.close()

    def test_quick_sketch_keys_work_once_and_focused_fields_do_not_trigger_tools(self):
        self.sketch()
        w = _Workspace(self.model)
        w.viewer.window.set_visible(False)
        try:
            w._run_action("edit_sketch", "profile")
            w.viewer._draw_frame()
            state = w.sketch_session["state"]
            key = w.viewer._pyglet.window.key
            w.ui.on_key_press(key.L, 0)
            w.ui.on_key_release(key.L, 0)
            w.viewer._draw_frame()
            self.assertEqual("line", state["tool"])
            state["pending"] = [(1, 2)]
            w.ui.on_key_press(key.ESCAPE, 0)
            w.ui.on_key_release(key.ESCAPE, 0)
            w.viewer._draw_frame()
            self.assertFalse(state["pending"])
            state["pending"] = [(3, 4)]
            w.viewer._draw_frame()
            self.assertEqual([(3, 4)], state["pending"])
            editor = next(iter(w.ui._editors))
            w.ui._focus = editor
            w.ui._capture()
            w.ui.on_key_press(key.C, 0)
            w.ui.on_key_release(key.C, 0)
            w.viewer._draw_frame()
            self.assertEqual("line", state["tool"])
        finally:
            w.viewer.close()

    def test_sketch_ribbon_keeps_finish_and_view_commands_visible_on_a_laptop(self):
        self.sketch()
        w = _Workspace(self.model)
        w.viewer.window.set_visible(False)
        try:
            w._run_action("edit_sketch", "profile")
            for width in (1920, 1280, 1024):
                w.viewer.window.set_size(width, 900)
                w.viewer._draw_frame()
                panel = w.ui._panels["Model"]
                positions = {args[2]: args[0] for kind, args, owner in w.ui._commands
                             if kind == "text_align" and owner is panel}
                for label in ("Finish sketch", "Cancel sketch", "Undo"):
                    self.assertIn(label, positions)
                    self.assertLess(positions[label][0]+30, width, (width, label))
        finally:
            w.viewer.close()

    def test_new_sketch_uses_the_preselected_face_and_starts_named_picking(self):
        source = self.model.document.insert("body", '    body = part.cuboid((0, 0, 0), (10, 10, 4), name="body")\n')
        self.assertTrue(self.model.rebuild(source), self.model.error)
        w = _Workspace(self.model)
        w.viewer.window.set_visible(False)
        try:
            w._choose_operation("Part.sketch")
            self.assertEqual(("Part.sketch", "plane", "plane"), w._pick_reference)
            w.picked = ["body:body-ExtrudeTop"]
            w._choose_operation("Part.sketch")
            self.assertEqual(w.picked[0], w.fields["plane"])
            self.assertIsNone(w._pick_reference)
            self.assertTrue(w._new_feature(), w.status)
            self.assertAlmostEqual(4, w.sketch_session["state"]["frame"].origin.z)
        finally:
            w.viewer.close()

    def test_first_extrusion_returns_to_a_fitted_three_dimensional_view(self):
        source = self.model.document.insert("profile", '    profile = part.sketch("xy", constrained=True, name="profile")\n'
                                             '    profile.add_circle((0, 0), 5, name="Circle")\n')
        self.assertTrue(self.model.rebuild(source), self.model.error)
        w = _Workspace(self.model)
        w.viewer.window.set_visible(False)
        try:
            w.viewer.camera.eye, w.viewer.camera.center = (0, 0, 20), (0, 0, 0)
            w._choose_operation("Sketch.extrude")
            w.fields["height"] = "4"
            w._new_feature()
            self.assertIsNone(w.pending, w.status)
            self.assertNotEqual(0, w.viewer.camera.eye[0])
            self.assertAlmostEqual(2, w.viewer.camera.center[2])
        finally:
            w.viewer.close()

    def test_sketch_tree_and_finish_restore_part_context_and_named_placement(self):
        from camber.feature_source import ModelDocument
        source = self.model.document.insert("body", '    body = part.cuboid((0, 0, 0), (10, 10, 4), name="body")\n')
        source = ModelDocument(source).insert("profile", '    profile = part.sketch(frame=body.face_frame("body:body-ExtrudeTop"), constrained=True, name="profile")\n'
                                             '    profile.add_line((0, 0), (4, 0), name="Edge")\n'
                                             '    profile.horizontal("Edge")\n    profile.length("Edge", 4)\n')
        self.assertTrue(self.model.rebuild(source), self.model.error)
        w = _Workspace(self.model)
        w.viewer.window.set_visible(False)
        try:
            w._run_action("edit_sketch", "profile")
            self.assertIsNotNone(w.sketch_session, w.status)
            self.assertTrue(w.ui.docks.visible["Features"])
            for panel in ("Document", "Model", "Properties", "Status"):
                self.assertTrue(w.ui.docks.visible[panel])
            bounds = w.sketch_session["bounds"]
            center = w.viewer.camera.center
            frame = w.sketch_session["state"]["frame"]
            expected = sketch_ui._uv_to_world(((bounds[0]+bounds[1])*0.5, (bounds[2]+bounds[3])*0.5), frame, 0)
            for actual, wanted in zip(center, expected):
                self.assertAlmostEqual(wanted, actual)
            w.viewer._draw_frame()
            texts = [args[2] for kind, args, panel in w.ui._commands if kind == "text_align"]
            for group in ("Curves (1)", "Constraints (1)", "Dimensions (1)"):
                self.assertIn(group, texts)
            for command in ("Finish sketch", "Cancel sketch", "Horizontal", "Circle"):
                self.assertIn(command, texts)
            self.assertNotIn("Sketch", w.ui._panels)
            state = w.sketch_session["state"]
            sketch_ui.select_tree_action(state, 2, edit=True)
            state["dimension"]["value"] = 6
            sketch_ui._commit_dimension(state)
            state["done"] = "finish"
            w._finish_sketch()
            self.assertIsNone(w.sketch_session)
            self.assertTrue(w.ui.docks.visible["Model"])
            self.assertIn("body.face_frame", w.model.document.block("profile").source)
            self.assertEqual(2, w.model.values["profile"].constraint_count)
        finally:
            w.viewer.close()


class FilletPreviewTests(_SketchFixture):
    def test_preview_edges_stay_editable_and_cancel_restores_the_model(self):
        import ast
        from camber.gui_testing import GuiDriver
        source = self.model.document.insert("body",
            '    body = part.cuboid((0, 0, 0), (20, 30, 10), name="body")\n')
        self.assertTrue(self.model.rebuild(source), self.model.error)
        w = _Workspace(self.model)
        w.viewer.window.set_visible(False)
        try:
            w._choose_operation("Solid.fillet")
            lines = [name for name, kind in w.model.packed["entity_types"].items() if kind == "lin"]
            edge, other = lines[0], lines[1]
            w.fields["edges"] = repr((edge,))
            w.fields["radius"] = "2"
            w._pick_reference = ("Solid.fillet", "edges", "edges")
            driver = GuiDriver(w.ui, w.viewer._draw_frame)
            driver.click("Preview", panel="Properties")
            self.assertTrue(w._draft_preview, w.status)
            self.assertTrue(any(name.startswith("fillet1:") for name in w.viewer._solid["packed"]["entity_types"]))
            preview_other = next(name for name in w.viewer._solid["packed"]["entity_types"]
                                 if name.split(":", 1)[-1] == other.split(":", 1)[-1])
            blend = next(name for name, kind in w.viewer._solid["packed"]["entity_types"].items()
                         if kind != "lin" and "BlendEdge_" in name)
            available = w._available_values(None)
            self.assertTrue(w._apply_entity_pick(preview_other, available), w.status)
            selected = ast.literal_eval(w.fields["edges"])
            self.assertIn(edge, selected)
            self.assertIn(other, selected)
            self.assertTrue(w._draft_preview)
            self.assertTrue(w._apply_entity_pick(preview_other, available), w.status)
            self.assertNotIn(other, ast.literal_eval(w.fields["edges"]))
            self.assertTrue(w._apply_entity_pick(blend, available), w.status)
            self.assertNotIn(edge, ast.literal_eval(w.fields["edges"]))
            driver.click("Cancel", panel="Properties")
            self.assertIsNone(w.pending)
            self.assertFalse(w._draft_preview)
            self.assertIs(w.model.packed, w.viewer._solid["packed"])
            self.assertIn(edge, w.viewer._solid["packed"]["entity_types"])
        finally:
            w.viewer.close()
