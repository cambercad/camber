"""Undo/redo exercised by window events, without authoring geometry in Python."""
import os
from pathlib import Path
import tempfile
import unittest
from camber import set_progress_log
from camber.gui_testing import GuiDriver
from camber.modeling import ModelSession, _Workspace
from camber.modeling_commands import operations


@unittest.skipUnless(os.environ.get('CAMBER_TEST_GL') == '1', 'requires OpenGL')
class HistoryWorkflowTests(unittest.TestCase):
    def setUp(self):
        set_progress_log(False)
        self.folder = tempfile.TemporaryDirectory(dir=Path(__file__).parent)
        self.model = ModelSession(Path(self.folder.name) / 'housing.py')
        self.assertTrue(self.model.rebuild(), self.model.error)
        self.workspace = _Workspace(self.model)
        self.workspace.viewer.window.set_visible(False)
        self.driver = GuiDriver(self.workspace.ui, self.workspace.viewer._draw_frame)

    def tearDown(self):
        self.workspace.viewer.close()
        self.folder.cleanup()

    def enter(self, label, value):
        self.driver.click(label, panel='Properties')
        self.driver.text(str(value), replace=True)

    def operation(self, key):
        d = self.driver
        query = operations()[key].label
        d.click('Operations##tab', panel='Model')
        d.click('Find operation', panel='Model')
        d.text(query, replace=True)
        entries = sorted((op.owner.__name__+' · '+op.group+' · '+op.label, name)
                         for name, op in operations().items()
                         if query.casefold() in (name+' '+op.label+' '+op.group).casefold())
        index = next(i for i, (_, name) in enumerate(entries) if name == key)
        d.click('Operation', panel='Model')
        d.key('HOME')
        for _ in range(index):
            d.key('DOWN')
        d.key('RETURN')
        d.click('Set up', panel='Model')
        self.assertEqual(key, self.workspace.pending.key)

    def point(self, x, y):
        ndc = self.workspace.viewer.camera.project((x,y,0))
        self.driver.click(x=(ndc[0]+1)*self.driver.window.width/2,
                          y=(1-ndc[1])*self.driver.window.height/2)

    def test_properties_callback_failure_does_not_leak_gui_state(self):
        from unittest.mock import patch
        ui = self.workspace.ui
        ui.new_frame()
        with patch.object(self.workspace, '_field', side_effect=ValueError('field failed')):
            with self.assertRaisesRegex(ValueError, 'field failed'):
                self.workspace._properties()
        self.assertIsNone(ui._current)
        self.assertFalse(ui._styles)
        self.assertFalse(ui._colors)
        ui.new_frame()
        self.workspace._properties()
        ui.render()

    def test_part_parameter_delete_history_and_redo_branch(self):
        d = self.driver
        d.click('Add parameter', panel='Features')
        self.enter('Feature name', 'shaft_radius')
        self.enter('value##named_parameter', 10)
        d.click('Create feature', panel='Properties')
        self.operation('Part.cylinder')
        self.enter('origin##Part.cylinder##X', 0)
        self.enter('origin##Part.cylinder##Y', 0)
        self.enter('origin##Part.cylinder##Z', 0)
        self.enter('radius##Part.cylinder', 'shaft_radius')
        self.enter('height##Part.cylinder', 20)
        d.click('Create feature', panel='Properties')
        self.assertIsNone(self.workspace.pending, self.workspace.form_build_error)
        original = self.model.source
        volume = self.model.displayed.volume()
        d.click('##undo', panel='Document')
        self.assertNotIn('cylinder1', self.model.values)
        d.click('##redo', panel='Document')
        self.assertEqual(original, self.model.source)
        self.assertAlmostEqual(volume, self.model.displayed.volume())
        self.enter('height##Part.cylinder', 30)
        from pyglet.window import key
        d.key('Z', modifiers=key.MOD_CTRL)
        self.assertEqual('20', self.workspace.fields['height'])
        self.assertEqual(original, self.model.source)
        d.key('Y', modifiers=key.MOD_CTRL)
        self.assertEqual('30', self.workspace.fields['height'])
        d.click('Apply', panel='Properties')
        d.click('##undo', panel='Document')
        self.assertAlmostEqual(volume, self.model.displayed.volume())
        d.click('##redo', panel='Document')
        self.assertAlmostEqual(1.5*volume, self.model.displayed.volume())
        d.click('##undo', panel='Document')
        self.enter('height##Part.cylinder', 25)
        d.click('Apply', panel='Properties')
        self.assertFalse(self.model.redo_stack)
        before_delete = self.model.source
        parameter_button = d.control('Add parameter', panel='Features')
        d.click(x=parameter_button.rect[0]+60, y=parameter_button.rect[1]-12)
        d.click('Delete parameter', panel='Properties')
        d.click('Cancel')
        self.assertEqual(before_delete, self.model.source)
        d.click('Delete parameter', panel='Properties')
        d.click('Replace references and delete')
        deleted = self.model.source
        self.assertNotIn('shaft_radius', self.model.values)
        d.click('##undo', panel='Document')
        self.assertEqual(before_delete, self.model.source)
        d.click('##redo', panel='Document')
        self.assertEqual(deleted, self.model.source)
        from pyglet.window import key
        d.key('S', modifiers=key.MOD_CTRL)
        self.assertFalse(self.model.dirty)
        d.click('##undo', panel='Document')
        self.assertTrue(self.model.dirty)
        d.click('##redo', panel='Document')
        self.assertFalse(self.model.dirty)

    def test_assembly_occurrence_placement_undo_redo(self):
        d = self.driver
        self.operation('Part.cylinder')
        for axis in 'XYZ':
            self.enter('origin##Part.cylinder##'+axis, 0)
        self.enter('radius##Part.cylinder', 8)
        self.enter('height##Part.cylinder', 15)
        d.click('Create feature', panel='Properties')
        self.operation('Part.assembly')
        d.click('Create feature', panel='Properties')
        self.operation('Assembly.add_part')
        d.click('solid##Assembly.add_part', panel='Properties')
        d.key('HOME')
        d.key('RETURN')
        d.click('Create feature', panel='Properties')
        self.assertIsNone(self.workspace.pending, self.workspace.form_build_error)
        instance = self.model.document.blocks[-1].result
        original = self.model.source
        self.enter('position##Assembly.add_part##X', 20)
        d.click('Apply', panel='Properties')
        self.assertAlmostEqual(20, self.model.values[instance].pose[0])
        placed = self.model.source
        d.click('##undo', panel='Document')
        self.assertEqual(original, self.model.source)
        self.assertAlmostEqual(0, self.model.values[instance].pose[0])
        d.click('##redo', panel='Document')
        self.assertEqual(placed, self.model.source)
        self.assertAlmostEqual(20, self.model.values[instance].pose[0])
        self.operation('Assembly.fix')
        d.click('part##Assembly.fix', panel='Properties')
        d.key('HOME')
        d.key('RETURN')
        d.click('Create mate', panel='Properties')
        self.assertEqual(1, len(self.model.displayed.constraints))
        d.click('##undo', panel='Document')
        self.assertEqual(0, len(self.model.displayed.constraints))
        d.click('##redo', panel='Document')
        self.assertEqual(1, len(self.model.displayed.constraints))

    def test_sketch_curve_undo_and_redo_through_keyboard(self):
        d = self.driver
        d.click('Create sketch', panel='Properties')
        d.click('Create sketch', panel='Properties')
        d.scroll(900, 600, -2)
        d.click('Rectangle', panel='Model')
        self.point(-20,-10)
        self.point(20,10)
        d.key('ESCAPE')
        state = self.workspace.sketch_session['state']
        self.assertEqual(4, state['sketch'].curve_count)
        from pyglet.window import key
        d.move(900,600)
        d.key('Z', modifiers=key.MOD_CTRL)
        self.assertEqual(0, state['sketch'].curve_count)
        d.click('##redo', panel='Document')
        self.assertEqual(4, state['sketch'].curve_count)
        d.click('Circle', panel='Model')
        self.point(0,0)
        self.point(6,0)
        d.key('ESCAPE')
        constraints = state['sketch'].constraint_count
        d.click('Radius', panel='Model')
        self.point(6,0)
        self.enter('##sk_dimension', 6)
        d.click('Apply##sk_act_dimension_apply', panel='Properties')
        constrained = state['sketch'].constraint_count
        self.assertGreater(constrained, constraints)
        d.key('ESCAPE')
        d.move(900,600)
        d.key('Z', modifiers=key.MOD_CTRL)
        self.assertEqual(constraints, state['sketch'].constraint_count)
        d.click('Redo', panel='Model')
        self.assertEqual(constrained, state['sketch'].constraint_count)
        d.click('Finish sketch', panel='Model')
        source = self.model.source
        d.click('##undo', panel='Document')
        d.click('##redo', panel='Document')
        self.assertEqual(source, self.model.source)
        d.click('Extrude', panel='Properties')
        self.enter('height##Sketch.extrude', 12)
        d.click('Create feature', panel='Properties')
        self.assertTrue(self.model.displayed.is_watertight())
        volume = self.model.displayed.volume()
        solid_source = self.model.source
        d.click('##undo', panel='Document')
        self.assertEqual(source, self.model.source)
        d.click('##redo', panel='Document')
        self.assertEqual(solid_source, self.model.source)
        self.assertAlmostEqual(volume, self.model.displayed.volume(), places=6)

    def test_sketch_drag_and_construction_are_undoable(self):
        from copy import deepcopy
        from pyglet.window import key
        d = self.driver
        d.click('Create sketch', panel='Properties')
        d.click('Create sketch', panel='Properties')
        d.scroll(900,600,-2)
        d.click('Line', panel='Model')
        self.point(-10,0)
        self.point(10,0)
        d.key('ESCAPE')
        d.click('Select', panel='Model')
        state = self.workspace.sketch_session['state']
        before = deepcopy(state['actions'])
        def screen(x,y):
            ndc = self.workspace.viewer.camera.project((x,y,0))
            return ((ndc[0]+1)*d.window.width/2, (1-ndc[1])*d.window.height/2)
        d.drag(screen(10,0), screen(16,8), steps=3)
        dragged = deepcopy(state['actions'])
        self.assertNotEqual(before, dragged)
        d.move(900,600)
        d.key('Z', modifiers=key.MOD_CTRL)
        self.assertEqual(before, state['actions'])
        d.key('Y', modifiers=key.MOD_CTRL)
        self.assertEqual(dragged, state['actions'])
        d.click('Line1##sketch_action_0', panel='Features', count=2)
        self.assertIsNotNone(state.get('curve_edit'), state['status'])
        self.enter('Coordinate##curve_editor_0', 18)
        self.enter('Coordinate##curve_editor_1', 6)
        d.click('Apply##curve_editor', panel='Properties')
        edited = deepcopy(state['actions'])
        self.assertNotEqual(dragged, edited)
        d.move(900,600)
        d.key('Z', modifiers=key.MOD_CTRL)
        self.assertEqual(dragged, state['actions'])
        d.key('Z', modifiers=key.MOD_CTRL | key.MOD_SHIFT)
        self.assertEqual(edited, state['actions'])
        line = next(a for a in state['actions'] if a['kind'] == 'line')
        self.point(*line['p0'])
        d.key('X')
        self.assertTrue(next(a for a in state['actions'] if a['kind']=='line').get('construction'))
        self.assertFalse(state['history'].can_redo)
        d.key('Z', modifiers=key.MOD_CTRL)
        self.assertFalse(next(a for a in state['actions'] if a['kind']=='line').get('construction'))
        d.key('Y', modifiers=key.MOD_CTRL)
        self.assertTrue(next(a for a in state['actions'] if a['kind']=='line').get('construction'))

