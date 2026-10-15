"""Create and edit a component with real window events and the generated form."""
import os
import unittest
import test_modeling_history_workflow as workflow
from examples.architecture.roofs import roof_component
from camber.modeling import ModelSession


@unittest.skipUnless(os.environ.get('CAMBER_TEST_GL') == '1', 'requires OpenGL')
class BuildingGuiTests(unittest.TestCase):
    setUp = workflow.HistoryWorkflowTests.setUp
    tearDown = workflow.HistoryWorkflowTests.tearDown
    enter = workflow.HistoryWorkflowTests.enter
    operation = workflow.HistoryWorkflowTests.operation

    def test_create_validate_edit_undo_save_and_reload_roof(self):
        from pyglet.window import key
        d, w = self.driver, self.workspace
        self.operation('roof_component')
        for field, value in (('width', 1200), ('length', 1800), ('eave_height', 1000), ('overhang', 100)):
            self.enter(field+'##roof_component', value)
        d.click('Create feature', panel='Properties')
        self.assertIsNone(w.pending, w.form_build_error)
        ndc = w.viewer.camera.project((600, 900, 1200))
        self.assertTrue(all(-1 < coordinate < 1 for coordinate in ndc[:2]), ndc)
        original = self.model.source
        self.enter('pitch##roof_component', 0)
        self.assertFalse(d.control('Apply', panel='Properties').enabled)
        self.assertEqual(original, self.model.source)
        self.enter('pitch##roof_component', 40)
        d.click('More options', panel='Properties')
        d.click('shape##roof_component', panel='Properties')
        d.key('HOME')
        d.key('DOWN')
        d.key('RETURN')
        d.click('Apply', panel='Properties')
        changed = self.model.source
        self.assertNotEqual(original, changed)
        self.assertEqual(4, len(self.model.displayed.leaves()))
        d.click('##undo', panel='Document')
        self.assertEqual(original, self.model.source)
        d.click('##redo', panel='Document')
        self.assertEqual(changed, self.model.source)
        d.move(900, 500)
        d.key('S', modifiers=key.MOD_CTRL)
        self.assertFalse(self.model.dirty)
        reopened = ModelSession(self.model.path)
        self.assertTrue(reopened.rebuild(), reopened.error)
        self.assertEqual(changed, reopened.source)
        self.assertEqual(4, len(reopened.displayed.leaves()))
