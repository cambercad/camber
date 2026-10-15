"""GUI contracts and history transactions, independently of Camber's CAD code."""
import importlib.util
import os
from pathlib import Path
import unittest


def load(name):
    path = Path(__file__).parents[1] / 'camber' / (name+'.py')
    spec = importlib.util.spec_from_file_location('standalone_safety_'+name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


History = load('history').History


class HistoryContractTests(unittest.TestCase):
    def test_failed_and_raised_restores_leave_both_stacks_unchanged(self):
        history = History()
        history.record('first', 'second')
        self.assertFalse(history.step('undo', 'second', lambda target: False))
        self.assertEqual(('first',), history.undo_stack)
        self.assertFalse(history.can_redo)
        def fail(target):
            raise ValueError('restore failed')
        with self.assertRaisesRegex(ValueError, 'restore failed'):
            history.step('undo', 'second', fail)
        self.assertEqual(('first',), history.undo_stack)
        self.assertFalse(history.can_redo)

    def test_branching_and_noop_edits_keep_the_correct_redo_revision(self):
        history = History()
        history.record(1, 2)
        self.assertTrue(history.step('undo', 2, lambda target: True))
        history.record(1, 1)
        self.assertTrue(history.can_redo)
        history.record(1, 3)
        self.assertFalse(history.can_redo)
        self.assertEqual((1,), history.undo_stack)

    def test_mutable_snapshots_are_isolated_from_callers_and_failed_callbacks(self):
        history = History()
        before = {'values':[1]}
        history.record(before, {'values':[2]})
        before['values'].append(3)
        history.undo_stack[0]['values'].append(4)
        def fail(target):
            target['values'].append(5)
            return False
        history.step('undo', {'values':[2]}, fail)
        restored = []
        self.assertTrue(history.step('undo', {'values':[2]}, lambda target: restored.append(target) or True))
        self.assertEqual([{'values':[1]}], restored)
        restored[0]['values'].append(6)
        self.assertEqual(({'values':[2]},), history.redo_stack)

    def test_reentrancy_and_missing_callback_results_are_errors(self):
        history = History()
        history.record(1,2)
        with self.assertRaisesRegex(RuntimeError, 'while restoring'):
            history.step('undo', 2, lambda target: history.record(2,3))
        with self.assertRaisesRegex(TypeError, 'True or False'):
            history.step('undo', 2, lambda target: None)
        with self.assertRaises(ValueError):
            history.step('sideways', 2, lambda target: True)
        with self.assertRaisesRegex(RuntimeError, 'while restoring'):
            history.transformed(transform=lambda target: history.record(2,3))
        self.assertEqual((1,), history.undo_stack)
        self.assertTrue(history.step('undo',2,lambda target: True))

    def test_save_as_transforms_independent_copies_of_both_stacks(self):
        history = History()
        history.record('old/a','old/b')
        history.record('old/b','old/c')
        history.step('undo','old/c',lambda target: True)
        copy = history.transformed(transform=lambda value: value.replace('old/','new/'))
        self.assertEqual(('new/a',),copy.undo_stack)
        self.assertEqual(('new/c',),copy.redo_stack)
        self.assertEqual(('old/a',),history.undo_stack)


@unittest.skipUnless(os.environ.get('CAMBER_TEST_GL') == '1', 'requires OpenGL')
class GuiScopeTests(unittest.TestCase):
    def setUp(self):
        import pyglet
        self.window = pyglet.window.Window(width=600,height=400,visible=False)
        self.ui = load('gui').UI(self.window)
        self.ui.new_frame()

    def tearDown(self):
        self.ui.shutdown()
        self.window.close()

    def test_nested_scopes_restore_state_after_an_exception(self):
        ui = self.ui
        with self.assertRaisesRegex(ValueError,'control failed'):
            with ui.style(variables={0:(12,8)}, colors={0:(1,0,0,1)}):
                with ui.panel('Form') as visible:
                    self.assertTrue(visible)
                    with ui.child('Fields',height=100):
                        ui.Text('Visible before failure')
                        raise ValueError('control failed')
        self.assertIsNone(ui._current)
        self.assertFalse(ui._styles)
        self.assertFalse(ui._colors)
        with ui.panel('Recovered'):
            ui.Button('Apply')
        ui.render()
        ui.new_frame()

    def test_zero_pop_is_a_noop_and_underflow_does_not_modify_styles(self):
        ui = self.ui
        ui.PushStyleVar(0,(8,8))
        ui.PopStyleVar(0)
        self.assertEqual(1,len(ui._styles))
        with self.assertRaises(RuntimeError):
            ui.PopStyleVar(2)
        with self.assertRaises(ValueError):
            ui.PopStyleVar(-1)
        self.assertEqual(1,len(ui._styles))
        ui.PopStyleVar()
        with self.assertRaises(RuntimeError):
            ui.PopItemWidth()

    def test_incorrect_panel_pairing_and_leaked_styles_fail_clearly(self):
        ui = self.ui
        ui.Begin('Form')
        with self.assertRaisesRegex(RuntimeError,'current panel'):
            ui.Begin('Accidental second panel')
        ui.BeginChild('Fields',height=100)
        with self.assertRaisesRegex(RuntimeError,'EndChild'):
            ui.End()
        ui.EndChild()
        ui.End()
        ui.PushStyleColor(0,(1,0,0,1))
        with self.assertRaisesRegex(RuntimeError,'scopes'):
            ui.render()
        ui.PopStyleColor()
        ui.render()

    def test_tab_selection_and_layout_queries_use_public_api(self):
        ui = self.ui
        with self.assertRaisesRegex(RuntimeError, 'panel'):
            ui.SetSelectedTab('Modes','Second')
        with ui.panel('Form'):
            ui.SetSelectedTab('Modes','Second')
            ui.SetSelectedTab('Modes','First',default=True)
            ui.BeginTabBar('Modes')
            self.assertFalse(ui.BeginTabItem('First'))
            self.assertTrue(ui.BeginTabItem('Second'))
            ui.Button('Apply',width=90)
            self.assertEqual(90,ui.GetItemRect()[2])
            self.assertEqual((8,6),ui.GetItemSpacing())
            ui.EndTabItem()
            ui.EndTabBar()
