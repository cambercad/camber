"""Document parameters and registered components share the ordinary feature editor."""
import unittest
import os
from camber.feature_source import ModelDocument
from camber.modeling import _TEMPLATE, _parameter_form, _parameter_source, _run_model
from camber.modeling_commands import operations
from examples.architecture.windows import window_component


class ParameterDocumentTests(unittest.TestCase):
    def document(self):
        source = ModelDocument(_TEMPLATE).insert("width", "    width = named_parameter(value=900, unit='mm')")
        source = ModelDocument(source).insert("opening", "    opening = named_parameter(value=width / 2)")
        source = ModelDocument(source).with_imports({"named_parameter"})
        source = ModelDocument(source).insert("window", "    window = window_component(part=part, name='window', width=width, height=1100, leaves=1)")
        source = ModelDocument(source).with_imports({"window_component"})
        return ModelDocument(ModelDocument(source).display_result("window"))

    def test_parameters_round_trip_expressions_and_dependencies(self):
        document = self.document()
        self.assertTrue(all(block.editable for block in document.blocks))
        self.assertEqual('width / 2', dict(_parameter_form(document.block('opening')))['value'])
        self.assertIn('window', document.dependents('width'))
        extra = ModelDocument(document.insert('frame_clearance', '    frame_clearance = named_parameter(value=width / 10)'))
        self.assertLess(extra.block('frame_clearance').start, extra.block('window').start)
        with self.assertRaisesRegex(ValueError, 'Used by'):
            document.delete('width')
        command = operations()['named_parameter']
        self.assertTrue(command.field_errors({'value':'missing + 2'}, {}))
        self.assertTrue(command.field_errors({'value':'1 / 0'}, {}))

    def test_generator_validation_and_edit_preserve_parameter_references(self):
        document = self.document()
        fields = dict(_parameter_form(document.block('window')))
        self.assertEqual('width', fields['width'])
        fields['height'] = '1250'
        body = _parameter_source(document.block('window'), fields, {'part':object(), 'width':900})
        self.assertIn('width=width', body)
        self.assertIn('height=1250', body)
        fields['leaves'] = '20'
        from camber import Part
        errors = operations()['window_component'].field_errors(fields, {'part':Part((-2000,-2000,-500),(3000,3000,3000)), 'width':900})
        self.assertIn('__parameters', errors)

    def test_delete_parameter_freezes_all_load_references_without_changing_text(self):
        document = self.document()
        source = document.source
        source = source.replace("unit='mm'", "unit='mm', description='width stays in this string'")
        source = source.replace("width=width", "width=width + width / 2")
        document = ModelDocument(source)
        result = document.delete_parameter('width', 900)
        self.assertNotIn('camber:begin width', result)
        self.assertIn('value=900 / 2', result)
        self.assertIn('width=900 + 900 / 2', result)
        self.assertFalse(ModelDocument(result).parameter_references('opening'))

    def test_parameter_deletion_preserves_unicode_and_rejects_shadowed_scopes(self):
        document = self.document()
        source = document.source.replace('value=width / 2', "value=width / 2, description='maß width'")
        result = ModelDocument(source).delete_parameter('width', 900)
        self.assertIn("description='maß width'", result)
        nested = document.source.replace('    return part, window', '    def custom(width):\n        return width\n    return part, window')
        with self.assertRaisesRegex(ValueError, 'custom Python scopes'):
            ModelDocument(nested).delete_parameter('width', 900)

    def test_delete_dialog_cancel_and_confirm_are_explicit(self):
        from types import SimpleNamespace
        from camber.modeling import _Workspace
        workspace = object.__new__(_Workspace)
        document = self.document()
        workspace.model = SimpleNamespace(document=document, values={'width':900})
        dialogs, applied = [], []
        workspace.ui = SimpleNamespace(OpenMessageBox=lambda *args, **kwargs: dialogs.append((args, kwargs)),
                                       MessageBox=lambda name: 'Cancel')
        workspace._apply = lambda source: applied.append(source) or False
        workspace._delete_feature('width')
        self.assertEqual('Cancel', dialogs[0][1]['default'])
        self.assertIn('900', dialogs[0][0][1])
        workspace._parameter_delete_dialog()
        self.assertFalse(applied)
        workspace._delete_feature('width')
        workspace.ui.MessageBox = lambda name: 'Replace references and delete'
        workspace._parameter_delete_dialog()
        self.assertEqual(1, len(applied))
        self.assertNotIn('camber:begin width', applied[0])
        self.assertIn('width=900', applied[0])

    def test_parameter_change_rebuilds_component(self):
        from pathlib import Path
        document = self.document()
        part, displayed, packed, values = _run_model(document, Path('parameter_test.py'))
        self.assertEqual(450, values['opening'])
        self.assertIs(displayed, values['window'])
        fields = dict(_parameter_form(document.block('width')))
        fields['value'] = '1200'
        updated = ModelDocument(document.replace('width', _parameter_source(document.block('width'), fields)))
        _, displayed, _, values = _run_model(updated, Path('parameter_test.py'))
        self.assertEqual(600, values['opening'])
        self.assertTrue(displayed.solve().converged)

    def test_module_document_save_reload_and_edit(self):
        from pathlib import Path
        path = Path(__file__).parents[1] / 'examples/architecture/parametric_openings.py'
        document = ModelDocument(path.read_text(encoding='utf-8'))
        self.assertTrue(document.module_level)
        self.assertTrue(all(b.editable for b in document.blocks))
        _, displayed, _, values = _run_model(document, path)
        self.assertEqual(1800, values['window_width'])
        _, _, _, prefix = _run_model(document, path, before=document.block('window').start)
        self.assertEqual(1800, prefix['window_width'])
        self.assertNotIn('window', prefix)
        fields = dict(_parameter_form(document.block('window_width')))
        fields['value'] = '2000'
        source = document.replace('window_width', _parameter_source(document.block('window_width'), fields))
        source = ModelDocument(source).insert('half_width', '    half_width = named_parameter(value=window_width / 2)')
        document = ModelDocument(source)
        _, displayed, _, values = _run_model(document, path)
        self.assertEqual(1000, values['half_width'])
        self.assertTrue(displayed.solve().converged)

    @unittest.skipUnless(os.environ.get('CAMBER_TEST_GL') == '1', 'requires OpenGL')
    def test_gui_parameter_and_generator_editors_apply_and_reject_invalid_sizes(self):
        import tempfile
        from pathlib import Path
        from camber.modeling import ModelSession, _Workspace
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as folder:
            path = Path(folder) / 'openings.py'
            path.write_text(self.document().source, encoding='utf-8')
            session = ModelSession(path)
            self.assertTrue(session.rebuild(), session.error)
            workspace = _Workspace(session)
            workspace.viewer.window.set_visible(False)
            try:
                workspace._select('width')
                workspace.viewer._draw_frame()
                workspace.fields['value'] = '1000'
                workspace._commit_edits()
                self.assertEqual(1000, session.values['width'])
                workspace._select('window')
                workspace.viewer._draw_frame()
                self.assertEqual('width', workspace.fields['width'])
                workspace.fields['leaves'] = '40'
                self.assertTrue(workspace._form_errors(operations()['window_component'], workspace._available_values(session.document.block('window').start), 'window'))
                previous = session.displayed
                workspace._commit_edits()
                self.assertIs(previous, session.displayed)
                session.save()
                loaded = ModelSession(path)
                self.assertTrue(loaded.rebuild(), loaded.error)
                self.assertEqual(1000, loaded.values['width'])
            finally:
                workspace.viewer.close()


if __name__ == '__main__':
    unittest.main()
