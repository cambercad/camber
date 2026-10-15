"""Inspection through the modeler's ribbon, task pane and viewport events."""
import os
import unittest

import test_modeling_history_workflow as workflow


@unittest.skipUnless(os.environ.get('CAMBER_TEST_GL') == '1', 'requires OpenGL')
class ModelingInspectionTests(unittest.TestCase):
    setUp = workflow.HistoryWorkflowTests.setUp
    tearDown = workflow.HistoryWorkflowTests.tearDown
    enter = workflow.HistoryWorkflowTests.enter
    operation = workflow.HistoryWorkflowTests.operation

    def cylinder(self):
        self.operation('Part.cylinder')
        for axis in 'XYZ':
            self.enter('origin##Part.cylinder##'+axis, 0)
        self.enter('radius##Part.cylinder', 10)
        self.enter('height##Part.cylinder', 20)
        self.driver.click('Create feature', panel='Properties')
        self.assertIsNone(self.workspace.pending, self.workspace.form_build_error)
        self.driver.click('View##tab', panel='Model')

    def pick(self, point, *, add=False):
        from pyglet.window import key
        ndc = self.workspace.viewer.camera.project(point)
        self.driver.click(x=(ndc[0]+1)*self.driver.window.width/2,
                          y=(1-ndc[1])*self.driver.window.height/2,
                          modifiers=key.MOD_CTRL if add else 0)

    def test_measure_viewport_selection_and_rebuild(self):
        self.cylinder()
        d, w = self.driver, self.workspace
        source, undo = self.model.source, len(self.model.undo_stack)
        d.click('Measure', panel='Model')
        self.pick((10, 0, 20))
        self.pick((10, 0, 0), add=True)
        self.assertEqual(2, len(w.picked), w.picked)
        self.assertEqual(1, len(w.viewer.inspection.measurements), w.viewer.inspection.error)
        self.assertAlmostEqual(20, w.viewer.inspection.measurements[-1].length, places=5)
        self.assertEqual(source, self.model.source)
        self.assertEqual(undo, len(self.model.undo_stack))
        d.click('Done', panel='Properties')
        self.enter('height##Part.cylinder', 30)
        d.click('Apply', panel='Properties')
        self.assertEqual([], w.viewer.inspection.measurements)
        d.click('Measure', panel='Model')
        self.pick((10, 0, 30))
        self.pick((10, 0, 0), add=True)
        self.assertAlmostEqual(30, w.viewer.inspection.measurements[-1].length, places=5)
        d.click('Clear measurements', panel='Properties')
        self.assertEqual([], w.viewer.inspection.measurements)
        d.click('Cylinder1', panel='Features')
        self.assertIsNone(w._inspection_mode)
        self.assertIsNotNone(d.control('height##Part.cylinder', panel='Properties'))

    def test_section_controls_and_return_to_modeling(self):
        self.cylinder()
        d, w = self.driver, self.workspace
        source, undo = self.model.source, len(self.model.undo_stack)
        d.click('Section view', panel='Model')
        self.assertEqual(((0, 0, 1), 10), w.viewer.section_plane)
        d.click('Keep opposite side', panel='Properties')
        self.assertEqual(((0, 0, -1), -10), w.viewer.section_plane)
        self.enter('##section_position_value', 3)
        self.assertEqual(((0, 0, -1), -13), w.viewer.section_plane)
        d.click('Fill cut faces (closed solids)', panel='Properties')
        self.assertFalse(w.viewer.section_fill)
        d.click('Done', panel='Properties')
        self.assertIsNotNone(w.viewer.section_plane)
        self.assertEqual(source, self.model.source)
        self.assertEqual(undo, len(self.model.undo_stack))
        d.click('Section view', panel='Model')
        d.click('Enable cross section', panel='Properties')
        self.assertIsNone(w.viewer.section_plane)
        d.click('Enable cross section', panel='Properties')
        self.assertEqual(((0, 0, -1), -13), w.viewer.section_plane)
        self.operation('Part.cylinder')
        self.assertIsNone(w._inspection_mode)
        self.assertIsNotNone(d.control('radius##Part.cylinder', panel='Properties'))

    def test_section_face_reference_and_sketch_suspension(self):
        self.cylinder()
        d, w = self.driver, self.workspace
        d.click('Top', panel='Model')
        d.click('Section view', panel='Model')
        d.click('Use selected face', panel='Properties')
        self.assertEqual('Select one planar face first.', w.viewer.inspection.section_error)
        d.click('Enable cross section', panel='Properties')
        self.pick((3, 2, 20))
        self.assertEqual(1, len(w.picked))
        d.click('Enable cross section', panel='Properties')
        d.click('Use selected face', panel='Properties')
        self.assertFalse(w.viewer.inspection.section_error)
        normal, offset = w.viewer.section_plane
        self.assertAlmostEqual(1, normal[2])
        self.assertAlmostEqual(20, offset)
        d.click('Done', panel='Properties')
        self.operation('Part.sketch')
        d.click('Create sketch', panel='Properties')
        self.assertIsNotNone(w.sketch_session, w.status)
        self.assertIsNone(w.viewer.section_plane)
        d.click('Finish sketch', panel='Model')
        self.assertIsNone(w.sketch_session)
        self.assertEqual((normal, offset), w.viewer.section_plane)

    def test_assembly_occurrences_measure_and_supply_section_plane(self):
        self.cylinder()
        d, w = self.driver, self.workspace
        self.operation('Part.assembly')
        d.click('Create feature', panel='Properties')
        for x in (0, 30):
            self.operation('Assembly.add_part')
            d.click('solid##Assembly.add_part', panel='Properties')
            d.key('HOME')
            d.key('RETURN')
            self.enter('position##Assembly.add_part##X', x)
            d.click('Create feature', panel='Properties')
            self.assertIsNone(w.pending, w.form_build_error)
        d.click('View##tab', panel='Model')
        d.click('Top', panel='Model')
        d.click('Fit', panel='Model')
        source = self.model.source
        d.click('Measure', panel='Model')
        self.pick((2, 1, 20))
        self.pick((32, 1, 20), add=True)
        self.assertEqual(2, len(w.picked), w.picked)
        self.assertAlmostEqual(10, w.viewer.inspection.measurements[-1].length, places=4)
        self.pick((2, 1, 20))
        d.click('Section view', panel='Model')
        d.click('Use selected face', panel='Properties')
        self.assertFalse(w.viewer.inspection.section_error)
        self.assertAlmostEqual(20, w.viewer.section_plane[1], places=4)
        self.assertEqual(source, self.model.source)
