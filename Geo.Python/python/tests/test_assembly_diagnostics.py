import contextlib
import io
import os
import sys
import unittest
sys.path.insert(0,os.path.join(os.path.dirname(__file__),'..'))
from camber import AssemblyConstraint, AssemblyConstraintDatum, Part, AssemblySolveResult, set_progress_log

class AssemblyDiagnosticsTests(unittest.TestCase):
    def setUp(self):
        set_progress_log(False)
        self.part=Part((-20,-20,-20),(20,20,20),tolerance=.01)

    def test_failed_solve_reports_exact_unsatisfied_mate_without_printing(self):
        part=self.part
        assembly=part.assembly('contradictory')
        assembly.solve_after_every_constraint=False
        first=assembly.add_part(part.cuboid((0,0,0),(1,1,1),name='first'))
        second=assembly.add_part(part.cuboid((0,0,0),(1,1,1),name='second'))
        assembly.fix(first);assembly.fix(second)
        assembly.coincident(first.point_at((0,0,0)),second.point_at((0,0,0)))
        assembly.coincident(first.point_at((10,0,0)),second.point_at((0,0,0)))
        output=io.StringIO()
        with contextlib.redirect_stdout(output):
            result=assembly.solve()
        self.assertIsInstance(result,AssemblySolveResult)
        self.assertFalse(result.converged)
        self.assertEqual('',output.getvalue())
        self.assertEqual(4,len(result.mates))
        self.assertEqual((3,),tuple(mate.index for mate in result.unsatisfied))
        failure=result.unsatisfied[0]
        self.assertEqual('CoincidentPoints',failure.kind)
        self.assertAlmostEqual(10/result.characteristic_length,failure.max_residual)
        self.assertIn(failure.label,str(result))
        self.assertGreater(result.sum_squared_error,0)
        with self.assertRaises(AttributeError):
            result.converged=True

    def test_success_reports_named_entities_and_final_pose_errors(self):
        part=self.part;assembly=part.assembly('seated')
        assembly.solve_after_every_constraint=False
        first=assembly.add_part(part.cuboid((0,0,0),(1,1,1),name='first'))
        second=assembly.add_part(part.cuboid((0,0,0),(1,1,1),name='second'),(0,0,4))
        assembly.fix(first)
        assembly.coincident(first.plane('first-ExtrudeTop'),second.plane('second-ExtrudeBottom'),opposite_normals=True)
        assembly.concentric(first.axis_at((0,0,0),(0,0,1)),second.axis_at((0,0,0),(0,0,1)))
        assembly.parallel(first.axis_at((0,0,0),(1,0,0)),second.axis_at((0,0,0),(1,0,0)))
        result=assembly.solve()
        self.assertTrue(result.converged)
        self.assertEqual((),result.unsatisfied)
        self.assertEqual(('first:first-ExtrudeTop','second:second-ExtrudeBottom'),result.mates[1].entities)
        self.assertLessEqual(result.mates[1].max_residual,result.mates[1].tolerance)
        self.assertGreaterEqual(result.num_parameters,0)
        self.assertGreaterEqual(result.num_equations,0)

    def test_constraints_are_native_immutable_records_with_physics_anchors(self):
        part=self.part;assembly=part.assembly('seated')
        assembly.solve_after_every_constraint=False
        first=assembly.add_part(part.cuboid((0,0,0),(1,1,1),name='first'))
        second=assembly.add_part(part.cuboid((0,0,0),(1,1,1),name='second'),(0,0,4))
        assembly.fix(first)
        assembly.coincident(first.plane('first-ExtrudeTop'),second.plane('second-ExtrudeBottom'),opposite_normals=True)
        assembly.concentric(first.axis_at((0,0,0),(0,0,1)),second.axis_at((0,0,0),(0,0,1)))
        self.assertTrue(assembly.solve().converged)

        records=assembly.constraints
        self.assertIsInstance(records,tuple)
        self.assertEqual(('FixPart','CoincidentPlanes','Concentric'),tuple(record.kind for record in records))
        self.assertIsInstance(records[1],AssemblyConstraint)
        plane=records[1].first
        self.assertIsInstance(plane,AssemblyConstraintDatum)
        self.assertEqual('plane',plane.kind)
        self.assertEqual('seated/first[1]',plane.path)
        self.assertEqual(('first:first-ExtrudeTop','second:second-ExtrudeBottom'),records[1].entities)
        self.assertAlmostEqual(1.0,plane.world_direction.z)
        self.assertAlmostEqual(-1.0,records[1].second.world_direction.z)
        self.assertEqual('axis',records[2].first.kind)
        self.assertEqual((0.0,0.0,1.0),tuple(records[2].first.local_direction))
        with self.assertRaises(AttributeError):
            records[1].kind='other'

    def test_python_can_author_every_kernel_constraint_kind(self):
        part=self.part;assembly=part.assembly('all_kinds')
        assembly.solve_after_every_constraint=False
        first=assembly.add_part(part.cuboid((0,0,0),(1,1,1),name='first'))
        second=assembly.add_part(part.cuboid((0,0,0),(1,1,1),name='second'))
        xy_first=first.plane_at((0,0,0),(0,0,1))
        xy_second=second.plane_at((0,0,0),(0,0,1))
        yz_second=second.plane_at((0,0,0),(1,0,0))
        point=first.point_at((0,0,0))
        assembly.parallel(xy_first,xy_second)
        assembly.perpendicular(xy_first,yz_second)
        assembly.on_plane(point,xy_second)
        assembly.contact(point,xy_second)
        self.assertEqual(('ParallelPlanes','PerpendicularPlanes','PointOnPlane','Contact'),
                         tuple(record.kind for record in assembly.constraints))
