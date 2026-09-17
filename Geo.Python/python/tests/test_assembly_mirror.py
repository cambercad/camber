"""Opposite-handed assembly components retain hierarchy and internal mates."""
import math
import unittest
from camber import Frame, Part, set_progress_log


class AssemblyMirrorTests(unittest.TestCase):
    def setUp(self):
        set_progress_log(False)
        self.part = Part((-100, -100, -100), (100, 100, 100), tolerance=.001)
        self.body = self.part.cuboid(Frame((1, 2, 3)), (1, 2, 4), name='source_body')

    def test_mirror_solved_part_preserves_source_and_opposite_handed_geometry(self):
        assembly = self.part.assembly('parent')
        assembly.solve_after_every_constraint = False
        seed = assembly.add_part(self.body, (7, -5, 9),
                                 (0, 0, math.sin(.23), math.cos(.23)))
        assembly.fix(seed)
        self.assertTrue(assembly.solve().converged)
        points, _ = self.body.mesh()
        before = [(p.x, p.y, p.z) for p in points]
        mirror = assembly.mirror(seed, Frame((0, 0, 2)), name='opposite')
        self.assertTrue(assembly.solve().converged)
        reflected = self.part.solid(mirror.name)
        actual, _ = reflected.mesh()
        for x, y, z in before:
            self.assertTrue(any(abs(p.x-x)+abs(p.y-y)+abs(p.z-(4-z))<1e-5 for p in actual))
        self.assertTrue(reflected.is_watertight())
        self.assertGreater(reflected.signed_volume(), 0)
        self.assertEqual((7, -5, 9), seed.pose)

    def test_mirrored_subassembly_retains_revolute_motion_and_nested_support(self):
        support = self.part.assembly('support')
        support.solve_after_every_constraint = False
        fixed = support.add_part(self.body)
        support.fix(fixed)
        mechanism = self.part.assembly('mechanism')
        mechanism.solve_after_every_constraint = False
        housing = mechanism.add_subassembly(support)
        mechanism.fix(housing)
        rotor = mechanism.add_part(self.body, (0, 0, 5))
        mechanism.concentric(fixed.axis_at((0, 0, 0), (0, 0, 1)), rotor.axis_at((0, 0, 0), (0, 0, 1)))
        mechanism.distance(rotor.plane_at((0, 0, 0), (0, 0, 1)), fixed.plane_at((0, 0, 0), (0, 0, 1)), 5)
        self.assertTrue(mechanism.solve().converged)
        parent = self.part.assembly('parent')
        parent.solve_after_every_constraint = False
        seed = parent.add_subassembly(mechanism, (10, 2, 4))
        parent.fix(seed)
        mirror = parent.mirror(seed, name='opposite_mechanism')
        self.assertEqual(1, len(mirror.subassemblies))
        self.assertTrue(parent.solve().converged)
        mirrored_rotor = mirror.parts[0]
        mirrored_housing = mirror.subassemblies[0].parts[0]
        for got, expected in zip(parent.world_pose(mirrored_rotor), (10, 2, -9)):
            self.assertAlmostEqual(got, expected, delta=1e-6)
        clone = mirror.assembly
        cloned_report = clone.solve()
        self.assertTrue(cloned_report.converged)
        self.assertEqual(['FixPart', 'Concentric', 'DistancePlanes'], [m.kind for m in cloned_report.mates])
        clone.angle(mirrored_housing.axis_at((0, 0, 0), (1, 0, 0)),
                    mirrored_rotor.axis_at((0, 0, 0), (1, 0, 0)), .4)
        self.assertTrue(clone.solve().converged)
        self.assertEqual((0, 0, 5), rotor.pose)

    def test_mirror_rejects_empty_and_foreign_seeds(self):
        parent = self.part.assembly('parent')
        with self.assertRaisesRegex(TypeError, 'AssemblyPart or AssemblyOccurrence'):
            parent.mirror(object())
        empty = parent.add_subassembly(self.part.assembly('empty'))
        with self.assertRaisesRegex(Exception, 'empty subassembly'):
            parent.mirror(empty)
        owner = self.part.assembly('owner')
        foreign = owner.add_part(self.body)
        with self.assertRaisesRegex(Exception, 'direct part of this assembly'):
            parent.mirror(foreign)
        self.assertEqual(0, len(parent.parts))
        self.assertEqual(1, len(parent.subassemblies))


if __name__ == '__main__': unittest.main()
