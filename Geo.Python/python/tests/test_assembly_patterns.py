"""Assembly patterns keep their explicit relative mates when the seed moves."""
import math
import unittest
from camber import Frame, Part, set_progress_log


class AssemblyPatternTests(unittest.TestCase):
    def setUp(self):
        set_progress_log(False)
        self.part = Part((-100, -100, -100), (100, 100, 100), tolerance=.01)
        self.solid = self.part.cuboid(Frame(), (1, 2, 3), name='pattern_seed')

    def assert_point(self, actual, expected):
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, delta=1e-5)

    def test_linear_instances_follow_seed_translation_and_rotation(self):
        assembly = self.part.assembly('linear')
        assembly.solve_after_every_constraint = False
        seed = assembly.add_part(self.solid)
        copies = assembly.pattern_linear(seed, 3, (10, 0, 0))
        self.assertIs(seed, copies[0])
        ground = assembly.add_part(self.solid, (3, 4, 0),
                                   (0, 0, math.sqrt(.5), math.sqrt(.5)))
        assembly.fix(ground)
        # Three noncollinear locating points move the seed after the pattern
        # exists; its copies must retain their original relative placements.
        for point in ((0, 0, 0), (1, 0, 0), (0, 1, 0)):
            assembly.coincident(seed.point_at(point), ground.point_at(point))
        probe = assembly.add_part(self.solid, copies[-1].pose)
        assembly.coincident(probe.point_at((0, 0, 0)), copies[-1].point_at((1, 0, 0)))
        result = assembly.solve()
        self.assertTrue(result.converged, repr(result))
        self.assertFalse(result.unsatisfied)
        for i, copy in enumerate(copies):
            self.assert_point(copy.pose, (3, 4 + 10*i, 0))
        self.assert_point(probe.pose, (3, 25, 0))

    def test_circular_full_and_partial_sweeps_place_and_rotate_instances(self):
        for angle, count in ((2*math.pi, 4), (math.pi, 3), (-math.pi, 3)):
            with self.subTest(angle=angle):
                assembly = self.part.assembly('circle_' + str(angle))
                assembly.solve_after_every_constraint = False
                initial_angle = math.pi / 6
                seed = assembly.add_part(self.solid, (12, 20, 30),
                    (0, 0, math.sin(initial_angle/2), math.cos(initial_angle/2)))
                copies = assembly.pattern_circular(seed, count, Frame((10, 20, 30)), angle)
                self.assertIs(seed, copies[0])
                assembly.fix(seed)
                step = angle / (count if abs(angle) == 2*math.pi else count-1)
                expected = []
                probes = []
                for i, copy in enumerate(copies):
                    theta = i*step
                    center = (10+2*math.cos(theta), 20+2*math.sin(theta), 30)
                    tip = (center[0]+math.cos(theta+initial_angle),
                           center[1]+math.sin(theta+initial_angle), 30)
                    # A point follower measures actual instance orientation;
                    # it imposes no orientation constraint on the patterned part.
                    probe = assembly.add_part(self.solid, center)
                    assembly.coincident(probe.point_at((0, 0, 0)), copy.point_at((1, 0, 0)))
                    probes.append(probe)
                    expected.append((center, tip))
                result = assembly.solve()
                self.assertTrue(result.converged, repr(result))
                for copy, probe, (center, tip) in zip(copies, probes, expected):
                    self.assert_point(copy.pose, center)
                    self.assert_point(probe.pose, tip)

    def test_nested_pattern_keeps_hierarchy_and_follows_seed(self):
        leaf = self.part.assembly('leaf')
        leaf.solve_after_every_constraint = False
        body = leaf.add_part(self.solid)
        leaf.fix(body)
        middle = self.part.assembly('middle')
        middle.solve_after_every_constraint = False
        nested = middle.add_subassembly(leaf, (2, 0, 0))
        middle.fix(nested)
        parent = self.part.assembly('nested_pattern')
        parent.solve_after_every_constraint = False
        seed = parent.add_subassembly(middle)
        copies = parent.pattern_linear(seed, 3, (10, 0, 0))
        self.assertIs(seed, copies[0])
        self.assertEqual(1, len(copies[1].subassemblies))
        self.assertEqual(0, len(copies[1].parts))
        self.assertTrue(copies[1].assembly.solve().converged)
        ground = parent.add_part(self.solid, (3, 4, 0),
                                 (0, 0, math.sqrt(.5), math.sqrt(.5)))
        parent.fix(ground)
        for point in ((0, 0, 0), (1, 0, 0), (0, 1, 0)):
            parent.coincident(body.point_at(point), ground.point_at(point))
        result = parent.solve()
        self.assertTrue(result.converged, repr(result))
        for i, copy in enumerate(copies):
            copied_body = copy.subassemblies[0].parts[0]
            self.assert_point(parent.world_pose(copied_body), (3, 4+10*i, 0))
        self.assertTrue(leaf.solve().converged)

    def test_foreign_and_nested_seeds_are_rejected_without_adding_instances(self):
        owner = self.part.assembly('owner')
        seed = owner.add_part(self.solid)
        target = self.part.assembly('target')
        target.add_subassembly(owner)
        for operation in (lambda: target.pattern_linear(seed, 2, (1, 0, 0)),
                          lambda: target.pattern_circular(seed, 3)):
            with self.assertRaisesRegex(Exception, 'seed must be a direct part of this assembly'):
                operation()
            self.assertEqual(0, len(target.parts))
        other = self.part.assembly('unrelated')
        with self.assertRaisesRegex(Exception, 'seed must be a direct part of this assembly'):
            other.pattern_linear(seed, 2, (1, 0, 0))
        self.assertEqual(0, len(other.parts))
        self.assertEqual(1, len(owner.parts))


if __name__ == '__main__': unittest.main()
