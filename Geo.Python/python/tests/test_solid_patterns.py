"""Public solid patterns preserve geometry and leave their seed untouched."""
import math
import unittest
from camber import Frame, Part, set_progress_log


class SolidPatternTests(unittest.TestCase):
    def setUp(self):
        set_progress_log(False)
        self.part = Part((-20, -20, -20), (20, 20, 20), tolerance=.001)
        self.seed = self.part.cuboid(Frame((2, 1, 1)), (1, 2, 3), name='seed')

    def test_linear_and_partial_circular_lists_include_seed(self):
        p, s = self.part, self.seed
        row = p.pattern_linear(s, 3, (3, 0, 0), name='row')
        self.assertIs(row[0], s)
        self.assertEqual(['seed', 'row_1', 'row_2'], [x.name for x in row])
        arc = p.pattern_circular(s, 3, angle=math.pi, name='arc')
        self.assertIs(arc[0], s)
        for copy in row + arc:
            self.assertTrue(copy.is_watertight())
            self.assertAlmostEqual(s.signed_volume(), copy.signed_volume(), places=8)
        source_points, _ = s.mesh()
        end_points, _ = arc[-1].mesh()
        for v in source_points:
            self.assertTrue(any(abs(q.x+v.x)+abs(q.y+v.y)+abs(q.z-v.z)<1e-9 for q in end_points))

    def test_mirror_is_an_independent_oriented_solid_and_can_be_mirrored_back(self):
        p, s = self.part, self.seed
        plane = Frame((.13, -.27, .31), x=(1,0,0), y=(0,.8,.6), z=(0,-.6,.8))
        reflected = p.mirror(s, plane, name='reflected')
        restored = p.mirror(reflected, plane, name='restored')
        self.assertTrue(reflected.is_watertight())
        self.assertAlmostEqual(s.signed_volume(), reflected.signed_volume(), places=8)
        before, _ = s.mesh()
        after, _ = restored.mesh()
        for v in before:
            self.assertTrue(any(abs(q.x-v.x)+abs(q.y-v.y)+abs(q.z-v.z)<1e-10 for q in after))
        self.assertEqual('seed', s.name)

    def test_invalid_counts_and_names_are_rejected(self):
        for count in (0, -1, 1.5, True):
            with self.subTest(count=count):
                with self.assertRaises(ValueError): self.part.pattern_linear(self.seed, count, (1,0,0))
                with self.assertRaises(ValueError): self.part.pattern_circular(self.seed, count)
        with self.assertRaisesRegex(Exception, 'Mirror output name already exists'):
            self.part.mirror(self.seed, name='seed')
        self.assertIs(self.seed, self.part.pattern_linear(self.seed, 1, (0,0,0))[0])


if __name__ == '__main__': unittest.main()
