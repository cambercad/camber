import unittest
from camber import Part, Interference, set_progress_log


class AssemblyInterferenceTests(unittest.TestCase):
    def test_nested_repeated_parts_sorted_filterable_and_displayable(self):
        set_progress_log(False)
        part=Part((-10,-10,-10),(10,10,10),tolerance=.01)
        cube=part.cuboid((0,0,0),(2,2,2),name='cube')
        child=part.assembly('child')
        child.add_part(cube)
        assembly=part.assembly('fixture')
        assembly.add_part(cube,(1,0,0))
        assembly.add_part(cube,(1.75,0,0))
        assembly.add_subassembly(child)
        before=[tuple(v) for v in cube.mesh()[0]]
        hits=assembly.interferences()
        self.assertEqual(3,len(hits))
        self.assertTrue(all(isinstance(hit,Interference) for hit in hits))
        self.assertEqual(sorted((hit.volume for hit in hits),reverse=True),[hit.volume for hit in hits])
        self.assertTrue(any('/child[1]/cube[1]' in path for hit in hits for path in (hit.first,hit.second)))
        self.assertEqual(1,len(assembly.interferences(min_volume=4.5)))
        for hit in hits:
            self.assertTrue(hit.geometry.is_watertight())
            self.assertAlmostEqual(hit.volume,hit.geometry.volume(),places=7)
        self.assertEqual(before,[tuple(v) for v in cube.mesh()[0]])
        self.assertEqual([p.pose for p in assembly.parts],[(1,0,0),(1.75,0,0)])

    def test_empty_and_invalid_threshold(self):
        part=Part((-10,-10,-10),(10,10,10))
        assembly=part.assembly('empty')
        self.assertEqual([],assembly.interferences())
        for value in (-1,float('nan'),float('inf')):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):assembly.interferences(min_volume=value)
