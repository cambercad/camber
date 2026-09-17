"""Independent CAD sessions may build concurrently; assembly ownership stays explicit."""
from concurrent.futures import ThreadPoolExecutor
import unittest

from camber import Part, set_progress_log


class PartConcurrencyTests(unittest.TestCase):
    def setUp(self):
        set_progress_log(False)

    def test_constructing_another_part_does_not_reset_existing_session_names(self):
        first = Part((-10, -10, -10), (10, 10, 10))
        sketch = first.sketch()
        second = Part((-10, -10, -10), (10, 10, 10))
        later = first.sketch()
        self.assertNotEqual(sketch.name, later.name)
        self.assertNotEqual(first.name, second.name)

    @staticmethod
    def build(index):
        owner = Part((-10, -10, -10), (20, 20, 20), tolerance=.01)
        sketch = owner.sketch()
        sketch.add_rectangle((0, 0), (12, 4))
        blank = owner.extrude(sketch, 2, name='blank')
        tool = owner.cuboid((4+index*.1, -1, -1), (5+index*.1, 5, 3), name='slot')
        body = owner.cut(blank, tool, name='finished')
        # Materialize on the owning worker; no mutable body is shared by workers.
        edges = tuple(sorted(body.edge_names))
        points, triangles = body.mesh()
        geometry = tuple(sorted(tuple(sorted(tuple(points[i]) for i in triangle)) for triangle in triangles))
        return owner, body, (geometry, edges), sketch.name

    def test_serial_and_threaded_geometry_and_provenance_match(self):
        expected = [self.build(i)[2] for i in range(4)]
        with ThreadPoolExecutor(max_workers=4) as pool:
            actual = list(pool.map(self.build, range(4)))
        self.assertEqual(expected, [item[2] for item in actual])
        self.assertEqual(4, len({owner.name for owner, _, _, _ in actual}))
        self.assertEqual(4, len({item[3] for item in actual}))
        destination = Part((-10, -10, -10), (20, 20, 20), tolerance=.02)
        assembly = destination.assembly('collected')
        for i, (owner, body, signature, _) in enumerate(actual):
            with self.assertRaisesRegex(Exception, 'not registered'):
                assembly.add_part(body)
            copied = destination.copy_solid(body, 'collected_'+str(i))
            points, triangles = copied.mesh()
            geometry = tuple(sorted(tuple(sorted(tuple(points[j]) for j in triangle)) for triangle in triangles))
            self.assertEqual(signature[0], geometry)
            self.assertEqual(signature[1], tuple(sorted(copied.edge_names)))
            assembly.add_part(copied, (0, 0, i*4))
        self.assertEqual(4, len(assembly.parts))

    def test_incompatible_lattice_copy_rejected_without_registering_result(self):
        source, body, _, _ = self.build(0)
        destination = Part((-20, -20, -20), (20, 20, 20))
        with self.assertRaisesRegex(ValueError, 'working.*lattice'):
            destination.copy_solid(body, 'bad_copy')
        self.assertIsNone(destination.solid('bad_copy'))
        # Different box shape is valid when the actual origin and step match.
        compatible = Part((-10, -10, -10), (20, 15, 20))
        copied = compatible.copy_solid(body, 'compatible_copy')
        self.assertEqual(body.mesh(), copied.mesh())

    def test_worker_error_does_not_corrupt_another_session(self):
        def failing_worker():
            owner = Part((-10, -10, -10), (20, 20, 20))
            owner.sketch(name='duplicate')
            owner.sketch(name='duplicate')
        with ThreadPoolExecutor(max_workers=2) as pool:
            failure = pool.submit(failing_worker)
            success = pool.submit(self.build, 2)
            with self.assertRaisesRegex(Exception, 'already registered'):
                failure.result()
            owner, body, actual, _ = success.result()
        self.assertEqual(self.build(2)[2], actual)
        self.assertTrue(body.is_watertight())


if __name__ == '__main__':
    unittest.main()
