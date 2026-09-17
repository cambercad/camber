"""Named section frames stay body-local until their occurrence pose is applied."""
import math
import unittest

from camber import Part, set_progress_log


class AssemblyPlaneFrameTests(unittest.TestCase):
    def setUp(self):
        set_progress_log(False)
        self.part = Part((-100, -100, -100), (100, 100, 100), tolerance=.01)
        sketch = self.part.sketch(name='profile')
        sketch.add_rectangle((0, 0), (2, 3))
        self.solid = self.part.extrude(sketch, 4, name='block')
        # Independent exact-mesh witnesses retain the actual modeled planes,
        # including construction-lattice rounding of the nominal extrusion.
        self.local_top = self.part.raycast(self.solid, (1, 1, 10), (0, 0, -1)).point.z
        self.local_bottom = self.part.raycast(self.solid, (1, 1, -10), (0, 0, 1)).point.z

    def assert_vector(self, actual, expected):
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, delta=1e-8)

    def assert_display_invariant(self, assembly, reference):
        before = assembly.plane_frame(reference)
        # Packing is the real viewer/export path and updates the shared mesh.
        assembly._n.dump_display()
        after = assembly.plane_frame(reference)
        for name in ('origin', 'x', 'y', 'z'):
            self.assert_vector(getattr(after, name), getattr(before, name))
        return after

    def test_translated_rotated_face_frame_is_unchanged_by_display(self):
        assembly = self.part.assembly('root')
        assembly.add_part(self.solid, (10, 20, 30),
                          (0, math.sqrt(.5), 0, math.sqrt(.5)))
        frame = self.assert_display_invariant(assembly, 'block:ExtrudeTop')
        self.assertAlmostEqual(frame.origin.x, 10+self.local_top, delta=1e-8)
        self.assert_vector(frame.z, (1, 0, 0))
        self.assert_vector(frame.x, (0, 0, -1))
        self.assert_vector(frame.y, (0, 1, 0))
        # Bottom winding is outward, opposite the extrusion's nominal axis.
        bottom = self.assert_display_invariant(assembly, 'block:ExtrudeBottom')
        self.assertAlmostEqual(bottom.origin.x, 10+self.local_bottom, delta=1e-8)
        self.assert_vector(bottom.z, (-1, 0, 0))

    def test_repeated_parts_require_and_resolve_occurrence_paths(self):
        assembly = self.part.assembly('repeated')
        assembly.add_part(self.solid, (10, 0, 0))
        assembly.add_part(self.solid, (30, 0, 0))
        with self.assertRaisesRegex(Exception, 'ambiguous.*repeated/block\\[1\\]'):
            assembly.plane_frame('block:ExtrudeTop')
        first = self.assert_display_invariant(assembly, 'repeated/block[1]:ExtrudeTop')
        second = self.assert_display_invariant(assembly, 'repeated/block[2]:ExtrudeTop')
        self.assert_vector(second.origin-first.origin, (20, 0, 0))
        self.assert_vector(first.z, (0, 0, 1))

    def test_nested_shared_body_uses_composed_occurrence_pose(self):
        root = self.part.assembly('root')
        for index, x in enumerate((10, 30)):
            child = self.part.assembly(f'child{index}')
            child.add_part(self.solid, (1, 2, 3))
            root.add_subassembly(child, (x, 20, 30),
                                 (0, math.sqrt(.5), 0, math.sqrt(.5)))
        first = self.assert_display_invariant(root, 'root/child0[1]/block[1]:ExtrudeTop')
        second = self.assert_display_invariant(root, 'root/child1[2]/block[1]:ExtrudeTop')
        self.assertAlmostEqual(first.origin.x, 10+3+self.local_top, delta=1e-8)
        self.assert_vector(first.z, (1, 0, 0))
        self.assert_vector(second.origin-first.origin, (20, 0, 0))


if __name__ == '__main__':
    unittest.main()
