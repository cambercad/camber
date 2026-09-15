import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from camber import Part, vec3, set_progress_log

set_progress_log(False)


def _box(part, name, size=4.0):
    sk = part.sketch("xy", name=name + "_sk")
    sk.add_rectangle((-0.5 * size, -0.5 * size), (0.5 * size, 0.5 * size))
    return part.extrude(sk, size, name=name)


def _near(got, expected, tol=0.15):
    return all(abs(a - b) < tol for a, b in zip(got, expected))


class AssemblyNestingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from camber.api import _native_mod
        native_asm = _native_mod().get("NativeAssembly")
        if native_asm is None or not hasattr(native_asm, "add_sub_assembly"):
            raise unittest.SkipTest(
                "native wheel has no add_sub_assembly; rebuild with Geo.Python/publish-wheel.ps1")

    def test_two_level_subassembly_composes_poses(self):
        part = Part(vec3(-80), vec3(80), tolerance=0.05)
        inner_solid = _box(part, "inner")
        mid_solid = _box(part, "mid")
        top_solid = _box(part, "top")

        inner = part.assembly("inner_asm")
        inner.solve_after_every_constraint = False
        inner_p = inner.add_part(inner_solid)
        inner.fix(inner_p)
        inner.solve()

        mid = part.assembly("mid_asm")
        mid.solve_after_every_constraint = False
        mid_p = mid.add_part(mid_solid)
        inner_occ = mid.add_subassembly(inner, (10.0, 0.0, 0.0))
        mid.fix(mid_p)
        mid.solve()

        top = part.assembly("top_asm")
        top.solve_after_every_constraint = False
        top_p = top.add_part(top_solid)
        mid_occ = top.add_subassembly(mid, (0.0, 20.0, 0.0))
        top.fix(top_p)
        top.solve()

        self.assertEqual(1, len(mid.subassemblies))
        self.assertEqual("inner_asm", inner_occ.name)
        self.assertEqual(1, len(top.subassemblies))
        self.assertEqual("mid_asm", mid_occ.name)
        self.assertEqual(1, len(mid_occ.subassemblies))
        self.assertTrue(_near(top.world_pose(inner_p), (10.0, 20.0, 0.0)))
        self.assertTrue(_near(top.world_pose(mid_p), (0.0, 20.0, 0.0)))
        self.assertTrue(_near(top.world_pose(top_p), (0.0, 0.0, 0.0)))

    def test_parent_can_mate_to_nested_part(self):
        part = Part(vec3(-80), vec3(80), tolerance=0.05)
        nested_solid = _box(part, "nested")
        ground_solid = _box(part, "ground")

        nested = part.assembly("nested_asm")
        nested.solve_after_every_constraint = False
        nested_p = nested.add_part(nested_solid)
        nested.fix(nested_p)
        nested.solve()

        top = part.assembly("top_asm")
        top.solve_after_every_constraint = False
        ground = top.add_part(ground_solid)
        occ = top.add_subassembly(nested, (12.0, 8.0, 0.0))
        top.fix(ground)
        top.coincident(ground.point_at((0.0, 0.0, 0.0)), nested_p.point_at((0.0, 0.0, 0.0)))
        top.solve()

        self.assertTrue(_near(top.world_pose(nested_p), (0.0, 0.0, 0.0)))
        self.assertTrue(_near(occ.pose, (0.0, 0.0, 0.0)))

    def test_rejects_cycles_and_double_nesting(self):
        part = Part(vec3(-80), vec3(80), tolerance=0.05)
        a_solid = _box(part, "a")
        b_solid = _box(part, "b")

        a = part.assembly("a_asm")
        a.add_part(a_solid)
        b = part.assembly("b_asm")
        b.add_part(b_solid)
        b.add_subassembly(a, (1.0, 0.0, 0.0))

        with self.assertRaises(Exception):
            a.add_subassembly(b)
        with self.assertRaises(Exception):
            b.add_subassembly(b)
        extra = part.assembly("extra_asm")
        with self.assertRaises(Exception):
            extra.add_subassembly(a)


if __name__ == "__main__":
    unittest.main()
