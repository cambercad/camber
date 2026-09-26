import math
import unittest

from camber import Frame, Part, set_progress_log


class RibAndHoleTests(unittest.TestCase):
    def setUp(self):
        set_progress_log(False)
        self.part = Part((-10, -10, -10), (10, 10, 10), tolerance=.001)

    def test_l_rib_terminates_at_named_face(self):
        roof = self.part.cuboid((-5, -5, 3), (5, 5, 4), name="roof")
        rib = roof.rib([(0, 0), (4, 0), (4, 4)], 1,
                            (roof, "roof-ExtrudeBottom"), name="ribbed")
        self.assertTrue(rib.is_watertight())
        self.assertAlmostEqual(124, rib.volume(), delta=.02)
        self.assertEqual("Union", self.part.operations[-1].kind)

    def test_rib_height_and_surface_termination(self):
        roof = self.part.cuboid((-5, -5, 3), (5, 5, 4), name="roof")
        surface = roof.face_surface("roof-ExtrudeBottom")
        from_height = roof.rib([(0, 0), (4, 0)], 1, 3)
        from_surface = roof.rib([(0, 0), (4, 0)], 1, surface)
        from_solid = roof.rib([(0, 0), (4, 0)], 1, roof)
        for result in (from_height, from_surface, from_solid):
            self.assertTrue(result.is_watertight())
            self.assertAlmostEqual(112, result.volume(), delta=.02)

    def test_hole_variants_and_draft(self):
        block = self.part.cuboid((-4, -4, 0), (4, 4, 4), name="block")
        mouth = Frame(origin=(0, 0, 4), x=(1, 0, 0), y=(0, -1, 0), z=(0, 0, -1))
        drilled = block.hole(mouth, 1, name="drilled")
        counterbored = block.counterbore_hole(mouth, 1, 2, 1, name="counterbored")
        countersunk = block.countersink_hole(mouth, 1, 2, math.pi / 2,
                                                  name="countersunk")
        for result in (drilled, counterbored, countersunk):
            self.assertTrue(result.is_watertight())
            self.assertLess(result.volume(), block.volume())
        self.assertLess(counterbored.volume(), drilled.volume())
        self.assertLess(countersunk.volume(), drilled.volume())
        self.assertEqual(["Hole", "Counterbore hole", "Countersink hole"],
                         [operation.kind for operation in self.part.operations[-3:]])

        side = next(name for name in block.patch_names if name.endswith("Line2"))
        drafted = block.draft_faces(side, Frame(), math.pi / 18,
                                        name="drafted")
        self.assertTrue(drafted.is_watertight())
        self.assertIn(side.split(":", 1)[-1],
                      [name.split(":", 1)[-1] for name in drafted.patch_names])


if __name__ == "__main__":
    unittest.main()
