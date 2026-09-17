"""Physical curvature and receiving-fit checks for the moulded palm frond."""

import unittest
from camber import Part, set_progress_log
from brick_models import foliage, build_model
from brick_parts import WORKING_LOW, WORKING_HIGH


class CurvedPalmTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        set_progress_log(False)
        cls.owner = Part(WORKING_LOW, WORKING_HIGH, 0.025)
        cls.leaf = foliage(cls.owner, "measured_palm_frond", True)

    def test_actual_blade_is_curved_and_has_material_thickness(self):
        p = self.owner
        hits = [p.raycast(self.leaf, (x, 0, 10), (0, 0, -1)) for x in (16, 43, 70)]
        self.assertTrue(all(hit is not None for hit in hits))
        self.assertGreater(hits[0].point.z, hits[1].point.z)
        self.assertGreater(hits[1].point.z, hits[2].point.z)
        # A merely tilted planar extrusion has zero midpoint departure.
        departure = hits[1].point.z - (hits[0].point.z + hits[2].point.z) / 2
        self.assertGreater(departure, 1.5)
        self.assertLess(hits[0].normal.dot(hits[2].normal), 0.97)
        for hit in hits:
            underside = p.raycast(self.leaf, hit.point - hit.normal * 0.2, -hit.normal)
            self.assertIsNotNone(underside)
            self.assertAlmostEqual(
                (hit.point - underside.point).norm(), 1.2, delta=0.04
            )

    def test_flat_heel_and_hollow_studs_remain_intact(self):
        p = self.owner
        top = [p.raycast(self.leaf, (4, y, 10), (0, 0, -1)) for y in (-1, 1)]
        for hit in top:
            self.assertAlmostEqual(hit.point.z, 1.6, delta=0.002)
        for x in (0, 8):
            self.assertIsNone(p.raycast(self.leaf, (x, 0, 10), (0, 0, -1)))
        self.assertTrue(self.leaf.is_watertight())
        self.assertGreater(self.leaf.signed_volume(), 0)

    def test_curved_fronds_clear_the_trunk_and_other_fronds(self):
        model = build_model("tree_palm")
        self.assertEqual(model.components, 14)
        self.assertEqual(model.assembly.interferences(min_volume=0.005), [])


if __name__ == "__main__":
    unittest.main()
