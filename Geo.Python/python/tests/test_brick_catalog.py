"""Pure-data catalogue integrity; geometry validation belongs to the builders."""

from collections import Counter
import unittest
from brick_catalog import CATALOG, PARTS, PLATE_HEIGHT, STUD_PITCH


class BrickCatalogTests(unittest.TestCase):
    def test_expanded_distinct_supported_parts(self):
        self.assertEqual(len(CATALOG), 130)
        self.assertEqual(len(PARTS), 130)
        self.assertEqual(
            Counter(p.family for p in CATALOG),
            {
                "brick": 23,
                "plate": 28,
                "tile": 18,
                "slope": 11,
                "roof_ridge": 4,
                "round": 8,
                "arch": 8,
                "beam": 6,
                "axle": 8,
                "bush": 2,
                "pin": 4,
                "jumper": 3,
                "grille": 1,
                "panel": 3,
                "window_frame": 2,
                "door_frame": 1,
            },
        )
        for spec in CATALOG:
            with self.subTest(key=spec.key):
                self.assertGreater(spec.studs_x, 0)
                self.assertGreater(spec.studs_y, 0)
                self.assertGreater(spec.height_plates, 0)
        identities = {
            (p.family, p.studs_x, p.studs_y, p.height_plates, p.variant)
            for p in CATALOG
        }
        self.assertEqual(len(identities), 130)

    def test_published_nominal_grid_and_verified_identity(self):
        self.assertEqual(STUD_PITCH, 8)
        self.assertAlmostEqual(PLATE_HEIGHT * PARTS["brick_2x4"].height_plates, 9.6)
        self.assertEqual(PARTS["brick_2x4"].source_id, "3001")
        self.assertEqual(PARTS["arch_1x5_h12"].source_id, "2339")
        self.assertEqual(PARTS["beam_15"].source_id, "32278")


if __name__ == "__main__":
    unittest.main()
