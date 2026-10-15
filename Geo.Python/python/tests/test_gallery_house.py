"""Integration checks for the complete house and its component placements."""
from pathlib import Path
import tempfile
import unittest

from examples.architecture.gallery_house import HouseParameters, ROOMS, build_house, write_plans


class HousePlanTests(unittest.TestCase):
    def test_options_require_booleans(self):
        for field in ('cutaway', 'rainwater', 'terrace'):
            with self.assertRaises(ValueError):
                HouseParameters(**{field: 'yes'})

    def test_plan_rooms_are_inside_the_envelope_and_do_not_overlap(self):
        for i, (level, label, (x0, y0, x1, y1)) in enumerate(ROOMS):
            self.assertTrue(350 <= x0 < x1 <= 11650, label)
            self.assertTrue(350 <= y0 < y1 <= 9650, label)
            for other_level, other, (a, b, c, d) in ROOMS[i+1:]:
                if level == other_level:
                    self.assertFalse(min(x1, c) > max(x0, a) and min(y1, d) > max(y0, b), (label, other))
        with tempfile.TemporaryDirectory() as temporary:
            target = Path(temporary)/'plan.svg'
            write_plans(target)
            import xml.etree.ElementTree as ET
            root = ET.parse(target).getroot()
            self.assertEqual('{http://www.w3.org/2000/svg}svg', root.tag)
            self.assertIn('Guest WC / shower', target.read_text(encoding='utf-8'))


class HouseGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = build_house()
        cls.solids = {s.name: s for s in cls.model.solids}

    def test_complete_house_contains_closed_positive_volume_components(self):
        for solid in self.model.solids:
            with self.subTest(solid=solid.name):
                self.assertTrue(solid.is_watertight())
                self.assertGreater(solid.volume(), 0)
        names = self.solids
        for name in ('house_roof_slope0', 'house_roof_slope1', 'house_dormer_shell0',
                     'house_dormer_shell1', 'house_dormer_shell2', 'house_upper_enclosure'):
            self.assertIn(name, names)
        self.assertTrue(any('rooflight' in n for n in names))
        self.assertTrue(any('gutter' in n for n in names))
        self.assertTrue(any('downpipe' in n for n in names))

    def test_stair_arrives_at_floor_without_a_gap_and_living_void_stays_open(self):
        landing = self.solids['house_upper_floor3']
        hit = landing.raycast((7400, 6801, 3400), (0, 0, -600))
        self.assertIsNotNone(hit)
        self.assertAlmostEqual(3200, hit.point.z, delta=.025)
        self.assertIsNone(landing.raycast((7400, 6790, 3400), (0, 0, -600)))
        for name, solid in self.solids.items():
            if name.startswith('house_upper_floor'):
                self.assertIsNone(solid.raycast((2000, 2500, 3500), (0, 0, -700)))

    def test_stair_headroom_and_landing_access_are_clear_of_partitions(self):
        partition = self.solids['house_upper_north_partition_clipped']
        self.assertIsNone(partition.raycast((7400, 6400, 4400), (0, 400, 0)))
        bedroom = self.solids['house_bedroom3_side_clipped']
        self.assertIsNone(bedroom.raycast((8000, 7300, 4400), (400, 0, 0)))

    def test_gable_window_is_a_real_opening(self):
        enclosure = self.solids['house_upper_enclosure']
        self.assertIsNone(enclosure.raycast((2000, -100, 4300), (0, 500, 0)))
        self.assertIsNotNone(enclosure.raycast((400, -100, 4300), (0, 500, 0)))

    def test_every_room_has_a_recorded_exterior_window(self):
        rows = self.model.daylight_schedule()
        self.assertEqual(len(ROOMS), len(rows))
        for row in rows:
            with self.subTest(room=row['room']):
                self.assertTrue(row['windows'])
                self.assertGreater(row['clear_glazing_area'], 0)
                for name in row['windows']:
                    self.assertTrue(any(n.startswith(name+'_') for n in self.solids), name)

    def test_service_room_windows_are_actual_wall_openings(self):
        self.assertIsNone(self.solids['house_north_wall'].raycast((7200, 9600, 2000), (0, 500, 0)))
        self.assertIsNone(self.solids['house_upper_enclosure'].raycast((5200, 9600, 4800), (0, 500, 0)))
        self.assertIsNone(self.solids['house_east_wall'].raycast((11600, 8100, 1600), (500, 0, 0)))

    def test_partitions_reach_the_roof_instead_of_stopping_above_doors(self):
        wall = self.solids['house_bath_front_clipped']
        self.assertIsNotNone(wall.raycast((6000, 7850, 6500), (0, 300, 0)))
        self.assertIsNone(wall.raycast((6000, 7850, 9000), (0, 300, 0)))


if __name__ == '__main__':
    unittest.main()
