"""Wheelbuilding datums, actual J-bend seating and woven material clearance."""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from camber import Part, set_progress_log
from bike_wheel import (
    spoke_placements,
    weave_contacts,
    create_spokes,
    create_front_hub,
    THROUGH_LEN,
)
from racing_hub import rear_hub_parts


def layout(front=False):
    right = 32 if front else 21
    return dict(
        left_z=-32,
        right_z=right,
        left_outboard=-35,
        right_outboard=right + 3,
        pcd=42,
        holes=12,
        rim_z=0,
        rim_radius=262.5,
        cross=2,
    )


class SpokeTests(unittest.TestCase):
    def setUp(self):
        set_progress_log(False)

    def test_pairing_preserves_holes_and_contact_datums(self):
        design = layout()
        spokes = spoke_placements(design)
        legacy = spoke_placements(dict(design, cross=3))
        self.assertEqual(
            {tuple(s.rim_point) for s in spokes}, {tuple(s.rim_point) for s in legacy}
        )
        self.assertEqual(
            [tuple(s.head) for s in spokes], [tuple(s.head) for s in legacy]
        )
        contacts = weave_contacts(spokes)
        self.assertEqual(24, len(contacts))
        for i, spoke in enumerate(spokes):
            self.assertAlmostEqual(math.hypot(spoke.head.x, spoke.head.y), 21)
            # The head is on a real planar flange face, with alternate insertion.
            expected = (24, 21)[i % 2] if i < 12 else (-35, -32)[i % 2]
            self.assertEqual(expected, spoke.head.z)
            point = contacts[spoke.label]
            others = [q for key, q in contacts.items() if key != spoke.label]
            mate = min(others, key=lambda q: (q - point).norm())
            self.assertAlmostEqual((mate - point).norm(), 2, places=9)
            self.assertAlmostEqual(mate.x, point.x, places=9)
            self.assertAlmostEqual(mate.y, point.y, places=9)
            # The final crossing reverses the original flange-side ordering.
            self.assertGreater((point.z - mate.z) * (-spoke.through.z), 0)

    def test_actual_hubs_and_all_woven_spokes_have_no_interference(self):
        for front in (False, True):
            with self.subTest(front=front):
                p = Part((-350, -350, -350), (350, 350, 350), tolerance=0.15)
                design = layout(front)
                hub = (
                    create_front_hub(p, design)
                    if front
                    else rear_hub_parts(p)[1]["shell"]
                )
                assembly = p.assembly("wheel_fit")
                assembly.fix(assembly.add_part(hub))
                for spoke in create_spokes(
                    p, design, max_deviation=0.15, tip_extension=1.7
                ):
                    self.assertTrue(spoke.is_watertight())
                    assembly.fix(assembly.add_part(spoke))
                self.assertEqual([], assembly.interferences(min_volume=1e-6))
                if not front:
                    inner = p.raycast(hub, (0, 0, -37), (1, 0, 0)).point.x
                    outer = p.raycast(hub, (20, 0, -37), (-1, 0, 0)).point.x
                    self.assertAlmostEqual(inner, 14.05, delta=0.002)
                    self.assertAlmostEqual(outer, 16, delta=0.002)
                    self.assertGreater(outer - inner, 1.94)


if __name__ == "__main__":
    unittest.main()
