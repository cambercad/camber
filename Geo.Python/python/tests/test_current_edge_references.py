"""Enumerated current edges remain selectable without reviving stale aliases."""

import os
import sys
import unittest

from camber import Part, set_progress_log

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from planetary_gearbox import HOUSING_R, TOLERANCE, build_carrier


class CurrentEdgeReferenceTests(unittest.TestCase):
    def test_current_split_patch_name_resolves_as_an_assembly_axis(self):
        set_progress_log(False)
        extent = HOUSING_R + 20
        part = Part((-extent, -extent, -extent), (extent, extent, extent), tolerance=TOLERANCE)
        carrier = build_carrier(part)

        current = next(name for name in carrier.patch_names
                       if "hub-Circle1" in name and "#current=" in name)
        occurrence = part.assembly("assembly").add_part(carrier)
        datum = occurrence.axis(current)
        self.assertEqual(current, datum.reference)

    def test_ambiguous_current_edges_roundtrip_but_do_not_transfer_to_a_copy(self):
        set_progress_log(False)
        part = Part((-10, -10, -10), (20, 20, 20), tolerance=0.02)
        hub = part.cylinder((0, 0, 0), 5, 5, name="hub")
        crossing = part.cylinder((-7, 0, 2.5), 1, 14, axis="x", name="crossing")
        body = part.union(hub, crossing, name="joined")
        edge = next(
            name
            for name in body.curve_names
            if "crossing-Circle1" in name and "crossing-ExtrudeBottom" in name
        )
        self.assertIn("#current=", edge)
        self.assertIn(edge, body.curve_names)
        for operation in (part.fillet, part.chamfer):
            with self.subTest(feature=operation.__name__):
                result = operation(
                    body, [edge], 0.1, name=operation.__name__ + "_result"
                )
                self.assertTrue(result.is_watertight())
                self.assertGreater(body.volume() - result.volume(), 0.001)
                self.assertLess(body.volume() - result.volume(), 0.1)
        copy = part.copy_solid(body, name="copied")
        with self.assertRaisesRegex(Exception, "another or rebuilt solid"):
            part.fillet(copy, [edge], 0.1)
        with self.assertRaisesRegex(Exception, "selects 2 connected edges"):
            part.fillet(body, ["[hub-Circle1,crossing-Circle1]"], 0.1)
        with self.assertRaisesRegex(Exception, "Indexed support-pair"):
            part.fillet(body, ["[hub-Circle1,crossing-Circle1]_1"], 0.1)


if __name__ == "__main__":
    unittest.main()
