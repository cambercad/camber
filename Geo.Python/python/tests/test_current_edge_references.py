"""Current face and edge names stay unique without snapshot tokens."""

import os
import sys
import unittest

from camber import Part, set_progress_log
from camber.display import decode_native_solid

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from planetary_gearbox import HOUSING_R, TOLERANCE, build_carrier


class CurrentEdgeReferenceTests(unittest.TestCase):
    def test_current_split_patch_name_resolves_as_an_assembly_axis(self):
        set_progress_log(False)
        extent = HOUSING_R + 20
        part = Part((-extent, -extent, -extent), (extent, extent, extent), tolerance=TOLERANCE)
        carrier = build_carrier(part)

        current = next(name for name in carrier.patch_names
                       if "hub-Circle1" in name and not name.endswith("hub-Circle1"))
        self.assertNotIn("#current=", current)
        occurrence = part.assembly("assembly").add_part(carrier)
        datum = occurrence.axis(current)
        self.assertEqual(current, datum.reference)

    def test_current_edges_roundtrip_and_ancestor_aliases_stay_strict(self):
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
        self.assertNotIn("#current=", edge)
        self.assertIn(edge, body.curve_names)
        self.assertEqual(len(body.curve_names), len(set(body.curve_names)))
        self.assertEqual(len(body.patch_names), len(set(body.patch_names)))
        display = decode_native_solid(body._n)
        self.assertEqual(set(body.curve_names), {curve["name"] for curve in display.curves})
        self.assertEqual(set(body.patch_names), {patch["name"] for patch in display.patches})
        for operation in (body.fillet, body.chamfer):
            with self.subTest(feature=operation.__name__):
                result = operation(
                    [edge], 0.1, name=operation.__name__ + "_result"
                )
                self.assertTrue(result.is_watertight())
                self.assertGreater(body.volume() - result.volume(), 0.001)
                self.assertLess(body.volume() - result.volume(), 0.1)
        copy = part.copy_solid(body, name="copied")
        copied_edge = next(name for name in copy.curve_names
                           if name.split(":", 1)[-1] == edge.split(":", 1)[-1])
        self.assertIn(copied_edge, copy.curve_names)
        with self.assertRaisesRegex(Exception, "selects 2 connected edges"):
            body.fillet(["[hub-Circle1,crossing-Circle1]"], 0.1)
        with self.assertRaisesRegex(Exception, "Indexed support-pair"):
            body.fillet(["[hub-Circle1,crossing-Circle1]_1"], 0.1)


if __name__ == "__main__":
    unittest.main()
