import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from camber import naming
from camber.api import _parse_sketch_curve_dump as parse_sketch_curve_dump


class NamingTests(unittest.TestCase):
    def test_sketch_handle_addresses_match_csharp(self):
        self.assertEqual("Line1@0.000", naming.format_sketch_handle_address("Line1", "line", 0))
        self.assertEqual("Line1@1.000", naming.format_sketch_handle_address("Line1", "line", 1))
        self.assertEqual("Line1@0.500", naming.format_sketch_handle_address("Line1", "line", 3))
        self.assertEqual("Circle1@center", naming.format_sketch_handle_address("Circle1", "circle", 2))
        self.assertEqual("Circle1@0.000", naming.format_sketch_handle_address("Circle1", "circle", 3))
        self.assertEqual("Circle1@0.250", naming.format_sketch_handle_address("Circle1", "circle", 4))
        self.assertEqual("Circle1@0.750", naming.format_sketch_handle_address("Circle1", "circle", 6))
        self.assertEqual("Arc1@center", naming.format_sketch_handle_address("Arc1", "arc", 2))
        self.assertEqual("", naming.format_sketch_handle_address("Line1", "line", 2))

    def test_format_patch_component_name(self):
        self.assertEqual("hole_z-Circle1", naming.format_patch_component_name("hole_z-Circle1", 0))
        self.assertEqual("hole_z-Circle1_1", naming.format_patch_component_name("hole_z-Circle1", 1))
        self.assertEqual("hole_z-Circle1_2", naming.format_patch_component_name("hole_z-Circle1", 2))

    def test_qualify_and_parse_round_trip(self):
        local = naming.format_sketch_curve_address("Line1", 0.5)
        qualified = naming.qualify("Sketch1", local)
        self.assertEqual("Sketch1:Line1@0.500", qualified)
        owner, rest = naming.parse_qualified(qualified)
        self.assertEqual("Sketch1", owner)
        parsed = naming.parse_sketch_curve_address(rest)
        self.assertEqual("Line1", parsed["curve"])
        self.assertAlmostEqual(0.5, parsed["uniform"])
        self.assertTrue(naming.is_point_address(qualified))
        self.assertTrue(naming.is_curve_address("Sketch1:Line1"))
        self.assertFalse(naming.is_curve_address(qualified))
        self.assertTrue(naming.is_offset_curve_name("north@out_offset"))
        self.assertEqual("north@in_offset", naming.format_sketch_offset_curve("north", False))
        self.assertEqual("north@out_offset_2", naming.format_sketch_offset_curve("north", True, 1))
        self.assertEqual("h@start_cap", naming.format_sketch_offset_end_cap("h", True))
        self.assertEqual("h@end_cap", naming.format_sketch_offset_end_cap("h", False))
        self.assertEqual(
            "cap[h@out_offset,v@out_offset]",
            naming.format_sketch_offset_join_cap("v@out_offset", "h@out_offset"),
        )
        self.assertTrue(naming.is_offset_curve_name("h@end_cap"))
        self.assertTrue(naming.is_offset_curve_name("h@start_cap_2"))
        self.assertTrue(naming.is_offset_curve_name("cap[h@out_offset,v@out_offset]"))
        self.assertTrue(naming.is_curve_address("north@out_offset"))
        self.assertTrue(naming.is_curve_address("h@end_cap"))
        self.assertTrue(naming.is_curve_address("cap[h@out_offset,v@out_offset]"))
        self.assertFalse(naming.is_point_address("north@out_offset"))
        parsed = naming.parse_sketch_curve_address("north@out_offset@0.500")
        self.assertEqual("north@out_offset", parsed["curve"])
        self.assertTrue(naming.is_origin_name("Origin"))
        self.assertTrue(naming.is_origin_name("origin"))
        self.assertTrue(naming.is_origin_name("Sketch1:Origin"))
        self.assertEqual(0, naming.sketch_axis_index("x"))
        self.assertEqual(1, naming.sketch_axis_index("Sketch1:y"))
        self.assertIsNone(naming.sketch_axis_index("origin"))
        self.assertEqual("origin", naming.canonicalize_point_name("Origin"))
        self.assertEqual("Sketch1:origin", naming.canonicalize_point_name("Sketch1:Origin"))
        self.assertEqual("Circle1@center", naming.canonicalize_point_name("Circle1@Center"))

    def test_highlight_names_filters_by_selection(self):
        all_names = ["box:[A,B]@0.000", "Sketch1:Line1", "Sketch1:Line1@1.000"]
        self.assertEqual(
            ["Sketch1:Line1@1.000"],
            naming.highlight_names(all_names, ["Sketch1:Line1@1.000"]),
        )

    def test_parse_named_and_legacy_dump(self):
        named = parse_sketch_curve_dump("L\tLine1\t0\t0\t2\t0\nC\tCircle1\t1\t1\t0.5\n")
        self.assertEqual("line", named[0]["kind"])
        self.assertEqual("Line1", named[0]["name"])
        self.assertEqual((0.0, 0.0), named[0]["p0"])
        self.assertEqual("Circle1", named[1]["name"])
        legacy = parse_sketch_curve_dump("L\t0\t0\t2\t0\n")
        self.assertEqual("", legacy[0]["name"])
        self.assertEqual((2.0, 0.0), legacy[0]["p1"])


if __name__ == "__main__":
    unittest.main()
