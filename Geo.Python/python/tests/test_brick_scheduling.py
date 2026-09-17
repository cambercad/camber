"""Build scheduling must not change catalogue identity or display order."""

import unittest
from unittest.mock import patch

import bricks


class BrickSchedulingTests(unittest.TestCase):
    def test_expensive_parts_start_first_and_display_order_is_preserved(self):
        requested = ["brick_1x1", "brick_2x4", "brick_1x2"]
        with patch.object(bricks, "build_part", wraps=bricks.build_part) as build:
            gallery, colors, report = bricks.build_catalog(requested, workers=1)
        self.assertEqual(
            [call.args[0] for call in build.call_args_list],
            ["brick_2x4", "brick_1x2", "brick_1x1"],
        )
        self.assertEqual([entry["key"] for entry in report["entries"]], requested)
        self.assertEqual(report["parts"], 3)
        self.assertTrue(all(entry["watertight"] for entry in report["entries"]))
        self.assertEqual(gallery.constraints, [])
        self.assertEqual(len(colors), 3)

    def test_largest_existing_tile_heads_the_queue(self):
        ordered = sorted(bricks.PARTS, key=bricks._build_priority, reverse=True)
        self.assertEqual(ordered[0], "tile_8x16")


if __name__ == "__main__":
    unittest.main()
