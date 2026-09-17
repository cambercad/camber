"""Check dimensioned frame walls and the receiving seats after all features."""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from camber import Part, set_progress_log, vec3
from racing_bike import (
    HEAD_BOTTOM, HEAD_TOP, STEERING_AXIS,
    build_down_tube, build_frame_body, washer,
)


class FrameWallTests(unittest.TestCase):
    def setUp(self):
        set_progress_log(False)
        self.part = Part((-420, -300, -50), (1420, 300, 1120), tolerance=.2)

    def assert_down_tube_sections(self, solid):
        """Measure both sides of the cavity at the authored loft stations."""
        self.assertTrue(solid.is_watertight())
        for center, width in [((475, 0, 360), 29), ((685, 0, 585), 25)]:
            for side in (-1, 1):
                with self.subTest(center=center, side=side):
                    axis = vec3(0, side, 0)
                    inside = self.part.raycast(solid, center, axis)
                    outside = self.part.raycast(solid, vec3(center) + axis * 40, -axis)
                    self.assertIsNotNone(inside)
                    self.assertIsNotNone(outside)
                    self.assertAlmostEqual(
                        width - 3, (inside.point - vec3(center)).dot(axis), delta=.02,
                    )
                    self.assertAlmostEqual(
                        width, (outside.point - vec3(center)).dot(axis), delta=.02,
                    )

    def test_down_tube_has_an_inner_wall_and_preserves_outer_stations(self):
        tube = build_down_tube(self.part, hollow=True)
        # The original solid envelope was 1.65 million mm³. A real cavity
        # removes the bulk while retaining a substantial continuous wall.
        self.assertGreater(tube.volume(), 300000)
        self.assertLess(tube.volume(), 400000)
        self.assert_down_tube_sections(tube)

    def test_finished_frame_preserves_walls_and_receives_headset_bearings(self):
        # Exercise the final frame builder, including both head-tube junction
        # rounds and the subsequent receiving bores and mount machining.
        frame = build_frame_body(self.part)
        self.assert_down_tube_sections(frame)
        self.assertTrue(
            any(
                "BlendEdge_" in name
                and "frame_down_tube-Side" in name
                and "frame_head_tube-Side" in name
                for name in frame.edge_names
            ),
            "The finished frame must retain the down/head transition patch.",
        )
        seats = [
            ('lower', HEAD_BOTTOM, 22),
            ('upper', HEAD_TOP - STEERING_AXIS * 7, 20),
        ]
        for name, center, radius in seats:
            with self.subTest(bearing=name):
                bearing = washer(
                    self.part, f'frame_fit_{name}_bearing', center,
                    radius, 14.35, 7, axis=STEERING_AXIS,
                )
                self.assertLess(self.part.intersect(frame, bearing).volume(), 1e-6)


if __name__ == '__main__':
    unittest.main()
