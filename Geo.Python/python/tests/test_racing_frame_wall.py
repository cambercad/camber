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

    def assert_down_tube_sections(self, solid, *, reinforced=False):
        """Measure both sides of the cavity at the authored loft stations."""
        self.assertTrue(solid.is_watertight())
        for center, width in [((475, 0, 360), 29), ((685, 0, 585), 25)]:
            for side in (-1, 1):
                with self.subTest(center=center, side=side):
                    axis = vec3(0, side, 0)
                    inside = self.part.raycast(solid, center, axis)
                    outside = self.part.raycast(solid, vec3(center) + axis * 60, -axis)
                    self.assertIsNotNone(inside)
                    self.assertIsNotNone(outside)
                    self.assertAlmostEqual(
                        width - 3, (inside.point - vec3(center)).dot(axis), delta=.02,
                    )
                    outer_radius = (outside.point - vec3(center)).dot(axis)
                    if reinforced and side == 1:
                        # The external hose guide intentionally thickens this
                        # wall. Its neck must leave the inner bore unchanged
                        # (checked above), and its outer rim stays at the
                        # authored guide radius, rather than inside the tube.
                        guide_center = 39 if center[0] == 475 else 35
                        self.assertAlmostEqual(guide_center + 4.6, outer_radius, delta=.05)
                    else:
                        self.assertAlmostEqual(width, outer_radius, delta=.02)

    def test_rear_mount_retains_material_around_bolt_bores(self):
        from racing_bike import build_chainstay, brake_support_features
        bosses, tools = brake_support_features(self.part, front=False)
        body = self.part.batch_union([build_chainstay(self.part, 1), *bosses])
        for tool in tools:
            body = self.part.cut(body, tool)
        self.assertTrue(body.is_watertight())
        # These rays flank both intended bores. The former oversized head
        # recess excavated the stay here instead of seating below it.
        for x in (60, 94):
            for y in (60, 61, 69, 70):
                with self.subTest(x=x, y=y):
                    top = self.part.raycast(body, (x,y,350), (0,0,-1))
                    bottom = self.part.raycast(body, (x,y,290), (0,0,1))
                    self.assertIsNotNone(top)
                    self.assertIsNotNone(bottom)
                    self.assertAlmostEqual(top.point.z, 337, delta=.005)
                    self.assertAlmostEqual(bottom.point.z, 312, delta=.005)

    def test_seat_clamp_tools_clear_the_top_tube(self):
        from racing_bike import SEAT, SEAT_COLLAR_TOP, SEATPOST_AXIS, tube
        from racing_saddle import seat_collar_frame_tools
        top = tube(self.part, 'top_tube_clamp_check', [
            (SEAT-SEATPOST_AXIS*18,13,14), ((380,0,764),14,17),
            ((659,0,799),17,20), (HEAD_TOP-STEERING_AXIS*24,18,20)])
        for tool in seat_collar_frame_tools(self.part, SEAT_COLLAR_TOP, SEATPOST_AXIS):
            with self.subTest(tool=tool.name):
                self.assertEqual(self.part.intersect(top,tool).volume(), 0)

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
        self.assert_down_tube_sections(frame, reinforced=True)
        self.assertTrue(
            any(
                "BlendEdge_" in name
                and "frame_down_tube-Side" in name
                and "frame_head_tube-Side" in name
                for name in frame.edge_names
            ),
            "The finished frame must retain the down/head transition patch.",
        )
        self.assertTrue(any("BlendEdge_" in name and "frame_top_tube-Side" in name
                            and "frame_seat_tube-Side" in name for name in frame.edge_names),
                        "The finished frame must retain the top/seat transition patch.")
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
