"""Real inclined spoke seats, tool access and standard nipple engagement."""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from camber import Part, vec3, set_progress_log
from bike_wheel import spoke_placements, create_spoke
from racing_nipple import nipple, nipple_frame, mount_nipples, TIP_EXTENSION, SLOT_FLOOR
from racing_tyre import tyre_rim_parts


def layout(depth=50):
    return dict(
        left_z=-32,
        right_z=21,
        left_outboard=-35,
        right_outboard=24,
        pcd=42,
        holes=12,
        rim_z=0,
        rim_radius=312.5 - depth,
        cross=2,
    )


class NippleTests(unittest.TestCase):
    def setUp(self):
        set_progress_log(False)

    def test_turned_nipple_bore_flats_and_slot(self):
        p = Part((-20, -20, -20), (20, 20, 20), tolerance=0.15)
        body = nipple(p)
        self.assertTrue(body.is_watertight())
        self.assertGreater(body.volume(), 100)
        self.assertLess(body.volume(), 120)
        self.assertIsNone(p.raycast(body, (0, 0, -1), (0, 0, 1)))
        flat = p.raycast(body, (5, 0, 3), (-1, 0, 0))
        self.assertAlmostEqual(flat.point.x, 3.23 / 2, delta=0.0001)
        slot = p.raycast(body, (2, 0, 13), (0, 0, -1))
        self.assertAlmostEqual(slot.point.z, SLOT_FLOOR, delta=0.0001)
        top = p.raycast(body, (0, 2, 13), (0, 0, -1))
        self.assertAlmostEqual(top.point.z, 12, delta=0.0001)

    def test_mounted_inclined_seats_and_spoke_engagement(self):
        for depth, index in ((25, 0), (50, 1), (80, 12)):
            with self.subTest(depth=depth, station=index):
                p = Part((-350, -350, -350), (350, 350, 350), tolerance=0.15)
                placement = spoke_placements(layout(depth))[index]
                raw = tyre_rim_parts(p, rim_depth=depth)["rim"]
                rim, assembly = mount_nipples(p, raw, [placement], rim_depth=depth)
                self.assertTrue(rim.is_watertight())
                self.assertLess(rim.volume(), raw.volume())
                frame = nipple_frame(placement)
                self.assertAlmostEqual(
                    (placement.tip(TIP_EXTENSION) - frame.origin).dot(frame.z),
                    SLOT_FLOOR,
                )
                self.assertLess((frame.z - placement.shaft_axis).norm(), 1e-12)
                radial = vec3(
                    placement.rim_point.x, placement.rim_point.y, 0
                ).normalized()
                self.assertIsNone(p.raycast(rim, radial * 300, radial))
                # A surrounding annulus remains solid: the bore has a bearing seat.
                self.assertIsNotNone(
                    p.raycast(rim, frame.origin + frame.x * 3.5, frame.z)
                )
                spoke = create_spoke(
                    p,
                    placement.head,
                    placement.through,
                    placement.tip(TIP_EXTENSION),
                    "spoke",
                    0.15,
                )
                assembly.fix(assembly.add_part(spoke))
                vertices, facets = nipple(p, name="bearing_reference").mesh()
                bearing_planes = []
                for triangle in facets:
                    a, b, c = (vertices[i] for i in triangle)
                    if all(
                        9.199 <= v.z <= 11.301 and math.hypot(v.x, v.y) > 1.99
                        for v in (a, b, c)
                    ):
                        normal = (b - a).cross(c - a).normalized()
                        if abs(normal.z) < 0.9:
                            bearing_planes.append((a, normal))
                self.assertTrue(bearing_planes)
                for hit in assembly.interferences(min_volume=1e-6):
                    # The native world-coordinate lattice and transformed local
                    # mesh round a nominal touching seat independently. Require
                    # every residual facet to remain on the bearing envelope;
                    # never accept spoke/shank interference by volume alone.
                    self.assertNotIn("spoke", hit.first)
                    self.assertNotIn("spoke", hit.second)
                    self.assertLess(hit.volume, 0.01)
                    positions, triangles = hit.geometry.mesh()
                    for i in {i for triangle in triangles for i in triangle}:
                        point = positions[i] - frame.origin
                        z = point.dot(frame.z)
                        radius = math.hypot(point.dot(frame.x), point.dot(frame.y))
                        self.assertGreaterEqual(z, 9.19)
                        self.assertLessEqual(z, 11.31)
                        self.assertLessEqual(radius - min(3, z - 7.2), 0.002)
                        self.assertGreaterEqual(radius - min(3, z - 7.2), -0.025)
                        local = vec3(point.dot(frame.x), point.dot(frame.y), z)
                        # Independent distance to the real faceted bearing, not
                        # an inflated analytic cone or only an overlap volume.
                        self.assertLess(
                            min(
                                abs((local - a).dot(normal))
                                for a, normal in bearing_planes
                            ),
                            0.002,
                        )


if __name__ == "__main__":
    unittest.main()
