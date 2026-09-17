"""Physical fit checks for the original-design M5 cage mounting interfaces."""
import os
import sys
import unittest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from camber import Part, set_progress_log
from racing_bike import (BOTTLE_AXIS, build_accessories, build_down_tube,
                         bottle_mount_frame, bottle_support_features, assembly, rigid)


class BottleMountTests(unittest.TestCase):
    def test_screws_seat_without_overlap_and_blind_bosses_receive_shanks(self):
        set_progress_log(False)
        part = Part((-420,-300,-50),(1420,300,1120),tolerance=.2)
        accessories = build_accessories(part)
        tube = build_down_tube(part)
        bosses, tools = bottle_support_features(part)
        for boss in bosses:
            self.assertGreater(part.intersect(tube,boss).volume(),50)
        support = part.batch_union([tube,*bosses])
        for tool in tools:
            support = part.cut(support,tool)
        self.assertTrue(support.is_watertight())
        fitted = assembly(part,'bottle_mount_verification')
        rigid(fitted,support)
        occurrence = fitted.add_subassembly(accessories)
        fitted.fix(occurrence)
        fitted.solve()
        self.assertEqual([],fitted.interferences(min_volume=.001))
        for i,height in enumerate((30,94)):
            with self.subTest(mount=i):
                frame = bottle_mount_frame(height)
                screw = part.solid(f'bottle_cage_bolt_{i}')
                spacer = part.solid(f'bottle_cage_mount_{i}')
                self.assertTrue(screw.is_watertight())
                # The blind bore has 2 mm beyond the 12 mm shank; the 4 mm
                # head retains a physical hex recess, rather than an open washer.
                floor = part.raycast(support,frame.origin,frame.z)
                self.assertAlmostEqual(14,(floor.point-frame.origin).dot(frame.z),delta=.003)
                tip = part.raycast(screw,frame.origin+frame.z*16,-frame.z)
                self.assertAlmostEqual(12,(tip.point-frame.origin).dot(frame.z),delta=.003)
                socket = part.raycast(screw,frame.origin-frame.z*5,frame.z)
                self.assertAlmostEqual(-1.5,(socket.point-frame.origin).dot(frame.z),delta=.003)
                land = part.raycast(support,frame.origin+frame.x*3,frame.z)
                front = part.raycast(spacer,frame.origin+frame.x*3,frame.z)
                self.assertAlmostEqual(4,(land.point-frame.origin).dot(frame.z),delta=.003)
                self.assertAlmostEqual(2,(front.point-frame.origin).dot(frame.z),delta=.003)
        # A straight pull cannot pass the narrowed shoulder hoop without cage
        # flex, confirming geometric retention rather than a floating bottle.
        retained = assembly(part,'bottle_shoulder_retention')
        rigid(retained,part.solid(accessories.parts[0].name))
        rigid(retained,part.solid('bottle_shell'),BOTTLE_AXIS*15)
        self.assertGreater(sum(hit.volume for hit in retained.interferences(min_volume=.001)),1)


if __name__ == '__main__':
    unittest.main()
