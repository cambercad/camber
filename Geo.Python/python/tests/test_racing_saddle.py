"""Receiving surfaces and real fasteners of the original saddle head."""
import os
import sys
import unittest
sys.path.insert(0,os.path.join(os.path.dirname(__file__),'..'))
from camber import Part,set_progress_log,vec3
from racing_bike import build_saddle
from racing_saddle import BOLT_X,RAIL_Z,rail_socket_frames


class SaddleClampTests(unittest.TestCase):
    def test_clamped_rails_post_socket_and_fasteners_fit(self):
        set_progress_log(False)
        part=Part((-420,-300,-50),(1420,300,1120),tolerance=.2)
        saddle=build_saddle(part)
        fit=part.assembly('saddle_head_verification')
        for occurrence in saddle.parts:
            solid=part.solid(occurrence.name)
            with self.subTest(solid=solid.name):
                self.assertTrue(solid.is_watertight())
                self.assertGreater(solid.volume(),0)
            item=fit.add_part(solid);fit.fix(item)
        self.assertEqual([],fit.interferences(min_volume=.001))
        lower=part.solid('saddle_lower_cradle')
        upper=part.solid('saddle_upper_clamp')
        shell=part.solid('saddle_shell')
        for side in (-1,1):
            for x in (190,205,220):
                # The gripping span is a true straight 7 mm cylinder. Both
                # semicircular seats leave the specified 0.02 mm radial fit.
                origin=(x,side*22,RAIL_Z)
                rail=part.raycast(part.solid(f'saddle_rail_{side}'),origin,(0,0,1))
                top=part.raycast(upper,origin,(0,0,1))
                bottom=part.raycast(lower,origin,(0,0,-1))
                self.assertAlmostEqual(RAIL_Z+3.5,rail.point.z,delta=.015)
                self.assertAlmostEqual(RAIL_Z+3.52,top.point.z,delta=.015)
                self.assertAlmostEqual(RAIL_Z-3.52,bottom.point.z,delta=.015)
            for end,datum in rail_socket_frames(side):
                for depth in (3,7,10):
                    with self.subTest(side=side,end=end,depth=depth):
                        center=datum.origin+datum.z*depth
                        inside=part.raycast(shell,center,(0,side,0))
                        outside=part.raycast(shell,center+vec3(0,side*25,0),(0,-side,0))
                        self.assertIsNotNone(inside)
                        self.assertIsNotNone(outside)
                        inner_radius=(inside.point-center).dot(vec3(0,side,0))
                        outer_radius=(outside.point-center).dot(vec3(0,side,0))
                        self.assertAlmostEqual(inner_radius,3.5,delta=.02)
                        self.assertGreater(outer_radius-inner_radius,1.5)
                # The pocket is blind: the rail end stops against material,
                # with shell/socket material remaining behind the end face.
                floor=part.raycast(shell,datum.origin+datum.z*5,-datum.z)
                self.assertIsNotNone(floor)
                self.assertAlmostEqual((floor.point-datum.origin).dot(datum.z),0,delta=.02)
        # Socket bosses must belong to the shell, never float beside it.
        points,triangles=shell.mesh()
        parents=list(range(len(points)))
        def root(i):
            while parents[i]!=i:
                parents[i]=parents[parents[i]];i=parents[i]
            return i
        used=set()
        for a,b,c in triangles:
            used.update((a,b,c));parents[root(b)]=root(a);parents[root(c)]=root(a)
        self.assertEqual(1,len({root(i) for i in used}))
        for x in BOLT_X:
            screw=part.solid(f'saddle_clamp_screw_{x}')
            bearing_face=part.raycast(lower,(x,3,918),(0,0,1))
            self.assertAlmostEqual(924,bearing_face.point.z,delta=.003)
            bore=part.raycast(upper,(x,0,924),(0,0,1))
            socket=part.raycast(screw,(x,0,918),(0,0,1))
            tip=part.raycast(screw,(x,0,940),(0,0,-1))
            self.assertAlmostEqual(939,bore.point.z,delta=.003)
            self.assertAlmostEqual(922.5,socket.point.z,delta=.003)
            self.assertAlmostEqual(938,tip.point.z,delta=.003)


class SeatCollarTests(unittest.TestCase):
    def test_split_collar_seats_on_actual_frame_neck(self):
        from racing_bike import build_frame_body,SEAT_COLLAR_TOP as SEAT,SEATPOST_AXIS
        from racing_saddle import seat_collar_frame
        set_progress_log(False)
        part=Part((-420,-300,-50),(1420,300,1120),tolerance=.2)
        frame=build_frame_body(part)
        saddle=build_saddle(part)
        self.assertTrue(frame.is_watertight())
        fit=part.assembly('seat_collar_frame_verification')
        for solid in [frame,*[part.solid(name) for name in
                       ('carbon_seatpost','seat_collar','seat_clamp_bolt')]]:
            item=fit.add_part(solid);fit.fix(item)
        self.assertEqual([],fit.interferences(min_volume=.001))
        datum=seat_collar_frame(SEAT,SEATPOST_AXIS)
        middle=SEAT-SEATPOST_AXIS*6
        neck=part.raycast(frame,middle+datum.x*25,-datum.x)
        bore=part.raycast(part.solid('seat_collar'),middle,datum.x)
        self.assertAlmostEqual(15.9,(neck.point-middle).dot(datum.x),delta=.02)
        self.assertAlmostEqual(15.95,(bore.point-middle).dot(datum.x),delta=.02)
        # The rear slit opens into the receiving bore, but stops below the
        # clamp without splitting the tube farther down toward its junctions.
        open_side=part.raycast(frame,middle+datum.y*30,-datum.y)
        self.assertLess((open_side.point-middle).dot(datum.y),0)
        below=SEAT-SEATPOST_AXIS*21
        closed_side=part.raycast(frame,below+datum.y*30,-datum.y)
        self.assertGreater((closed_side.point-below).dot(datum.y),15.9)
        screw_origin=middle+datum.x*6.6+datum.y*23
        bearing=part.raycast(part.solid('seat_collar'),
                              screw_origin+datum.x*5+datum.y*3,-datum.x)
        socket=part.raycast(part.solid('seat_clamp_bolt'),screw_origin+datum.x*5,-datum.x)
        self.assertAlmostEqual(0,(bearing.point-screw_origin).dot(datum.x),delta=.003)
        self.assertAlmostEqual(1.5,(socket.point-screw_origin).dot(datum.x),delta=.003)


if __name__=='__main__': unittest.main()
