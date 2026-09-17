import itertools
import os
import sys
import unittest
sys.path.insert(0,os.path.join(os.path.dirname(__file__),".."))
from camber import Part,set_progress_log
from racing_control import control_bar_frame
from racing_bike import build_cockpit, washer, HEAD_TOP, HEAD_BOTTOM, STEERING_AXIS


class ControlTests(unittest.TestCase):
    def test_pivots_and_mounted_control_clearances(self):
        set_progress_log(False)
        part=Part((-420,-300,-50),(1420,300,1120),tolerance=.2)
        cockpit=build_cockpit(part)
        cockpit.solve()
        self.assertTrue(part.solid("carbon_handlebar").is_watertight())
        stem_fit = part.assembly("stem_interface_check")
        for occurrence in cockpit.parts:
            if occurrence.name in ("carbon_stem", "stem_faceplate", "stem_clamp_screw",
                                   "stem_top_cap", "carbon_handlebar"):
                solid = part.solid(occurrence.name)
                self.assertTrue(solid.is_watertight(), occurrence.name)
                stem_fit.add_part(solid, occurrence.pose)
        stem_fit.add_part(washer(part, "steerer_fit", HEAD_BOTTOM, 14.3, 0,
            (HEAD_TOP-HEAD_BOTTOM).norm()+48, axis=STEERING_AXIS))
        from racing_stem import headset_stack
        for spacer in headset_stack(part,HEAD_TOP,STEERING_AXIS):
            stem_fit.add_part(spacer)
        self.assertEqual([], stem_fit.interferences(min_volume=.01))
        for side in (-1,1):
            with self.subTest(side=side, interface="hood_receives_wrapped_bar"):
                self.assertAlmostEqual(0, part.intersect(
                    part.solid(f"rubber_shift_hood_{side}"),
                    part.solid(f"rubber_bar_tape_{side}")).volume(), delta=1e-6)
            outlet=part.solid(f'control_{side}_hydraulic_hose_nipple')
            hose=part.solid(f'rubber_hydraulic_hose_{side}')
            bracket=part.solid(f'control_{side}_pivot_bracket')
            for first,second in [(outlet,bracket),(outlet,hose),(hose,bracket),
                                 (hose,part.solid('carbon_handlebar')),
                                 (hose,part.solid(f'rubber_shift_hood_{side}')),
                                 (outlet,part.solid(f'rubber_shift_hood_{side}'))]:
                with self.subTest(hydraulic_fit=(first.name,second.name)):
                    self.assertAlmostEqual(0,part.intersect(first,second).volume(),delta=1e-6)
            self.assertTrue(hose.is_watertight())
            self.assertTrue(outlet.is_watertight())
            # An actual receiving floor sits 1 mm beyond the inserted nipple.
            hit=part.raycast(bracket,(978,side*211+2,886),(1,0,0))
            self.assertIsNotNone(hit)
            self.assertAlmostEqual(989,hit.point.x,delta=.002)
            pieces={name:part.solid(f'control_{side}_{name}')
                    for name in ('brake_blade','pivot_bracket','pivot_pin','pivot_retainer','pivot_bush_-1','pivot_bush_1','bar_band','band_screw','switch_pod','shift_button_upper','shift_button_lower')}
            # The captured band grips bare structural carbon, and all metal
            # carrier material stays outside that tube and its installed tape.
            for name in ('bar_band','band_screw','pivot_bracket'):
                self.assertAlmostEqual(0,part.intersect(pieces[name],
                    part.solid('carbon_handlebar')).volume(),delta=1e-6)
            band_frame,_,_,_=control_bar_frame(side)
            bolt_origin=band_frame.origin+band_frame.y*18+band_frame.x*6.6
            bolt_axis=-band_frame.x
            # M5 seats on the outer lug; its 12 mm shank ends 0.5 mm
            # before the blind receiving floor. The socket remains accessible.
            for solid,expected in ((pieces['bar_band'],12.5),(pieces['band_screw'],-1.5)):
                hit=part.raycast(solid,bolt_origin-bolt_axis*6,bolt_axis)
                self.assertIsNotNone(hit)
                self.assertAlmostEqual(expected,(hit.point-bolt_origin).dot(bolt_axis),delta=.002)
            # A machinable carrier is one connected solid, not detached islands
            # left by its band pocket or driver access cut.
            points,triangles=pieces['pivot_bracket'].mesh()
            parents=list(range(len(points)))
            def root(index):
                while parents[index]!=index:
                    parents[index]=parents[parents[index]];index=parents[index]
                return index
            used=set()
            for a,b,c in triangles:
                used.update((a,b,c));parents[root(b)]=root(a);parents[root(c)]=root(a)
            self.assertEqual(1,len({root(index) for index in used}))
            # The drive recess is accessible from outside and has a solid floor;
            # the surrounding head retains its original bearing face.
            pin = pieces["pivot_pin"]
            socket_floor = part.raycast(pin, (1033, side*200+18, 878), (0,-1,0))
            head_face = part.raycast(pin, (1036, side*200+18, 878), (0,-1,0))
            self.assertIsNotNone(socket_floor)
            self.assertIsNotNone(head_face)
            self.assertAlmostEqual(side*200+14.6, socket_floor.point.y, delta=.002)
            self.assertAlmostEqual(side*200+16, head_face.point.y, delta=.002)
            # An actual opposing fastener prevents the pin sliding out. Its
            # shank stops 0.5 mm short of the blind receiving bore floor.
            receiving_floor = part.raycast(pin,(1033,side*200-18,878),(0,1,0))
            retainer_floor = part.raycast(pieces["pivot_retainer"],
                                         (1033,side*200-18,878),(0,1,0))
            self.assertAlmostEqual(side*200-9,receiving_floor.point.y,delta=.002)
            self.assertAlmostEqual(side*200-15.4,retainer_floor.point.y,delta=.002)
            for end in (-1,1):
                bush=pieces[f"pivot_bush_{end}"]
                # Both bearing bores are concentric Ø5 envelopes; the flange
                # physically occupies the available gap beside the blade.
                bearing=part.raycast(bush,(1033,side*200+end*3,878),(1,0,0))
                self.assertAlmostEqual(1035.5,bearing.point.x,delta=.01)
                flange=part.raycast(bush,(1037,side*200+end*9,878),(0,-end,0))
                self.assertAlmostEqual(side*200+end*7.8,flange.point.y,delta=.002)
            for name,solid in pieces.items():
                with self.subTest(side=side,solid=name):
                    self.assertTrue(solid.is_watertight())
                    self.assertGreater(solid.volume(),0)
            for (a,first),(b,second) in itertools.combinations(pieces.items(),2):
                with self.subTest(side=side,pair=(a,b)):
                    self.assertAlmostEqual(0,part.intersect(first,second).volume(),delta=1e-6)
            for name,solid in pieces.items():
                for obstacle in ('rubber_shift_hood','rubber_bar_tape'):
                    with self.subTest(side=side,part=name,obstacle=obstacle):
                        self.assertAlmostEqual(0,part.intersect(solid,part.solid(f'{obstacle}_{side}')).volume(),delta=1e-6)
