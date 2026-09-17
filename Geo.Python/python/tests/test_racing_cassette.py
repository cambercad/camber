import math
import os
import sys
import unittest

sys.path.insert(0,os.path.join(os.path.dirname(__file__),".."))
from camber import Frame,Part,set_progress_log
from racing_cassette import build_cassette,lockring_tool,make_sprocket
from racing_hub import build_rear_hub


class CassetteTests(unittest.TestCase):
    def test_lockring_seats_and_tool_engagement(self):
        set_progress_log(False)
        part = Part((-420,-300,-50),(1420,300,1120),tolerance=.2)
        cassette = build_cassette(part)
        self.assertTrue(cassette.solve().converged)
        self.assertEqual(2,sum(c["kind"]=="Concentric" for c in cassette.constraints))
        ring = part.solid("cassette_lockring")
        self.assertTrue(ring.is_watertight())
        self.assertGreater(ring.volume(),0)
        for occurrence in cassette.parts:
            if occurrence.name == ring.name:
                continue
            with self.subTest(component=occurrence.name):
                self.assertLessEqual(part.intersect(ring,part.solid(occurrence.name)).volume(),.001)
        # Independent nominal tool, smaller than its receiving spline. A half
        # tooth-pitch misalignment must engage the flanks rather than spin freely.
        tool = lockring_tool(part,"nominal_tool",major=11.7,minor=10.6,half_width=5)
        self.assertLessEqual(part.intersect(ring,tool).volume(),.001)
        misaligned = lockring_tool(part,"misaligned_tool",major=11.7,minor=10.6,
                                   half_width=5,clocking=15)
        self.assertGreater(part.intersect(ring,misaligned).volume(),1)
        mounted = build_rear_hub(part)
        self.assertTrue(mounted.solve().converged)
        freehub = part.solid("rear_freehub")
        self.assertTrue(freehub.is_watertight())
        self.assertLessEqual(part.intersect(ring,freehub).volume(),.001)


    def test_hg_sliding_fit_clocking_and_torque_engagement(self):
        set_progress_log(False)
        part = Part((-100,-100,-100),(100,100,100),tolerance=.2)
        hub = build_rear_hub(part)
        self.assertTrue(hub.solve().converged)
        cassette = build_cassette(part)
        self.assertTrue(cassette.solve().converged)
        freehub = part.solid("rear_freehub")
        for occurrence in cassette.parts:
            solid = part.solid(occurrence.name)
            with self.subTest(component=solid.name):
                self.assertTrue(solid.is_watertight())
                self.assertGreater(solid.volume(),0)
                self.assertLessEqual(part.intersect(freehub,solid).volume(),.001)
        # A small assembly allowance must permit sliding; both torque directions
        # meet a flank, and a large wrong clocking must be rejected by the key.
        for angle,engaged in ((-.1,False),(.1,False),(-2,True),(2,True),(40,True)):
            a = math.radians(angle)
            frame = Frame((0,0,40),x=(math.cos(a),math.sin(a),0),
                          y=(-math.sin(a),math.cos(a),0),z=(0,0,1))
            cog = make_sprocket(part,f"clocking_{angle}",frame,19,"HG",1.65)
            with self.subTest(clocking=angle):
                overlap = part.intersect(freehub,cog).volume()
                if engaged:
                    self.assertGreater(overlap,1)
                else:
                    self.assertLessEqual(overlap,.001)
