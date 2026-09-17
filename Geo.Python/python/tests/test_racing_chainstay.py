"""Actual frame clearance to both chainrings and the rear tyre."""
import math
import unittest
from camber import Part,set_progress_log
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from racing_bike import build_frame_body,REAR
from racing_chainring import chainring
from racing_tyre import tyre_rim_parts

class ChainstayClearanceTests(unittest.TestCase):
    def test_frame_clears_both_rings_and_rear_tyre(self):
        set_progress_log(False)
        part=Part((-420,-300,-50),(1420,300,1120),tolerance=.2)
        frame=build_frame_body(part)
        self.assertTrue(frame.is_watertight())
        for teeth in (36,52):
            with self.subTest(chainring=teeth):
                self.assertAlmostEqual(0,part.intersect(frame,chainring(part,teeth)).volume(),delta=1e-7)
        tyre=tyre_rim_parts(part,name='clearance_rear')['tyre']
        assembly=part.assembly('frame_tyre_clearance')
        assembly.solve_after_every_constraint=False
        fixed=assembly.add_part(frame)
        rear=assembly.add_part(tyre,REAR,(math.sqrt(.5),0,0,math.sqrt(.5)))
        assembly.fix(fixed);assembly.fix(rear);assembly.solve()
        self.assertEqual([],assembly.interferences(min_volume=1e-7))
