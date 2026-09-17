"""Selected-gear phase and the nominal 12-speed material envelope."""
import math
import unittest
from camber import Part, Frame, set_progress_log
from racing_bike import chain_layout, build_chain
from racing_cassette import make_sprocket
from racing_chain import (CHAIN_Y, INNER_WIDTH, PLATE_THICKNESS, PIN_LENGTH,
                          ROLLER_RADIUS, engagement_phase, crank_pose)
from bike_cassette import CHAIN_PITCH


class RacingChainTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        set_progress_log(False)
        cls.tension_z, cls.pins = chain_layout()
        cls.part = Part((-420,-300,-50),(1420,300,1120),tolerance=.2)
        cls.chain = build_chain(cls.part,cls.pins)

    def test_closed_pitch_and_published_envelope(self):
        self.assertEqual(len(self.pins),108)
        self.assertEqual(INNER_WIDTH,25.4*11/128)
        self.assertEqual(PIN_LENGTH,5.2)
        self.assertEqual(ROLLER_RADIUS,3.885)
        for a,b in zip(self.pins,self.pins[1:]+self.pins[:1]):
            self.assertAlmostEqual(math.dist(a,b),CHAIN_PITCH,delta=.001)
        for name,width in [('chain_pin_58',PIN_LENGTH),('chain_plate_58_1',PLATE_THICKNESS)]:
            vertices,_ = self.part.solid(name).mesh()
            self.assertAlmostEqual(max(v.y for v in vertices)-min(v.y for v in vertices),width,delta=.004)

    def test_selected_cog_seated_rollers_and_adjacent_cog_clearances(self):
        phase=engagement_phase(self.pins,(0.,339.),17)
        self.assertAlmostEqual(phase,-.16537128653754357,places=12)
        for teeth,start in [(19,40.7),(17,44.25),(16,47.8)]:
            frame=Frame((0,-start,339),x=(math.cos(phase),0,math.sin(phase)),
                        y=(-math.sin(phase),0,math.cos(phase)),z=(0,-1,0))
            cog=make_sprocket(self.part,f'chain_fit_cog_{teeth}',frame,teeth,'HG',1.65)
            # The six fully seated rollers must fit the tooth valleys. Entry
            # and exit kinematics are tracked separately as an open issue.
            indices=range(58,64) if teeth==17 else range(56,66)
            for i in indices:
                names=[f'chain_roller_{i}',f'chain_pin_{i}',
                       f'chain_plate_{i}_-1',f'chain_plate_{i}_1']
                for name in names:
                    with self.subTest(teeth=teeth,body=name):
                        self.assertLess(self.part.intersect(cog,self.part.solid(name)).volume(),1e-8)

    def test_actual_freehub_cassette_pose_keeps_seated_rollers_clear(self):
        from racing_cassette import build_cassette
        from racing_hub import build_rear_hub
        from racing_bike import WHEEL_ROTATION
        part=Part((-420,-300,-50),(1420,300,1120),tolerance=.2)
        phase=engagement_phase(self.pins,(0.,339.),17)
        hub=build_rear_hub(part,cassette=build_cassette(part),freehub_angle=phase)
        mounted=part.assembly('mounted_chain_cassette_fit')
        mounted.solve_after_every_constraint=False
        mounted.fix(mounted.add_subassembly(hub,(0,0,339),WHEEL_ROTATION))
        self.assertTrue(mounted.solve().converged)
        from racing_bike import washer
        from racing_chain import ROLLER_LENGTH
        cog=part.solid('cassette_cog_17')
        for i in range(58,64):
            x,z=self.pins[i]
            roller=washer(part,f'mounted_roller_{i}',(x,CHAIN_Y+ROLLER_LENGTH/2,z),
                          ROLLER_RADIUS,1.6,ROLLER_LENGTH)
            with self.subTest(roller=i):
                self.assertLess(part.intersect(cog,roller).volume(),1e-8)

    def test_initial_crank_pose_preserves_bearing_and_seats_rollers(self):
        phase=engagement_phase(self.pins,(404.,267.),52)
        position,rotation=crank_pose(self.pins)
        self.assertAlmostEqual(rotation[1],-math.sin(phase/2),places=14)
        c,s=math.cos(phase),math.sin(phase)
        self.assertAlmostEqual(position[0]+c*404-s*267,404,places=10)
        self.assertAlmostEqual(position[2]+s*404+c*267,267,places=10)
        frame=Frame((404,-44,267),x=(c,0,s),y=(-s,0,c),z=(0,-1,0))
        ring=make_sprocket(self.part,'chain_fit_ring52',frame,52,28,2)
        radius=CHAIN_PITCH/(2*math.sin(math.pi/52))
        engaged=[i for i,(x,z) in enumerate(self.pins)
                 if abs(math.hypot(x-404,z-267)-radius)<.1]
        self.assertGreater(len(engaged),20)
        for i in engaged:
            with self.subTest(roller=i):
                self.assertLess(self.part.intersect(ring,self.part.solid(f'chain_roller_{i}')).volume(),1e-8)
                for side in (-1,1):
                    self.assertLess(self.part.intersect(ring,self.part.solid(f'chain_plate_{i}_{side}')).volume(),1e-8)


if __name__=='__main__':
    unittest.main()
