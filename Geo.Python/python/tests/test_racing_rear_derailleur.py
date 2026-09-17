import math
import os
import sys
import unittest
import numpy as np
sys.path.insert(0,os.path.join(os.path.dirname(__file__),".."))
from camber import Part,set_progress_log
from racing_rear_derailleur import (build_rear_derailleur,frame_support_tools,
                                    MOUNT_DATUM,GUIDE,DEFAULT_TENSION_Z)


def new_part():
    set_progress_log(False)
    return Part((-420,-300,-50),(1420,300,1120),tolerance=.2)


class RearDerailleurTests(unittest.TestCase):
    def test_mechanical_groups_are_closed_clear_and_pulleys_spin_independently(self):
        p=new_part();a=build_rear_derailleur(p)
        self.assertTrue(a.solve().converged)
        self.assertEqual(8,sum(c['kind']=='Concentric' for c in a.constraints))
        self.assertEqual([],a.interferences(min_volume=.001))
        for body in list(a.parts)+[b for g in a.subassemblies for b in g.parts]:
            with self.subTest(body=body.name):
                solid=p.solid(body.name)
                self.assertTrue(solid.is_watertight());self.assertGreater(solid.volume(),0)
        guide=next(b for b in a.parts if b.name=='rear_derailleur_guide_pulley')
        tension=next(b for b in a.parts if b.name=='rear_derailleur_tension_pulley')
        cage=next(b for g in a.subassemblies for b in g.parts if b.name=='rear_derailleur_outer_cage')
        # Ground the cage only for this independent-spin test; the production
        # mechanism keeps its actual cage and four-bar degrees of freedom.
        a.fix(cage)
        before=np.asarray(p.solid(tension.name).mesh()[0]).copy()
        a.angle(cage.axis_at(GUIDE,(1,0,0)),guide.axis_at(GUIDE,(0,0,1)),math.pi/2-.15)
        self.assertTrue(a.solve().converged)
        np.testing.assert_allclose(p.solid(tension.name).mesh()[0],before,atol=1e-8,rtol=0)
        self.assertEqual([],a.interferences(min_volume=.001))

    def test_service_cover_is_hollow_and_fasteners_have_blind_sockets(self):
        p=new_part();a=build_rear_derailleur(p)
        self.assertTrue(a.solve().converged)
        cover=p.solid('rear_derailleur_service_cover')
        # The receiving side is open; only the crowned end wall closes the
        # central ray. The two sides leave a real 1.5 mm central wall.
        inside=p.raycast(cover,(-41,-76,300),(-1,0,0))
        outside=p.raycast(cover,(-60,-76,300),(1,0,0))
        self.assertIsNotNone(inside);self.assertIsNotNone(outside)
        self.assertAlmostEqual(inside.point.x,-55.5,delta=.004)
        self.assertAlmostEqual(outside.point.x,-57,delta=.004)
        for i,z in enumerate((296,306)):
            screw=p.solid(f'rear_derailleur_service_screw_{i}')
            floor=p.raycast(screw,(-60,-80,z),(1,0,0))
            self.assertIsNotNone(floor)
            self.assertAlmostEqual(floor.point.x,-55.6,delta=.004)

    def test_covered_four_bar_moves_without_binding(self):
        from racing_rear_derailleur import STATIC_PIVOTS, MOVING_PIVOTS
        for angle in (-.2,.2):
            with self.subTest(angle=angle):
                p=new_part();a=build_rear_derailleur(p)
                self.assertTrue(a.solve().converged)
                bodies={b.name:b for g in a.subassemblies for b in g.parts}
                upper=bodies['rear_derailleur_fixed_knuckle']
                lower=bodies['rear_derailleur_moving_knuckle']
                cage=bodies['rear_derailleur_outer_cage']
                # Hold B adjustment and cage spring angle for this isolated
                # four-bar travel test. Production leaves both joints free.
                a.fix(upper)
                a.parallel(lower.axis_at((0,0,0),(0,0,1)),cage.axis_at((0,0,0),(0,0,1)))
                link=next(b for b in a.parts if b.name=='rear_derailleur_link_0')
                before=np.asarray(p.solid(lower.name).mesh()[0]).copy()
                cover_before=np.asarray(p.solid('rear_derailleur_service_cover').mesh()[0]).copy()
                a.angle(upper.axis_at((0,0,0),(0,0,1)),link.axis_at((0,0,0),(0,1,0)),math.pi/2-angle)
                self.assertTrue(a.solve().converged)
                dy=MOVING_PIVOTS[0][1]-STATIC_PIVOTS[0][1]
                dz=MOVING_PIVOTS[0][2]-STATIC_PIVOTS[0][2]
                shift=(0,dy*(math.cos(angle)-1)-dz*math.sin(angle),
                       dy*math.sin(angle)+dz*(math.cos(angle)-1))
                after=np.asarray(p.solid(lower.name).mesh()[0])
                np.testing.assert_allclose(after-before,np.broadcast_to(shift,after.shape),atol=.001,rtol=0)
                np.testing.assert_allclose(p.solid('rear_derailleur_service_cover').mesh()[0],cover_before,atol=1e-8,rtol=0)
                self.assertEqual([],a.interferences(min_volume=.001))

    def test_actual_chain_entry_wrap_exit_clears_jockey_teeth_and_cage(self):
        from racing_bike import chain_layout,build_chain
        p=new_part();z,pins=chain_layout();a=build_rear_derailleur(p,z,chain_pins=pins)
        f=p.assembly('rear_chain_fit');f.solve_after_every_constraint=False
        f.fix(f.add_subassembly(a));chain=build_chain(p,pins)
        for body in chain.parts:
            x,z=pins[int(body.name.split('_')[2])]
            if x<65 and z<325:f.fix(f.add_part(p.solid(body.name)))
        self.assertTrue(f.solve().converged)
        overlaps=[h for h in f.interferences(min_volume=.001)
                  if any('rear_derailleur' in path for path in (h.first,h.second))]
        self.assertEqual([],overlaps)

    def test_original_jockey_crest_is_dimensioned_and_invalid_sizes_are_rejected(self):
        from racing_cassette import make_sprocket
        from racing_rear_derailleur import side,PULLEY_TIP_RADIUS
        from bike_cassette import sprocket_dims
        p=new_part()
        wheel=make_sprocket(p,"dimensioned_jockey",side(0),11,3.1,2,tip_radius=PULLEY_TIP_RADIUS)
        points,triangles=wheel.mesh()
        radius=max(math.hypot(points[i].x,points[i].z) for tri in triangles for i in tri)
        # Arc tessellation is inscribed; a sampled crest need not land at
        # its analytic radial maximum. Bound both chord and lattice error.
        self.assertGreaterEqual(radius,PULLEY_TIP_RADIUS-.01-math.sqrt(2)*1840/999999)
        self.assertLessEqual(radius,PULLEY_TIP_RADIUS+math.sqrt(2)*1840/999999)
        for index,value in enumerate((0,sprocket_dims(11)[1],30,float("nan"),float("inf"))):
            with self.subTest(tip_radius=value):
                with self.assertRaisesRegex(ValueError,"tip_radius"):
                    make_sprocket(p,f"invalid_jockey_{index}",side(0),11,3.1,2,tip_radius=value)

    def test_hanger_seats_on_actual_dropout_dimensions_and_blind_receivers(self):
        from racing_bike import washer
        p=new_part()
        # Same drive-side dropout blank and axle bore as build_frame_body.
        dropout=washer(p,'drive_dropout',(0,-78,339),13,6,7,axis=(0,1,0))
        for tool in frame_support_tools(p):dropout=p.cut(dropout,tool)
        self.assertTrue(dropout.is_watertight())
        f=p.assembly('hanger_receiver_fit');f.solve_after_every_constraint=False
        f.fix(f.add_part(dropout));f.fix(f.add_subassembly(build_rear_derailleur(p)))
        self.assertTrue(f.solve().converged)
        self.assertEqual([],f.interferences(min_volume=.001))
        for x in (-9,9):
            floor=p.raycast(dropout,(x,-90,339),(0,1,0))
            self.assertIsNotNone(floor)
            self.assertAlmostEqual(floor.point.y,-74,delta=.003)
        self.assertEqual(MOUNT_DATUM,(0,-78,339))


if __name__=='__main__':unittest.main()
