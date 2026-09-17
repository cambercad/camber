import math
import os
import sys
import unittest
import numpy as np

sys.path.insert(0,os.path.join(os.path.dirname(__file__),".."))
from camber import Part,set_progress_log
from racing_front_derailleur import (build_front_derailleur,CAGE_RADIUS,
    CAGE_OUTSIDE_Y,CAGE_INSIDE_Y,SHEET,STATIC_PIVOTS,MOVING_PIVOTS)
from bike_cassette import sprocket_dims


def new_part():
    set_progress_log(False)
    return Part((-420,-300,-50),(1420,300,1120),tolerance=.2)


class FrontDerailleurTests(unittest.TestCase):
    def test_four_bar_mates_keep_cage_parallel_and_parts_clear(self):
        part=new_part()
        mechanism=build_front_derailleur(part)
        self.assertTrue(mechanism.solve().converged)
        self.assertEqual(4,sum(c["kind"]=="Concentric" for c in mechanism.constraints))
        self.assertEqual([],mechanism.interferences(min_volume=.001))
        bodies=list(mechanism.parts)
        for group in mechanism.subassemblies:
            bodies.extend(group.parts)
        for body in bodies:
            solid=part.solid(body.name)
            with self.subTest(body=body.name):
                self.assertTrue(solid.is_watertight())
                self.assertGreater(solid.volume(),0)
        outer=part.solid("front_derailleur_steel_outer_cage")
        points,triangles=outer.mesh()
        before=np.asarray(points)
        support=next(p for p in mechanism.subassemblies[0].parts if p.name=="front_derailleur_mount")
        link=mechanism.parts[0]
        angle=.1
        mechanism.angle(support.axis_at((0,0,0),(0,0,1)),link.axis_at((0,0,0),(0,1,0)),math.pi/2-angle)
        self.assertTrue(mechanism.solve().converged)
        after=np.asarray(outer.mesh()[0])
        dy=MOVING_PIVOTS[0][1]-STATIC_PIVOTS[0][1]
        dz=MOVING_PIVOTS[0][2]-STATIC_PIVOTS[0][2]
        shift=(0,dy*(math.cos(angle)-1)-dz*math.sin(angle),dy*math.sin(angle)+dz*(math.cos(angle)-1))
        np.testing.assert_allclose(after-before,np.broadcast_to(shift,after.shape),atol=.015,rtol=0)
        self.assertEqual([],mechanism.interferences(min_volume=.001))
        self.assertAlmostEqual(CAGE_RADIUS-sprocket_dims(52)[3],2.)
        self.assertLess(CAGE_OUTSIDE_Y+SHEET,-48.9)
        self.assertGreater(CAGE_INSIDE_Y,-41.5)

    def test_existing_chain_and_52_36_rings_clear_the_cage(self):
        from racing_bike import sprocket,chain_layout,build_chain
        part=new_part()
        derailleur=build_front_derailleur(part)
        fixture=part.assembly("front_derailleur_chain_fit")
        fixture.solve_after_every_constraint=False
        fixture.fix(fixture.add_subassembly(derailleur))
        for teeth,y,bore in ((52,-44,86),(36,-36,56)):
            fixture.fix(fixture.add_part(sprocket(part,f"fit_chainring_{teeth}",(404,y,267),teeth,bore,2)))
        _,pins=chain_layout()
        chain=build_chain(part,pins)
        # Reuse actual nearby chain solids, avoiding unrelated chain/spider
        # contacts elsewhere in the drivetrain when testing this interface.
        near={i for i,(x,z) in enumerate(pins) if 325<x<465 and z>345}
        for occurrence in chain.parts:
            tokens=occurrence.name.split("_")
            if int(tokens[2]) in near:
                fixture.fix(fixture.add_part(part.solid(occurrence.name)))
        self.assertTrue(fixture.solve().converged)
        overlaps=[hit for hit in fixture.interferences(min_volume=.001)
                  if any(path.rsplit("/",1)[-1].startswith("front_derailleur_") for path in (hit.first,hit.second))]
        self.assertEqual([],overlaps)


class FrontDerailleurFrameSupportTests(unittest.TestCase):
    def test_receiver_joins_actual_seat_tube_and_has_blind_screw_clearance(self):
        from racing_bike import build_seat_tube
        from racing_front_derailleur import frame_support_features,MOUNT_DATUM
        part=new_part()
        seat=build_seat_tube(part)
        boss,tools=frame_support_features(part)
        self.assertGreater(part.intersect(seat,boss).volume(),3000)
        joined=part.union(seat,boss)
        support=part.cut(joined,part.batch_union(tools),name="fitted_derailleur_receiver")
        self.assertTrue(support.is_watertight())
        self.assertGreater(support.volume(),seat.volume())
        fixture=part.assembly("front_receiver_fit")
        fixture.solve_after_every_constraint=False
        fixture.fix(fixture.add_part(support))
        fixture.fix(fixture.add_subassembly(build_front_derailleur(part)))
        self.assertTrue(fixture.solve().converged)
        self.assertEqual([],fixture.interferences())
        # Independent ray intersections verify the facing plane and actual
        # blind floor, within the model's 1.84 micrometre coordinate grid.
        face=part.raycast(support,(370,-30,412),(0,1,0))
        floor=part.raycast(support,(376,-30,412),(0,1,0))
        self.assertIsNotNone(face)
        self.assertIsNotNone(floor)
        self.assertAlmostEqual(face.point.y,MOUNT_DATUM[1],delta=.003)
        self.assertAlmostEqual(floor.point.y,-12.5,delta=.003)
        screw=part.solid("front_derailleur_mount_screw")
        points,triangles=screw.mesh()
        end=max(points[i].y for triangle in triangles for i in triangle)
        self.assertAlmostEqual(end-MOUNT_DATUM[1],7.,delta=.003)
        self.assertAlmostEqual(floor.point.y-end,.5,delta=.003)
        # A coaxial probe behind the floor must be completely embedded in
        # material: this catches through-holes and a missing boss back wall.
        from racing_front_derailleur import round_section
        from camber import Frame
        probe=round_section(part,"blind_floor_probe",Frame((376,-12.4,412),
            x=(1,0,0),y=(0,0,-1),z=(0,1,0)),2.,6.3)
        self.assertEqual(part.cut(probe,support).volume(),0.)


if __name__=="__main__":
    unittest.main()
