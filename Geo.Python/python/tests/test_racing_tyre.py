"""Section fit tests for the original clincher: inspect materials, not pictures."""
import itertools
import math
import os
import sys
import unittest
sys.path.insert(0,os.path.join(os.path.dirname(__file__),".."))
from camber import Part,set_progress_log
from racing_tyre import tyre_rim_sections,tyre_rim_parts,build_tyre_rim,meridian


class TyreRimTests(unittest.TestCase):
    def setUp(self):
        set_progress_log(False)
        self.part=Part((-350,-350,-30),(350,350,30),tolerance=.1)

    def test_crown_dimensions_and_tangent_shoulders(self):
        tyre=tyre_rim_sections(self.part)["tyre"]
        for surface,radius,height in (("outer",14,325),("inner",13,324)):
            center=tyre.eval_xy(surface+"_crown@center")
            self.assertAlmostEqual(center[0],0,places=8)
            self.assertAlmostEqual(center[1],height,places=8)
            for side in (-1,1):
                shoulder=tyre.eval_xy(f"{surface}_shoulder_{side}@center")
                # Common contact lies on the horizontal line of centres:
                # its tangent is vertical, independently of sampled mesh.
                self.assertAlmostEqual(shoulder[1],height,places=7)
            self.assertAlmostEqual(abs(tyre.eval_xy(surface+"_crown@0")[0]),radius,places=7)

    def test_material_sections_are_disjoint_at_seats_hooks_and_core_cavities(self):
        for depth in (25,50,80):
            with self.subTest(rim_depth=depth):
                sections=tyre_rim_sections(self.part,name=f"d{depth}",rim_depth=depth)
                solids={key:self.part.extrude(sketch,1,name=f"section_{depth}_{key}",max_deviation=.005)
                        for key,sketch in sections.items()}
                for key,solid in solids.items():
                    self.assertTrue(solid.is_watertight(),key)
                    self.assertGreater(solid.volume(),0)
                for (a,first),(b,second) in itertools.combinations(solids.items(),2):
                    self.assertAlmostEqual(self.part.intersect(first,second).volume(),0,delta=1e-8,msg=f"{a}/{b}")

    def test_drop_centre_slopes_have_real_laminate_thickness(self):
        rim_section=tyre_rim_sections(self.part)["rim"]
        rim=self.part.extrude(rim_section,1,name="rim_probe_section",max_deviation=.005)
        for side in (-1,1):
            probe=self.part.sketch(frame=meridian(),name=f"laminate_probe_{side}")
            probe.add_circle((side*7,307.6),.6)
            disc=self.part.extrude(probe,1,name=f"laminate_disc_{side}",max_deviation=.005)
            # A 1.2 mm disk must fit inside each sloping laminate wall.
            self.assertAlmostEqual(self.part.cut(disc,rim).volume(),0,delta=1e-8)

    def test_revolved_materials_and_rigid_mount_mates(self):
        bodies=tyre_rim_parts(self.part,max_deviation=.02)
        for key,solid in bodies.items():
            self.assertTrue(solid.is_watertight(),key)
            self.assertGreater(solid.volume(),0)
        # Plausible rubber envelope for a 28 mm road tyre, excluding cords.
        self.assertGreater(bodies["tyre"].volume(),180000)
        self.assertLess(bodies["tyre"].volume(),260000)
        # Pappus' theorem checks the separate bead envelopes independently.
        expected=2*math.pi*312.05*math.pi*.65**2
        self.assertAlmostEqual(bodies["bead_1"].volume(),expected,delta=expected*.03)
        assembly=build_tyre_rim(self.part,name="mounted")
        self.assertEqual([],assembly.interferences(min_volume=1e-8))
        self.assertEqual(len(assembly.parts),4)
        self.assertEqual(sum(c['kind']=='FixPart' for c in assembly.constraints),1)
        self.assertEqual(sum(c['kind']=='Concentric' for c in assembly.constraints),3)

    def test_tape_covers_access_apertures_and_leaves_valve_seat_exposed(self):
        from racing_tyre import rim_tape
        from racing_valve import valve_frame
        from camber import vec3
        tape=rim_tape(self.part)
        self.assertTrue(tape.is_watertight())
        self.assertGreater(tape.volume(),0)
        # Check the centre and aperture perimeter of all 24 radial 6.5 mm
        # access holes, including the spoke stations closest to the valve.
        for station in range(24):
            angle=2*math.pi*station/24
            radial=vec3(math.cos(angle),math.sin(angle),0)
            tangent=vec3(-math.sin(angle),math.cos(angle),0)
            for u,v in ((0,0),(3.25,0),(-3.25,0),(0,3.25),(0,-3.25)):
                hit=self.part.raycast(tape,radial*320+tangent*u+vec3(0,0,v),-radial)
                self.assertIsNotNone(hit,(station,u,v))
                self.assertGreater(hit.point.dot(radial),306.8)
                self.assertLess(hit.point.dot(radial),307.3)
        # Two opposite probes independently measure actual film thickness.
        outer=self.part.raycast(tape,(320,0,0),(-1,0,0))
        inner=self.part.raycast(tape,(306,0,0),(1,0,0))
        self.assertAlmostEqual(.15,outer.point.x-inner.point.x,delta=.002)
        valve=valve_frame()
        self.assertIsNone(self.part.raycast(tape,valve.origin,valve.z))


if __name__=="__main__":
    unittest.main()
