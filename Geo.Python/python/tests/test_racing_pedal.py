import unittest
from camber import Part, set_progress_log
from racing_pedal import pedal_parts, build_pedal, crank_pedal_center


class RacingPedalTests(unittest.TestCase):
    def test_components_and_clearances_survive_assembly_update(self):
        set_progress_log(False)
        part = Part((-100, -100, -50), (100, 100, 50), tolerance=.035)
        pieces = pedal_parts(part)
        assembly = part.assembly("inspection")
        assembly.solve_after_every_constraint = False
        for name, solid in pieces.items():
            with self.subTest(component=name):
                self.assertTrue(solid.is_watertight())
                self.assertGreater(solid.volume(), 0)
            assembly.fix(assembly.add_part(solid))
        # The identity solve used to re-quantize CSG vertices and corrupt
        # exact cylindrical contacts. Audit the assembled geometry as well.
        assembly.solve()
        for name in ("spindle", "jaw", "jaw_pin", "wear_centre", "wear_left",
                     "wear_right", "inboard_outer", "outboard_outer", "bearing_spacer",
                     "bearing_nut", "inboard_cap", "outboard_cap"):
            with self.subTest(body_interface=name):
                overlap = part.intersect(pieces["body"], pieces[name])
                self.assertEqual(0, overlap.triangle_count)

    def test_pedal_mechanism_solves_with_two_revolute_interfaces(self):
        set_progress_log(False)
        part = Part((-100, -100, -50), (100, 100, 50), tolerance=.05)
        pedal = build_pedal(part, side=-1)
        self.assertTrue(pedal.solve().converged)
        self.assertEqual(2, sum(c["kind"] == "Concentric" for c in pedal.constraints))
        self.assertEqual(2, sum(c["kind"] == "CoincidentPlanes" for c in pedal.constraints))
        self.assertEqual(1, sum(c["kind"] == "FixPart" for c in pedal.constraints))

    def test_translated_left_wear_plate_contact_in_bike_operating_space(self):
        set_progress_log(False)
        part = Part((-420, -300, -50), (1420, 300, 1120), tolerance=.1)
        pieces = pedal_parts(part, side=-1)
        assembly = part.assembly("wear_plate_contact")
        assembly.solve_after_every_constraint = False
        position = crank_pedal_center(-1)
        position.y -= 52
        for key in ("wear_right", "body"):
            assembly.fix(assembly.add_part(pieces[key], position))
        assembly.solve()
        # CSG previously tried to triangulate a zero-area polygon when a
        # constraint passed through existing collinear vertices on this pair.
        self.assertEqual([], assembly.interferences())


class MountedCrankInitializationTests(unittest.TestCase):
    def test_both_pedals_preserve_requested_free_crank_phase(self):
        import math
        import numpy as np
        from racing_bike import (build_drivetrain, chain_layout, mount_crankset,
                                 washer, BB)
        from racing_chain import crank_pose, engagement_phase
        from racing_pedal import mount_pedal
        set_progress_log(False)
        part=Part((-420,-300,-50),(1420,300,1120),tolerance=.2)
        bike=part.assembly('crank_and_pedals_fit')
        bike.solve_after_every_constraint=False
        frame=part.assembly('bearing_support')
        frame.fix(frame.add_part(washer(part,'frame_shell',(404,34,267),25,15,68)))
        support=bike.add_subassembly(frame)
        bike.fix(support)
        pins=chain_layout()[1]
        pose=crank_pose(pins)
        phase=engagement_phase(pins,(BB.x,BB.z),52)
        drivetrain=build_drivetrain(part)
        points,triangles=part.solid('crank_arm_-1').mesh()
        used=sorted({i for triangle in triangles for i in triangle})
        before=np.asarray([tuple(points[i]) for i in used])
        crankset=bike.add_subassembly(drivetrain,*pose)
        mount_crankset(bike,support,crankset)
        for side in (-1,1):
            arm=next(p for p in crankset.parts if p.name==f'crank_arm_{side}')
            mount_pedal(part,bike,arm,crank_pedal_center(side,BB),side,
                        initial_parent_pose=pose)
        self.assertTrue(bike.solve().converged)
        self.assertEqual(1,sum(c['kind']=='FixPart' for c in bike.constraints))
        self.assertFalse(any(c['kind']=='Angle' for c in bike.constraints))
        solved_points,_=part.solid('crank_arm_-1').mesh()
        after=np.asarray([tuple(solved_points[i]) for i in used])
        a=before[:,(0,2)]-np.asarray((BB.x,BB.z))
        b=after[:,(0,2)]-np.asarray((BB.x,BB.z))
        measured=math.atan2(np.sum(a[:,0]*b[:,1]-a[:,1]*b[:,0]),np.sum(a*b))
        self.assertAlmostEqual(measured,phase,delta=1e-7)
        c,s=math.cos(phase),math.sin(phase)
        expected=before.copy()
        expected[:,0]=BB.x+c*a[:,0]-s*a[:,1]
        expected[:,2]=BB.z+s*a[:,0]+c*a[:,1]
        np.testing.assert_allclose(after,expected,atol=.002,rtol=0)
