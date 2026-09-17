import math
import os
import sys
import unittest
sys.path.insert(0,os.path.join(os.path.dirname(__file__),'..'))
from camber import Part,set_progress_log
from racing_computer import build_computer,MOUNT_DATUM,RECEIVER_CENTER


def new_part():
    set_progress_log(False)
    return Part((-420,-300,-50),(1420,300,1120),tolerance=.2)


class CyclingComputerTests(unittest.TestCase):
    def test_enclosure_and_mount_have_closed_bodies_and_clear_receiving_interfaces(self):
        p=new_part();a=build_computer(p)
        self.assertTrue(a.solve().converged)
        self.assertEqual([],a.interferences(min_volume=.001))
        bodies=[body for group in a.subassemblies for body in group.parts]
        self.assertGreaterEqual(len(bodies),20)
        for body in bodies:
            with self.subTest(body=body.name):
                solid=p.solid(body.name)
                self.assertTrue(solid.is_watertight());self.assertGreater(solid.volume(),0)
        self.assertEqual(1,sum(c['kind']=='Concentric' for c in a.constraints))
        self.assertEqual(1,sum(c['kind']=='DistancePlanes' for c in a.constraints))

    def test_quarter_turn_sweeps_clear_and_lips_retain_the_locked_device(self):
        for angle,lift,retained in ((0,0,False),(math.pi/4,0,False),
                (math.pi/2,0,False),(math.pi/2,4.,False),(0,1.,True)):
            with self.subTest(clocking=angle,lift=lift):
                p=new_part();a=build_computer(p,clocking=angle,lift=lift)
                self.assertTrue(a.solve().converged)
                hits=a.interferences(min_volume=.001)
                if retained:
                    self.assertTrue(hits)
                    self.assertTrue(all('computer_quarter_turn_receiver' in h.first+h.second and
                                        'computer_lower_case' in h.first+h.second for h in hits))
                else:self.assertEqual([],hits)

    def test_actual_stem_faceplate_and_handlebar_receive_the_adapter(self):
        from racing_bike import build_cockpit
        p=new_part();cockpit=build_cockpit(p);report=cockpit.solve()
        self.assertTrue(report.converged,repr(report))
        computer=next(child for child in cockpit.subassemblies if child.name=='cycling_computer')
        self.assertEqual(2,len(computer.subassemblies))
        self.assertNotIn('computer',[body.name for body in cockpit.parts])
        self.assertNotIn('computer_mount',[body.name for body in cockpit.parts])
        screws=[body for body in cockpit.parts if body.name=='stem_clamp_screw']
        self.assertEqual(2,len(screws))
        for screw in screws:
            self.assertAlmostEqual(screw.pose[2],882,delta=1e-6)
        # Audit the integrated component against every actual cockpit body,
        # including hoses, controls, tape and the retained upper screw pair.
        computer_hits=[hit for hit in cockpit.interferences(min_volume=.001)
                       if 'computer' in hit.first+hit.second]
        self.assertEqual([],computer_hits)
        # Explicit datums carry empty entity names; inspect the recorded physical
        # connection by its two component names in the solver's mate labels.
        receiving=[mate for mate in report.mates
                   if 'computer_mount_bracket' in mate.label and 'stem_faceplate' in mate.label]
        self.assertEqual(['Concentric','CoincidentPlanes','CoincidentPlanes'],[mate.kind for mate in receiving])
        # Actual lower screw receiving lands: same tip x876 and 12 mm of
        # existing thread engagement up to the rear clamp split at x888.
        screw=p.solid('computer_faceplate_screw_0')
        points,triangles=screw.mesh()
        self.assertAlmostEqual(min(points[i].x for tri in triangles for i in tri),876,delta=.003)
        sleeve=p.solid('computer_faceplate_sleeve_0')
        points,triangles=sleeve.mesh()
        self.assertAlmostEqual(min(points[i].x for tri in triangles for i in tri),905.5,delta=.003)
        self.assertAlmostEqual(max(points[i].x for tri in triangles for i in tri),909,delta=.003)


if __name__=='__main__':unittest.main()
