"""Check real valve mounting cuts, retention and air passage in deep rims."""
import os
import sys
import unittest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from camber import Part, set_progress_log
from racing_tyre import tyre_rim_parts
from racing_valve import mount_valve, valve_frame


class ValveTests(unittest.TestCase):
    def test_mounting_faces_air_passage_and_material_fit(self):
        set_progress_log(False)
        for depth in (25,50,80):
            with self.subTest(rim_depth=depth):
                p=Part((-350,-350,-350),(350,350,350),tolerance=.15)
                raw=tyre_rim_parts(p,rim_depth=depth)['rim']
                rim,assembly=mount_valve(p,raw,rim_depth=depth)
                self.assertLess(rim.volume(),raw.volume())
                self.assertTrue(rim.is_watertight())
                self.assertEqual([],assembly.interferences(min_volume=1e-6))
                f=valve_frame(depth)
                # Inspect the bare stem: the axial air passage must be open.
                self.assertIsNone(p.raycast(p.solid('road_valve_stem'),f.origin-f.z,f.z))
                self.assertIsNone(p.raycast(rim,f.origin,f.z))
                # Both retaining parts seat on actual machined rim faces.
                nut_seat=p.raycast(rim,f.origin+f.x*4,f.z)
                seal_seat=p.raycast(rim,f.origin+f.x*4+f.z*(depth+40),-f.z)
                self.assertAlmostEqual(30.1,(nut_seat.point-f.origin).dot(f.z),delta=.002)
                self.assertAlmostEqual(depth+25.5,(seal_seat.point-f.origin).dot(f.z),delta=.002)
                # The axis sits halfway between neighbouring 15-degree spoke stations.
                import math
                angle=math.degrees(math.atan2(f.z.y,f.z.x))
                self.assertAlmostEqual(7.5,angle % 15)

    def test_actual_road_wheel_uses_mounted_material_assembly(self):
        from bike_wheel import build_road_wheel
        set_progress_log(False)
        wheel=build_road_wheel(front=True,max_deviation=.2)
        mounted=next(child.assembly for child in wheel.subassemblies
                     if child.name=='front_rim_valve')
        names={occurrence.name for occurrence in mounted.parts}
        self.assertTrue({'front_carbon_rim','front_rubber_tyre',
                         'front_aramid_bead_-1','front_aramid_bead_1',
                         'front_valve_stem','front_valve_seal','front_rim_tape'}.issubset(names))
        for contact in mounted.interferences(min_volume=1e-6):
            names=contact.first+" "+contact.second
            self.assertIn('front_nipple[',names)
            self.assertIn('front_carbon_rim[',names)
            self.assertLess(contact.volume,.01)
        # Nipple-specific tests additionally confine every residual-contact
        # vertex to the independently measured 0.002 mm bearing envelope.
        self.assertFalse(any('tan_sidewall' in occurrence.name for occurrence in wheel.parts))


if __name__ == '__main__':
    unittest.main()
