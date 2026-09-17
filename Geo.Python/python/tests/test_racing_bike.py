import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from racing_bike import build_cassette, build_road_wheel, WHEEL_ROTATION, build_brakes, build_fork, assembly, rigid, HEAD_BOTTOM, SEAT, SEATPOST_TOP, STEERING_AXIS, SEATPOST_AXIS, build_frame_body, washer, BB, FRONT, HEAD_TOP, REAR, CHAIN_PITCH, chain_layout, build_down_tube, bottle_cage_parts


from camber import Part, set_progress_log, vec3


class BikeLayoutTests(unittest.TestCase):
    def test_down_tube_clears_front_wheel_envelope(self):
        # A sphere encloses the tyre in its nominal orientation. Testing the
        # intersection checks faces and edges too, not only sampled vertices.
        set_progress_log(False)
        part = Part((-420, -400, -50), (1420, 400, 1120), tolerance=.1)
        down_tube = build_down_tube(part)
        envelope = part.sphere(FRONT, 339 + 6, name="tyre_with_6mm_clearance")
        overlap = part.intersect(down_tube, envelope)
        self.assertTrue(down_tube.is_watertight())
        self.assertAlmostEqual(0, overlap.volume(), delta=1e-6)

    def test_bottle_cage_solids_and_mounted_clearances(self):
        set_progress_log(False)
        part = Part((-420, -300, -50), (1420, 300, 1120), tolerance=.1)
        pieces = bottle_cage_parts(part)
        for name, solid in pieces.items():
            with self.subTest(component=name):
                self.assertTrue(solid.is_watertight())
                self.assertGreater(solid.volume(), 0)
        self.assertAlmostEqual(0, part.intersect(pieces["bottle"], pieces["cage"]).volume(), delta=1e-6)
        self.assertAlmostEqual(0, part.intersect(pieces["bottle"], build_down_tube(part)).volume(), delta=1e-6)

    def test_frame_machined_interfaces_receive_components(self):
        # Test the finished union, where an adjoining tube can otherwise plug
        # a bore that was made in only one member before joining the frame.
        set_progress_log(False)
        part = Part((-420, -300, -50), (1420, 300, 1120), tolerance=.2)
        frame = build_frame_body(part)
        self.assertTrue(frame.is_watertight())
        components = [
            washer(part, "steerer_check", HEAD_BOTTOM, 14.3, 0,
                   (HEAD_TOP-HEAD_BOTTOM).norm(), axis=STEERING_AXIS),
            washer(part, "lower_headset_check", HEAD_BOTTOM, 22, 14.3, 7, axis=STEERING_AXIS),
            washer(part, "upper_headset_check", HEAD_TOP-STEERING_AXIS*7, 20, 14.3, 7, axis=STEERING_AXIS),
            washer(part, "seatpost_check", SEAT-SEATPOST_AXIS*100, 13.6, 11.6,
                   (SEATPOST_TOP-SEAT).norm()+100, axis=SEATPOST_AXIS),
            washer(part, "crank_spindle_check", BB+vec3(0, 69, 0), 15, 10, 138),
            washer(part, "rear_axle_check", REAR+vec3(0, 78, 0), 6, 0, 156),
        ]
        for side in (-1, 1):
            components.append(washer(part, f"bb_bearing_check_{side}", BB+vec3(0, side*34, 0),
                                     20.5, 15, 7, axis=(0, -side, 0)))
        for component in components:
            with self.subTest(component=component.name):
                self.assertTrue(component.is_watertight())
                self.assertAlmostEqual(0, part.intersect(frame, component).volume(), delta=1e-6)

    def test_mounted_caliper_housings_clear_frame_and_fork(self):
        set_progress_log(False)
        part = Part((-420,-300,-50),(1420,300,1120),tolerance=.2)
        frame = build_frame_body(part)
        fork, brakes = build_fork(part), build_brakes(part)
        mounted = assembly(part, "mounted_brake_check")
        rigid(mounted, frame)
        for sub in (fork, brakes):
            mounted.fix(mounted.add_subassembly(sub))
        mounted.solve()
        obstacles = [("frame", frame)]+[(x.name,part.solid(x.name)) for x in fork.parts]
        for name in ("front", "rear"):
            housing = part.solid(name+"_caliper_housing")
            for label, obstacle in obstacles:
                with self.subTest(caliper=name, obstacle=label):
                    self.assertAlmostEqual(0,part.intersect(housing,obstacle).volume(),delta=1e-6)

    def test_fork_receives_axle_headset_and_clears_rotor(self):
        set_progress_log(False)
        part = Part((-420,-300,-50),(1420,300,1120),tolerance=.2)
        fork = build_fork(part)
        shell = part.solid("carbon_fork")
        self.assertTrue(shell.is_watertight())
        obstacles = [build_frame_body(part),
            washer(part, "front_rotor_sweep", (991,46,339),80,0,1.8)]
        obstacles.extend(part.solid(x.name) for x in fork.parts if x.name != "carbon_fork")
        # Nominal contacting seats may accumulate sub-microlitre mesh overlap;
        # accept at most 0.001 mm³, while still rejecting physical clashes.
        for obstacle in obstacles:
            with self.subTest(interface=obstacle.name):
                contact=part.intersect(shell,obstacle)
                if obstacle.name == "frame_shell":
                    # Both faces use the same exact construction plane. A
                    # quantized, nonplanar loft cap must not create penetration.
                    self.assertEqual(0,contact.volume())
                else:
                    self.assertLessEqual(contact.volume(),.001)

    def test_rear_dropouts_clear_cassette_and_mounted_hub(self):
        set_progress_log(False)
        part = Part((-420,-300,-50),(1420,300,1120),tolerance=.2)
        frame = build_frame_body(part)
        self.assertTrue(frame.is_watertight())
        for obstacle in (
            washer(part,"rear_axle_insertion",REAR+vec3(0,90,0),6,0,180),
            washer(part,"rear_axle_head_fit",REAR+vec3(0,82,0),9,3,4),
        ):
            with self.subTest(component=obstacle.name):
                self.assertLessEqual(part.intersect(frame,obstacle).volume(),.001)
        wheel = build_road_wheel(part,front=False,rim_depth=60,max_deviation=.2)
        mounted = assembly(part,"rear_hub_fit")
        rigid(mounted,frame)
        wheel_occurrence = mounted.add_subassembly(wheel,REAR,WHEEL_ROTATION)
        mounted.fix(wheel_occurrence)
        mounted.solve()
        drive = next(c for c in wheel_occurrence.subassemblies if c.name == "hub_drive")
        cassette = next(c for c in drive.subassemblies if c.name == "cassette_12_speed")
        for occurrence in cassette.parts:
            with self.subTest(component=occurrence.name):
                self.assertLessEqual(part.intersect(frame,part.solid(occurrence.name)).volume(),.001)
        for name in ("rear_hub","rear_freehub","rear_hub_axle","rear_hub_left_cap","rear_hub_right_cap"):
            with self.subTest(component=name):
                contact = part.intersect(frame,part.solid(name))
                # Nominal endcap contact: allow under 0.1 mm³ only if every
                # referenced point lies within 1 micron of a 142 mm seating face.
                self.assertLessEqual(contact.volume(),.1)
                points, triangles = contact.mesh()
                for index in {i for triangle in triangles for i in triangle}:
                    self.assertLessEqual(abs(abs(points[index].y)-71),.001)

    def test_reference_geometry_and_closed_even_chain(self):
        self.assertEqual(991, FRONT.x-REAR.x)
        self.assertEqual(72, REAR.z-BB.z)
        self.assertEqual(395, HEAD_TOP.x-BB.x)
        self.assertEqual(565, HEAD_TOP.z-BB.z)
        tension_z, pins = chain_layout()
        self.assertTrue(171 <= tension_z <= 191)
        self.assertEqual(0, len(pins)%2)
        for i, pin in enumerate(pins):
            self.assertAlmostEqual(CHAIN_PITCH, math.dist(pin, pins[(i+1)%len(pins)]), delta=.001)


if __name__ == "__main__":
    unittest.main()
