import itertools
import math
import os
import sys
import unittest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from camber import Frame, Part, set_progress_log, vec3
from racing_brake import (caliper_parts, cylinder, build_caliper, build_mounted_caliper,
                          brake_frame, brake_mount_frame, FRONT_BRAKE_ANGLE, REAR_BRAKE_ANGLE,
                          mounting_screw, round_in_frame)


class CaliperTests(unittest.TestCase):
    def test_parts_and_swept_rotor_have_no_interference(self):
        set_progress_log(False)
        part = Part((-100,-100,-100),(100,100,100),tolerance=.2)
        pieces = caliper_parts(part)
        # A solid disc includes every rotor phase and every scallop/spoke.
        pieces["rotor_sweep"] = cylinder(part,"rotor_sweep",(0,-72,-.9),80,1.8)
        for name, solid in pieces.items():
            with self.subTest(solid=name):
                self.assertTrue(solid.is_watertight())
                self.assertGreater(solid.volume(),0)
        for (a, first),(b, second) in itertools.combinations(pieces.items(),2):
            with self.subTest(pair=(a,b)):
                self.assertAlmostEqual(0,part.intersect(first,second).volume(),delta=1e-6)

    def test_component_mates_solve(self):
        set_progress_log(False)
        part = Part((-100,-100,-100),(100,100,100),tolerance=.2)
        caliper = build_caliper(part)
        self.assertEqual(9,len(caliper.parts))
        self.assertEqual(1,sum(c['kind']=='FixPart' for c in caliper.constraints))
        self.assertEqual(8,sum(c['kind']=='Concentric' for c in caliper.constraints))


class BrakeInstallationTests(unittest.TestCase):
    def test_receiving_planes_match_frame_and_fork_layout(self):
        # Published pitch and original support positions are independent of
        # the rendered mesh and caliper piston placement.
        front = brake_frame((991,0,339),FRONT_BRAKE_ANGLE).to_global(brake_mount_frame(front=True))
        fork_seat = front.origin-front.z*6
        self.assertLess((fork_seat-vec3(948,66,375)).norm(),1e-8)
        self.assertAlmostEqual(70,(front.x*70).norm(),places=8)
        rear = brake_frame((0,0,339),REAR_BRAKE_ANGLE).to_global(brake_mount_frame(front=False))
        self.assertLess((rear.origin-vec3(77,65,337)).norm(),1e-8)
        self.assertLess((rear.z-vec3(0,0,1)).norm(),1e-8)
        self.assertLess((rear.origin-rear.x*17-vec3(60,65,337)).norm(),1e-8)
        self.assertLess((rear.origin+rear.x*17-vec3(94,65,337)).norm(),1e-8)

    def test_mounted_brakes_clear_full_rotor_sweep_and_each_other(self):
        set_progress_log(False)
        for front in (True,False):
            with self.subTest(front=front):
                part = Part((-150,-150,-150),(150,150,150),tolerance=.2)
                assembly = build_mounted_caliper(part,"front" if front else "rear",front=front)
                expected_fasteners = 4 if front else 2
                self.assertEqual(expected_fasteners,sum("screw" in p.name for p in assembly.parts))
                self.assertEqual(expected_fasteners+(1 if front else 0),
                                 sum(c['kind']=='Concentric' for c in assembly.constraints))
                rotor = assembly.add_part(cylinder(part,"rotor_sweep",(0,-72,-.9),80,1.8))
                assembly.fix(rotor)
                self.assertEqual([],assembly.interferences(min_volume=.001))
                for item in (*assembly.parts,*assembly.subassemblies[0].parts):
                    self.assertTrue(part.solid(item.name).is_watertight())
                    self.assertGreater(part.solid(item.name).volume(),0)


    def test_integral_mounting_feet_form_one_connected_housing(self):
        set_progress_log(False)
        for mount in ("front","rear"):
            with self.subTest(mount=mount):
                part = Part((-150,-150,-150),(150,150,150),tolerance=.2)
                housing = caliper_parts(part,mount=mount)["housing"]
                points,triangles = housing.mesh()
                adjacent = {}
                for triangle in triangles:
                    vertices = {tuple(points[i]) for i in triangle}
                    for point in vertices:
                        adjacent.setdefault(point,set()).update(vertices)
                # Follow shared geometric vertices: rendering mesh patches may
                # use distinct indices at one and the same topological vertex.
                reached = set()
                pending = [next(iter(adjacent))]
                while pending:
                    point = pending.pop()
                    if point not in reached:
                        reached.add(point)
                        pending.extend(adjacent[point]-reached)
                self.assertEqual(len(adjacent),len(reached))
                self.assertTrue(housing.is_watertight())


    def test_turned_mounting_screw_preserves_seats_and_socket(self):
        set_progress_log(False)
        for direction in (-1,1):
            with self.subTest(direction=direction):
                part = Part((-150,-150,-150),(150,150,150),tolerance=.2)
                frame = brake_mount_frame(front=True)
                screw = mounting_screw(part,"mount_screw",frame,17,-6,12,direction)
                datum = frame.origin+frame.x*17-frame.z*6
                axis = frame.z*direction
                points,triangles = screw.mesh()
                vertices = [vec3(points[i])-datum for i in {i for t in triangles for i in t}]
                self.assertAlmostEqual(-4,min(p.dot(axis) for p in vertices),delta=.001)
                self.assertAlmostEqual(12,max(p.dot(axis) for p in vertices),delta=.001)
                self.assertAlmostEqual(4.25,max((p-axis*p.dot(axis)).norm() for p in vertices),delta=.001)
                # Boolean reconstruction exposes the operating-space lattice.
                # Verify the replacement preserves the previous extruded blank's
                # actual seating planes, independently of its nominal 1 µm bound.
                old_frame = Frame(datum,x=frame.x,y=frame.y*direction,z=axis)
                old_blank = part.union(
                    round_in_frame(part,"old_head",old_frame.offset(-axis*4),4.25,4),
                    round_in_frame(part,"old_shank",old_frame,2.45,12))
                old_points,old_triangles = old_blank.mesh()
                old_axial = [(vec3(old_points[i])-datum).dot(axis)
                             for i in {i for t in old_triangles for i in t}]
                self.assertAlmostEqual(min(old_axial),min(p.dot(axis) for p in vertices),places=9)
                self.assertAlmostEqual(max(old_axial),max(p.dot(axis) for p in vertices),places=9)
                nominal_volume = math.pi*(4.25**2*4+2.45**2*12)-8*math.sqrt(3)*2.5
                self.assertAlmostEqual(nominal_volume,screw.volume(),delta=nominal_volume*.005)
                self.assertTrue(screw.is_watertight())
                socket_frame = Frame(datum-axis*3.99,x=frame.x,y=frame.y*direction,z=axis)
                gauge = round_in_frame(part,"socket_go_gauge",socket_frame,1.98,2.4)
                self.assertAlmostEqual(0,part.intersect(screw,gauge).volume(),delta=1e-6)
                oversized = round_in_frame(part,"socket_no_go_gauge",socket_frame,2.1,2.4)
                self.assertGreater(part.intersect(screw,oversized).volume(),.1)


    def test_installed_mount_contacts_are_confined_to_the_seating_planes(self):
        from racing_bike import build_frame,build_fork,FRONT,REAR

        set_progress_log(False)
        for front in (False,True):
            part = Part((-420,-300,-50),(1420,300,1120),tolerance=.2)
            model = (build_fork if front else build_frame)(part,brakes=True)
            self.assertTrue(model.solve().converged)
            support = part.solid("carbon_fork" if front else "frame_shell")
            mount = brake_frame(FRONT if front else REAR,
                                FRONT_BRAKE_ANGLE if front else REAR_BRAKE_ANGLE).to_global(
                                    brake_mount_frame(front=front))
            brake = model.subassemblies[0]
            frame_screws = [item for item in model.parts if item.name.startswith("rear_caliper_mount_screw_")]
            for item in (*brake.parts,*brake.subassemblies[0].parts,*frame_screws):
                with self.subTest(front=front,component=item.name):
                    contact = part.intersect(support,part.solid(item.name))
                    seat_depth = None
                    if front and item.name == "front_brake_adapter":
                        seat_depth = -6
                    elif not front and "mount_screw" in item.name:
                        from racing_brake import REAR_FRAME_MOUNT_THICKNESS
                        seat_depth = -REAR_FRAME_MOUNT_THICKNESS
                    elif not front and item.name == "rear_caliper_housing":
                        seat_depth = 0
                    if seat_depth is None:
                        # In particular, front screw tips must clear the blind
                        # hole bottoms, rather than merely fit inside a tolerance.
                        self.assertAlmostEqual(0,contact.volume(),delta=1e-6)
                    else:
                        # Only nominal seating surfaces may have lattice-sized
                        # overlap; every contact vertex must lie on that seat.
                        self.assertLess(contact.volume(),.1)
                        points,triangles = contact.mesh()
                        for i in {i for triangle in triangles for i in triangle}:
                            depth = (vec3(points[i])-mount.origin).dot(mount.z)
                            self.assertLessEqual(abs(depth-seat_depth),.001)
