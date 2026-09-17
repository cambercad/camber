"""Working-gauge checks for the unbranded 19 mm combination wrench."""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from camber import Frame, set_progress_log, vec3
from wrench_19 import (build_wrench, ring_frame, JAW_X, JAW_Y, C, S,
                       RING_X, RING_Z, RING_THICKNESS, HANDLE_THICKNESS,
                       JAW_THICKNESS, DEFAULT_TOLERANCE, NECK_RUNOUT_START)


class WrenchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        set_progress_log(False)
        cls.wrench = build_wrench()
        cls.part = cls.wrench._part

    def hex_gauge(self, name, frame, size, angle=0):
        sketch = self.part.sketch(frame=frame, name=name+"_section")
        r = size/math.sqrt(3)
        vertices = [(r*math.cos(angle+i*math.pi/3), r*math.sin(angle+i*math.pi/3))
                    for i in range(6)]
        for i, point in enumerate(vertices):
            sketch.add_line(point, vertices[(i+1) % 6])
        return self.part.extrude(sketch, 5, name=name)

    def test_single_connected_watertight_forging_and_envelope(self):
        self.assertTrue(self.wrench.is_watertight())
        self.assertGreater(self.wrench.signed_volume(), 0)
        points, triangles = self.wrench.mesh()
        # Raw Boolean meshes can retain unused tool vertices. Only indexed
        # surface vertices contribute to the manufactured part envelope.
        active = {i for triangle in triangles for i in triangle}
        surface_points = [points[i] for i in active]
        self.assertAlmostEqual(230, max(p.x for p in surface_points)-min(p.x for p in surface_points), delta=.06)
        self.assertAlmostEqual(42, max(p.y for p in surface_points)-min(p.y for p in surface_points), delta=.1)
        neighbours = {}
        for a, b, c in triangles:
            neighbours.setdefault(a, set()).update((b, c))
            neighbours.setdefault(b, set()).update((a, c))
            neighbours.setdefault(c, set()).update((a, b))
        visited, pending = set(), [next(iter(neighbours))]
        while pending:
            vertex = pending.pop()
            if vertex not in visited:
                visited.add(vertex)
                pending.extend(neighbours[vertex]-visited)
        self.assertEqual(len(neighbours), len(visited), "Heads and grip must be one connected forging")

    def test_ring_accepts_19mm_hex_at_both_bihex_positions_and_transmits_torque(self):
        for index, angle in enumerate((0, math.pi/6)):
            gauge = self.hex_gauge(f"ring_gauge_{index}", ring_frame(-2.5), 19, angle)
            self.assertLess(self.part.intersect(self.wrench, gauge).volume(), 1e-6)
        rotated = self.hex_gauge("ring_torque_gauge", ring_frame(-2.5), 19, math.pi/12)
        self.assertGreater(self.part.intersect(self.wrench, rotated).volume(), 1)

    def test_ground_open_jaws_accept_nominal_hex_and_reject_oversize(self):
        center = vec3(JAW_X, JAW_Y, -2.5)+vec3(-C, -S, 0)*6
        frame = Frame(center, x=(-C, -S, 0), y=(S, -C, 0), z=(0, 0, 1))
        gauge = self.hex_gauge("open_19mm_gauge", frame, 19)
        self.assertLess(self.part.intersect(self.wrench, gauge).volume(), 1e-5)
        oversize = self.hex_gauge("open_oversize_gauge", frame, 19.2)
        self.assertGreater(self.part.intersect(self.wrench, oversize).volume(), 1)

    def test_open_head_has_two_straight_flats_and_a_curved_clearance_throat(self):
        origin = vec3(JAW_X, JAW_Y, 0)
        along = vec3(C, S, 0)
        across = vec3(-S, C, 0)
        # Three actual axial stations distinguish straight opposing gripping
        # faces from a circular or tapered opening that only fits one gauge.
        for station in (-10, -6, -2):
            center = origin+along*station
            upper = self.part.raycast(self.wrench, center, across)
            lower = self.part.raycast(self.wrench, center, -across)
            self.assertIsNotNone(upper)
            self.assertIsNotNone(lower)
            width = sum(a*b for a, b in zip(upper.point-lower.point, across))
            self.assertAlmostEqual(width, 19.06, delta=.003)
            self.assertAlmostEqual(sum(a*b for a, b in zip(upper.normal, across)), -1, delta=.003)
            self.assertAlmostEqual(sum(a*b for a, b in zip(lower.normal, across)), 1, delta=.003)
        # The back of the opening is a continuous clearance arc, not the old
        # straight crossbar with two small corner rounds.
        for angle in (-math.pi/3, 0, math.pi/3):
            direction = along*math.cos(angle)+across*math.sin(angle)
            hit = self.part.raycast(self.wrench, origin, direction)
            self.assertIsNotNone(hit)
            radius = sum(a*b for a, b in zip(hit.point-origin, direction))
            self.assertAlmostEqual(radius, 19.06/2, delta=DEFAULT_TOLERANCE+.001)
        crown_x = JAW_X+6
        upper = self.part.raycast(self.wrench, (crown_x, 35, 0), (0, -1, 0))
        lower = self.part.raycast(self.wrench, (crown_x, -35, 0), (0, 1, 0))
        self.assertIsNotNone(upper)
        self.assertIsNotNone(lower)
        self.assertAlmostEqual(upper.point.y, 22, delta=DEFAULT_TOLERANCE+.001)
        self.assertAlmostEqual(lower.point.y, -20, delta=DEFAULT_TOLERANCE+.001)

    def test_ring_has_both_offset_and_inclined_parallel_faces(self):
        top = self.part.raycast(self.wrench, (RING_X,12.5,30), (0,0,-1))
        bottom = self.part.raycast(self.wrench, (RING_X,12.5,-20), (0,0,1))
        self.assertIsNotNone(top)
        self.assertIsNotNone(bottom)
        self.assertAlmostEqual((top.point.z+bottom.point.z)/2, RING_Z, delta=.01)
        self.assertAlmostEqual(top.point.z-bottom.point.z, RING_THICKNESS/C, delta=.02)
        self.assertAlmostEqual(abs(top.normal.x), S, delta=.003)
        self.assertAlmostEqual(abs(top.normal.z), C, delta=.003)
        shank = self.part.raycast(self.wrench, (100,5.5,20), (0,0,-1))
        jaw = self.part.raycast(self.wrench, (20,14,20), (0,0,-1))
        self.assertAlmostEqual(shank.point.z,HANDLE_THICKNESS/2,delta=.01)
        self.assertAlmostEqual(jaw.point.z,JAW_THICKNESS/2,delta=.01)
        self.assertGreater(RING_Z, HANDLE_THICKNESS/2)

    def test_handle_outer_edge_is_a_real_round_not_only_smooth_shading(self):
        hit = self.part.raycast(self.wrench, (100,6.9,20), (0,0,-1))
        self.assertIsNotNone(hit)
        self.assertGreater(hit.point.z,2.0)
        self.assertLess(hit.point.z,2.25)
        self.assertGreater(hit.normal.y,.5)

    def test_ring_neck_starts_from_both_shank_faces_without_a_thickened_belly(self):
        thicknesses = []
        for x in (NECK_RUNOUT_START, 190, 195, 200):
            with self.subTest(station=x):
                top = self.part.raycast(self.wrench, (x, 0, 20), (0, 0, -1))
                bottom = self.part.raycast(self.wrench, (x, 0, -20), (0, 0, 1))
                self.assertIsNotNone(top)
                self.assertIsNotNone(bottom)
                thicknesses.append(top.point.z-bottom.point.z)
                if x == NECK_RUNOUT_START:
                    self.assertAlmostEqual(top.point.z, HANDLE_THICKNESS/2, delta=.01)
                    self.assertAlmostEqual(bottom.point.z, -HANDLE_THICKNESS/2, delta=.01)
                    self.assertLess(abs(top.normal.x), .01)
                    self.assertLess(abs(bottom.normal.x), .01)
        self.assertEqual(thicknesses, sorted(thicknesses))
        self.assertLess(thicknesses[-1], RING_THICKNESS/C)

    def test_outer_ring_and_broach_share_one_axis_through_the_wall_thickness(self):
        frame = ring_frame()
        def dot(a,b):
            return sum(x*y for x,y in zip(a,b))
        # Three axial stations are necessary: an outside wall left parallel to
        # global Z can look right at its centre but drifts at the two faces.
        for angle in (0., math.pi/3, -math.pi/3):
            radial = frame.x*math.cos(angle)+frame.y*math.sin(angle)
            outer_radii = []
            for station in (-3.,0.,3.):
                center = frame.origin+frame.z*station
                hit = self.part.raycast(self.wrench,center+radial*25,-radial)
                self.assertIsNotNone(hit)
                radius = dot(hit.point-center,radial)
                outer_radii.append(radius)
                # Ray casting hits tessellated chords; radial error is bounded
                # by the requested CAD tessellation tolerance. Axial drift is
                # checked independently and substantially more tightly.
                self.assertAlmostEqual(radius,14.,delta=DEFAULT_TOLERANCE+.001)
                self.assertAlmostEqual(dot(hit.normal,frame.z),0.,delta=.003)
            self.assertLess(max(outer_radii)-min(outer_radii),.003)
        # A point strictly inside a drive flank avoids the bihex vertex where
        # either adjoining normal would be a valid ray-cast answer.
        radial = frame.x*math.cos(.2)+frame.y*math.sin(.2)
        radii = []
        for station in (-3.,0.,3.):
            center = frame.origin+frame.z*station
            hit = self.part.raycast(self.wrench,center,radial)
            self.assertIsNotNone(hit)
            radii.append(dot(hit.point-center,radial))
            self.assertAlmostEqual(dot(hit.normal,frame.z),0.,delta=.003)
        self.assertLess(max(radii)-min(radii),.015)
