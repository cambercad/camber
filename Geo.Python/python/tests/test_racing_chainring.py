"""Mounting sleeve clearance and preservation of the working tooth band."""
import os
import math
import sys
import unittest
sys.path.insert(0,os.path.join(os.path.dirname(__file__),'..'))
from camber import Part,set_progress_log
from racing_chainring import chainring,crank_spider,chainring_fasteners,mounting_points,ring_frame,spider_seat_face,crank_clamp_face
from racing_cassette import make_sprocket


def minimum_facet_radius(patches, center):
    """Radius of the disk contained by every projected bore-wall half-plane."""
    cx,cz=center
    distances=[]
    for patch in patches:
        vertices=patch['vertices']
        for tri in patch['faces']:
            for ia,ib in zip(tri,(tri[1],tri[2],tri[0])):
                a,b=vertices[ia],vertices[ib]
                dx,dz=b[0]-a[0],b[2]-a[2]
                length2=dx*dx+dz*dz
                t=0 if length2<1e-20 else max(0,min(1,((cx-a[0])*dx+(cz-a[2])*dz)/length2))
                distances.append(math.hypot(a[0]+t*dx-cx,a[2]+t*dz-cz))
    return min(distances)


class ChainringTests(unittest.TestCase):
    def test_fastener_tool_access_and_thread_envelopes(self):
        set_progress_log(False)
        p=Part((-100,-100,-100),(600,100,600),tolerance=.15)
        nut,screw=chainring_fasteners(p)
        self.assertTrue(nut.is_watertight())
        self.assertTrue(screw.is_watertight())
        self.assertAlmostEqual(0,p.intersect(nut,screw).volume(),delta=1e-7)
        # The driver reaches the socket floor; it is not a painted hexagon.
        floor=p.raycast(screw,(0,-14,0),(0,1,0))
        self.assertIsNotNone(floor)
        self.assertAlmostEqual(-9.6,floor.point.y,delta=.001)
        # The nut has a continuous through bore and a recessed tool slot.
        self.assertIsNone(p.raycast(nut,(0,3,0),(0,-1,0)))
        floor=p.raycast(nut,(5.5,3,0),(0,-1,0))
        self.assertIsNotNone(floor)
        self.assertAlmostEqual(1.2,floor.point.y,delta=.001)

    def test_spider_has_clear_spindle_and_shared_ring_seats(self):
        set_progress_log(False)
        p=Part((-100,-100,-100),(600,100,600),tolerance=.15)
        spider=crank_spider(p)
        self.assertTrue(spider.is_watertight())
        spindle=p.cylinder(ring_frame(-35),15,12,max_deviation=.01)
        self.assertAlmostEqual(0,p.intersect(spider,spindle).volume(),delta=1e-7)
        for teeth in (36,52):
            with self.subTest(teeth=teeth):
                ring=chainring(p,teeth)
                self.assertAlmostEqual(0,p.intersect(spider,ring).volume(),delta=1e-7)
        for x,z in mounting_points():
            frame=ring_frame(-35)
            origin=frame.origin+frame.x*x+frame.y*z
            sleeve=p.cylinder(frame.offset(origin-frame.origin),5,12,max_deviation=.01)
            self.assertAlmostEqual(0,p.intersect(spider,sleeve).volume(),delta=1e-7)
            # A 7 mm radius bearing footprint surrounds each through hole.
            # Its inner face must meet the spider at each ring seat.
            for y,direction in ((-37,(0,-1,0)),(-45,(0,1,0))):
                hit=p.raycast(spider,(404+x,y,267+z+6),direction)
                self.assertIsNotNone(hit)
                self.assertAlmostEqual(-38 if y == -37 else -44,hit.point.y,delta=.001)

    def test_mounts_receive_sleeves_without_removing_working_teeth(self):
        set_progress_log(False)
        p=Part((-100,-100,-100),(600,100,600),tolerance=.15)
        for teeth,depth,inner_radius in ((36,-36,61),(52,-44,90)):
            with self.subTest(teeth=teeth):
                ring=chainring(p,teeth)
                self.assertTrue(ring.is_watertight())
                frame=ring_frame(depth)
                for x,y in mounting_points():
                    origin=frame.origin+frame.x*x+frame.y*y
                    sleeve=p.cylinder(frame.offset(origin-frame.origin),5,2,max_deviation=.02)
                    self.assertAlmostEqual(0,p.intersect(ring,sleeve).volume(),delta=1e-7)
                    self.assertIsNone(p.raycast(ring,origin-frame.z,frame.z))
                # The complete original working-tooth annulus must survive
                # both mounting drilling and web lightening.
                working=make_sprocket(p,f'working_band_{teeth}',frame,teeth,inner_radius,2)
                self.assertAlmostEqual(0,p.cut(working,ring).volume(),delta=1e-7)
                self.assertGreater(ring.volume(),working.volume())


    def test_production_facets_preserve_bolt_clearance(self):
        # A 12-sided nominal Ø10.2 hole previously had only 4.92487 mm
        # minimum radius, so a Ø10 sleeve intersected the actual solid.
        from camber.glview import _as_scene
        set_progress_log(False)
        p=Part((-420,-300,-50),(1420,300,1120),tolerance=.2)
        for teeth in (36,52):
            patches=_as_scene(chainring(p,teeth)).patches
            for index,(dx,dz) in enumerate(mounting_points()):
                walls=[patch for patch in patches if patch['name'].endswith(f'-mount_{index}')]
                self.assertTrue(walls)
                self.assertGreater(minimum_facet_radius(walls,(404+dx,267+dz)),5.08)

    def test_integrated_crank_has_real_seats_connected_carrier_and_clear_hardware(self):
        from camber.glview import _as_scene
        from racing_bike import build_drivetrain,BB
        from racing_pedal import crank_pedal_center
        set_progress_log(False)
        p=Part((-420,-300,-50),(1420,300,1120),tolerance=.2)
        assembly=build_drivetrain(p)
        status=assembly.solve()
        self.assertTrue(status.converged)
        self.assertEqual((),status.unsatisfied)
        names=[body.name for body in assembly.parts]
        self.assertEqual(1,names.count('crank_spindle'))
        self.assertEqual(1,names.count('crank_arm_-1'))
        self.assertEqual(1,names.count('crank_arm_1'))
        self.assertEqual(4,names.count('chainring_sleeve_nut'))
        self.assertEqual(4,names.count('chainring_socket_screw'))
        self.assertFalse(any(name.startswith('crank_spider_') for name in names))
        self.assertEqual(1,sum(c['kind']=='FixPart' for c in assembly.constraints))
        for side in (-1,1):
            arm=p.solid(f'crank_arm_{side}')
            self.assertTrue(arm.is_watertight())
            # Body adjacency plus a ray across the joined root proves this is
            # material connected to the crank, not eight overlapping loose arms.
            vertices,triangles=arm.mesh()
            adjacency={}
            for tri in triangles:
                for vertex in tri:adjacency.setdefault(vertex,set()).update(tri)
            remaining=set(adjacency);pending=[remaining.pop()]
            while pending:
                vertex=pending.pop()
                for neighbor in adjacency[vertex]&remaining:
                    remaining.remove(neighbor);pending.append(neighbor)
            self.assertFalse(remaining)
            inside=p.raycast(arm,(BB.x+12,0,BB.z),(0,side,0))
            self.assertIsNotNone(inside)
            self.assertAlmostEqual(side*69,inside.point.y,delta=.002)
            self.assertIsNone(p.raycast(arm,(BB.x,-90,BB.z),(0,1,0)))
            pedal=crank_pedal_center(side,BB)
            self.assertIsNone(p.raycast(arm,(pedal.x,-90,pedal.z),(0,1,0)))
        # Inspect actual solved patches. Hole half-planes contain a disk whose
        # radius exceeds every sleeve point: an independent, bounded radial
        # clearance proof avoids expensive all-pairs CSG on nominal face contact.
        patches=_as_scene(assembly).patches
        def faces(body,suffix):
            return [patch for patch in patches if patch['name'].startswith(body+':')
                    and patch['name'].endswith(suffix)]
        def all_vertices(selected):
            return [vertex for patch in selected for vertex in patch['vertices']]
        def face_plane(selected):
            self.assertEqual(1, len(selected))
            patch = selected[0]
            candidates = []
            for triangle in patch['faces']:
                a, b, c = [patch['vertices'][i] for i in triangle]
                u = [b[i]-a[i] for i in range(3)]
                v = [c[i]-a[i] for i in range(3)]
                normal = (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0])
                candidates.append((math.hypot(*normal), a, normal))
            area, origin, normal = max(candidates, key=lambda item: item[0])
            self.assertGreater(area, 0)
            normal = tuple(value/area for value in normal)
            if normal[1] < 0:
                normal = tuple(-value for value in normal)
            plane = origin, normal
            # A rigid planar ring had 8.34e-8 mm world-Y spread after solving.
            # Test real flatness at 1e-8, not exact alignment with a world axis.
            for vertex in patch['vertices']:
                self.assertLess(abs(plane_distance(plane, vertex)), 1e-8)
            return plane

        def plane_distance(plane, vertex):
            origin, normal = plane
            return sum((vertex[i]-origin[i])*normal[i] for i in range(3))

        def plane_mate(first, second):
            matches = [mate for mate in status.mates if mate.kind == 'CoincidentPlanes'
                       and any(entity.startswith(first+':') for entity in mate.entities)
                       and any(entity.startswith(second+':') for entity in mate.entities)]
            self.assertEqual(1, len(matches))
            return matches[0]

        def contact_tolerance(plane, vertices, mate):
            self.assertTrue(vertices)
            lever_arm = max(math.dist(vertex, plane[0]) for vertex in vertices)
            # Plane offsets use normalized length; the normal-component residuals
            # add bounded angular error across the measured face.
            tolerance = mate.tolerance * (status.characteristic_length + math.sqrt(3)*lever_arm)
            self.assertLess(tolerance, 1e-5)  # Catches the former .0003 mm nominal-seat error.
            return tolerance

        def assert_face_contact(receiving, contacting, mate):
            plane = face_plane(receiving)
            for patch in contacting:
                face_plane([patch])
            vertices = all_vertices(contacting)
            tolerance = contact_tolerance(plane, vertices, mate)
            for vertex in vertices:
                self.assertLess(abs(plane_distance(plane, vertex)), tolerance)
            return plane

        for side in (-1, 1):
            assert_face_contact(faces(f'crank_arm_{side}', crank_clamp_face(side)),
                                faces(f'crank_bolt_{side}', f'crank_bolt_head_{side}-ExtrudeBottom'),
                                plane_mate(f'crank_arm_{side}', f'crank_bolt_{side}'))
        bearing_planes = {}
        bearing_mates = {}
        for teeth, body, cap in ((36, 'chainring_sleeve_nut', 'ExtrudeBottom'),
                                 (52, 'chainring_socket_screw', 'ExtrudeTop')):
            bearing = faces(body, body+'-edge_3')
            self.assertEqual(4, len(bearing))
            mate = plane_mate(f'chainring_{teeth}', body)
            bearing_planes[body] = assert_face_contact(
                faces(f'chainring_{teeth}', f'chainring_{teeth}_blank-'+cap), bearing, mate)
            bearing_mates[body] = mate
        # Full creation-root sets identify one actual receiving face after fusion.
        for teeth, cap in ((36, 'ExtrudeTop'), (52, 'ExtrudeBottom')):
            assert_face_contact(faces('crank_arm_-1', spider_seat_face(teeth)),
                                faces(f'chainring_{teeth}', f'chainring_{teeth}_blank-'+cap),
                                plane_mate('crank_arm_-1', f'chainring_{teeth}'))
        centers=[(404+x,267+z) for x,z in mounting_points()]
        def radial(v):return min(math.hypot(v[0]-x,v[2]-z) for x,z in centers)
        barrels=faces('chainring_sleeve_nut','chainring_sleeve_nut-edge_4')
        self.assertEqual(4,len(barrels))
        sleeve_radius=max(map(radial,all_vertices(barrels)))
        for body in ('chainring_36','chainring_52','crank_arm_-1'):
            for index,(cx,cz) in enumerate(centers):
                suffix=f'-bolt_{index}' if body=='crank_arm_-1' else f'-mount_{index}'
                walls=faces(body,suffix)
                self.assertTrue(walls,(body,suffix))
                self.assertGreater(minimum_facet_radius(walls,(cx,cz))-sleeve_radius,.08)
        # The unthreaded screw envelope similarly clears the sleeve bore.
        screw_points=all_vertices(faces('chainring_socket_screw','chainring_socket_screw-edge_2'))
        screw_radius=max(map(radial,screw_points))
        bores=faces('chainring_sleeve_nut','chainring_sleeve_nut-edge_7')
        self.assertEqual(4,len(bores))
        for patch in bores:
            vertex=patch['vertices'][0]
            center=min(centers,key=lambda c:math.hypot(vertex[0]-c[0],vertex[2]-c[1]))
            self.assertGreater(minimum_facet_radius([patch],center)-screw_radius,.08)
        # Larger head material is outside the ring stack; the barrel fits the
        # verified bore. All four instances use their actual solved geometry.
        for body, sign in (('chainring_sleeve_nut', 1), ('chainring_socket_screw', -1)):
            points=all_vertices([patch for patch in patches if patch['name'].startswith(body+':')])
            plane = bearing_planes[body]
            tolerance = contact_tolerance(plane, points, bearing_mates[body])
            self.assertTrue(all(sign*plane_distance(plane, v) >= -tolerance for v in points if radial(v)>5.08))
        # Nut ends stop short of the screw bearing plane, not bottomed in it.
        tips=all_vertices(faces('chainring_sleeve_nut','chainring_sleeve_nut-edge_6'))
        self.assertGreater(min(plane_distance(bearing_planes['chainring_socket_screw'], v) for v in tips),.49)


if __name__=='__main__': unittest.main()
