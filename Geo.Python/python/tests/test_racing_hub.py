import math
import numpy as np
import os
import sys
import unittest

sys.path.insert(0,os.path.join(os.path.dirname(__file__),".."))
from camber import Part,set_progress_log
from racing_hub import build_rear_hub


class RearHubTests(unittest.TestCase):
    def test_internal_fit_and_independent_bearing_rotations(self):
        set_progress_log(False)
        reference = {}
        for angles in ((0,0),(math.pi/3,-math.pi/5)):
            part = Part((-100,-100,-100),(100,100,100),tolerance=.2)
            hub = build_rear_hub(part,wheel_angle=angles[0],freehub_angle=angles[1])
            self.assertTrue(hub.solve().converged)
            self.assertEqual(2,sum(c["kind"]=="Concentric" for c in hub.constraints))
            self.assertEqual([],hub.interferences(min_volume=.001))
            for group,angle in zip(hub.subassemblies,(0,*angles)):
                for occurrence in group.parts:
                    solid = part.solid(occurrence.name)
                    with self.subTest(angles=angles,part=solid.name):
                        self.assertTrue(solid.is_watertight())
                        self.assertGreater(solid.volume(),0)
                        points,triangles = solid.mesh()
                        used = {i for triangle in triangles for i in triangle}
                        if angles == (0,0):
                            reference[solid.name] = (np.asarray([tuple(points[i]) for i in used]),solid.volume())
                        else:
                            original,volume = reference[solid.name]
                            c,s = math.cos(angle),math.sin(angle)
                            expected = original @ np.array(((c,s,0),(-s,c,0),(0,0,1)))
                            actual = np.asarray([tuple(points[i]) for i in used])
                            # Independent Boolean builds may reorder or subdivide
                            # triangles. Compare geometric supports, never indices.
                            directions = np.asarray([(math.cos(a),math.sin(a),z)
                                                     for a in np.linspace(0,2*math.pi,65)
                                                     for z in (-1,0,1)])
                            np.testing.assert_allclose((actual @ directions.T).max(axis=0),
                                                       (expected @ directions.T).max(axis=0),
                                                       atol=1e-6,rtol=0)
                            self.assertAlmostEqual(solid.volume(),volume,places=7)


    def test_wheel_and_cassette_follow_independent_angle_mates(self):
        import numpy as np
        from bike_wheel import build_road_wheel

        set_progress_log(False)
        part = Part((-500,-500,-150),(500,500,150),tolerance=.2)
        hub = build_road_wheel(part,front=False)
        self.assertTrue(hub.solve().converged)
        stationary,wheel,drive = hub.subassemblies
        axle = next(p for p in stationary.parts if p.name == "rear_hub_axle")
        shell = next(p for p in wheel.parts if p.name == "rear_hub")
        freehub = next(p for p in drive.parts if p.name == "rear_freehub")

        def members(group):
            yield from group.parts
            for child in group.subassemblies:
                yield from members(child)

        def vertices(occurrence):
            points,triangles = part.solid(occurrence.name).mesh()
            used = sorted({i for triangle in triangles for i in triangle})
            return np.asarray([tuple(points[i]) for i in used])

        reference = {p.name: vertices(p) for g in hub.subassemblies for p in members(g)}
        origin,x = (0,0,0),(1,0,0)
        hub.angle(axle.axis_at(origin,x),shell.axis_at(origin,x),.4)
        hub.angle(axle.axis_at(origin,x),freehub.axis_at(origin,x),.2)
        self.assertTrue(hub.solve().converged)
        for group,body,requested in ((stationary,axle,0),(wheel,shell,.4),(drive,freehub,.2)):
            before,after = reference[body.name],vertices(body)
            index = np.argmax(np.linalg.norm(before[:,:2],axis=1))
            phase = math.atan2(after[index,1],after[index,0])-math.atan2(before[index,1],before[index,0])
            # An unsigned angle mate can choose either rotation direction.
            self.assertAlmostEqual(math.cos(requested),math.cos(phase),places=7)
            c,s = math.cos(phase),math.sin(phase)
            rotation = np.array(((c,-s,0),(s,c,0),(0,0,1)))
            for occurrence in members(group):
                with self.subTest(part=occurrence.name):
                    np.testing.assert_allclose(vertices(occurrence),reference[occurrence.name] @ rotation.T,atol=1e-5,rtol=0)
