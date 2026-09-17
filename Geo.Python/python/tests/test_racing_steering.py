"""Motion checks on the actual complete bicycle, including nested brake hardware."""
import math
import os
import sys
import time
import unittest
import numpy as np

sys.path.insert(0,os.path.join(os.path.dirname(__file__),".."))
from racing_bike import build_bike,HEAD_BOTTOM,FRONT,BB,STEERING_AXIS,chain_layout,set_progress_log


class SteeringTests(unittest.TestCase):
    def test_fork_cockpit_and_front_wheel_follow_independent_bearing_mates(self):
        set_progress_log(False)
        started = time.perf_counter()
        bike=build_bike()
        print(f"Bike construction and solve: {time.perf_counter()-started:.2f} seconds", flush=True)
        part=bike._part
        groups={g.name:g for g in bike.subassemblies}
        frame,fork,cockpit,wheel=(groups[n] for n in ("frameset","fork_and_headset","cockpit","front_wheel"))

        def members(group):
            yield from group.parts
            for child in group.subassemblies:
                yield from members(child)

        def vertices(occurrence):
            points,triangles=part.solid(occurrence.name).mesh()
            used=sorted({i for triangle in triangles for i in triangle})
            return np.asarray([tuple(points[i]) for i in used])

        drivetrain = groups["drivetrain"]
        shafts = [groups[f"pedal_{side}_shaft_group"] for side in (-1, 1)]
        reference={p.name:vertices(p) for g in (frame,fork,cockpit,wheel,drivetrain,*shafts) for p in members(g)}
        # Standard nipples share one Solid definition. Keep each occurrence's
        # position separately, so repeated names cannot collapse motion coverage.
        wheel_positions=[(occurrence,np.asarray(bike.world_pose(occurrence)))
                         for occurrence in members(wheel)]
        crank_positions=[(occurrence,np.asarray(bike.world_pose(occurrence)))
                         for group in (drivetrain,*shafts) for occurrence in members(group)]
        head=next(p for p in frame.parts if p.name=="frame_shell")
        steerer=next(p for p in fork.parts if p.name=="carbon_fork")
        axle=next(p for p in fork.parts if p.name=="front_thru_axle")
        hub=next(p for p in wheel.parts if p.name=="front_hub")
        bike.angle(head.axis_at(HEAD_BOTTOM,(0,1,0)),steerer.axis_at(HEAD_BOTTOM,(0,1,0)),.2)
        bike.angle(axle.axis_at(FRONT,(1,0,0)),hub.axis_at((0,0,0),(1,0,0)),.3)
        crank = next(p for p in drivetrain.parts if p.name == "crank_arm_-1")
        from racing_chain import engagement_phase
        initial_phase = engagement_phase(chain_layout()[1], (BB.x,BB.z), 52)
        initial_direction = (math.cos(initial_phase), 0, math.sin(initial_phase))
        bike.angle(head.axis_at(BB,initial_direction),crank.axis_at(BB,(1,0,0)),.17)
        result = bike.solve()
        self.assertTrue(result.converged, f"{result}; message={result.message}; sse={result.sum_squared_error}; unsatisfied={result.unsatisfied}")

        def rotation(axis,angle):
            x,y,z=axis
            skew=np.array(((0,-z,y),(z,0,-x),(-y,x,0)))
            return np.eye(3)*math.cos(angle)+(1-math.cos(angle))*np.outer(axis,axis)+math.sin(angle)*skew

        def measured_angle(before,after,axis):
            before=before-axis*np.dot(before,axis)
            after=after-axis*np.dot(after,axis)
            return math.atan2(np.dot(axis,np.cross(before,after)),np.dot(before,after))

        origin=np.array(tuple(HEAD_BOTTOM))
        axis=np.array(tuple(STEERING_AXIS))
        phase=measured_angle(reference[axle.name].mean(axis=0)-origin,vertices(axle).mean(axis=0)-origin,axis)
        # Equality mates now use 1e-8 normalized residuals. These geometric
        # checks also include tessellation and fitted-angle error at tyre radius.
        self.assertAlmostEqual(math.cos(.2),math.cos(phase),delta=1e-6)
        steer=rotation(axis,phase)
        for group in (frame,fork,cockpit):
            for occurrence in members(group):
                expected=reference[occurrence.name]
                if group is not frame:
                    expected=(expected-origin) @ steer.T+origin
                with self.subTest(component=occurrence.name):
                    np.testing.assert_allclose(vertices(occurrence),expected,atol=.002,rtol=0)

        # Remove steering from wheel geometry; its remaining motion must be
        # pure rotation about the through-axle, independently of steering.
        wheel_origin=np.array(tuple(FRONT))
        wheel_axis=np.array((0.,-1.,0.))
        before=reference[hub.name]-wheel_origin
        after=(vertices(hub)-origin) @ steer+origin-wheel_origin
        radial=before-np.outer(before @ wheel_axis,wheel_axis)
        rotated=after-np.outer(after @ wheel_axis,wheel_axis)
        # Fit the rotation to all hub vertices. A single lattice-rounded point
        # on the small hub amplifies its positional error at the tyre radius.
        spin=math.atan2(np.sum(np.cross(radial,rotated) @ wheel_axis),
                        np.sum(radial*rotated))
        self.assertAlmostEqual(math.cos(.3),math.cos(spin),delta=1e-6)
        wheel_rotation=rotation(wheel_axis,spin)
        for index,(occurrence,position) in enumerate(wheel_positions):
            expected=(position-wheel_origin) @ wheel_rotation.T+wheel_origin
            expected=(expected-origin) @ steer.T+origin
            with self.subTest(wheel_occurrence=index,name=occurrence.name):
                np.testing.assert_allclose(bike.world_pose(occurrence),expected,atol=.002,rtol=0)
        for occurrence in members(wheel):
            expected=(reference[occurrence.name]-wheel_origin) @ wheel_rotation.T+wheel_origin
            expected=(expected-origin) @ steer.T+origin
            with self.subTest(component=occurrence.name):
                np.testing.assert_allclose(vertices(occurrence),expected,atol=.002,rtol=0)

        # Crank rotation is independent of steering and wheel spin. The pedal
        # shafts are threaded to the arms; their bodies retain bearing rotation.
        crank_origin = np.array(tuple(BB))
        crank_axis = np.array((0.,1.,0.))
        phase = measured_angle(reference[crank.name].mean(axis=0)-crank_origin,
                               vertices(crank).mean(axis=0)-crank_origin, crank_axis)
        self.assertAlmostEqual(math.cos(.17), math.cos(phase), delta=1e-6)
        crank_rotation = rotation(crank_axis, phase)
        for index,(occurrence,position) in enumerate(crank_positions):
            expected = (position-crank_origin) @ crank_rotation.T+crank_origin
            with self.subTest(crank_occurrence=index,name=occurrence.name):
                np.testing.assert_allclose(bike.world_pose(occurrence),expected,atol=.002,rtol=0)
        for group in (drivetrain,*shafts):
            for occurrence in members(group):
                expected = (reference[occurrence.name]-crank_origin) @ crank_rotation.T+crank_origin
                with self.subTest(crank_component=occurrence.name):
                    np.testing.assert_allclose(vertices(occurrence),expected,atol=.002,rtol=0)

        # Steering changes the tyre's swept position as well as the fork pose.
        # Check real transformed geometry, beyond the nominal axle-centered
        # clearance envelope in the separate layout test.
        tyre=part.solid("front_rubber_tyre")
        frame_shell=part.solid("frame_shell")
        self.assertAlmostEqual(0,part.intersect(frame_shell,tyre).volume(),delta=1e-6)
