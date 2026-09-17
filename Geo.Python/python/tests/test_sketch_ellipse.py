import math
import unittest

from camber import Frame, Part, set_progress_log


class EllipseTests(unittest.TestCase):
    def setUp(self):
        set_progress_log(False)
        self.part = Part((-100,-100,-100),(100,100,100),tolerance=.02)

    def test_dimensions_drive_true_ellipse_and_extrusion(self):
        frame = Frame((3,5,7),x=(0,1,0),y=(0,0,1),z=(1,0,0))
        sk = self.part.sketch(frame=frame,constrained=True,name="section")
        sk.solve_after_every_constraint = False
        e = sk.add_ellipse((0,0),(8,5),rotation=.2,name="oval")
        sk.coincident(e @ "center",sk @ "origin")
        sk.point_on_line(e @ 0,sk @ "x")
        sk.distance(e @ "center",e @ 0,10)
        sk.distance(e @ "center",e @ .25,4)
        sk.solve()
        for point,expected in ((e @ 0,(10,0)),(e @ .25,(0,4)),(e @ .5,(-10,0)),
                               (e @ .75,(0,-4)),(e @ "center",(0,0))):
            with self.subTest(point=point):
                self.assertLess(math.dist(sk.eval_xy(point),expected),1e-6)
        solid = self.part.extrude(sk,6,max_deviation=.01)
        self.assertTrue(solid.is_watertight())
        self.assertAlmostEqual(solid.volume(),math.pi*10*4*6,delta=3)
        # A construction axis uses standard line constraints and is excluded
        # from the profile, rather than creating a second ellipse API.
        guide = sk.add_line(e @ "center",e @ 0,construction=True)
        self.assertNotEqual(guide.name,e.name)
        sk.horizontal(guide)
        sk.solve()
        self.assertLess(math.dist(sk.eval_xy(e @ .25),(0,4)),1e-6)
        with_guide = self.part.extrude(sk,6,max_deviation=.01)
        self.assertTrue(with_guide.is_watertight())
        self.assertAlmostEqual(with_guide.volume(),solid.volume(),places=8)

    def test_constrained_ellipses_loft_into_a_tapered_frame_member(self):
        sections = []
        for z,scale in ((0,1),(10,1.5)):
            sk = self.part.sketch(frame=Frame((z/10,0,z)),constrained=True)
            sk.solve_after_every_constraint = False
            e = sk.add_ellipse((0,0),(9*scale,5*scale),name="section")
            sk.coincident(e @ "center",sk @ "origin")
            sk.point_on_line(e @ 0,sk @ "x")
            sk.distance(e @ "center",e @ 0,10*scale)
            sk.distance(e @ "center",e @ .25,4*scale)
            sk.solve()
            sections.append(sk)
        solid = self.part.loft(sections,max_deviation=.02)
        self.assertTrue(solid.is_watertight())
        self.assertAlmostEqual(solid.volume(),math.pi*40*10*(1+1.5+1.5**2)/3,delta=10)

    def test_unconstrained_named_center_and_construction(self):
        sk = self.part.sketch(name="reference")
        e = sk.add_ellipse((2,3),(7,4),rotation=math.pi/2,construction=True)
        self.assertLess(math.dist(sk.eval_xy(e @ "center"),(2,3)),1e-12)
        self.assertLess(math.dist(sk.eval_xy(e @ 0),(2,10)),1e-12)

    def test_radius_is_rejected_with_axis_guidance(self):
        sk = self.part.sketch(constrained=True)
        e = sk.add_ellipse((0,0),(10,4))
        with self.assertRaisesRegex(Exception,"two semiaxes"):
            sk.radius(e,5)

    def test_invalid_axes_and_rotation(self):
        sk = self.part.sketch(constrained=True)
        for radii,rotation in (((0,4),0),((-1,4),0),((3,float("nan")),0),((3,4),float("inf"))):
            with self.subTest(radii=radii,rotation=rotation):
                with self.assertRaises(ValueError):
                    sk.add_ellipse((0,0),radii,rotation)
