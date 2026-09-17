import unittest
from camber import Part, set_progress_log


class SolidEdgeTests(unittest.TestCase):
    def test_inspected_edges_and_reversed_face_pairs_blend_identically(self):
        set_progress_log(False)
        part = Part((-10, -10, -10), (10, 10, 10), tolerance=.01)
        sketch = part.sketch("xy", name="profile")
        sketch.add_rectangle((0, 0), (4, 6), names=("south", "east", "north", "west"))
        solid = part.extrude(sketch, 3, name="block")
        self.assertEqual(12, len(solid.edge_names))
        edge = "[block-south,block-east]"
        reverse = "[block-east,block-south]"
        self.assertIn(edge, solid.edge_names)
        for operation in (part.fillet, part.chamfer):
            with self.subTest(operation=operation.__name__):
                a = operation(solid, edge, .3, name=operation.__name__+"_forward")
                b = operation(solid, reverse, .3, name=operation.__name__+"_reversed")
                self.assertTrue(a.is_watertight())
                self.assertTrue(b.is_watertight())
                self.assertAlmostEqual(a.volume(), b.volume(), places=9)
                self.assertLess(a.volume(), solid.volume())
                self.assertGreater(len(a.edge_names), len(solid.edge_names))

    def test_disconnected_fillet_groups_get_unique_corner_patch_names(self):
        set_progress_log(False)
        part = Part((-10,-10,-10),(60,60,20),tolerance=.02)
        sketch = part.sketch("xy")
        points = [(0,0),(40,0),(40,10),(20,10),(20,30),(0,30)]
        for i, point in enumerate(points):
            sketch.add_line(point,points[(i+1)%len(points)],name=f"edge_{i}")
        solid = part.extrude(sketch,5,name="step")
        edges = [f"[step-edge_{(i-1)%len(points)},step-edge_{i}]" for i in range(len(points))]
        rounded = part.fillet(solid,edges,1,name="rounded")
        self.assertTrue(rounded.is_watertight())
        self.assertGreater(rounded.volume(),0)
        self.assertLess(rounded.volume(),solid.volume())


    def test_equal_radius_end_round_from_constrained_circle(self):
        import math
        from camber import Frame
        set_progress_log(False)
        radius=.5
        for bottom,angle in ((False,0),(True,0),(False,.7)):
            with self.subTest(bottom=bottom,angle=angle):
                part=Part((-5,-5,-5),(5,5,5),tolerance=.001)
                axis=(math.sin(angle),0,math.cos(angle))
                frame=Frame((0,0,0),x=(math.cos(angle),0,-math.sin(angle)),y=(0,1,0),z=axis)
                sketch=part.sketch(frame=frame,constrained=True)
                sketch.solve_after_every_constraint=False
                sketch.add_circle((0,0),radius,name="section")
                sketch.radius("section",radius).fix("section@center")
                sketch.solve()
                blank=part.extrude(sketch,radius,name="cap_blank")
                end="ExtrudeBottom" if bottom else "ExtrudeTop"
                edges=[edge for edge in blank.edge_names if end in edge]
                self.assertEqual(1,len(edges))
                hemisphere=part.fillet(blank,edges,radius,name="hemisphere",max_deviation=.001)
                self.assertTrue(hemisphere.is_watertight())
                expected=2*math.pi*radius**3/3
                # Bound volume error by area times the tessellation deviation.
                self.assertAlmostEqual(expected,hemisphere.volume(),delta=3*math.pi*radius**2*.0011)
                self.assertLess(hemisphere.volume(),blank.volume())

    def test_equal_radius_sector_with_its_tangent_rim_segments(self):
        import math
        from camber import Frame
        for bottom, angle in ((False, 0), (True, .7)):
            with self.subTest(bottom=bottom, angle=angle):
                part = Part((30, 14, 5), (38, 22, 16), tolerance=.001)
                frame = Frame((33, 17, 8), x=(math.cos(angle), 0, -math.sin(angle)),
                              y=(0, 1, 0), z=(math.sin(angle), 0, math.cos(angle)))
                sketch = part.sketch(frame=frame)
                sketch.add_rectangle((0, 0), (1, 1), names=("south", "east", "north", "west"))
                blank = part.extrude(sketch, 3, name="sector_blank")
                rounded = part.fillet(blank, "[sector_blank-north,sector_blank-west]", .5,
                                      name="vertical_round", max_deviation=.001)
                end = "ExtrudeBottom" if bottom else "ExtrudeTop"
                rim = [edge for edge in rounded.edge_names if end in edge
                       and "sector_blank-south" not in edge and "sector_blank-east" not in edge]
                self.assertEqual(3, len(rim))
                result = part.fillet(rounded, rim, .5, name="spherical_corner", max_deviation=.001)
                self.assertTrue(result.is_watertight())
                self.assertAlmostEqual(result.volume(), 2.752673239922555, delta=.01)
