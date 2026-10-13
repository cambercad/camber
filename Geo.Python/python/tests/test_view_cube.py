"""Orientation control geometry and real viewer input, independent of models."""
from itertools import product
import math
import os
import unittest

from camber.glview import Camera, _basis, _norm3
from camber.view import _fit_view
from camber.view_cube import ViewCube


class CubeGeometryTests(unittest.TestCase):
    def test_all_26_face_edge_and_corner_directions_have_unambiguous_targets(self):
        camera, cube = Camera(), ViewCube()
        for direction in product((-1, 0, 1), repeat=3):
            if not any(direction):
                continue
            with self.subTest(direction=direction):
                camera.look_from(direction)
                cube.layout(camera.frame(), (20, 30, 112, 112))
                self.assertEqual(direction, cube.hit((76, 86)))
                self.assertEqual(sum(bool(v) for v in direction), len(cube.faces))
                self.assertIsNone(cube.hit((0, 0)))

    def test_orientation_preserves_center_distance_zoom_and_projection(self):
        camera = Camera()
        camera.center, camera.eye, camera.zoom = (100, 20, 30), (100, 10, 30), 12
        for direction, up in [((0,0,1),(0,1,0)), ((0,0,-1),(0,-1,0)), ((1,-1,1),(0,0,1))]:
            camera.look_from(direction)
            self.assertEqual((100,20,30), camera.center)
            self.assertAlmostEqual(100, sum((a-b)**2 for a,b in zip(camera.eye,camera.center)))
            self.assertEqual(12,camera.zoom)
            self.assertEqual(up,camera.up)
            self.assertTrue(camera.ortho)
        for direction in ((0,0,0),(float('nan'),0,1)):
            with self.assertRaises(ValueError): camera.look_from(direction)

    def test_face_turn_uses_the_nearest_quarter_roll_and_the_short_arc(self):
        camera = Camera()
        camera.center = (0.0, 0.0, 0.0)
        camera.eye = (0.0, -10.0, 0.0)
        rolled = math.radians(170.0)
        camera.up = (math.sin(rolled), 0.0, math.cos(rolled))
        start = camera.frame()
        camera.start_turn((0.0, -1.0, 0.0), now=0.0)
        camera.step_turn(0.25)
        mid = camera.frame()
        camera.step_turn(0.5)
        end = camera.frame()
        self.assertLess(end[1][2], -0.99)
        self.assertAlmostEqual(end[1][0], 0.0, places=5)
        self.assertGreater(sum(a*b for a, b in zip(start[1], mid[1])), 0.9)
        self.assertLess(_frame_angle(start, end), math.radians(20.0))
        self.assertLess(_frame_angle(start, mid), _frame_angle(start, end) + 1e-6)

    def test_turn_to_the_opposite_face_never_spins_the_long_way(self):
        camera = Camera()
        camera.center = (0.0, 0.0, 0.0)
        camera.eye = (0.0, -8.0, 0.0)
        camera.up = (0.0, 0.0, 1.0)
        start = camera.frame()
        camera.start_turn((0.0, 1.0, 0.0), now=0.0)
        camera.step_turn(0.5)
        end = camera.frame()
        for actual, target in zip(end[0], (0.0, -1.0, 0.0)):
            self.assertAlmostEqual(actual, target, places=5)
        self.assertLessEqual(_frame_angle(start, end), math.pi + 1e-6)

    def test_edge_and_corner_turns_use_the_nearest_quarter_roll(self):
        for direction in ((1.0, 0.0, 1.0), (1.0, -1.0, 1.0)):
            with self.subTest(direction=direction):
                normal = _norm3(direction)
                look = tuple(-component for component in normal)
                right, up, _look = _basis(look, (0.0, 0.0, 1.0))
                tilt = math.radians(20.0)
                camera = Camera()
                camera.center = (0.0, 0.0, 0.0)
                camera.eye = tuple(component * 10.0 for component in normal)
                camera.up = tuple(math.cos(tilt) * right[i] + math.sin(tilt) * up[i] for i in range(3))
                start = camera.frame()
                camera.start_turn(direction, now=0.0)
                camera.step_turn(0.5)
                end = camera.frame()
                for actual, target in zip(end[0], look):
                    self.assertAlmostEqual(actual, target, places=5)
                self.assertGreater(sum(a * b for a, b in zip(end[1], right)), 0.99)
                self.assertLess(_frame_angle(start, end), math.radians(25.0))

    def test_turn_then_fit_keeps_the_model_between_the_clip_planes(self):
        camera = Camera()
        camera.eye = (5.0, 5.0, 53.0)
        camera.center = (5.0, 5.0, 50.0)
        camera.up = (0.0, 1.0, 0.0)
        camera.near, camera.far = 0.05, 10.0
        scene = _BoundsScene(camera, ((0.0, 0.0, 0.0), (10.0, 10.0, 100.0)))
        camera.start_turn((1.0, 0.0, 0.0), now=0.0)
        camera.step_turn(0.5)
        _fit_view(scene)
        look = camera.frame()[0]
        for axis, value in zip(look, (-1.0, 0.0, 0.0)):
            self.assertAlmostEqual(axis, value, places=5)
        lows, highs = scene.bounds
        for corner in product((lows[0], highs[0]), (lows[1], highs[1]), (lows[2], highs[2])):
            depth = sum((corner[i] - camera.eye[i]) * look[i] for i in range(3))
            self.assertGreater(depth, camera.near)
            self.assertLess(depth, camera.far)


def _frame_angle(a, b):
    trace = sum(sum(x*y for x, y in zip(a[i], b[i])) for i in range(3))
    return math.acos(max(-1.0, min(1.0, (trace - 1.0) * 0.5)))


class _BoundsScene:
    def __init__(self, camera, bounds):
        self.camera = camera
        self.bounds = bounds

    def get_bounding_box(self):
        return self.bounds

    def get_window_size(self):
        return (800, 600)


@unittest.skipUnless(os.environ.get('CAMBER_TEST_GL') == '1', 'requires OpenGL')
class CubeViewerTests(unittest.TestCase):
    def test_edge_colors_follow_edge_axis_and_remain_stable_on_hover(self):
        from camber.host import Viewer
        from camber.gui_testing import GuiDriver
        viewer=Viewer('Axis colors',size=(480,360))
        viewer.window.set_visible(False)
        driver=GuiDriver(viewer._ui,viewer._draw_frame)
        try:
            for theme in ('light','dark'):
                viewer._ui.set_theme(theme)
                viewer.camera.look_from((1,-1,1))
                driver.move(400,20)
                cube=viewer._view_cube
                for direction,_,polygon in cube.cells:
                    if sum(bool(v) for v in direction)!=2: continue
                    axis=direction.index(0)
                    point=tuple(sum(p[i] for p in polygon)/4 for i in (0,1))
                    for hover in (False,True):
                        driver.move(*point if hover else (400,20))
                        fill=next(args[-1] for kind,args,_ in viewer._ui._commands
                                  if kind=='quad' and args[:4]==polygon)
                        self.assertTrue(all(fill[axis]>fill[other] for other in range(3) if other!=axis))
        finally:
            viewer.close()

    def test_faces_edges_and_corners_change_view_without_leaking_scene_clicks(self):
        from camber.host import Viewer
        from camber.gui_testing import GuiDriver
        viewer = Viewer('Orientation', size=(480,360))
        viewer.window.set_visible(False)
        driver = GuiDriver(viewer._ui,viewer._draw_frame)
        captured=[]
        def tick():
            if viewer._ui.io.MouseClicked[0]: captured.append(viewer._ui.io.WantCaptureMouse)
        viewer._tick=tick
        try:
            for theme in ('light','dark'):
                viewer._ui.set_theme(theme)
                for direction in ((0,0,1),(1,-1,0),(1,-1,1)):
                    viewer.camera.look_from((1,-1,1))
                    driver.frame()
                    cube=viewer._view_cube
                    if sum(bool(v) for v in direction)==1:
                        indices=next(indices for normal,indices in cube.faces if normal==direction)
                        point=tuple(sum(cube.points[i][axis] for i in indices)/4 for axis in (0,1))
                    elif sum(bool(v) for v in direction)==2:
                        from camber.view_cube import _VERTICES
                        a,b=next((a,b) for a,b in cube.edges
                                 if tuple((x+y)//2 for x,y in zip(_VERTICES[a],_VERTICES[b]))==direction)
                        point=tuple((cube.points[a][axis]+cube.points[b][axis])/2 for axis in (0,1))
                    else:
                        from camber.view_cube import _VERTICES
                        point=cube.points[_VERTICES.index(direction)]
                    driver.click(x=point[0],y=point[1])
                    viewer.camera.finish_turn()
                    driver.frame()
                    look=viewer.camera.frame()[0]
                    norm=sum(v*v for v in direction)**.5
                    for a,b in zip(look,direction): self.assertAlmostEqual(-b/norm,a)
            self.assertEqual([True]*6,captured)
            viewer.window.set_size(640,420)
            driver.frame()
            control=driver.control('View direction',panel='View cube')
            self.assertGreaterEqual(control.rect[0],viewer._ui.viewport_rect[0])
            self.assertLessEqual(control.rect[1]+control.rect[3],420)
        finally:
            viewer.close()

    def test_right_click_on_the_viewport_fits_without_opening_on_a_drag(self):
        from camber.gui_testing import GuiDriver
        from camber.host import Viewer
        viewer = Viewer('Viewport menu', size=(480, 360))
        viewer.window.set_visible(False)
        driver = GuiDriver(viewer._ui, viewer._draw_frame)
        mouse = viewer._pyglet.window.mouse
        bounds = ((-2.0, -3.0, -4.0), (2.0, 3.0, 4.0))
        viewer.get_bounding_box = lambda: bounds
        viewer.camera.eye = (0.0, -2.0, 40.0)
        viewer.camera.center = (0.0, 0.0, 40.0)
        viewer.camera.near, viewer.camera.far = 0.02, 3.0
        try:
            x, y, width, height = viewer._ui.viewport_rect
            point = (x + width * 0.5, y + height * 0.55)
            driver.drag(point, (point[0] + 40, point[1] + 24), button='right')
            self.assertIsNone(viewer._ui._menu)
            driver.move(*point)
            viewer._ui.on_mouse_press(point[0], viewer.window.height - point[1], mouse.RIGHT, 0)
            viewer._ui.on_mouse_release(point[0], viewer.window.height - point[1], mouse.RIGHT, 0)
            driver.frame()
            self.assertEqual('Fit', viewer._ui._menu.items[0].label)
            rect = dict(viewer._ui._menu.rows)[0]
            driver.click(x=rect[0] + 30, y=rect[1] + 10)
            look = viewer.camera.frame()[0]
            lows, highs = bounds
            corners = list(product((lows[0], highs[0]), (lows[1], highs[1]), (lows[2], highs[2])))
            for corner in corners:
                depth = sum((corner[i] - viewer.camera.eye[i]) * look[i] for i in range(3))
                self.assertGreater(depth, viewer.camera.near)
                self.assertLess(depth, viewer.camera.far)
            self.assertEqual((0.0, 0.0, 0.0), tuple(round(v, 5) for v in viewer.camera.center))
        finally:
            viewer.close()


if __name__ == '__main__': unittest.main()
