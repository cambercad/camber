"""Picking and annotations must use the same camera as the rendered workpiece."""

from types import SimpleNamespace
import unittest

from camber.glview import Camera
from camber.sketch_ui import _world_to_screen
from camber.sketch_ui import _origin_axes
from camber.view import camera_ray
from camber.view import _world_per_pixel


class ViewportCoordinatesTests(unittest.TestCase):
    def test_origin_axes_follow_a_placed_sketch_frame(self):
        frame = SimpleNamespace(origin=(7, 8, 9), x=(0, 1, 0), y=(0, 0, 1), z=(1, 0, 0))
        x, y = _origin_axes(frame, .5, 4)
        self.assertEqual((7.5, 8, 9), x[1][0])
        self.assertEqual(x[1][0], y[1][0])
        self.assertEqual((7.5, 12, 9), x[1][1])
        self.assertEqual((7.5, 8, 13), y[1][1])

    def test_docked_camera_projection_and_picking_agree_in_both_modes_and_dpi_scales(self):
        for ortho in (True, False):
            for scale in (1, 2):
                with self.subTest(ortho=ortho, scale=scale):
                    camera = Camera()
                    camera.width, camera.height = 1920*scale, 1080*scale
                    camera.viewport = tuple(v*scale for v in (240, 168, 1320, 880))
                    camera.eye, camera.center, camera.up = (0, 0, 50), (0, 0, 0), (0, 1, 0)
                    camera.zoom, camera.ortho = 20, ortho
                    viewer = SimpleNamespace(camera=camera, get_window_size=lambda: (1920, 1080))
                    ui = SimpleNamespace(GetIO=lambda: SimpleNamespace(DisplaySize=(1920, 1080)))
                    self.assertEqual((900, 608), _world_to_screen(viewer, ui, camera.center))
                    self.assertAlmostEqual(2*camera.view_scale/880, _world_per_pixel(viewer, ui))
                    self.assertEqual(_world_per_pixel(viewer, ui), _world_per_pixel(viewer, None))
                    for screen in ((900, 608), (700, 420), (1100, 790)):
                        origin, direction = camera_ray(viewer, ui, screen)
                        t = -origin[2]/direction[2]
                        point = tuple(o+t*d for o,d in zip(origin, direction))
                        projected = _world_to_screen(viewer, ui, point)
                        for wanted, actual in zip(screen, projected):
                            self.assertAlmostEqual(wanted, actual, places=6)

    def test_perspective_annotation_behind_camera_is_not_projected(self):
        camera = Camera()
        camera.ortho = False
        camera.eye, camera.center = (0, 0, 5), (0, 0, 0)
        self.assertIsNone(camera.project((0, 0, 6)))
