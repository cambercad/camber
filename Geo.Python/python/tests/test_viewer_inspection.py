"""Viewer distance tools and live sections, including real GL rendering/events."""
import math
import os
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from camber import glview
from camber.display import DisplayScene
from camber.inspection import _ViewerInspection


def cube_scene(hollow=False):
    scene = DisplayScene()
    faces = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7),
             (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5),
             (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    for radius, name in ((2, "box:outside"), (1, "box:inside")) if hollow else ((2, "box:outside"),):
        r = radius
        vertices = [(-r, -r, -r), (r, -r, -r), (r, r, -r), (-r, r, -r),
                    (-r, -r, r), (r, -r, r), (r, r, r), (-r, r, r)]
        triangles = [(c, b, a) for a, b, c in faces] if name.endswith("inside") else faces
        scene.patches.append({"name": name, "vertices": vertices, "faces": triangles})
    scene.points.extend([{"name": "box:p0", "position": (-2, -2, -2)},
                         {"name": "box:p1", "position": (2, -2, -2)}])
    return scene


class InspectionStateTests(unittest.TestCase):
    def tools(self):
        packed = glview.pack_scene(cube_scene())
        viewer = SimpleNamespace(visible_part_packeds=lambda: [packed],
                                 get_bounding_box=lambda: packed["bounds"])
        return _ViewerInspection(viewer)

    def test_measurement_cache_and_clear(self):
        tools = self.tools()
        result = tools.measure_selected(["box:p0", "box:p1"])
        self.assertEqual(4, result.length)
        cached = tools._geometry["box:p0"]
        tools.measure_selected(["box:p0", "box:p1"])
        self.assertIs(cached, tools._geometry["box:p0"])
        tools.measurements.clear()
        self.assertEqual([], tools.measurements)

    def test_arbitrary_section_normal_offset_and_reverse(self):
        tools = self.tools()
        tools.set_section((1, 2, 2), 3)
        self.assertEqual(((1/3, 2/3, 2/3), 1), tools.viewer.section_plane)
        lo, hi = tools._limits()
        self.assertAlmostEqual(-10/3, lo)
        self.assertAlmostEqual(10/3, hi)
        tools.set_section((1, 2, 2), 3, reverse=True)
        self.assertEqual(((-1/3, -2/3, -2/3), -1), tools.viewer.section_plane)
        for normal, offset in (((0, 0, 0), 0), ((math.nan, 0, 1), 0), ((0, 0, 1), math.inf)):
            with self.assertRaises(ValueError):
                tools.set_section(normal, offset)


@unittest.skipUnless(os.environ.get("CAMBER_TEST_GL") == "1", "requires OpenGL")
class LiveInspectionGlTests(unittest.TestCase):
    def viewer(self, hollow=False, visible=False):
        from camber.host import Viewer
        viewer = Viewer(visible=visible, size=(640, 480))
        self.addCleanup(viewer.close)
        viewer.set_solid(glview.pack_scene(cube_scene(hollow), colors={"*": (.35, .5, .75)}))
        viewer.checker = False
        viewer.camera.eye, viewer.camera.center, viewer.camera.up = (0, 0, 10), (0, 0, 0), (0, 1, 0)
        viewer.camera.ortho, viewer.camera.zoom = True, 3
        return viewer

    def test_section_is_capped_and_motion_reuses_the_original_mesh(self):
        viewer = self.viewer()
        tools = _ViewerInspection(viewer)
        mesh = viewer._gpu_parts()[0]["mesh"]
        original = viewer.capture_rgba()
        tools.set_section((0, 0, 1), 0)
        cut = viewer.capture_rgba()
        red, green, blue = map(int, cut[240, 320, :3])
        self.assertGreater(red, 200)
        self.assertLess(blue, 100)
        self.assertGreater(green, 100)
        self.assertFalse((original[240, 320] == cut[240, 320]).all())
        cap = viewer._section_cap
        for offset in (-1, .5, 1):
            tools.set_section((0, 0, 1), offset)
            viewer.capture_rgba()
        self.assertIs(cap, viewer._section_cap)
        self.assertIs(mesh, viewer._gpu_parts()[0]["mesh"])
        self.assertEqual(viewer._gl.GL_NO_ERROR, viewer._gl.glGetError())

    def test_hollow_section_preserves_the_cavity(self):
        viewer = self.viewer(hollow=True)
        tools = _ViewerInspection(viewer)
        tools.set_section((0, 0, 1), 0)
        pixels = viewer.capture_rgba()
        # Solid ring is ochre; the center exposes the blue cavity wall.
        self.assertGreater(int(pixels[240, 435, 0]), 200)
        self.assertLess(int(pixels[240, 435, 2]), 100)
        self.assertGreater(int(pixels[240, 320, 2]), int(pixels[240, 320, 0]))
        viewer._solid_transparent = True
        viewer.set_transparency_mode("pretty")
        viewer.capture_rgba()
        self.assertEqual(viewer._gl.GL_NO_ERROR, viewer._gl.glGetError())

    def test_reverse_section_and_arbitrary_plane_render(self):
        viewer = self.viewer(hollow=True)
        tools = _ViewerInspection(viewer)
        tools.set_section((1, .4, .2), .1)
        pixels = viewer.capture_rgba()
        tools.set_section((1, .4, .2), .1, reverse=True)
        reverse = viewer.capture_rgba()
        self.assertGreater(viewer._np.count_nonzero(pixels != reverse), 5000)
        self.assertEqual(viewer._gl.GL_NO_ERROR, viewer._gl.glGetError())

    def test_main_viewer_initializes_for_an_ordinary_solid_scene(self):
        from camber.host import Viewer, run_solid

        def drive(viewer):
            self.addCleanup(viewer.close)
            viewer.on_draw()
            result = viewer.inspection.measure_selected(["box:p0", "box:p1"])
            self.assertEqual(4, result.length)
            viewer.on_draw()
            self.assertEqual(viewer._gl.GL_NO_ERROR, viewer._gl.glGetError())
            viewer.close()

        with patch.object(Viewer, "show", drive):
            run_solid(cube_scene())
