"""Standalone primitive coverage and caching on a framebuffer without MSAA."""
import importlib.util
import math
import os
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location("coverage_gui", Path(__file__).resolve().parents[1] / "camber" / "gui.py")
gui = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gui)


@unittest.skipUnless(os.environ.get("CAMBER_TEST_GL") == "1", "requires OpenGL")
class GuiCoverageTests(unittest.TestCase):
    def test_convex_quad_has_no_internal_diagonal_and_accepts_both_windings(self):
        import pyglet
        def capture(reverse=False):
            self.window.clear()
            self.ui.new_frame()
            self.ui.SetNextWindowPos((0,0),self.ui.ImGuiCond_Always)
            self.ui.SetNextWindowSize((220,180),self.ui.ImGuiCond_Always)
            self.ui.Begin('Quad',self.ui.ImGuiWindowFlags_NoDecoration)
            dl=self.ui.GetWindowDrawList()
            dl.AddRectFilled((0,0),(220,180),(255,255,255,255))
            points=((40,30),(160,50),(140,150),(20,130))
            dl.AddQuadFilled(*(points[::-1] if reverse else points),(30,30,30,255))
            self.ui.End()
            self.ui.render()
            return pyglet.graphics.framebuffer.get_screenshot().get_bytes('RGBA',220*4)
        pixels=capture()
        self.assertTrue(all(v==30 for v in self.region(pixels,(60,65,60,40))))
        self.assertEqual(bytes(pixels),bytes(capture(True)))
        self.assertGreater(sum(30<v<255 for v in self.region(pixels,(19,29,142,123))),40)

    def setUp(self):
        import pyglet
        config = pyglet.config.Config()
        config.opengl = pyglet.config.OpenGLUserConfig(double_buffer=True, sample_buffers=0, samples=0, major_version=3, minor_version=3)
        self.window = pyglet.window.Window(220, 180, config=config, visible=False)
        self.ui = gui.UI(self.window)

    def tearDown(self):
        self.ui.shutdown()
        self.window.close()

    def frame(self, color=(0, 0, 0, 255), reverse=False, degenerate=False):
        import pyglet
        self.window.switch_to()
        self.window.clear()
        self.ui.new_frame()
        self.ui.SetNextWindowPos((0, 0), self.ui.ImGuiCond_Always)
        self.ui.SetNextWindowSize((220, 180), self.ui.ImGuiCond_Always)
        self.ui.Begin("Coverage", self.ui.ImGuiWindowFlags_NoDecoration)
        dl = self.ui.GetWindowDrawList()
        dl.AddRectFilled((0, 0), (220, 180), (255, 255, 255, 255))
        dl.AddLine((20.25, 40.75), (90.25, 68.75), color, 1.25)
        dl.AddCircle((60, 112), 18, color, 32, 1.5)
        dl.AddCircleFilled((126, 50), 12, color)
        points = ((120, 92), (145, 117), (170, 142)) if degenerate else ((120, 92), (175, 104), (140, 144))
        dl.AddTriangleFilled(*(points[::-1] if reverse else points), color)
        dl.AddEllipseArc((174, 42), (16, 10), 0, 1.5*math.pi, color, 1.5)
        self.ui.End()
        self.ui.render()
        return pyglet.graphics.framebuffer.get_screenshot().get_bytes("RGBA", 220*4)

    @staticmethod
    def region(pixels, rect):
        x, y, w, h = rect
        return [pixels[((179-row)*220+column)*4] for row in range(y, y+h) for column in range(x, x+w)]

    def test_every_diagonal_and_curved_primitive_has_subpixel_coverage_without_msaa(self):
        from pyglet.graphics.api import gl
        samples = gl.GLint()
        gl.glGetIntegerv(gl.GL_SAMPLES, samples)
        self.assertEqual(0, samples.value)
        pixels = self.frame()
        for rect in ((18, 38, 75, 33), (39, 91, 42, 42), (112, 36, 28, 28),
                     (118, 90, 60, 57), (156, 29, 37, 26)):
            with self.subTest(region=rect):
                values = self.region(pixels, rect)
                self.assertGreater(sum(10 < value < 245 for value in values), 10)
        self.assertEqual(0, self.region(pixels, (126, 50, 1, 1))[0])
        self.assertEqual(255, self.region(pixels, (60, 112, 1, 1))[0])
        reversed_pixels = self.frame(reverse=True)
        self.assertEqual(self.region(pixels, (118, 90, 60, 57)), self.region(reversed_pixels, (118, 90, 60, 57)))

    def test_coverage_shapes_update_color_without_reallocating_cached_geometry(self):
        self.frame()
        before = [obj for signature, obj in self.ui._cache if signature[0] in ("line", "arc", "circle", "ellipse_arc", "triangle")]
        self.frame((10, 100, 220, 255))
        after = [obj for signature, obj in self.ui._cache if signature[0] in ("line", "arc", "circle", "ellipse_arc", "triangle")]
        self.assertEqual(before, after)
        self.assertTrue(all(obj.color == (10, 100, 220, 255) for obj in after))

    def test_collinear_triangle_does_not_fill_its_bounding_rectangle(self):
        pixels = self.frame(degenerate=True)
        self.assertTrue(all(value == 255 for value in self.region(pixels, (118, 90, 60, 57))))


if __name__ == "__main__":
    unittest.main()
