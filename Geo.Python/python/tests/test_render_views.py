import os
import struct
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from camber import Part, render_views, set_progress_log
from camber.display import DisplayScene
from camber.glview import pack_scene


class RenderInputTests(unittest.TestCase):
    def test_color_patterns_preserve_names_and_override_in_order(self):
        scene = DisplayScene()
        scene.patches = [{"name": "fork:blade", "vertices": [(0, 0, 0), (1, 0, 0), (0, 1, 0)], "faces": [(0, 1, 2)]}]
        packed = pack_scene(scene, colors={"*": (.5, .5, .5), "fork:*": (.8, .1, .2)})
        self.assertEqual([(.8, .1, .2)]*3, packed["mesh_color"])
        self.assertEqual(["fork:blade"], packed["face_names"])
        with self.assertRaises(ValueError):
            pack_scene(scene, colors={"*": (255, 0, 0)})

    def test_invalid_views_fail_before_opening_gl(self):
        for kwargs in [{"columns": 0}, {"tile_size": (10, 10)}, {"views": []},
                       {"views": {"bad": ((0, 0, 1), (0, 0, 1))}},
                       {"views": {"bad": ((float("nan"), 0, 1), (0, 1, 0))}}]:
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                render_views(None, "unused.png", **kwargs)


@unittest.skipUnless(os.environ.get("CAMBER_TEST_GL") == "1", "set CAMBER_TEST_GL=1 with a working GL context")
class RenderCaptureTests(unittest.TestCase):
    def test_checkerboard_can_be_enabled_with_custom_colors(self):
        from unittest.mock import patch
        from camber.host import Viewer
        import numpy as np

        part = Part((-20, -20, -20), (20, 20, 20), tolerance=.1)
        block = part.cuboid((-5, -5, -5), (5, 5, 5), name="checker_block")
        images = []
        capture = Viewer.capture_rgba

        def record(viewer, label):
            pixels = capture(viewer, label)
            images.append(pixels.copy())
            return pixels

        with tempfile.TemporaryDirectory() as tmp, patch.object(Viewer, "capture_rgba", record):
            for checker in (False, True):
                render_views(block, Path(tmp) / f"checker-{checker}.png", views=["+Z"],
                             tile_size=(128, 128), colors={"*": (.6, .7, .8)}, checker=checker)
        # UV checks must alter the rendered interior, not geometry or background.
        self.assertGreater(np.count_nonzero(np.any(images[0] != images[1], axis=2)), 500)
        np.testing.assert_array_equal(images[0][0], images[1][0])

    def test_png_streams_close_after_success_and_encoder_failure(self):
        from unittest.mock import patch
        import pyglet

        set_progress_log(False)
        part = Part((-20, -20, -20), (20, 20, 20), tolerance=.1)
        block = part.cuboid((-2, -2, -2), (2, 2, 2), name="png_stream_owner")
        original = pyglet.image.ImageData.save
        for fail in (False, True):
            with self.subTest(encoder_failure=fail), tempfile.TemporaryDirectory() as tmp:
                streams = []
                def save(image, *args, **kwargs):
                    stream = kwargs.get("file")
                    self.assertIsNotNone(stream, "renderer must own its PNG stream")
                    self.assertFalse(stream.closed)
                    streams.append(stream)
                    if fail:
                        raise OSError("encoder failed")
                    return original(image, *args, **kwargs)
                with patch.object(pyglet.image.ImageData, "save", autospec=True, side_effect=save):
                    if fail:
                        with self.assertRaisesRegex(OSError, "encoder failed"):
                            render_views(block, Path(tmp)/"sheet.png", views=["+Z", "iso"],
                                         tile_size=(128, 128), individual=True)
                    else:
                        render_views(block, Path(tmp)/"sheet.png", views=["+Z", "iso"],
                                     tile_size=(128, 128), individual=True)
                self.assertEqual(1 if fail else 3, len(streams))
                self.assertTrue(all(stream.closed for stream in streams))

    def test_capture_process_exits_cleanly_with_native_objects_still_alive(self):
        # In-process resource assertions cannot detect native crashes during
        # interpreter shutdown. Keep CAD objects alive until natural exit.
        script = """
import sys
from pathlib import Path
from camber import Part, render_views, set_progress_log
set_progress_log(False)
part = Part((-20, -20, -20), (20, 20, 20), tolerance=.1)
block = part.cuboid((-5, -5, -5), (10, 10, 10), name="block")
for index in range(3):
    render_views(block, Path(sys.argv[1]) / f"capture{index}.png",
                 views=["+X", "iso"], tile_size=(64, 64))
print("captures complete", flush=True)
"""
        env = os.environ.copy()
        env["PYTHONPATH"] = str(Path(__file__).resolve().parents[1]) + os.pathsep + env.get("PYTHONPATH", "")
        env["PYTHONFAULTHANDLER"] = "1"
        with tempfile.TemporaryDirectory() as tmp:
            result = subprocess.run([sys.executable, "-c", script, tmp], env=env,
                                    capture_output=True, text=True, timeout=60)
            self.assertEqual(0, result.returncode, result.stdout + result.stderr)
            self.assertIn("captures complete", result.stdout)
            for index in range(3):
                self.assertEqual(b"\x89PNG\r\n\x1a\n", (Path(tmp) / f"capture{index}.png").read_bytes()[:8])

    def test_capture_releases_gl_resources_on_success_and_failure(self):
        from unittest.mock import patch
        from camber.host import Viewer

        set_progress_log(False)
        part = Part((-20,-20,-20),(20,20,20),tolerance=.1)
        block = part.cuboid((-5,-5,-5),(10,10,10),name="block")
        monitor = Viewer(visible=False,size=(64,64))
        created = []

        def create(*args,**kwargs):
            viewer = Viewer(*args,**kwargs)
            # Retain wrappers: correctness must not depend on garbage collection.
            created.append((viewer,viewer._dummy_depth,viewer._mesh_prog.id))
            return viewer

        try:
            with tempfile.TemporaryDirectory() as tmp, patch("camber.host.Viewer",side_effect=create):
                path = Path(tmp)/"capture.png"
                def check_released():
                    monitor.window.switch_to()
                    viewer,texture,program = created[-1]
                    self.assertFalse(monitor._gl.glIsTexture(texture))
                    self.assertFalse(monitor._gl.glIsProgram(program))
                    viewer.close()  # Repeated cleanup must be harmless.

                render_views(block,path,views=["iso"],tile_size=(64,64))
                check_released()
                with patch.object(Viewer,"capture_rgba",side_effect=RuntimeError("capture failed")):
                    with self.assertRaisesRegex(RuntimeError,"capture failed"):
                        render_views(block,path,views=["iso"],tile_size=(64,64))
                check_released()
                # Check before creating another context: GL can reuse deleted IDs.
                render_views(block,path,views=["iso"],tile_size=(64,64))
                check_released()
        finally:
            monitor.close()

    def test_partial_constructor_failure_releases_resources_and_allows_next_capture(self):
        from unittest.mock import patch
        from camber.host import Viewer

        set_progress_log(False)
        part = Part((-20,-20,-20),(20,20,20),tolerance=.1)
        block = part.cuboid((-5,-5,-5),(10,10,10),name="block")
        monitor = Viewer(visible=False,size=(64,64))
        allocated = []
        compile_program = Viewer._compile

        def fail_second_program(viewer, *args):
            if allocated:
                raise RuntimeError("shader compilation failed")
            program = compile_program(viewer, *args)
            allocated.append((viewer, program, program.id))
            return program

        try:
            with tempfile.TemporaryDirectory() as tmp:
                path = Path(tmp)/"capture.png"
                with patch.object(Viewer,"_compile",new=fail_second_program):
                    with self.assertRaisesRegex(RuntimeError,"shader compilation failed"):
                        render_views(block,path,views=["iso"],tile_size=(64,64))
                failed, program, program_id = allocated[0]
                self.assertTrue(failed._closed)
                self.assertIsNone(failed.window.context)
                monitor.window.switch_to()
                self.assertFalse(monitor._gl.glIsProgram(program_id))
                self.assertIsNone(program.id)
                failed.close()
                render_views(block,path,views=["iso"],tile_size=(64,64))
                self.assertTrue(path.is_file())
        finally:
            monitor.close()

    def test_close_finalizes_window_when_context_switch_fails(self):
        from unittest.mock import Mock
        from camber.host import Viewer

        viewer = object.__new__(Viewer)
        viewer._closed = False
        viewer.window = Mock()
        viewer.window.switch_to.side_effect = RuntimeError("context switch failed")
        with self.assertRaisesRegex(RuntimeError,"context switch failed"):
            viewer.close()
        self.assertTrue(viewer._closed)
        viewer.window.close.assert_called_once_with()
        viewer.close()
        viewer.window.close.assert_called_once_with()

    def test_repeated_capture_has_expected_geometry_and_sheet_dimensions(self):
        import numpy as np
        import pyglet
        set_progress_log(False)
        part = Part((-20, -20, -20), (20, 20, 20), tolerance=.1)
        block = part.cuboid((-5, -7, -9), (10, 14, 18), name="block")
        child = part.assembly("child")
        child.fix(child.add_part(block))
        root = part.assembly("root")
        root.fix(root.add_subassembly(child, (2, 0, 0)))
        with tempfile.TemporaryDirectory() as tmp:
            for i, colors in enumerate([{"*": (.8, .1, .1)}, {"*": (.1, .1, .8)}]):
                path = render_views(root, Path(tmp)/f"sheet{i}.png", tile_size=(256, 192), columns=4, colors=colors, individual=True)
                data = path.read_bytes()
                self.assertEqual(b"\x89PNG\r\n\x1a\n", data[:8])
                w, h = struct.unpack(">II", data[16:24])
                self.assertEqual((1024, 384), (w, h))
                with path.open("rb") as source:
                    image = pyglet.image.load(str(path), file=source).get_image_data()
                pixels = np.frombuffer(image.get_bytes("RGBA", -w*4), np.uint8).reshape(h, w, 4)
                for index in range(8):
                    single_path = path.with_name(f"{path.stem}_{index+1:02d}.png")
                    with single_path.open("rb") as source:
                        single = pyglet.image.load(str(single_path), file=source).get_image_data()
                    self.assertEqual((256, 192), (single.width, single.height))
                    actual = np.frombuffer(single.get_bytes("RGBA", -256*4), np.uint8).reshape(192, 256, 4)
                    row, col = divmod(index, 4)
                    np.testing.assert_array_equal(actual, pixels[row*192:(row+1)*192, col*256:(col+1)*256])
                # Known object pixels, not just file existence: catches stale or
                # unrelated desktop pixels from an unmapped window's back buffer.
                center = pixels[96, 128, :3]
                dominant = 0 if i == 0 else 2
                self.assertGreater(center[dominant], 100)
                self.assertLess(center[1], 60)
                self.assertGreater(center[dominant], center[2-dominant]*3)


if __name__ == "__main__":
    unittest.main()
