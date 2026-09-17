"""The NativeAOT runtime must outlive Python wrappers and their CFFI proxies."""
import ast
import importlib.util
from pathlib import Path
import subprocess
import sys
import textwrap
import unittest

import _camber_native.__dotwrap_generated.__camber_native as loader


SCENE_REPRO = '''
from camber import Part, glview, set_progress_log
set_progress_log(False)
for optional in (False, True):
    part = Part((-10, -10, -10), (20, 20, 20), tolerance=.01)
    base = part.cuboid((0, 0, 0), (12, 4, 2), name="base")
    if optional:
        notch = part.cuboid((1, -1, -1), (2, 5, 3), name="optional_slot")
        base = part.cut(base, notch, name="before_main")
        assert base.edge_names
    cutter = part.cuboid((4, -1, -1), (5, 5, 3), name="main_slot")
    final = part.cut(base, cutter, name="finished")
    packed = glview.pack_scene(glview._as_scene(final))
    assert packed["mesh_idx"] and packed["face_names"]
print("completed", flush=True)
'''


class NativeLifetimeTests(unittest.TestCase):
    def run_child(self, source):
        result = subprocess.run(
            [sys.executable, "-I", "-X", "faulthandler", "-c", source],
            capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        return result.stdout

    def test_installed_scene_packaging_exits_cleanly(self):
        # This CPU-only sequence previously exited with SIGSEGV after printing:
        # CFFI dlclose unmapped code still executing on .NET thread-pool workers.
        for attempt in range(3):
            with self.subTest(attempt=attempt):
                source = SCENE_REPRO
                if attempt == 2:
                    source += "\nimport gc\ndel final, cutter, base, part, notch\ngc.collect()\n"
                self.assertIn("completed", self.run_child(source))

    @unittest.skipUnless(sys.platform.startswith("linux"), "checks OS mapping ownership")
    def test_generated_loader_mapping_survives_proxy_collection(self):
        script = Path(__file__).resolve().parents[2] / "patch_wheel_metadata.py"
        spec = importlib.util.spec_from_file_location("camber_wheel_metadata", script)
        packaging = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(packaging)
        cdef = next(ast.literal_eval(node.value) for node in ast.parse(Path(loader.__file__).read_text()).body
                    if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "_CDEF" for t in node.targets))
        source = packaging.LOADER_TEMPLATE.replace("__CDEF_REPR__", repr(cdef))
        # Child has not imported camber: the generated loader is the sole owner.
        # Collecting every Python reference must not unmap the runtime library.
        self.run_child(textwrap.dedent(f'''
            import gc
            from pathlib import Path
            scope = {{"__file__": {loader.__file__!r}}}
            exec({source!r}, scope)
            path = scope["_native_lib_path"]()
            assert path in Path("/proc/self/maps").read_text()
            scope.clear()
            gc.collect()
            assert path in Path("/proc/self/maps").read_text(), "NativeAOT library was unloaded"
        '''))
