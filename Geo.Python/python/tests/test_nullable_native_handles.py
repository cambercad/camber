"""Generated proxy ownership: null has no owner, valid handles have one."""
import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile
import textwrap
import unittest

PACKAGING = Path(__file__).resolve().parents[2] / 'patch_wheel_metadata.py'


def normalizer():
    spec = importlib.util.spec_from_file_location('camber_packaging_handles', PACKAGING)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module._fix_null_objects


FIXTURE = '''
class CString:
    pass
class Proxy:
    @classmethod
    def _dotwrap_from_ptr(cls, ptr: int):
        instance = object.__new__(cls)
        instance._dotwrap_ptr = _dotwrap_ffi.cast("void *", ptr)
        return instance
    def __del__(self):
        _dotwrap_lib.proxy___dotwrapDestroy(self._dotwrap_ptr)
'''


class NullableNativeHandleTests(unittest.TestCase):
    def test_null_partial_and_live_owners_and_idempotence(self):
        import gc
        from cffi import FFI
        ffi = FFI()
        calls = []
        class Library:
            proxy___dotwrapDestroy = staticmethod(lambda pointer: calls.append(pointer))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'main.py'
            path.write_text(FIXTURE)
            fix = normalizer()
            fix(path)
            first = path.read_text()
            fix(path)
            self.assertEqual(first, path.read_text())
            scope = {'_dotwrap_ffi': ffi, '_dotwrap_lib': Library}
            exec(first, scope)
            proxy = scope['Proxy']
            self.assertIsNone(proxy._dotwrap_from_ptr(0))
            self.assertIsNone(proxy._dotwrap_from_ptr(ffi.NULL))
            partial = object.__new__(proxy)
            partial.__del__()
            partial._dotwrap_ptr = ffi.NULL
            partial.__del__()
            live = proxy._dotwrap_from_ptr(123)
            live.__del__()
            live.__del__()
            del live, partial
            gc.collect()
            self.assertEqual(len(calls), 1)
            self.assertEqual(calls[0], ffi.cast('void *', 123))

    def test_unexpected_generator_shape_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'main.py'
            path.write_text(FIXTURE.replace('object.__new__(cls)', 'cls()'))
            with self.assertRaisesRegex(RuntimeError, 'factory implementation'):
                normalizer()(path)

    def test_actual_generated_null_returns_and_failed_constructors_exit_cleanly(self):
        source = textwrap.dedent(f'''
            import gc, importlib.util, tempfile
            from pathlib import Path
            import _camber_native.__dotwrap_generated.main as generated
            spec = importlib.util.spec_from_file_location('packaging', {str(PACKAGING)!r})
            packaging = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(packaging)
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / 'main.py'
                path.write_text(Path(generated.__file__).read_text())
                packaging._fix_null_objects(path)
                scope = {{'__name__': '_camber_native.__dotwrap_generated.handle_probe',
                         '__package__': '_camber_native.__dotwrap_generated'}}
                exec(compile(path.read_text(), str(path), 'exec'), scope)
                classes = [value for value in scope.values() if isinstance(value, type)
                           and '_dotwrap_from_ptr' in value.__dict__]
                assert len(classes) > 10
                for cls in classes:
                    assert cls._dotwrap_from_ptr(0) is None
                for index in range(10):
                    part = scope['NativePart'](-10, -10, -10, 10, 10, 10, .01)
                    assert part.get_mesh_from_name('absent') is None
                    empty = scope['NativeSolidList']()
                    assert part.batch_union(empty) is None
                    solid = part.create_cuboid_aabb(0, 0, 0, 1, 1, 1, 'cube_' + str(index))
                    assert abs(solid.signed_volume() - 1) < .001
                    frame = part.get_plane_frame('xy')
                    del frame, solid, empty, part
                    gc.collect()
                try:
                    scope['NativePart'](-1, -1, -1, 1, 1, 1, -1)
                except Exception:
                    pass
                else:
                    raise AssertionError('negative tessellation tolerance should reject')
                gc.collect()
            print('completed', flush=True)
        ''')
        result = subprocess.run([sys.executable, '-I', '-X', 'faulthandler', '-c', source],
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('completed', result.stdout)


if __name__ == '__main__':
    unittest.main()
