"""Boundary regression for DotWrap empty-string results, without new packages."""
import ast
import importlib.util
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from cffi import FFI
import _camber_native.__dotwrap_generated.main as native


class NativeStringTests(unittest.TestCase):
    def test_generated_decoder_accepts_null_empty_and_utf8(self):
        script = Path(__file__).resolve().parents[2] / "patch_wheel_metadata.py"
        spec = importlib.util.spec_from_file_location("camber_wheel_metadata", script)
        packaging = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(packaging)
        source = Path(native.__file__).read_text()
        with tempfile.TemporaryDirectory() as tmp:
            generated = Path(tmp) / "main.py"
            generated.write_text(source)
            packaging._fix_null_cstrings(generated)
            once = generated.read_text()
            packaging._fix_null_cstrings(generated)
            self.assertEqual(once, generated.read_text())
        # Exercise the actual generated decoder, with memory owned by cffi here.
        tree = ast.parse(once)
        decoder = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "CString")
        ffi = FFI()
        scope = {"_dotwrap_ffi": ffi,
                 "_dotwrap_lib": SimpleNamespace(DotWrap_BuiltIn_CString_Free=lambda pointer: None)}
        exec(compile(ast.Module(body=[decoder], type_ignores=[]), "generated CString", "exec"), scope)
        for pointer, expected in ((ffi.NULL, ""), (ffi.new("char[]", b""), ""),
                                  (ffi.new("char[]", "Größe 19".encode()), "Größe 19")):
            self.assertEqual(expected, str(scope["CString"](pointer)))
