"""The combined gallery must not share same-named helper modules across folders."""
import runpy
import tempfile
import unittest
from pathlib import Path


class GalleryImportTests(unittest.TestCase):
    def test_each_gallery_uses_its_own_common_helper(self):
        runner = Path(__file__).resolve().parents[1] / "examples" / "show_all_examples.py"
        collect = runpy.run_path(str(runner))["_collect_gallery"]
        failures = []
        with tempfile.TemporaryDirectory() as temporary:
            results = []
            for label in ("first", "second"):
                folder = Path(temporary) / label
                folder.mkdir()
                (folder / "_common.py").write_text("value = %r\n" % label, encoding="utf-8")
                (folder / "01_sample.py").write_text(
                    "from _common import value\n"
                    "class Result:\n"
                    "    def display_scene(self): return value\n"
                    "def build(): return Result()\n", encoding="utf-8")
                results.extend(collect(folder, label, failures))
        self.assertEqual([], failures)
        self.assertEqual(["first", "second"], [scene for _, scene in results])


if __name__ == "__main__":
    unittest.main()
