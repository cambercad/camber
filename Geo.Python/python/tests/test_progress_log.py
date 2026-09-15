import io
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from camber.api import _require, progress_log_enabled, set_progress_log


class ProgressLogTests(unittest.TestCase):
    def setUp(self):
        self._previous = progress_log_enabled()

    def tearDown(self):
        set_progress_log(self._previous)

    def test_require_prints_the_native_name(self):
        set_progress_log(True)
        ran = []

        class Dummy(object):
            def extrude(self):
                ran.append(1)
                return 7

        buf = io.StringIO()
        old = sys.stdout
        sys.stdout = buf
        try:
            result = _require(Dummy(), "extrude")()
        finally:
            sys.stdout = old
        self.assertEqual(7, result)
        self.assertEqual([1], ran)
        self.assertEqual("camber: extrude\n", buf.getvalue())

    def test_opt_out_is_silent(self):
        set_progress_log(False)

        class Dummy(object):
            def boolean(self):
                return None

        buf = io.StringIO()
        old = sys.stdout
        sys.stdout = buf
        try:
            _require(Dummy(), "boolean")()
        finally:
            sys.stdout = old
        self.assertEqual("", buf.getvalue())


if __name__ == "__main__":
    unittest.main()
