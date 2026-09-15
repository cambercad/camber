import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from camber.api import set_progress_log

set_progress_log(False)
