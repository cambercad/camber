"""Profile the bike frame with the installed wheel, without a viewer."""

import argparse
import cProfile
import importlib.metadata
from pathlib import Path
import pstats
import sys
import time

sys.path.insert(
    0,
    str(
        Path(__file__).resolve().parents[2]
        / "Geo.Python/python/examples/engineering_models"
    ),
)
from camber import Part, set_progress_log
from racing_bike import build_frame_body


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("/tmp/camber-frame.prof"))
    args = parser.parse_args()
    set_progress_log(False)
    part = Part((-420, -300, -50), (1420, 300, 1120), tolerance=0.2)
    profile = cProfile.Profile()
    started = time.perf_counter()
    frame = profile.runcall(build_frame_body, part)
    print(
        f"cambercad {importlib.metadata.version('cambercad')}: "
        f"frame construction {time.perf_counter() - started:.2f}s"
    )
    profile.dump_stats(str(args.output))
    pstats.Stats(profile).strip_dirs().sort_stats("cumtime").print_stats(30)
    watertight = frame.is_watertight()
    print(f"Watertight: {watertight}; volume: {frame.volume():.9f} mm³")
    assert watertight


if __name__ == "__main__":
    main()
