"""9-speed cassette on a 700c rear wheel (hub, rim, tire, spokes)."""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from bike_cassette import build_cassette, new_part
from bike_wheel import add_wheel

# Construction / tessellation tolerance for every solid in this assembly.
TOLERANCE = 0.1


def build_rear_wheel(tolerance=TOLERANCE):
    part = new_part(extent=360.0, tolerance=tolerance)
    asm = build_cassette(part, max_deviation=tolerance)
    add_wheel(asm, part, max_deviation=tolerance)
    return asm


if __name__ == "__main__":
    wheel = build_rear_wheel()
    print(wheel)
    wheel.show(title="camber 9-speed cassette on 700c rear wheel")
