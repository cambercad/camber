"""Editable window/door document. Run this file to open the modeling workspace."""
from pathlib import Path
import sys
if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    from camber.modeling import open
    open(__file__)
    raise SystemExit

from camber import Part
from camber.modeling_parameters import named_parameter
from examples.architecture.windows import window_component
from examples.architecture.doors import door_component

# camber:begin setup
part = Part((-4000, -4000, -500), (6000, 6000, 4000), tolerance=.025)
# camber:end setup

# camber:begin window_width
window_width = named_parameter(value=1800, unit="mm", description="Window outside frame width")
# camber:end window_width

# camber:begin window_height
window_height = named_parameter(value=1400, unit="mm")
# camber:end window_height

# camber:begin window
window = window_component(part=part, name="living_room", width=window_width, height=window_height, leaves=3)
# camber:end window

# camber:begin door
door = door_component(part=part, name="entrance", width=1000, height=2150, style="panel")
# camber:end door

# camber:begin openings
openings = part.assembly(name="openings")
# camber:end openings

# camber:begin window_instance
window_instance = openings.add_subassembly(window, position=(2000, 0, 900))
# camber:end window_instance

# camber:begin window_fixed
openings.fix(window_instance)
# camber:end window_fixed

# camber:begin door_instance
door_instance = openings.add_subassembly(door)
# camber:end door_instance

# camber:begin door_fixed
openings.fix(door_instance)
# camber:end door_fixed

displayed = openings

