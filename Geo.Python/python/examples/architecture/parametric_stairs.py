"""GUI-editable straight stair. Run this file to open the modeler."""
from pathlib import Path
import sys
if __name__ == '__main__':
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    from camber.modeling import open
    open(__file__)
    raise SystemExit

from camber import Part
from camber.modeling_parameters import named_parameter
from examples.architecture.stairs import stair_component

# camber:begin setup
part = Part((-1000, -1000, -1000), (6000, 15000, 8000), tolerance=.025)
# camber:end setup

# camber:begin floor_height
floor_height = named_parameter(value=2800, unit='mm')
# camber:end floor_height

# camber:begin stairs
stairs = stair_component(part=part, name='stairs', total_rise=floor_height)
# camber:end stairs

displayed = stairs
