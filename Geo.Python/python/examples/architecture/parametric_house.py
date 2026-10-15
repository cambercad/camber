"""Open the family-house study in the experimental modeler."""
from pathlib import Path
import sys

if __name__ == '__main__':
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    from camber.modeling import open
    open(__file__)
    raise SystemExit

from camber import Part
from examples.architecture.gallery_house import house_component

# camber:begin setup
part = Part((-4000, -5000, -1200), (17000, 15000, 13000), tolerance=.025)
# camber:end setup

# camber:begin house
house = house_component(part=part, name='house', cutaway=False, rainwater=True, terrace=True)
# camber:end house

displayed = house
