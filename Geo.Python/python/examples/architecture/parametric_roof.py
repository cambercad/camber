"""GUI-editable roof, covering, gutter and downpipe. Run this file to edit."""
from pathlib import Path
import sys
if __name__ == '__main__':
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    from camber.modeling import open
    open(__file__)
    raise SystemExit

from camber import Part
from camber.modeling_parameters import named_parameter
from examples.architecture.roofs import roof_component
from examples.architecture.roof_coverings import roof_covering_component
from examples.architecture.rainwater import gutter_component, downpipe_component

# camber:begin setup
part = Part((-20000, -20000, -2000), (30000, 30000, 20000), tolerance=.025)
# camber:end setup

# camber:begin width
width = named_parameter(value=6000, unit='mm')
# camber:end width

# camber:begin length
length = named_parameter(value=8000, unit='mm')
# camber:end length

# camber:begin eaves
eaves = named_parameter(value=3000, unit='mm')
# camber:end eaves

# camber:begin roof
roof = roof_component(part=part, name='roof', width=width, length=length, eave_height=eaves, overhang=0)
# camber:end roof

# camber:begin covering
covering = roof_covering_component(part=part, name='covering', width=width, length=length, eave_height=eaves, overhang=0)
# camber:end covering

# camber:begin gutter
gutter = gutter_component(part=part, name='gutter', length=length, elevation=eaves-30)
# camber:end gutter

# camber:begin outlet_height
outlet_height = named_parameter(value=eaves-30-30*(length-100)/length-75*(1+(30/length)**2)**0.5-60, unit='mm', description='Gutter outlet bottom centre')
# camber:end outlet_height

# camber:begin downpipe
downpipe = downpipe_component(part=part, name='downpipe', points=((0, length-100, outlet_height), (0, length-100, outlet_height-400), (-150, length-100, outlet_height-600), (-150, length-100, 100)))
# camber:end downpipe

# camber:begin assembly
assembly = part.assembly(name='roof_system')
# camber:end assembly

# camber:begin roof_instance
roof_instance = assembly.add_subassembly(roof)
# camber:end roof_instance

# camber:begin roof_fixed
assembly.fix(roof_instance)
# camber:end roof_fixed

# camber:begin covering_instance
covering_instance = assembly.add_subassembly(covering)
# camber:end covering_instance

# camber:begin covering_fixed
assembly.fix(covering_instance)
# camber:end covering_fixed

# camber:begin gutter_instance
gutter_instance = assembly.add_subassembly(gutter, position=(-40, 0, 0))
# camber:end gutter_instance

# camber:begin gutter_fixed
assembly.fix(gutter_instance)
# camber:end gutter_fixed

# camber:begin downpipe_instance
downpipe_instance = assembly.add_subassembly(downpipe, position=(-40, 0, 0))
# camber:end downpipe_instance

# camber:begin downpipe_fixed
assembly.fix(downpipe_instance)
# camber:end downpipe_fixed

displayed = assembly
