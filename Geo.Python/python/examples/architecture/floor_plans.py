"""Vector plan sheets derived from model sections and placed component parameters.

No parallel wall layout is maintained here. Door swings are conventional 90-degree
opening symbols, independent of the display pose of the model's door leaves.
"""
from html import escape
from pathlib import Path
import math
import textwrap

from camber import Frame
from examples.architecture.doors import DoorParameters


def section_geometry(part, solids, height):
    """Return native section outlines and fill triangles, in model millimetres."""
    outlines, fills = [], []
    for solid in solids:
        sketch = part.section_sketch(solid, plane=Frame((0, 0, height)))
        try:
            lines = sketch.polylines()
            if not lines:
                continue
            outlines.extend([tuple((p.x, p.y) for p in line) for line in lines])
            points, triangles = sketch.triangulate()
            fills.extend(tuple((points[i].x, points[i].y) for i in triangle)
                         for triangle in triangles)
        finally:
            # Derived drawings must not accumulate features in the source Part.
            part._unregister_sketch(sketch)
    return outlines, fills


class _Drawing:
    def __init__(self):
        self.items = []

    def path(self, points, *, close=False, css='detail'):
        if not points:
            return
        d = 'M'+' L'.join(f'{x:.2f},{y:.2f}' for x, y in points)
        self.items.append(f'<path class="{css}" d="{d}{" Z" if close else ""}"/>')

    def rect(self, x, y, w, h, *, css='detail'):
        self.path(((x, y), (x+w, y), (x+w, y+h), (x, y+h)), close=True, css=css)

    def text(self, x, y, value, *, size=200, anchor='middle', css=''):
        # The plan group flips Y so north is up. Flip text back locally.
        self.items.append(f'<text transform="translate({x},{y}) scale(1,-1)" '
                          f'font-size="{size}" text-anchor="{anchor}" class="{css}">{escape(value)}</text>')

    def dimension(self, x0, x1, y, source_y):
        self.path(((x0, y), (x1, y)), css='dimension')
        for x in (x0, x1):
            self.path(((x, source_y-100), (x, y-100)), css='dimension')
            self.path(((x-65, y-65), (x+65, y+65)), css='detail')
        self.text((x0+x1)/2, y+90, f'{x1-x0:,.0f}'.replace(',', ' '), size=180)


def _joinery(d, openings, floor, height):
    for name, p, position, angle in openings:
        x, y, z = position
        if z+p.height <= floor or z >= floor+3200:
            continue
        is_door = isinstance(p, DoorParameters)
        # High windows are an overhead reference, not a cut at a false elevation.
        css = 'detail' if z < height < z+p.height else 'overhead'
        d.items.append(f'<g transform="translate({x},{y}) rotate({angle})"><title>{escape(name)}</title>')
        if is_door:
            d.rect(0, 0, p.jamb_width, p.frame_depth)
            d.rect(p.width-p.jamb_width, 0, p.jamb_width, p.frame_depth)
            available = p.width-2*p.jamb_width-(p.leaves+1)*p.gap
            start = p.jamb_width+p.gap
            for weight, side in zip(p.leaf_weights, p.hinge_sides):
                width = available*weight/sum(p.leaf_weights)
                pivot = start if side == 'left' else start+width
                direction = 1 if side == 'left' else -1
                back = p.frame_depth-10
                d.rect(pivot-p.leaf_thickness/2, back, p.leaf_thickness, width)
                arc = [(pivot+direction*width*math.cos(t*math.pi/48),
                        back+width*math.sin(t*math.pi/48)) for t in range(25)]
                d.path(arc, css='swing')
                start += width+p.gap
        else:
            d.rect(0, 0, p.width, p.frame_depth, css=css)
            for offset in (p.frame_depth*.35, p.frame_depth*.65):
                d.path(((p.frame_width, offset), (p.width-p.frame_width, offset)), css=css)
            available = p.width-2*p.frame_width-(p.leaves-1)*p.mullion_width
            start = p.frame_width
            for weight in p.leaf_weights[:-1]:
                start += available*weight/sum(p.leaf_weights)
                d.rect(start, 0, p.mullion_width, p.frame_depth, css=css)
                start += p.mullion_width
        d.items.append('</g>')


def _stairs(d, model, floor, height):
    p = model.stair_parameters
    x, y, z = model.stair_position
    run = (p.risers-1)*p.going
    for i in range(p.risers):
        css = 'overhead' if floor == z and z+(i+1)*p.total_rise/p.risers > height else 'detail'
        d.path(((x, y+i*p.going), (x+p.width, y+i*p.going)), css=css)
    for side in (x, x+p.width):
        d.path(((side, y), (side, y+run)))
    d.path(((x+p.width/2, y+150), (x+p.width/2, y+run-150)), css='detail')
    d.path(((x+p.width/2-100, y+run-350), (x+p.width/2, y+run-150),
            (x+p.width/2+100, y+run-350)))
    d.text(x+p.width/2, y-280, 'UP', size=180)
    d.text(x+p.width/2, y-520, f'{p.risers} × {p.total_rise/p.risers:.1f} / {p.going:g}', size=160)


def write_house_plans(path, model, *, rooms):
    """Write a printable A3 sheet at 1:100; regenerate after changing the model.

    Labels identify spaces, not calculated net areas. Geometry comes from native
    sections, with symbols from the same parameters used to construct joinery.
    """
    drawings = []
    for level, floor, title in ((0, 0, 'GROUND FLOOR'), (1, 3200, 'UPPER FLOOR')):
        height = floor+1600
        outlines, fills = section_geometry(model.part, model.plan_walls, height)
        d = _Drawing()
        # Floor edges expose the actual gallery void and stair opening.
        for label, low, high in model.plan_boxes:
            if abs(high[2]-floor) < .1 and 'floor' in label:
                d.rect(low[0], low[1], high[0]-low[0], high[1]-low[1], css='floor')
            elif floor <= low[2] < height and high[2] < floor+2800:
                if any(tag in label for tag in ('worktop', 'storage')):
                    d.rect(low[0], low[1], high[0]-low[0], high[1]-low[1], css='fitting')
        for triangle in fills:
            d.path(triangle, close=True, css='wall-fill')
        for line in outlines:
            d.path(line, css='wall-edge')
        _joinery(d, model.plan_joinery, floor, height)
        _stairs(d, model, floor, height)
        for room_level, label, (x0, y0, x1, y1) in rooms:
            if room_level != level:
                continue
            # Label zones are deliberately independent of the cut geometry.
            cx, cy = (x0+x1)/2, (y0+y1)/2
            if label == 'Dining':
                cx, cy = 5800, 1200  # keep the stair and its approach legible
            lines = textwrap.wrap(label, width=max(10, int((x1-x0)/115)))
            d.items.append(f'<g><title>{escape(label)}</title>')
            for i, text in enumerate(lines):
                d.text(cx, cy-i*260, text, size=210, css='room-label')
            d.items.append('</g>')
        if level:
            d.text(2700, 2700, 'VOID OVER LIVING', size=200, css='muted')
        xs = [x for line in outlines for x, _ in line]
        if xs:
            d.dimension(min(xs), max(xs), -1750, -900)
        d.text(0, 11800, title, size=320, anchor='start')
        d.text(0, 11280, f'LEVEL +{floor/1000:.2f} m  ·  CUT +{height/1000:.2f} m', size=180, anchor='start')
        d.path(((12500, 10800), (12500, 11900)))
        d.path(((12380, 11600), (12500, 11900), (12620, 11600)))
        d.text(12500, 12100, 'N', size=200)
        drawings.append(f'<g transform="translate({3500+level*18500},15700) scale(1,-1)">'+''.join(d.items)+'</g>')
    svg = '''<svg xmlns="http://www.w3.org/2000/svg" width="420mm" height="297mm" viewBox="0 0 42000 29700">
<rect width="42000" height="29700" fill="white"/>
<style>
text{font-family:Segoe UI,Arial,sans-serif;fill:#262b30}
path{fill:none;stroke:#34383b;stroke-width:18;stroke-linejoin:round}
.wall-fill{fill:#45494c;stroke:none}.wall-edge{stroke:#292d30;stroke-width:30}
.floor{fill:#fcfcfb;stroke:#a0a5a8;stroke-width:12}
.fitting{fill:#f1f2f2;stroke:#777e83;stroke-width:15}
.overhead{stroke:#93999d;stroke-dasharray:90 60;stroke-width:13}
.swing,.dimension{stroke:#747b80;stroke-width:12}
.room-label{paint-order:stroke;stroke:white;stroke-width:40;stroke-linejoin:round}
.muted{fill:#71797e}
</style>
'''+''.join(drawings)
    svg += '''<g font-family="Segoe UI,Arial,sans-serif" fill="#262b30">
<path d="M2000 21800H40000" stroke="#34383b" stroke-width="25"/>
<text x="2000" y="22600" font-size="420">GALLERY HOUSE · FLOOR PLANS</text>
<text x="2000" y="23200" font-size="240">Design development · model-derived wall sections · dimensions in mm</text>
<text x="2000" y="23800" font-size="220">Dark: cut walls   /   Thin: joinery and fittings   /   Dashed: above cut plane</text>
<text x="2000" y="24300" font-size="220">Door arcs show opening direction. Room names are labels; net floor areas are not yet measured.</text>
<text x="2000" y="24800" font-size="220">Roof, furniture, services and structural coordination require further development.</text>
<text x="35000" y="22600" font-size="300">A-101</text>
<text x="35000" y="23200" font-size="240">1:100 @ A3</text>
<text x="35000" y="23700" font-size="200">Print at 100%</text>
<path d="M2000 26600h5000m-5000 -100v200m1000 -200v200m4000 -200v200" stroke="#34383b" stroke-width="25"/>
<text x="2000" y="27000" font-size="200">0</text><text x="3000" y="27000" font-size="200">1</text>
<text x="7000" y="27000" font-size="200">5 m</text>
</g></svg>'''
    Path(path).write_text(svg, encoding='utf-8')
