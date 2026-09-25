"""CadQuery gallery: configurable embossed Braille label."""

def build():
    """Build and return this CadQuery gallery sample."""
    from collections import namedtuple

    from camber.cqcompat import Workplane


    # CadQuery example input: the raised Braille reads "free CAD".
    text_lines = ["⠠ ⠋ ⠗ ⠑ ⠑ ⠠ ⠉ ⠠ ⠁ ⠠ ⠙"]
    horizontal_interdot = 2.5
    vertical_interdot = 2.5
    horizontal_intercell = 6
    vertical_interline = 10
    dot_height = 0.5
    dot_diameter = 1.3
    base_thickness = 1.5

    BrailleCellGeometry = namedtuple(
        "BrailleCellGeometry",
        ("horizontal_interdot", "vertical_interdot", "intercell", "interline",
         "dot_height", "dot_diameter"),
    )


    class Point(object):
        def __init__(self, x, y):
            self.x = x
            self.y = y

        def __add__(self, other):
            return Point(self.x + other.x, self.y + other.y)

        def __len__(self):
            return 2

        def __getitem__(self, index):
            return (self.x, self.y)[index]

        def __str__(self):
            return "({}, {})".format(self.x, self.y)


    def brailleToPoints(text, cell_geometry):
        """Decode Unicode Braille bits into the configured six/eight-dot positions."""
        masks = (0b00000001, 0b00000010, 0b00000100, 0b00001000,
                 0b00010000, 0b00100000, 0b01000000, 0b10000000)
        w = cell_geometry.horizontal_interdot
        h = cell_geometry.vertical_interdot
        pos = (Point(0, 2 * h), Point(0, h), Point(0, 0), Point(w, 2 * h),
               Point(w, h), Point(w, 0), Point(0, -h), Point(w, -h))
        blank = "⠀"
        points = []
        character_origin = 0
        for character in text:
            delta_to_blank = ord(character) - ord(blank)
            for mask, point in zip(masks, pos):
                if mask & delta_to_blank:
                    points.append(point + Point(character_origin, 0))
            character_origin += cell_geometry.intercell
        return points


    def get_plate_height(lines, cell_geometry):
        return (2 * cell_geometry.vertical_interdot
                + 2 * cell_geometry.vertical_interdot
                + (len(lines) - 1) * cell_geometry.interline)


    def get_plate_width(lines, cell_geometry):
        max_len = max(len(line) for line in lines)
        return (2 * cell_geometry.horizontal_interdot
                + cell_geometry.horizontal_interdot
                + (max_len - 1) * cell_geometry.intercell)


    def get_cylinder_radius(cell_geometry):
        """Radius producing the requested spherical-cap dot height and diameter."""
        h = cell_geometry.dot_height
        r = cell_geometry.dot_diameter / 2
        return (r ** 2 + h ** 2) / (2 * h)


    def get_base_plate_thickness(plate_thickness, cell_geometry):
        return plate_thickness + get_cylinder_radius(cell_geometry) - cell_geometry.dot_height


    def make_base(lines, cell_geometry, plate_thickness):
        return Workplane("XY").box(
            get_plate_width(lines, cell_geometry),
            get_plate_height(lines, cell_geometry),
            get_base_plate_thickness(plate_thickness, cell_geometry),
            centered=False,
        )


    def make_embossed_plate(lines, cell_geometry):
        """Form rounded dot caps, then expose only their portion above the label plate."""
        base = make_base(lines, cell_geometry, base_thickness)
        dot_pos = []
        base_width = get_plate_width(lines, cell_geometry)
        base_height = get_plate_height(lines, cell_geometry)
        y = base_height - 3 * cell_geometry.vertical_interdot
        line_start_pos = Point(cell_geometry.horizontal_interdot, y)
        for line in lines:
            dots = brailleToPoints(line, cell_geometry)
            dot_pos += [point + line_start_pos for point in dots]
            line_start_pos += Point(0, -cell_geometry.interline)

        radius = get_cylinder_radius(cell_geometry)
        dots = (
            base.faces(">Z").vertices("<XY").workplane()
            .pushPoints(dot_pos).circle(radius).extrude(radius)
        )
        dots = dots.faces(">Z").edges().fillet(radius - 0.001)
        hiding_box = Workplane("XY").box(
            base_width, base_height,
            get_base_plate_thickness(base_thickness, cell_geometry),
            centered=False,
        )
        return hiding_box.union(dots)


    _cell_geometry = BrailleCellGeometry(
        horizontal_interdot, vertical_interdot, horizontal_intercell,
        vertical_interline, dot_height, dot_diameter,
    )
    if base_thickness < get_cylinder_radius(_cell_geometry):
        raise ValueError("Base thickness should be at least {}".format(dot_height))

    result = make_embossed_plate(text_lines, _cell_geometry)
    assert result.val().is_watertight()

    return result


if __name__ == "__main__":
    result = build()
    result.show(title='CadQuery gallery 37 — Braille Example')
