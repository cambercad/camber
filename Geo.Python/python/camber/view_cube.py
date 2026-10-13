"""A batched orientation control with face, edge and corner hit testing."""
from itertools import product


_VERTICES = tuple(product((-1, 1), repeat=3))
_FACES = tuple((axis, sign, tuple(i for i, vertex in enumerate(_VERTICES) if vertex[axis] == sign))
               for axis in range(3) for sign in (-1, 1))
_EDGES = tuple((a, b) for a in range(8) for b in range(a+1, 8)
               if sum(x != y for x, y in zip(_VERTICES[a], _VERTICES[b])) == 1)
_LABELS = (("Left", "Right"), ("Front", "Back"), ("Bottom", "Top"))
_EDGE_COLORS = ((212, 80, 82), (55, 151, 86), (54, 131, 217))


def _dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def _inside(point, polygon):
    crosses = [(b[0]-a[0])*(point[1]-a[1])-(b[1]-a[1])*(point[0]-a[0])
               for a, b in zip(polygon, polygon[1:]+polygon[:1])]
    return all(c >= -1e-7 for c in crosses) or all(c <= 1e-7 for c in crosses)


class ViewCube:
    """Draw with a camera's (look, up, right) frame; return a clicked direction.

    Coordinates and hit targets use logical pixels, top-down. The control uses
    the UI's existing geometry and text batches, with no separate GL resources.
    """

    size = 124

    def layout(self, frame, rect):
        look, up, right = frame
        x, y, width, height = rect
        scale = min(width, height)*.24
        def project(vertex):
            return x+width/2+scale*_dot(vertex, right), y+height/2-scale*_dot(vertex, up)
        self.points = tuple(project(vertex) for vertex in _VERTICES)
        self.faces = []
        self.cells = []
        visible = set()
        for axis, sign, indices in _FACES:
            if -look[axis]*sign <= 1e-8:
                continue
            # Binary vertex order traverses a square as 0,1,3,2.
            indices = tuple(indices[i] for i in (0, 1, 3, 2))
            direction = tuple(sign if i == axis else 0 for i in range(3))
            self.faces.append((direction, indices))
            visible.update(indices)
            others = [i for i in range(3) if i != axis]
            breaks = (-1, -.64, .64, 1)
            for row in range(3):
                for column in range(3):
                    cell_direction = list(direction)
                    cell_direction[others[0]], cell_direction[others[1]] = row-1, column-1
                    polygon = []
                    for u,v in ((breaks[row],breaks[column]), (breaks[row+1],breaks[column]),
                                (breaks[row+1],breaks[column+1]), (breaks[row],breaks[column+1])):
                        vertex = list(direction)
                        vertex[others[0]],vertex[others[1]] = u,v
                        polygon.append(project(vertex))
                    self.cells.append((tuple(cell_direction), direction, tuple(polygon)))
        self.edges = tuple((a, b) for a, b in _EDGES
                           if any(a in indices and b in indices for _, indices in self.faces))
        self.vertices = tuple(sorted(visible))

    def hit(self, point):
        if not hasattr(self, "points"):
            return None
        return next((direction for direction, _, polygon in self.cells if _inside(point, polygon)), None)

    @staticmethod
    def label(direction):
        return " · ".join(_LABELS[axis][int(sign > 0)] for axis, sign in enumerate(direction) if sign)

    def draw(self, ui, frame, rect):
        self.layout(frame, rect)
        ui.SetNextWindowPos(rect[:2], ui.ImGuiCond_Always)
        ui.SetNextWindowSize(rect[2:], ui.ImGuiCond_Always)
        ui.PushStyleVar(ui.ImGuiStyleVar_WindowPadding, (0, 0))
        ui.Begin("View cube", ui.ImGuiWindowFlags_NoDecoration | ui.ImGuiWindowFlags_NoMove |
                 ui.ImGuiWindowFlags_NoBackground | ui.ImGuiWindowFlags_NoScrollWithMouse)
        clicked = ui.InvisibleButton("View direction", rect[2:], tab_stop=False)
        target = self.hit(ui.GetIO().MousePos) if ui.IsItemHovered() else None
        dl = ui.GetWindowDrawList()
        text, accent = (ui.theme[key] for key in ("text", "accent"))
        base = ui.theme["field"]
        if ui.theme_name == "dark":
            base = tuple(round(.78*c+.22*t) for c,t in zip(ui.theme["control"][:3],text[:3]))+(255,)

        def color(normal, material=1, rgb=base[:3]):
            shade = .95 if normal[2] else .86 if normal[0] else .92
            return tuple(round(c*shade*material) for c in rgb)+(255,)

        # One continuous face fill prevents seams at the internal GPU triangles.
        for direction, indices in self.faces:
            polygon = tuple(self.points[i] for i in indices)
            dl.AddQuadFilled(*polygon, color(direction))
        for direction, normal, polygon in self.cells:
            count = sum(bool(v) for v in direction)
            fill = color(normal, rgb=_EDGE_COLORS[direction.index(0)]) if count == 2 else color(normal, .97 if count == 3 else 1)
            if target == direction:
                fill = (tuple(round(.8*c+.2*255) for c in fill[:3]) if count == 2 else
                        tuple(round(.65*c+.35*a) for c,a in zip(fill[:3],accent[:3])))+(255,)
            dl.AddQuadFilled(*polygon, fill)
        for direction, indices in self.faces:
            polygon = tuple(self.points[i] for i in indices)
            if max(p[0] for p in polygon)-min(p[0] for p in polygon) >= 34 and \
               max(p[1] for p in polygon)-min(p[1] for p in polygon) >= 18:
                center = tuple(sum(p[i] for p in polygon)/4 for i in (0, 1))
                dl.AddTextAligned(center, text, self.label(direction).upper(), "center", size=9)
        if target is not None:
            ui.SetTooltip("View from " + self.label(target).lower())
        ui.End()
        ui.PopStyleVar()
        return target if clicked else None
