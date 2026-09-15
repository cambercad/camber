"""Small Python vectors. Tuples and lists also work anywhere a vec2/vec3 is accepted."""

class vec2(object):
    """2D vector ``(x, y)``. Accepts ``vec2(x, y)``, ``vec2((x, y))``, or another vec2.

    Arithmetic with tuples works. ``*`` is scalar or per-component.
    """
    __slots__ = ("x", "y")

    def __init__(self, x, y=None):
        if y is None:
            if isinstance(x, vec2):
                self.x = x.x
                self.y = x.y
                return
            self.x = float(x[0])
            self.y = float(x[1])
            return
        self.x = float(x)
        self.y = float(y)

    def __iter__(self):
        yield self.x
        yield self.y

    def __len__(self):
        return 2

    def __getitem__(self, i):
        if i == 0:
            return self.x
        if i == 1:
            return self.y
        raise IndexError(i)

    def __add__(self, other):
        """Component-wise add. Tuples work."""
        ox, oy = _xy(other)
        return vec2(self.x + ox, self.y + oy)

    def __radd__(self, other):
        return self.__add__(other)

    def __sub__(self, other):
        """Component-wise subtract."""
        ox, oy = _xy(other)
        return vec2(self.x - ox, self.y - oy)

    def __rsub__(self, other):
        ox, oy = _xy(other)
        return vec2(ox - self.x, oy - self.y)

    def __mul__(self, other):
        """Scalar or component-wise multiply."""
        if isinstance(other, (int, float)):
            return vec2(self.x * other, self.y * other)
        ox, oy = _xy(other)
        return vec2(self.x * ox, self.y * oy)

    def __rmul__(self, other):
        return self.__mul__(other)

    def __neg__(self):
        """Negate both components."""
        return vec2(-self.x, -self.y)

    def __eq__(self, other):
        try:
            ox, oy = _xy(other)
        except (TypeError, IndexError, ValueError):
            return NotImplemented
        return self.x == ox and self.y == oy

    def __ne__(self, other):
        eq = self.__eq__(other)
        if eq is NotImplemented:
            return NotImplemented
        return not eq

    def tolist(self):
        """Return ``[x, y]``."""
        return [self.x, self.y]

    def copy(self):
        """Return a new vec2 with the same components."""
        return vec2(self.x, self.y)

    def dot(self, other):
        """Dot product with a vec2 or 2-tuple."""
        ox, oy = _xy(other)
        return self.x * ox + self.y * oy

    def norm(self):
        """Euclidean length."""
        return (self.x * self.x + self.y * self.y) ** 0.5

    def __repr__(self):
        return "vec2({0}, {1})".format(self.x, self.y)


class vec3(object):
    """3D vector ``(x, y, z)``. ``vec3(s)`` fills all axes; tuples and vec3 copy.

    Arithmetic with tuples works. ``*`` is scalar or per-component. ``/`` is scalar.
    """
    __slots__ = ("x", "y", "z")

    def __init__(self, x, y=None, z=None):
        if y is None and z is None:
            if isinstance(x, vec3):
                self.x, self.y, self.z = x.x, x.y, x.z
                return
            if isinstance(x, (int, float)):
                v = float(x)
                self.x = self.y = self.z = v
                return
            self.x = float(x[0])
            self.y = float(x[1])
            self.z = float(x[2])
            return
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    def __iter__(self):
        yield self.x
        yield self.y
        yield self.z

    def __len__(self):
        return 3

    def __getitem__(self, i):
        if i == 0:
            return self.x
        if i == 1:
            return self.y
        if i == 2:
            return self.z
        raise IndexError(i)

    def __add__(self, other):
        """Component-wise add. Tuples work."""
        ox, oy, oz = _xyz(other)
        return vec3(self.x + ox, self.y + oy, self.z + oz)

    def __radd__(self, other):
        return self.__add__(other)

    def __sub__(self, other):
        """Component-wise subtract."""
        ox, oy, oz = _xyz(other)
        return vec3(self.x - ox, self.y - oy, self.z - oz)

    def __rsub__(self, other):
        ox, oy, oz = _xyz(other)
        return vec3(ox - self.x, oy - self.y, oz - self.z)

    def __mul__(self, other):
        """Scalar or component-wise multiply."""
        if isinstance(other, (int, float)):
            return vec3(self.x * other, self.y * other, self.z * other)
        ox, oy, oz = _xyz(other)
        return vec3(self.x * ox, self.y * oy, self.z * oz)

    def __rmul__(self, other):
        return self.__mul__(other)

    def __truediv__(self, other):
        """Divide by a scalar."""
        scale = float(other)
        return vec3(self.x / scale, self.y / scale, self.z / scale)

    def __neg__(self):
        """Negate all components."""
        return vec3(-self.x, -self.y, -self.z)

    def dot(self, other):
        """Dot product with a vec3 or 3-tuple."""
        ox, oy, oz = _xyz(other)
        return self.x * ox + self.y * oy + self.z * oz

    def cross(self, other):
        """Right-handed cross product. Returns vec3."""
        ox, oy, oz = _xyz(other)
        return vec3(
            self.y * oz - self.z * oy,
            self.z * ox - self.x * oz,
            self.x * oy - self.y * ox,
        )

    def normalized(self):
        """Unit vector, or (0,0,0) if length is 0."""
        n = self.norm()
        if n == 0.0:
            return vec3(0, 0, 0)
        return vec3(self.x / n, self.y / n, self.z / n)

    def __eq__(self, other):
        try:
            ox, oy, oz = _xyz(other)
        except (TypeError, IndexError, ValueError):
            return NotImplemented
        return self.x == ox and self.y == oy and self.z == oz

    def __ne__(self, other):
        eq = self.__eq__(other)
        if eq is NotImplemented:
            return NotImplemented
        return not eq

    def tolist(self):
        """Return ``[x, y, z]``."""
        return [self.x, self.y, self.z]

    def copy(self):
        """Return a new vec3 with the same components."""
        return vec3(self.x, self.y, self.z)

    def norm(self):
        """Euclidean length."""
        return (self.x * self.x + self.y * self.y + self.z * self.z) ** 0.5

    def __repr__(self):
        return "vec3({0}, {1}, {2})".format(self.x, self.y, self.z)


def _xy(p):
    if isinstance(p, vec2):
        return p.x, p.y
    if isinstance(p, (int, float)):
        v = float(p)
        return v, v
    return float(p[0]), float(p[1])


def _xyz(p):
    if isinstance(p, vec3):
        return p.x, p.y, p.z
    if isinstance(p, (int, float)):
        v = float(p)
        return v, v, v
    return float(p[0]), float(p[1]), float(p[2])
