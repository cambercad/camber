"""Build and round the five regular convex polyhedra from sewn face meshes."""

import itertools
import math

from camber import Part


def _ordered_face(points, outward):
    center = tuple(sum(point[i] for point in points) / len(points) for i in range(3))
    length = math.sqrt(sum(component * component for component in outward))
    normal = tuple(component / length for component in outward)
    first = tuple(points[0][i] - center[i] for i in range(3))
    first_length = math.sqrt(sum(component * component for component in first))
    u = tuple(component / first_length for component in first)
    v = (normal[1] * u[2] - normal[2] * u[1],
         normal[2] * u[0] - normal[0] * u[2],
         normal[0] * u[1] - normal[1] * u[0])

    def angle(point):
        delta = tuple(point[i] - center[i] for i in range(3))
        return math.atan2(sum(delta[i] * v[i] for i in range(3)),
                          sum(delta[i] * u[i] for i in range(3)))

    return sorted(points, key=angle)


def _triangle_faces(vertices):
    pairs = list(itertools.combinations(range(len(vertices)), 2))
    edge_lengths = [sum((vertices[a][axis] - vertices[b][axis]) ** 2
                        for axis in range(3)) for a, b in pairs]
    shortest = min(edge_lengths)
    tolerance = shortest * 1e-9
    faces = []
    for indices in itertools.combinations(range(len(vertices)), 3):
        points = [vertices[index] for index in indices]
        lengths = [sum((points[a][axis] - points[b][axis]) ** 2
                       for axis in range(3))
                   for a, b in ((0, 1), (1, 2), (2, 0))]
        if any(abs(length - shortest) > tolerance for length in lengths):
            continue
        a, b, c = points
        ab = tuple(b[i] - a[i] for i in range(3))
        ac = tuple(c[i] - a[i] for i in range(3))
        normal = (ab[1] * ac[2] - ab[2] * ac[1],
                  ab[2] * ac[0] - ab[0] * ac[2],
                  ab[0] * ac[1] - ab[1] * ac[0])
        center = tuple(sum(point[i] for point in points) / 3 for i in range(3))
        if sum(normal[i] * center[i] for i in range(3)) < 0:
            normal = tuple(-component for component in normal)
        faces.append(_ordered_face(points, normal))
    return faces


def _platonic_faces():
    tetrahedron = [(1, 1, 1), (1, -1, -1), (-1, 1, -1), (-1, -1, 1)]
    cube = list(itertools.product((-1, 1), repeat=3))
    octahedron = [tuple(sign if axis == i else 0 for axis in range(3))
                  for i in range(3) for sign in (-1, 1)]
    phi = (1 + math.sqrt(5)) / 2
    icosahedron = (
        [(0, a, b * phi) for a in (-1, 1) for b in (-1, 1)]
        + [(a, b * phi, 0) for a in (-1, 1) for b in (-1, 1)]
        + [(a * phi, 0, b) for a in (-1, 1) for b in (-1, 1)]
    )

    cube_faces = []
    for axis in range(3):
        for sign in (-1, 1):
            points = [point for point in cube if point[axis] == sign]
            outward = tuple(sign if i == axis else 0 for i in range(3))
            cube_faces.append(_ordered_face(points, outward))

    icosa_faces = _triangle_faces(icosahedron)
    dodecahedron_faces = []
    for vertex in icosahedron:
        points = [tuple(sum(point[i] for point in face) / 3 for i in range(3))
                  for face in icosa_faces if vertex in face]
        dodecahedron_faces.append(_ordered_face(points, vertex))

    return (
        ("tetrahedron", _triangle_faces(tetrahedron), 6),
        ("cube", cube_faces, 12),
        ("octahedron", _triangle_faces(octahedron), 12),
        ("dodecahedron", dodecahedron_faces, 30),
        ("icosahedron", icosa_faces, 30),
    )


def _rounded_body(name, faces, edge_count, fillet_radius=1.5):
    points = [point for face in faces for point in face]
    circumradius = max(math.sqrt(sum(component * component for component in point))
                       for point in points)
    scale = 8.0 / circumradius
    part = Part((-10, -10, -10), (10, 10, 10), tolerance=0.001)
    surfaces = []
    for index, face in enumerate(faces):
        vertices = [tuple(component * scale for component in point) for point in face]
        triangles = [(0, point, point + 1) for point in range(1, len(vertices) - 1)]
        surfaces.append(part.solid_from_mesh(vertices, triangles,
                                             name=f"{name}_face_{index + 1}"))

    body = part.sew(surfaces, make_solid=True, name=name)
    assert body.is_watertight()
    assert len(body.curve_names) == edge_count

    rounded = body.fillet(
        body.curve_names, fillet_radius, name=f"rounded_{name}", max_deviation=0.001)

    assert rounded.is_watertight()
    assert rounded.volume() < body.volume()
    return rounded


def build():
    viewer_part = Part((-55, -12, -12), (55, 12, 12), tolerance=0.01)
    assembly = viewer_part.assembly(name="rounded_platonic_solids")
    positions = (-40, -20, 0, 20, 40)
    for (name, faces, edge_count), x in zip(_platonic_faces(), positions):
        assembly.add_part(_rounded_body(name, faces, edge_count), position=(x, 0, 0))
    return assembly


if __name__ == "__main__":
    build().show(title="All five Platonic solids with every edge rounded")
