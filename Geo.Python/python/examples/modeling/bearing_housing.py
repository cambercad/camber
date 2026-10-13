"""Source-backed modeling example. Open this file with camber.modeling.open()."""

from camber import Part, Frame, Curve3D, LoftOptions


def build_model():
    # camber:begin setup
    part = Part((-45, -35, -5), (45, 35, 45), tolerance=0.01)
    # camber:end setup

    # camber:begin base_profile
    base_profile = part.sketch(plane='xy', name='base_profile', origin_name=None, frame=Frame(), constrained=True)
    base_profile.add_rectangle((-34, -27), (34, 27))
    # camber:end base_profile

    # camber:begin base_pad
    base_pad = base_profile.extrude(height=8, name='base_pad', both_sides=False, max_deviation=-1, twist=0, taper_angle=0)
    # camber:end base_pad

    # camber:begin boss_profile
    boss_profile = part.sketch(plane='xy', name='boss_profile', origin_name=None, frame=Frame(origin=(0, 0, 8)), constrained=True)
    boss_profile.add_circle((0, 0), 19)
    # camber:end boss_profile

    # camber:begin boss_pad
    boss_pad = boss_profile.extrude(height=24, name='boss_pad', both_sides=False, max_deviation=-1, twist=0, taper_angle=0)
    # camber:end boss_pad

    # camber:begin housing_blank
    housing_blank = part.union(a=base_pad, b=boss_pad, name='housing_blank')
    # camber:end housing_blank

    # camber:begin right_rib_profile
    right_rib_profile = part.sketch(plane='xy', name='right_rib_profile', origin_name=None, frame=Frame(origin=(0, 0, 8), x=(1, 0, 0), y=(0, 0, 1), z=(0, -1, 0)), constrained=True)
    right_rib_profile.add_line((15, 0), (29, 0))
    right_rib_profile.add_line((29, 0), (15, 14))
    right_rib_profile.add_line((15, 14), (15, 0))
    # camber:end right_rib_profile

    # camber:begin right_rib
    right_rib = right_rib_profile.extrude(height=3, name='right_rib', both_sides=True, max_deviation=-1, twist=0, taper_angle=0)
    # camber:end right_rib

    # camber:begin right_braced
    right_braced = part.union(a=housing_blank, b=right_rib, name='right_braced')
    # camber:end right_braced

    # camber:begin left_rib_profile
    left_rib_profile = part.sketch(plane='xy', name='left_rib_profile', origin_name=None, frame=Frame(origin=(0, 0, 8), x=(1, 0, 0), y=(0, 0, 1), z=(0, -1, 0)), constrained=True)
    left_rib_profile.add_line((-15, 0), (-15, 14))
    left_rib_profile.add_line((-15, 14), (-29, 0))
    left_rib_profile.add_line((-29, 0), (-15, 0))
    # camber:end left_rib_profile

    # camber:begin left_rib
    left_rib = left_rib_profile.extrude(height=3, name='left_rib', both_sides=True, max_deviation=-1, twist=0, taper_angle=0)
    # camber:end left_rib

    # camber:begin braced_housing
    braced_housing = part.union(a=right_braced, b=left_rib, name='braced_housing')
    # camber:end braced_housing

    # camber:begin bearing_seat
    bearing_seat = braced_housing.counterbore_hole(mouth=Frame(origin=(0, 0, 32), y=(0, -1, 0), z=(0, 0, -1)), diameter=20, counterbore_diameter=26, counterbore_depth=5, depth=None, name='bearing_seat', max_deviation=-1)
    # camber:end bearing_seat

    # camber:begin mount_hole_1
    mount_hole_1 = bearing_seat.counterbore_hole(mouth=Frame(origin=(-26, -19, 8), y=(0, -1, 0), z=(0, 0, -1)), diameter=6, counterbore_diameter=10, counterbore_depth=3, depth=None, name='mount_hole_1', max_deviation=-1)
    # camber:end mount_hole_1

    # camber:begin mount_hole_2
    mount_hole_2 = mount_hole_1.counterbore_hole(mouth=Frame(origin=(26, -19, 8), y=(0, -1, 0), z=(0, 0, -1)), diameter=6, counterbore_diameter=10, counterbore_depth=3, depth=None, name='mount_hole_2', max_deviation=-1)
    # camber:end mount_hole_2

    # camber:begin mount_hole_3
    mount_hole_3 = mount_hole_2.counterbore_hole(mouth=Frame(origin=(26, 19, 8), y=(0, -1, 0), z=(0, 0, -1)), diameter=6, counterbore_diameter=10, counterbore_depth=3, depth=None, name='mount_hole_3', max_deviation=-1)
    # camber:end mount_hole_3

    # camber:begin mount_hole_4
    mount_hole_4 = mount_hole_3.counterbore_hole(mouth=Frame(origin=(-26, 19, 8), y=(0, -1, 0), z=(0, 0, -1)), diameter=6, counterbore_diameter=10, counterbore_depth=3, depth=None, name='mount_hole_4', max_deviation=-1)
    # camber:end mount_hole_4

    # camber:begin oil_port
    oil_port = mount_hole_4.hole(mouth=Frame(origin=(19, 0, 23), x=(0, 1, 0), y=(0, 0, -1), z=(-1, 0, 0)), diameter=4, depth=11, name='oil_port', max_deviation=-1)
    # camber:end oil_port

    return part, oil_port


if __name__ == "__main__":
    import camber
    camber.modeling.open(__file__)
