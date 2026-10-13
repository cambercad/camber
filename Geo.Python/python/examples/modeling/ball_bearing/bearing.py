from camber import Part, Frame, Curve3D, LoftOptions


from camber import load_component

def build_model():
    # camber:begin setup
    part = Part((-50, -50, -50), (50, 50, 50), tolerance=0.01)
    # camber:end setup

    # camber:begin assembly
    assembly = part.assembly(name="assembly")
    # camber:end assembly

    # camber:begin inner_race
    inner_race = load_component('inner_race.py')
    # camber:end inner_race

    # camber:begin inner_race_instance
    inner_race_instance = assembly.add_part(inner_race, position=(0, 0, 0))
    # camber:end inner_race_instance

    # camber:begin fixed_base
    assembly.fix(inner_race_instance)
    # camber:end fixed_base

    # camber:begin outer_race
    outer_race = load_component('outer_race.py')
    # camber:end outer_race

    # camber:begin outer_race_instance
    outer_race_instance = assembly.add_part(solid=outer_race, position=(0, 0, 0), orientation=(0, 0, 0, 1))
    # camber:end outer_race_instance

    # camber:begin ball
    ball = load_component('ball.py')
    # camber:end ball

    # camber:begin ball_instance
    ball_instance = assembly.add_part(solid=ball, position=(16.75, 0, 0), orientation=(0, 0, 0, 1))
    # camber:end ball_instance


    # camber:begin fixed_ball_1
    assembly.fix(part=ball_instance)
    # camber:end fixed_ball_1

    # camber:begin ball_2
    ball_2 = assembly.add_part(solid=ball, position=(12.831244422242882, 10.766692462249532, 0), orientation=(0, 0, 0, 1))
    # camber:end ball_2

    # camber:begin fixed_ball_2
    assembly.fix(part=ball_2)
    # camber:end fixed_ball_2

    # camber:begin ball_3
    ball_3 = assembly.add_part(solid=ball, position=(2.9086069759210846, 16.495529862954484, 0), orientation=(0, 0, 0, 1))
    # camber:end ball_3

    # camber:begin fixed_ball_3
    assembly.fix(part=ball_3)
    # camber:end fixed_ball_3

    # camber:begin ball_4
    ball_4 = assembly.add_part(solid=ball, position=(-8.374999999999996, 14.505925513389348, 0), orientation=(0, 0, 0, 1))
    # camber:end ball_4

    # camber:begin fixed_ball_4
    assembly.fix(part=ball_4)
    # camber:end fixed_ball_4

    # camber:begin ball_5
    ball_5 = assembly.add_part(solid=ball, position=(-15.739851398163964, 5.728837400704954, 0), orientation=(0, 0, 0, 1))
    # camber:end ball_5

    # camber:begin fixed_ball_5
    assembly.fix(part=ball_5)
    # camber:end fixed_ball_5

    # camber:begin ball_6
    ball_6 = assembly.add_part(solid=ball, position=(-15.739851398163966, -5.72883740070495, 0), orientation=(0, 0, 0, 1))
    # camber:end ball_6

    # camber:begin fixed_ball_6
    assembly.fix(part=ball_6)
    # camber:end fixed_ball_6

    # camber:begin ball_7
    ball_7 = assembly.add_part(solid=ball, position=(-8.375000000000007, -14.505925513389345, 0), orientation=(0, 0, 0, 1))
    # camber:end ball_7

    # camber:begin fixed_ball_7
    assembly.fix(part=ball_7)
    # camber:end fixed_ball_7

    # camber:begin ball_8
    ball_8 = assembly.add_part(solid=ball, position=(2.908606975921077, -16.495529862954488, 0), orientation=(0, 0, 0, 1))
    # camber:end ball_8

    # camber:begin fixed_ball_8
    assembly.fix(part=ball_8)
    # camber:end fixed_ball_8

    # camber:begin ball_9
    ball_9 = assembly.add_part(solid=ball, position=(12.831244422242879, -10.766692462249537, 0), orientation=(0, 0, 0, 1))
    # camber:end ball_9

    # camber:begin fixed_ball_9
    assembly.fix(part=ball_9)
    # camber:end fixed_ball_9

    # camber:begin coaxial_races
    assembly.concentric(a=inner_race_instance.axis('inner_race:race_blank-Line1'), b=outer_race_instance.axis('outer_race:race_blank-Line3'))
    # camber:end coaxial_races

    # camber:begin aligned_race_faces
    assembly.coincident(a=inner_race_instance.plane('inner_race:race_blank-Line2'), b=outer_race_instance.plane('outer_race:race_blank-Line2'), opposite_normals=False)
    # camber:end aligned_race_faces

    return part, assembly


if __name__ == "__main__":
    import camber
    camber.modeling.open(__file__)
