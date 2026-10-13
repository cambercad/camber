from camber import Part, Frame, Curve3D, LoftOptions


def build_model():
    # camber:begin setup
    part = Part((-50, -50, -50), (50, 50, 50), tolerance=0.01)
    # camber:end setup

    # camber:begin race_profile
    race_profile = part.sketch(plane='xy', name='race_profile', origin_name=None, frame=Frame(x=(0, 0, 1), y=(1, 0, 0), z=(0, 1, 0)), constrained=True)
    race_profile.add_line((-7, 10), (7, 10), name='Line1')
    race_profile.add_line((7, 10), (7, 14.5), name='Line2')
    race_profile.add_line((7, 14.5), (-7, 14.5), name='Line3')
    race_profile.add_line((-7, 14.5), (-7, 10), name='Line4')
    # camber:end race_profile

    # camber:begin race_blank
    race_blank = race_profile.revolve(angle=6.283185307179586, name='race_blank', max_deviation=-1)
    # camber:end race_blank

    # camber:begin raceway_profile
    raceway_profile = part.sketch(plane='xy', name='raceway_profile', origin_name=None, frame=Frame(x=(0, 0, 1), y=(1, 0, 0), z=(0, 1, 0)), constrained=True)
    raceway_profile.add_circle((0, 16.75), 3.8, name='Raceway')
    # camber:end raceway_profile

    # camber:begin raceway_tool
    raceway_tool = raceway_profile.revolve(angle=6.283185307179586, name='raceway_tool', max_deviation=-1)
    # camber:end raceway_tool

    # camber:begin inner_race
    inner_race = part.subtract(a=race_blank, b=raceway_tool, name='inner_race')
    # camber:end inner_race

    return part, inner_race


if __name__ == "__main__":
    import camber
    camber.modeling.open(__file__)
