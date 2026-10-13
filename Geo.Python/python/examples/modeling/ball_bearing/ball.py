from camber import Part, Frame, Curve3D, LoftOptions


def build_model():
    # camber:begin setup
    part = Part((-50, -50, -50), (50, 50, 50), tolerance=0.01)
    # camber:end setup

    # camber:begin ball
    ball = part.sphere(center=(0, 0, 0), radius=3.75, name='ball', max_deviation=-1)
    # camber:end ball

    return part, ball


if __name__ == "__main__":
    import camber
    camber.modeling.open(__file__)
