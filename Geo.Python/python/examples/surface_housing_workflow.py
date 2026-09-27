"""Build a thin-walled cylindrical housing from sewn surface patches."""

from camber import Part


def build():
    part = Part((-8, -8, -2), (8, 8, 14), tolerance=.01)
    profile = part.sketch("xy", name="housing_profile")
    profile.add_circle((0, 0), 5)

    # Reuse the same exact profile edges so the sheet boundaries sew without
    # snapping. The base closes one end; the other remains open.
    wall = profile.extrude_surface(12, name="housing_wall")
    base = profile.surface(name="housing_base")
    open_housing = part.sew((wall, base), name="open_housing")
    assert not open_housing.is_volume

    # These are two useful outcomes from the same authored sheet: a closed
    # solid blank, and a hollow housing with constant wall thickness.
    capped_blank = open_housing.cap_planar_boundaries(name="capped_blank")
    housing = open_housing.thicken(.4, name="housing")
    top_rim = next(name for name in housing.curve_names
                   if "ThickenPositive_housing_wall-Circle1" in name and
                   "ThickenRim_1" in name)
    housing = housing.fillet(top_rim, .15, name="rounded_housing")
    assert capped_blank.is_volume and capped_blank.is_watertight()
    assert housing.is_volume and housing.is_watertight()
    return housing


if __name__ == "__main__":
    build().show(title="Sewn cylindrical surface housing")
