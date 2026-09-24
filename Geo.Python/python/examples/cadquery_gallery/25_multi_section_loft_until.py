"""CadQuery gallery: multiple lofted sections followed by an until cut."""
from camber.cqcompat import Workplane

locations = [(-16, 1), (-8, 0), (7, 0.2), (17, -1.2)]
angles = iter([15, 0, -8, 10])


def make_tower(location):
    return (Workplane().rect(5, 16).workplane(offset=10)
            .ellipse(3, 8).workplane(offset=10).slot2D(20, 5, 90).loft()
            .rotateAboutCenter((0, 0, 1), next(angles))
            .val().located(location))


skyscrapers = Workplane().pushPoints(locations).eachpoint(make_tower)
result = (skyscrapers.transformed((0, -90, 0)).moveTo(15, 0)
          .rect(3, 3, forConstruction=True).vertices().circle(1)
          .cutBlind("last"))

if __name__ == "__main__":
    result.show(title="CadQuery gallery 25 — multi-section ellipse lofts")
