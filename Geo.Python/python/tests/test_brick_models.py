"""Physical assembly checks for original figures and modular trees."""

import unittest
from camber import set_progress_log
from brick_catalog import PartSpec
from brick_models import MODEL_FAMILIES, build_model
from brick_parts import build_rectangular


class BrickModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        set_progress_log(False)
        cls.models = {}
        failures = []
        for key in MODEL_FAMILIES:
            try:
                cls.models[key] = build_model(key)
            except Exception as error:
                failures.append(f"{key}: {error}")
        if failures:
            raise AssertionError("\n".join(failures))

    def test_components_and_nominal_envelopes(self):
        for key, model in self.models.items():
            with self.subTest(model=key):
                self.assertEqual(model.assembly.constraints, [])
                self.assertEqual(len(model.assembly.parts), model.components)
                self.assertGreater(model.volume, 0)
                self.assertAlmostEqual(model.bounds[0][2], 0, delta=0.002)
                if key in ("tree_conifer", "tree_broadleaf"):
                    self.assertEqual(model.components, 1)
                else:
                    self.assertGreater(model.components, 4)
        adult = self.models["figure_classic"]
        child = self.models["figure_short"]
        self.assertGreater(adult.bounds[1][2], child.bounds[1][2] + 5)
        self.assertLess(adult.bounds[1][2], 44)
        self.assertGreater(self.models["tree_conifer"].bounds[1][2], 60)

    def test_connections_have_no_significant_solid_overlap(self):
        for key, model in self.models.items():
            with self.subTest(model=key):
                hits = model.assembly.interferences(min_volume=0.005)
                self.assertEqual([(h.first, h.second, h.volume) for h in hits], [])

    def test_hip_and_wrist_sockets_have_closed_ends(self):
        from brick_models import _figure_wrist

        for key, pivot in (("figure_classic", 11.2), ("figure_short", 7.0)):
            with self.subTest(model=key):
                model = self.models[key]
                owner = model.assembly._part
                solids = {body.name: body for body, _ in model.placements}
                leg = solids[key + "_right_leg"]
                hip_end = owner.raycast(leg, (-1, 0, pivot), (1, 0, 0))
                self.assertIsNotNone(hip_end)
                self.assertAlmostEqual(hip_end.point.x, 0, delta=0.002)
                arm = solids[key + "_right_arm"]
                cuff, down = _figure_wrist()
                wrist_end = owner.raycast(arm, cuff - down * 2.2, -down)
                self.assertIsNotNone(wrist_end)
                self.assertAlmostEqual(
                    (wrist_end.point - cuff).dot(-down), 2.5, delta=0.003
                )

    def test_both_figures_seat_on_standard_stud_spacing(self):
        for key in ("figure_classic", "figure_short"):
            with self.subTest(model=key):
                model = self.models[key]
                owner = model.assembly._part
                plate = build_rectangular(
                    owner, PartSpec(key + "_test_plate", "plate", 2, 1, 1)
                )
                seated = owner.assembly(key + "_seated")
                seated.add_part(plate)
                seated.add_subassembly(model.assembly, (0, 0, 3.2))
                hits = seated.interferences(min_volume=0.005)
                self.assertEqual([(h.first, h.second, h.volume) for h in hits], [])


if __name__ == "__main__":
    unittest.main()
