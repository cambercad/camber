"""Architectural generators: parameter validation, closed parts and placement."""
from dataclasses import replace
import importlib.util
import math
import unittest

from camber import Part
from examples.architecture.windows import WindowParameters, build_window
from examples.architecture.doors import DoorParameters, build_door


class JoineryParameterTests(unittest.TestCase):
    def test_invalid_sizes_layouts_and_profile_combinations_are_rejected(self):
        for values in ({"width": float("nan")}, {"leaves": True}, {"leaves": 3, "leaf_weights": (1, 2)},
                       {"width": 500, "leaves": 3}, {"glazing": (4, 16)}, {"sash_depth": 20},
                       {"opening_angles": (0, 30), "hinge_sides": ("left", "fixed")},
                       {"transom_height": 250, "mullion_width": 0}):
            with self.subTest(window=values), self.assertRaises(ValueError):
                WindowParameters(**values)
        for values in ({"width": 300}, {"height": 400}, {"leaves": 3}, {"panel_rows": 0},
                       {"style": "unknown"}, {"threshold_height": float("inf")},
                       {"style": "glazed", "leaf_thickness": 30}, {"active_leaf": 1}):
            with self.subTest(door=values), self.assertRaises(ValueError):
                DoorParameters(**values)

    def test_parameter_sequences_are_immutable_and_replaceable(self):
        weights = [1, 2]
        params = WindowParameters(leaf_weights=weights)
        weights[0] = -1
        self.assertEqual((1, 2), params.leaf_weights)
        self.assertEqual(1800, replace(params, width=1800).width)


@unittest.skipUnless(importlib.util.find_spec("_camber_native"), "requires native wheel")
class JoineryGeometryTests(unittest.TestCase):
    def assert_closed_components(self, model):
        self.assertTrue(model.assembly.solve().converged)
        self.assertEqual(len(model.solids), len(model.assembly.leaves()))
        for solid in model.solids:
            with self.subTest(component=solid.name):
                self.assertTrue(solid.is_watertight())
                self.assertGreater(solid.volume(), 0)

    def test_single_small_and_three_unequal_open_window_leaves(self):
        for params in (WindowParameters(width=650, height=850, leaves=1, sill=False, glazing=(4, 16, 4)),
                       WindowParameters(width=1400, height=1200, leaves=2, mullion_width=0,
                                        opening_angles=(40, 0)),
                       WindowParameters(width=2400, height=1650, leaves=3, leaf_weights=(1, 1.2, .8),
                                        opening_angles=(35, 0, 60), hinge_sides=("left", "fixed", "right"), transom_height=280)):
            with self.subTest(parameters=params):
                model = build_window(params, name="tested_window")
                self.assert_closed_components(model)
                self.assertEqual(params.leaves, len(model.leaves))
                self.assertEqual(params.leaves * (len(params.glazing)+1)//2 + (3 if params.transom_height else 0), len(model.glass))
                # Opening rotates about the hinge, rather than the model origin.
                if params.opening_angles[0]:
                    back = 11+params.sash_depth
                    closed_x = params.frame_width+params.gap
                    theta = math.radians(params.opening_angles[0])
                    self.assertAlmostEqual(closed_x+(back+5)*math.sin(theta), model.leaves[0].pose[0], places=6)

    def test_doors_all_styles_and_right_active_double_leaf(self):
        configurations = (DoorParameters(width=900, height=2050, style="flush", hinge_sides=("right",), opening_angles=(45,)),
                          DoorParameters(width=1000, height=2150),
                          DoorParameters(width=1500, height=2300, leaves=2, style="glazed", active_leaf=1,
                                         opening_angles=(15, 50), threshold_height=18))
        for params in configurations:
            with self.subTest(parameters=params):
                model = build_door(params, name="tested_door")
                self.assert_closed_components(model)
                self.assertEqual(params.leaves, len(model.leaves))
                self.assertEqual(2*params.leaves if params.style == "glazed" else 0, len(model.glass))
                if params.leaves == 2:
                    names = [solid.name for solid in model.solids]
                    self.assertTrue(any("leaf1_lower_flush_bolt" in name for name in names))
                    self.assertFalse(any("leaf1_front_lever" in name for name in names))

    def test_window_and_door_are_reusable_in_one_house_assembly(self):
        owner = Part((-5000, -5000, -500), (5000, 5000, 4000), tolerance=.025)
        window = build_window(WindowParameters(width=800, height=1100, leaves=1), part=owner, name="kitchen")
        door = build_door(DoorParameters(style="flush"), part=owner, name="entrance")
        house = owner.assembly(name="house_openings")
        house.solve_after_every_constraint = False
        house.fix(house.add_subassembly(window.assembly, position=(1400, 0, 900)))
        house.fix(house.add_subassembly(door.assembly))
        self.assertTrue(house.solve().converged)
        self.assertEqual(len(window.solids)+len(door.solids), len(house.leaves()))


if __name__ == "__main__":
    unittest.main()
