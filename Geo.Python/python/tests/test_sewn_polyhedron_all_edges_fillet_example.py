"""Headless regression for the five-body rounded-polyhedra viewer scene."""

import importlib.util
from pathlib import Path
import unittest


EXAMPLE = Path(__file__).resolve().parents[1] / "examples" / "sewn_polyhedron_all_edges_fillet.py"
spec = importlib.util.spec_from_file_location("sewn_polyhedron_all_edges_fillet", EXAMPLE)
model = importlib.util.module_from_spec(spec)
spec.loader.exec_module(model)


class RoundedPlatonicSceneTests(unittest.TestCase):
    def test_all_five_solids_build_at_the_viewer_radius(self):
        assembly = model.build()
        leaves = assembly.leaves()

        self.assertEqual(5, len(leaves))
        self.assertEqual(
            ["rounded_tetrahedron", "rounded_cube", "rounded_octahedron",
             "rounded_dodecahedron", "rounded_icosahedron"],
            [leaf.solid.name for leaf in leaves],
        )
        self.assertEqual([-40, -20, 0, 20, 40],
                         [leaf.frame.origin.x for leaf in leaves])
        expected_volumes = (194.647, 739.146, 654.400, 1402.292, 1288.015)
        for leaf, expected_volume in zip(leaves, expected_volumes):
            self.assertTrue(leaf.solid.is_watertight(), leaf.solid.name)
            self.assertAlmostEqual(leaf.solid.volume(), expected_volume,
                                   delta=expected_volume * .005, msg=leaf.solid.name)


if __name__ == "__main__":
    unittest.main()
