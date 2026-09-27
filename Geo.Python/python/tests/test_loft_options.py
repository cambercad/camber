import unittest

from camber import LoftOptions


class LoftOptionsTests(unittest.TestCase):
    def test_general_style_and_crease_settings(self):
        options = LoftOptions(
            style="smooth_catmull_rom",
            correspondence="arc_length",
            crease_policy="none",
        )

        self.assertEqual("smooth_catmull_rom", options.style)
        self.assertEqual("arc_length", options.correspondence)
        self.assertEqual("none", options.crease_policy)

        options.style = "ruled"
        options.crease_policy = "first_profile"
        self.assertEqual("ruled", options.style)
        self.assertEqual("first_profile", options.crease_policy)

    def test_rejects_unknown_setting_names(self):
        options = LoftOptions()
        with self.assertRaises(ValueError):
            options.style = "propeller"
        with self.assertRaises(ValueError):
            options.crease_policy = "sharp_edges"

    def test_propeller_preset_is_not_public(self):
        self.assertFalse(hasattr(LoftOptions, "propeller_blade"))


if __name__ == "__main__":
    unittest.main()
