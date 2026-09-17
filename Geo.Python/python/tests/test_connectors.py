"""Actual component topology, contact apertures and seated-pair material fit."""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from camber import set_progress_log
from bricks import surface_bounds
from connector_catalog import CONNECTORS
from connector_parts import CONTACT, mate_pair
from connectors import build_pair, build_catalog


class ConnectorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        set_progress_log(False)
        cls.pairs = {key: build_pair(key) for key in CONNECTORS}

    def test_eighteen_distinct_pairs_and_separate_real_contacts(self):
        self.assertEqual(len(self.pairs), 18)
        for key, pair in self.pairs.items():
            for half in pair:
                with self.subTest(key=key, gender=half.gender):
                    contacts = [
                        s
                        for s in half.solids
                        if half.colors["*" + s.name + "*"] == CONTACT
                    ]
                    self.assertEqual(len(contacts), half.spec.contacts)
                    self.assertEqual(
                        len(set(half.contact_positions)), half.spec.contacts
                    )
                    self.assertTrue(half.assembly.solve().converged)
                    for solid in half.solids:
                        self.assertTrue(solid.is_watertight(), solid.name)
                        self.assertGreater(solid.signed_volume(), 0, solid.name)

    def test_all_actual_seated_material_pairs_are_clear(self):
        for key, (male, female) in self.pairs.items():
            with self.subTest(key=key):
                assembly = male.part.assembly(key + "_seated_test")
                assembly.add_subassembly(male.assembly)
                assembly.add_subassembly(female.assembly)
                overlaps = assembly.interferences(min_volume=1e-6)
                self.assertEqual([], [(o.first, o.second, o.volume) for o in overlaps])

    def test_round_contacts_have_open_receiving_sleeves(self):
        for key, (_, female) in self.pairs.items():
            if female.spec.family not in (
                "circular",
                "d_sub",
                "coaxial",
                "din",
                "rf_sma",
            ):
                continue
            sockets = [
                s for s in female.solids if female.colors["*" + s.name + "*"] == CONTACT
            ]
            for socket, (x, y) in zip(sockets, female.contact_positions):
                with self.subTest(key=key, socket=socket.name):
                    self.assertIsNone(
                        female.part.raycast(socket, (x, y, -1), (0, 0, 1))
                    )
                    radius = female.spec.contact_diameter / 2 + 0.1
                    self.assertIsNotNone(
                        female.part.raycast(socket, (x + radius, y, -1), (0, 0, 1))
                    )

    def test_axis_shoulder_and_clock_mates_seat_a_displaced_socket(self):
        male, female = build_pair("m8_a3")
        pair = mate_pair(male, female, initial_position=(2, 1, 8))
        position = pair.subassemblies[1].pose
        for coordinate in position:
            self.assertAlmostEqual(0, coordinate, delta=1e-6)
        self.assertEqual([], pair.interferences(min_volume=1e-6))
        self.assertEqual(4, len(pair.constraints))

    def test_published_usb_a_and_d_sub_envelopes(self):
        male, _ = self.pairs["usb_a"]
        lo, hi = surface_bounds(male.solids[0])
        self.assertAlmostEqual(12, hi[0] - lo[0], delta=0.001)
        self.assertAlmostEqual(4.5, hi[1] - lo[1], delta=0.001)
        for key, width in (("de9", 30.84), ("db25", 53.04)):
            lo, hi = surface_bounds(self.pairs[key][0].solids[0])
            self.assertAlmostEqual(width, hi[0] - lo[0], delta=0.001)
            self.assertAlmostEqual(12.55, hi[1] - lo[1], delta=0.001)

    def test_added_published_contact_patterns_and_socket_passages(self):
        import math

        din = self.pairs["din_5"][0]
        for x, y in din.contact_positions:
            self.assertAlmostEqual(3.49, math.hypot(x, y), places=8)
        self.assertEqual(
            (
                (-1.995, 1.6),
                (1.995, 1.6),
                (-2.5, -0.41),
                (2.5, -0.41),
                (-1.005, -2.39),
                (1.005, -2.39),
            ),
            self.pairs["mini_din_6"][0].contact_positions,
        )
        male, female = self.pairs["iec_c13_c14"]
        blades = [s for s in male.solids if male.colors["*" + s.name + "*"] == CONTACT]
        heights = [surface_bounds(s)[1][2] for s in blades]
        self.assertAlmostEqual(1, heights[2] - heights[0], places=5)
        sockets = [
            s for s in female.solids if female.colors["*" + s.name + "*"] == CONTACT
        ]
        for socket, (x, y) in zip(sockets, female.contact_positions):
            self.assertIsNone(female.part.raycast(socket, (x, y, -1), (0, 0, 1)))
            self.assertIsNotNone(
                female.part.raycast(socket, (x + 1.2, y, -1), (0, 0, 1))
            )
        sma = self.pairs["sma"][0]
        pin = next(s for s in sma.solids if sma.colors["*" + s.name + "*"] == CONTACT)
        lo, hi = surface_bounds(pin)
        self.assertAlmostEqual(0.92, hi[0] - lo[0], delta=0.002)

    def test_new_cable_housings_meet_their_shell_shoulders(self):
        for key in ("hdmi_a", "displayport", "din_5", "mini_din_6"):
            for half in self.pairs[key]:
                with self.subTest(key=key, gender=half.gender):
                    shell_lo, shell_hi = surface_bounds(half.solids[0])
                    housing_lo, housing_hi = surface_bounds(half.solids[1])
                    if half.gender == "male":
                        self.assertAlmostEqual(housing_hi[2], shell_lo[2], delta=0.001)
                    else:
                        self.assertAlmostEqual(shell_hi[2], housing_lo[2], delta=0.001)

    def test_worker_gallery_and_selection_validation(self):
        gallery, colors, report = build_catalog(["m8_a3", "usb_a"], workers=2)
        self.assertEqual(2, report["pairs"])
        self.assertEqual(4, len(gallery.subassemblies))
        self.assertEqual(4, len(report["entries"]))
        self.assertTrue(colors)
        for keys, workers in (
            ([], 1),
            (["missing"], 1),
            (["de9", "de9"], 1),
            (["de9"], 0),
        ):
            with self.subTest(keys=keys, workers=workers):
                with self.assertRaises(ValueError):
                    build_catalog(keys, workers=workers)


if __name__ == "__main__":
    unittest.main()
