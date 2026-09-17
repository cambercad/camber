"""Reference proportions and actual receiving fits of the refined figure parts."""

import unittest
from camber import Frame, Part, set_progress_log, vec3
from brick_models import (
    _figure_arm,
    _figure_upper_body,
    _figure_wrist,
    build_model,
)
from brick_parts import WORKING_LOW, WORKING_HIGH, cylinder


class FigureUpperBodyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        set_progress_log(False)
        cls.owner = Part(WORKING_LOW, WORKING_HIGH, 0.025)
        cls.torso, cls.head, cls.face = _figure_upper_body(cls.owner, "measured_figure")
        cls.arm, cls.hand = _figure_arm(cls.owner, "measured_right_arm")

    def test_reference_head_diameter_and_torso_taper_are_measured_on_geometry(self):
        p = self.owner
        left = p.raycast(self.head, (-20, 0, 34), (1, 0, 0))
        right = p.raycast(self.head, (20, 0, 34), (-1, 0, 0))
        self.assertAlmostEqual(right.point.x - left.point.x, 10.2, delta=0.003)
        # Sample unperforated side faces away from the shoulder journal and
        # mould rounds, then extrapolate their real plane to the shoulder top.
        lower = p.raycast(self.torso, (20, 2.6, 20), (-1, 0, 0))
        upper = p.raycast(self.torso, (20, 2.6, 26), (-1, 0, 0))
        self.assertLess(upper.point.x, lower.point.x)
        gradient = (upper.point.x - lower.point.x) / 6
        shoulder_half = upper.point.x + gradient * (28.8 - 26)
        self.assertAlmostEqual(shoulder_half * 2, 11.2, delta=0.005)

    def test_arm_is_round_and_its_angled_wrist_has_a_real_blind_end(self):
        front = self.owner.raycast(self.arm, (9.6, -20, 22), (0, 1, 0))
        back = self.owner.raycast(self.arm, (9.6, 20, 22), (0, -1, 0))
        self.assertAlmostEqual(back.point.y - front.point.y, 4.3, delta=0.035)
        cuff, down = _figure_wrist()
        end = self.owner.raycast(self.arm, cuff - down * 2.2, -down)
        self.assertIsNotNone(end)
        self.assertAlmostEqual((end.point - cuff).dot(-down), 2.5, delta=0.003)

    def test_hand_grips_a_nominal_bar_but_rejects_an_oversize_bar(self):
        cuff, down = _figure_wrist()
        up = -down
        across = (vec3(1, 0, 0) - up * up.x).normalized()
        normal = across.cross(up).normalized()
        center = cuff + down * 3.1
        frame = Frame(center - normal * 3, x=across, y=up, z=normal)
        bar = cylinder(self.owner, "nominal_hand_gauge", 1.55, 6, frame=frame)
        larger = cylinder(self.owner, "oversize_hand_gauge", 1.8, 6, frame=frame)
        self.assertLess(self.owner.intersect(self.hand, bar).volume(), 1e-6)
        self.assertGreater(self.owner.intersect(self.hand, larger).volume(), 0.2)

    def test_actual_upper_body_parts_have_clear_receiving_interfaces(self):
        assembly = self.owner.assembly("upper_body_fit")
        for body in (self.torso, self.head, self.face, self.arm, self.hand):
            self.assertTrue(body.is_watertight())
            assembly.add_part(body)
        self.assertEqual(assembly.interferences(min_volume=0.005), [])


class CompleteFigureTests(unittest.TestCase):
    def test_both_complete_figures_are_closed_and_have_no_component_penetration(self):
        set_progress_log(False)
        for name in ("figure_classic", "figure_short"):
            with self.subTest(figure=name):
                model = build_model(name)
                self.assertGreater(model.volume, 0)
                self.assertEqual(model.assembly.interferences(min_volume=0.005), [])
                self.assertAlmostEqual(model.bounds[0][2], 0, delta=0.002)
                self.assertLess(model.bounds[1][2], 44)


if __name__ == "__main__":
    unittest.main()
