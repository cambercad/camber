import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from camber.glview import Camera, _LIGHT_DIR, _light_in_view


class ViewerWorldLightTests(unittest.TestCase):
    def test_orbit_keeps_light_fixed_in_world_space(self):
        camera = Camera()
        first = (camera.view_matrix(), _light_in_view(camera))
        camera.orbit(.21, -.13)
        second = (camera.view_matrix(), _light_in_view(camera))
        self.assertGreater(math.dist(first[1], second[1]), .1)
        for matrix, view_light in (first, second):
            self.assertAlmostEqual(math.sqrt(sum(v * v for v in view_light)), 1.0, places=12)
            for axis in range(3):
                world = sum(matrix[row][axis] * view_light[row] for row in range(3))
                self.assertAlmostEqual(world, _LIGHT_DIR[axis], places=12)


if __name__ == "__main__":
    unittest.main()
