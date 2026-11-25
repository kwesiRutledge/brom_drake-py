import unittest

import numpy as np
from pydrake.all import (
    Box,
    Capsule,
    Cylinder,
    Ellipsoid,
    HalfSpace,
    RigidTransform,
    RollPitchYaw,
    Sphere,
)

from brom_drake.productions.grasping.grasp_selection import lowest_z_of_shape


class LowestZOfShapeTest(unittest.TestCase):
    def test_box1(self):
        X = RigidTransform(p=[0.0, 0.0, 1.0])
        self.assertAlmostEqual(lowest_z_of_shape(Box(0.2, 0.4, 0.6), X), 0.7)

    def test_rotated_box1(self):
        # A unit cube rotated 45 degrees about x stands on an edge.
        X = RigidTransform(RollPitchYaw(np.pi / 4, 0.0, 0.0), [0.0, 0.0, 0.0])
        self.assertAlmostEqual(lowest_z_of_shape(Box(1.0, 1.0, 1.0), X), -np.sqrt(0.5))

    def test_sphere1(self):
        X = RigidTransform(p=[1.0, 2.0, 0.5])
        self.assertAlmostEqual(lowest_z_of_shape(Sphere(0.1), X), 0.4)

    def test_cylinder1(self):
        upright = RigidTransform(p=[0.0, 0.0, 0.0])
        self.assertAlmostEqual(lowest_z_of_shape(Cylinder(0.1, 0.4), upright), -0.2)

        lying = RigidTransform(RollPitchYaw(-np.pi / 2, 0.0, 0.0), [0.0, 0.0, 0.0])
        self.assertAlmostEqual(lowest_z_of_shape(Cylinder(0.1, 0.4), lying), -0.1)

    def test_capsule_and_ellipsoid1(self):
        X = RigidTransform()
        self.assertAlmostEqual(lowest_z_of_shape(Capsule(0.1, 0.4), X), -0.3)
        self.assertAlmostEqual(lowest_z_of_shape(Ellipsoid(0.1, 0.2, 0.3), X), -0.3)

    def test_unsupported_shape1(self):
        with self.assertRaises(TypeError):
            lowest_z_of_shape(HalfSpace(), RigidTransform())


if __name__ == "__main__":
    unittest.main()
