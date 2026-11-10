import gc
from pathlib import Path
import tempfile
import unittest

import numpy as np
from pydrake.all import Box

from brom_drake.productions.grasping.object_description import (
    ObjectDescription,
    ObjectGeometry,
)

INERTIAL = """
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
"""

BOX_URDF = f"""<?xml version="1.0"?>
<robot name="box_object">
  <link name="base">
    {INERTIAL}
    <collision name="offset_box">
      <origin xyz="0.1 0 0.2"/>
      <geometry><box size="0.01 0.02 0.03"/></geometry>
    </collision>
  </link>
</robot>
"""

TWO_BODY_URDF = f"""<?xml version="1.0"?>
<robot name="two_body_object">
  <link name="base">{INERTIAL}</link>
  <link name="tip">{INERTIAL}</link>
  <joint name="hinge" type="revolute">
    <parent link="base"/>
    <child link="tip"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>
"""

NO_COLLISION_URDF = f"""<?xml version="1.0"?>
<robot name="no_collision_object">
  <link name="base">{INERTIAL}</link>
</robot>
"""


class TestObjectDescription(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.directory = Path(directory.name)

    def write_urdf(self, contents: str) -> str:
        path = self.directory / "object.urdf"
        path.write_text(contents)
        return str(path)

    def test_from_model_file1(self):
        """
        Description
        -----------
        This test verifies that a collision shape is collected
        with its dimensions and its pose in the object frame.
        """
        # Setup
        path = self.write_urdf(BOX_URDF)

        # Collect the geometry
        geometry = ObjectGeometry.from_model_file(path)
        gc.collect()  # The shape must outlive the temporary plant

        # Verify
        self.assertEqual(len(geometry.collision_shapes), 1)

        collision_shape = geometry.collision_shapes[0]
        self.assertIsInstance(collision_shape.shape, Box)
        np.testing.assert_allclose(collision_shape.shape.size(), [0.01, 0.02, 0.03])
        np.testing.assert_allclose(
            collision_shape.X_ObjectShape.translation(), [0.1, 0.0, 0.2]
        )

    def test_from_model_file2(self):
        """
        Description
        -----------
        This test verifies that a model with more than one body is rejected
        (because it would not define a single object frame).
        """
        with self.assertRaises(ValueError):
            ObjectGeometry.from_model_file(self.write_urdf(TWO_BODY_URDF))

    def test_from_model_file3(self):
        """
        Description
        -----------
        This test verifies that a model without collision geometry is rejected.
        """
        with self.assertRaises(ValueError):
            ObjectGeometry.from_model_file(self.write_urdf(NO_COLLISION_URDF))

    def test_from_model_file4(self):
        """
        Description
        -----------
        This test verifies that the description holds the path and the geometry.
        """
        # Setup
        path = self.write_urdf(BOX_URDF)

        # Create the description
        description = ObjectDescription.from_model_file(path)

        # Verify
        self.assertEqual(description.path_to_object, path)
        self.assertIsInstance(description.geometry, ObjectGeometry)
        self.assertEqual(len(description.geometry.collision_shapes), 1)


if __name__ == "__main__":
    unittest.main()
