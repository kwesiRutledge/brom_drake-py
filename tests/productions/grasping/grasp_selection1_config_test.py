from brom_drake.productions.grasping.config import (
    Configuration as GraspSelection1Configuration,
)
from brom_drake.productions.grasping.script import Script as GraspSelection1Script

import unittest


class TestGraspSelection1Configuration(unittest.TestCase):
    def test_defaults1(self):
        """
        Description
        -----------
        This test verifies that the default Configuration uses the
        GraspSelection1 Script and the base configuration's default meshcat port.
        """
        # Setup
        config = GraspSelection1Configuration()

        # Verify defaults
        self.assertIsInstance(config.script, GraspSelection1Script)
        self.assertEqual(config.meshcat_port_number, 7001)
        self.assertFalse(config.show_collision_geometries)
        self.assertFalse(config.show_gripper_base_frame)

    def test_defaults2(self):
        """
        Description
        -----------
        This test verifies that two default Configurations do not share
        their mutable fields (i.e., the script and base configuration).
        """
        # Setup
        config1 = GraspSelection1Configuration()
        config2 = GraspSelection1Configuration()

        # Modify config1
        config1.script.drop_time = 1234.0
        config1.base.time_step = 0.5

        # Verify config2 was not modified
        self.assertIsNot(config1.script, config2.script)
        self.assertNotEqual(config2.script.drop_time, 1234.0)
        self.assertNotEqual(config2.base.time_step, 0.5)

    def test_defaults3(self):
        """
        Description
        -----------
        This test verifies that the default gains of the gripper's puppet have one value
        per puppet joint and are not shared between Configurations.
        """
        # Setup
        config1 = GraspSelection1Configuration()
        config2 = GraspSelection1Configuration()

        # Verify
        self.assertEqual(len(config1.gripper_puppet_Kp), 6)
        self.assertEqual(len(config1.gripper_puppet_Kd), 6)
        self.assertIsNot(config1.gripper_puppet_Kp, config2.gripper_puppet_Kp)
        self.assertIsNot(config1.gripper_puppet_Kd, config2.gripper_puppet_Kd)

    def test_meshcat_port_number1(self):
        """
        Description
        -----------
        This test verifies that the meshcat_port_number property
        reads and writes the value in the base configuration.
        """
        # Setup
        config = GraspSelection1Configuration()

        # Set the port number
        config.meshcat_port_number = 7001

        # Verify
        self.assertEqual(config.meshcat_port_number, 7001)
        self.assertEqual(config.base.meshcat_port_number, 7001)


if __name__ == "__main__":
    unittest.main()
