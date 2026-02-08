from importlib import resources as impresources
from pydrake.all import (
    MultibodyPlant,
    Parser,
)
import unittest

# Internal Imports
from brom_drake import robots
from brom_drake.utils.model_instances import get_first_body_in


class ModelInstancesTest(unittest.TestCase):
    def setUp(self):
        # Create the gripper urdf
        self.gripper_urdf1 = str(
            impresources.files(robots)
            / "models/robotiq/2f_85_gripper-no-mimic/urdf/robotiq_2f_85.urdf"
        )

    def test_get_first_body_in1(self):
        """
        Description
        -----------
        This method tests the get_first_body_in method.
        """
        # Setup
        plant = MultibodyPlant(time_step=0.0)
        gripper_model = Parser(plant).AddModels(self.gripper_urdf1)[0]

        # Algorithm
        first_body = get_first_body_in(plant, gripper_model)

        # Assert
        self.assertEqual(first_body.name(), "robotiq_arg2f_base_link")


if __name__ == "__main__":
    unittest.main()
