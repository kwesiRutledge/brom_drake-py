from brom_drake import robots
from brom_drake.all import GripperType, ProductionID
from brom_drake.productions.debug.grasping.attempt_grasp.with_puppeteer_wrist.production import AttemptGraspWithPuppeteerWrist
import numpy as np
from pydrake.all import RigidTransform
import importlib.resources as impresources
import unittest

class TestAttemptGraspWithPuppeteerWristProduction(unittest.TestCase):
    def setUp(self):
        # Create dummy object
        self.object_file = str(
            impresources.files(robots) / "models/erlenmeyer_flask/500ml.urdf"
        )
    
    def test_suggested_roles1(self):
        """
        Description
        -----------
        This test verifies that the suggested_roles method
        returns NO expected roles for the AttemptGraspWithPuppeteerWrist production.
        """
        # Setup
        production = AttemptGraspWithPuppeteerWrist(
            path_to_object=self.object_file,
            gripper_choice=GripperType.Robotiq_2f_85,
            grasp_joint_positions=np.array([0.7]),
            X_WorldGripper_trajectory=[RigidTransform(), RigidTransform()],
        )

        # Get suggested roles
        roles = production.suggested_roles

        # Verify the expected roles are present
        self.assertLessEqual(roles, [])

    def test_id1(self):
        """
        Description
        -----------
        This test verifies that the id() method returns the appropriate 
        Enum value for the AttemptGraspWithPuppeteerWrist production.
        It should not be the undefined id.
        """
        # Setup
        production = AttemptGraspWithPuppeteerWrist(
            path_to_object=self.object_file,
            gripper_choice=GripperType.Robotiq_2f_85,
            grasp_joint_positions=np.array([0.7]),
            X_WorldGripper_trajectory=[RigidTransform(), RigidTransform()],
        )

        # Verify the id
        self.assertEqual(
            production.id,
            ProductionID.kAttemptGraspWithPuppeteer,
        )

if __name__ == "__main__":
    unittest.main()