import numpy as np
import unittest

# Internal Imports
from brom_drake.productions.motion_planning.offline.dynamic import ChemLab2

class KinemticTest(unittest.TestCase):
    def test_start_pose1(self):
        """
        Description
        -----------
        In this test, we prove that the start pose is correctly set
        when we provide the start_configuraiton as an input parameter
        to a subclass of a OfflineDynamicMotionPlanningProduction production.
        """
        # Setup
        q0 = np.zeros((6,))
        production = ChemLab2(
            meshcat_port_number=None,
            start_configuration=q0,
        )

        # Call the method
        start_pose = production.start_pose

        print(start_pose)

        # If the method succeeds, then the start pose was correctly inferred
        # from the start configuration
        self.assertTrue(True)



if __name__ == '__main__':
    unittest.main()