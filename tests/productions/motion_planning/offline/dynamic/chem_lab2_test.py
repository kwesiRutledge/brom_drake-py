import numpy as np
from pydrake.all import Simulator
import unittest

# Internal Imports
from brom_drake.motion_planning.algorithms.rrt.bidirectional_connect import (
    BidirectionalRRTConnectPlanner,
    BidirectionalRRTConnectPlannerConfig,
    BiRRTConnectSamplingProbabilities,
)
from brom_drake.productions.motion_planning.offline.dynamic.chem_lab2 import ChemLab2
from brom_drake.productions import ProductionID

class ChemLab2Test(unittest.TestCase):
    def test_init1(self):
        """
        Description
        -----------
        This test verifies that we can initialize the ChemLab2 without any errors.
        """
        # Setup
        production = ChemLab2(
            meshcat_port_number=None,
        )

        self.assertTrue(True)

    def test_easy_cast_and_build1(self):
        """
        Description
        -----------
        This test verifies that we can build the ChemLab2 without any errors.
        """
        # Setup
        
        # Create the production
        production = ChemLab2(
            meshcat_port_number=None, # Use None for CI
        )

        # Create a planner object which will be used to plan the motion
        config = BidirectionalRRTConnectPlannerConfig(
            steering_step_size=0.2,
        )
        planner2 = BidirectionalRRTConnectPlanner(
            production.arm,
            production.plant,
            production.scene_graph,
            config=config,
        )

        # To build the production, we only need to provide a planning function
        # (can come from anywhere, not just a BaseRRTPlanner object)
        diagram, diagram_context = production.easy_cast_and_build(
            planner2.plan,
            with_watcher=True,
        )

        # Return true if we get here
        self.assertTrue(True)

    def test_goal_pose1(self):
        """
        Description
        -----------
        This test verifies that the ChemLab2's goal_pose method.
        The method should not raise an error when the object
        is initialized with a goal configuration input (instead
        of a goal pose).
        """
        # Setup
        production = ChemLab2(
            meshcat_port_number=None,
            goal_configuration=np.zeros((6,)),
        )

        # Call goal_pose
        goal_pose = production.goal_pose

        # Return true if we get here
        self.assertTrue(True)

    def test_id1(self):
        """
        Description
        -----------
        This test verifies that the ChemLab2 is using the
        correct ID.
        """
        # Setup
        production = ChemLab2(
            meshcat_port_number=None,
        )

        # Check
        self.assertEqual(production.id, ProductionID.kChemLab2)

    def test_start_pose1(self):
        """
        Description
        -----------
        This test verifies that the ChemLab2's start_pose method.
        The method should not raise an error when the object
        is initialized with a start configuration input (instead
        of a start pose).
        """
        # Setup
        production = ChemLab2(
            meshcat_port_number=None,
            start_configuration=np.zeros((6,)),
        )

        # Call start_pose
        start_pose = production.start_pose

        # Return true if we get here
        self.assertTrue(True)

if __name__ == "__main__":
    unittest.main()