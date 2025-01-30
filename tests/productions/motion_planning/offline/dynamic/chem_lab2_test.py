from pydrake.all import Simulator
import unittest

# Internal Imports
from brom_drake.motion_planning.algorithms.rrt.bidirectional_connect import (
    BidirectionalRRTConnectPlanner,
    BidirectionalRRTConnectPlannerConfig,
    BiRRTConnectSamplingProbabilities,
)
from brom_drake.productions.motion_planning.offline.dynamic.chem_lab2 import ChemLab2

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

if __name__ == "__main__":
    unittest.main()