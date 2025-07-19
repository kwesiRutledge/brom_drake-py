import networkx as nx
from pathlib import Path
from pydrake.all import HPolyhedron
import unittest

# Internal Imports
from brom_drake.utils.pick_and_place_phase import PickAndPlacePhase
from brom_drake.utils.pick_and_place_target_description import PickAndPlaceTargetDescription
from brom_drake.productions.types.pick_and_place import (
    MotionPlanningAndGraspingProductionScript
)

class MotionPlanningAndGraspingProductionScriptTest(unittest.TestCase):
    def test_to_networkx_graph1(self):
        """
        Description
        -----------
        This test verifies that the method creates the following nodes:
        - "init"
        - for each target with name `name_ii`:
          + for each pick and place phase name `pp_phase_jj`:
            ~ `name_ii` + `_` `pp_phase_jj`
        """
        # Setup
        box0 = HPolyhedron.MakeUnitBox(3)
        box1 = box0.Scale(1.5)

        # Create the Script
        my_target0 = PickAndPlaceTargetDescription(
            file_path=Path("./outputs/mid.urdf"),
            goal_region=box0,
            model_instance_index=0,
        )

        my_target1 = PickAndPlaceTargetDescription(
            file_path=Path("./outputs/webster.urdf"),
            goal_region=box1,
            model_instance_index=1,
        )

        script0 = MotionPlanningAndGraspingProductionScript(
            grasping_targets=[my_target0, my_target1],
            time_outs=[0.1, 10.0],
            initial_settling_time=2.5,
        )

        # Call the function to a networkx graph
        graph_out: nx.DiGraph = script0.to_networkx_graph()

        # Check to see if the initial node is in the graph
        self.assertIn("init", [node for node in graph_out.nodes()])

        # Check to see if each of the targets contains a state for EACH pick and place phases
        pick_and_place_phases = [
            PickAndPlacePhase.kPreGrasp, PickAndPlacePhase.kGrasp, PickAndPlacePhase.kPostGrasp,
            PickAndPlacePhase.kPrePlace, PickAndPlacePhase.kPlace, PickAndPlacePhase.kPostPlace
        ]
        for phase_jj in pick_and_place_phases:
            self.assertIn(
                my_target0.name_for_pick_and_place_phase(phase_jj),
                [node for node in graph_out.nodes()]
            )

            self.assertIn(
                my_target1.name_for_pick_and_place_phase(phase_jj),
                [node for node in graph_out.nodes()]
            )

if __name__ == '__main__':
    unittest.main()
