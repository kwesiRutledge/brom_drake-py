from dataclasses import dataclass
import networkx as nx
from typing import List

# Internal Imports
from brom_drake.utils.leaf_systems.network_fsm import (
    NetworkXFSM, FSMOutputDefinition, FSMTransitionCondition, FSMTransitionConditionType
)
from brom_drake.utils.pick_and_place.phase import PickAndPlacePhase
from brom_drake.utils.pick_and_place.target_description import PickAndPlaceTargetDescription

@dataclass
class Script:
    """
    Description
    -----------
    This class defines the "script" or the sequence of events that is meant to happen in the
    Pick and Place (or Motion Planning and Grasping) task.
    
    Assumptions
    -----------
    We assume that the robot will try to grasp each of the targets in the order that they
    appear in the list below. (i.e., pick up target 0, then target 1, etc.)
    """
    grasping_targets: List[PickAndPlaceTargetDescription]
    time_outs: List[float] # The amount of time given for the robot to reach each of the targets
    initial_settling_time: float # The time after initialization before the robot begins planning or moving
    
    def to_networkx_graph(self) -> nx.DiGraph:
        # Setup
        graph = nx.DiGraph()

        # Create initial state
        graph.add_node("init")

        # Collect the names of all of the targets
        target_names = []
        for target_name_ii in self.grasping_targets:
            name_candidate_ii = target_name_ii.name
            if name_candidate_ii in target_names:
                raise NotImplementedError(
                    f"Unsure of how to add a second object with name \"{name_candidate_ii}\" to the list of targets for the MotionPlanningAndGraspingProductionScript."
                )
            
            target_names.append(name_candidate_ii)

        n_targets = len(target_names)

        # For each target name, we're going to create the following PickAndPlace states.
        pick_and_place_phases = [
            PickAndPlacePhase.kPreGrasp, PickAndPlacePhase.kGrasp, PickAndPlacePhase.kPostGrasp,
            PickAndPlacePhase.kPrePlace, PickAndPlacePhase.kPlace, PickAndPlacePhase.kPostPlace
        ]
        for ii, target_name_ii in enumerate(target_names):
            target_ii: PickAndPlaceTargetDescription = self.grasping_targets[ii]

            # Check the value of the model instance index
            assert target_ii.model_instance_index is not None, \
                f"The provided target does not have a model instance index defined yet. Please define one before calling to_networkx_graph()!"

            for phase_jj in pick_and_place_phases:
                graph.add_node(
                    target_ii.name_for_pick_and_place_phase(phase_jj),
                    outputs=[
                        FSMOutputDefinition(f"model_target", target_ii.model_instance_index),
                    ]
                )

        # Add edges
            
        return graph

    def to_networkx_fsm(self) -> NetworkXFSM:
        pass