from dataclasses import dataclass
import networkx as nx

# Internal Imports
from brom_drake.utils.leaf_systems.network_fsm import (
    NetworkXFSM, FSMOutputDefinition, FSMTransitionCondition, FSMTransitionConditionType
)

@dataclass
class AttemptGraspScript:
    """
    Description
    -----------
    This class defines the "script" or the sequence of events that is meant to happen in the
    AttemptGrasp production.
    """
    settling_time: float = 2.0
    grasp_closing_time: float = 2.0
    post_grasp_settling_time: float = 0.0
    drop_time: float = 10.0

    def to_fsm(self) -> NetworkXFSM:
        return NetworkXFSM(self.to_networkx_graph())

    def to_networkx_graph(self) -> nx.DiGraph:
        # Setup
        graph = nx.DiGraph()

        # Create NetworkX FSM to trigger the floor trajectory
        graph.add_node(
            0,
            outputs=[
                FSMOutputDefinition("start_floor", False),  # Floor trigger
                FSMOutputDefinition("start_gripper", False),  # Gripper trigger
            ]
        )
        graph.add_node(
            1,
            outputs=[
                FSMOutputDefinition("start_gripper", True),
            ]
        )
        graph.add_node(
            2,
            outputs=[
                FSMOutputDefinition("start_floor", True),  # second
            ]
        )

        # Create time from init -> gripper start
        graph.add_edge(0, 1, conditions=[
            FSMTransitionCondition(
                condition_type=FSMTransitionConditionType.kAfterThisManySeconds,
                condition_value=self.settling_time,
            )
        ])

        # Create time from gripper start to end of grasp closing
        graph.add_edge(1, 2, conditions=[
            FSMTransitionCondition(
                condition_type=FSMTransitionConditionType.kAfterThisManySeconds,
                condition_value=self.grasp_closing_time,
            )
        ])

        # Create time from end of grasp closing to floor drop start
        graph.add_edge(2, 3, conditions=[
            FSMTransitionCondition(
                condition_type=FSMTransitionConditionType.kAfterThisManySeconds,
                condition_value=self.post_grasp_settling_time,
            )
        ])

        # Create time from floor drop start to end of production
        graph.add_edge(3, 4, conditions=[
            FSMTransitionCondition(
                condition_type=FSMTransitionConditionType.kAfterThisManySeconds,
                condition_value=self.drop_time,
            )
        ])

        return graph

    def total_time(self) -> float:
        return self.settling_time + self.grasp_closing_time + self.post_grasp_settling_time + self.drop_time
    
