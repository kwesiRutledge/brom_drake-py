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
    post_grasp_settling_time: float = 0.1
    drop_time: float = 10.0

    def add_all_states_to_networkx_graph(self, graph: nx.DiGraph) -> list[int]:
        # Setup

        # First State: Wait for settling time
        graph.add_node(
            0,
            outputs=[
                FSMOutputDefinition("start_floor", False),  # Floor trigger
                FSMOutputDefinition("start_gripper", False),  # Gripper trigger
            ]
        )

        # Second State: Start the gripper closing
        graph.add_node(
            1,
            outputs=[
                FSMOutputDefinition("start_gripper", True),
            ]
        )
        # Third State: Finished closing the gripper
        graph.add_node(
            2,
        )

        # Fourth State: Start the floor drop
        graph.add_node(
            3,
            outputs=[
                FSMOutputDefinition("start_floor", True),  # second
            ]
        )


    def to_fsm(self) -> NetworkXFSM:
        return NetworkXFSM(self.to_networkx_graph())

    def to_networkx_graph(self) -> nx.DiGraph:
        # Setup
        graph = nx.DiGraph()

        # Create NetworkX FSM to trigger the floor trajectory
        self.add_all_states_to_networkx_graph(graph)

        # Create time from init -> gripper start
        graph.add_edge(0, 1, conditions=[
            FSMTransitionCondition(
                condition_type=FSMTransitionConditionType.kAfterThisManySeconds,
                condition_value=self.settling_time,
            )
        ])

        if self.post_grasp_settling_time > 0.0:
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

        else:
            # Create time from end of grasp closing to end of production
            graph.add_edge(1, 3, conditions=[
                FSMTransitionCondition(
                    condition_type=FSMTransitionConditionType.kAfterThisManySeconds,
                    condition_value=self.grasp_closing_time,
                )
            ])

        return graph

    def total_time(self) -> float:
        return self.settling_time + self.grasp_closing_time + self.post_grasp_settling_time + self.drop_time
    
