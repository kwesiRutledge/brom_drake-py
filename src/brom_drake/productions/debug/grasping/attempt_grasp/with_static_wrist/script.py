from dataclasses import dataclass
import networkx as nx

# Internal Imports
from brom_drake.utils.leaf_systems.network_fsm import (
    NetworkXFSM, FSMOutputDefinition, FSMTransitionCondition, FSMTransitionConditionType
)
from .phases import AttemptGraspWithStaticWristPhase

# Define some internal classes/enums
@dataclass
class Script:
    """
    Description
    -----------
    This class defines the "script" or the sequence of events that is meant to happen in the
    AttemptGrasp production.
    """
    settling_time_on_floor: float = 2.0
    grasp_closing_time: float = 5.0
    post_grasp_settling_time: float = 1.0
    drop_time: float = 10.0

    def add_all_states_to_networkx_graph(self, graph: nx.DiGraph) -> list[int]:
        # Setup

        # First State: Object settled
        graph.add_node(
            AttemptGraspWithStaticWristPhase.kObjectSettlingOnFloor,
            outputs=[
                FSMOutputDefinition("start_floor", False),  # Floor trigger
                FSMOutputDefinition("start_gripper", False),  # Gripper trigger
            ]
        )

        # Third State: Close gripper
        graph.add_node(
            AttemptGraspWithStaticWristPhase.kGripperClosing,
            outputs=[
                FSMOutputDefinition("start_gripper", True),
            ]
        )

        # Fourth State: Drop the floor
        graph.add_node(
            AttemptGraspWithStaticWristPhase.kFloorDrop,
            outputs=[
                FSMOutputDefinition("start_floor", True),
            ]
        )

    def start_time_of_phase(self, phase: AttemptGraspWithStaticWristPhase) -> float:
        if phase == AttemptGraspWithStaticWristPhase.kObjectSettlingOnFloor:
            return 0.0
        elif phase == AttemptGraspWithStaticWristPhase.kGripperClosing:
            return self.settling_time_on_floor
        elif phase == AttemptGraspWithStaticWristPhase.kObjectSettlingInGrasp:
            return self.settling_time_on_floor + self.grasp_closing_time
        elif phase == AttemptGraspWithStaticWristPhase.kFloorDrop:
            return self.settling_time_on_floor + self.grasp_closing_time + self.post_grasp_settling_time
        else:
            raise ValueError(f"Phase {phase} not recognized!")

    def to_fsm(self) -> NetworkXFSM:
        return NetworkXFSM(self.to_networkx_graph())

    def to_networkx_graph(self) -> nx.DiGraph:
        # Setup
        graph = nx.DiGraph()

        # Create NetworkX FSM to trigger the floor trajectory
        self.add_all_states_to_networkx_graph(graph)

        # Create time from init -> Let object settle
        graph.add_edge(
            AttemptGraspWithStaticWristPhase.kObjectSettlingOnFloor,
            AttemptGraspWithStaticWristPhase.kGripperClosing, 
            conditions=[
                FSMTransitionCondition(
                    condition_type=FSMTransitionConditionType.kAfterThisManySeconds,
                    condition_value=self.settling_time_on_floor,
                )
            ]
        )

        if self.post_grasp_settling_time > 0.0:
            # Create time from gripper start to end of grasp closing
            graph.add_edge(
                AttemptGraspWithStaticWristPhase.kGripperClosing,
                AttemptGraspWithStaticWristPhase.kObjectSettlingInGrasp,
                conditions=[
                    FSMTransitionCondition(
                        condition_type=FSMTransitionConditionType.kAfterThisManySeconds,
                        condition_value=self.grasp_closing_time,
                    )
                ]
            )

            # Create time from end of grasp closing to floor drop start
            graph.add_edge(
                AttemptGraspWithStaticWristPhase.kObjectSettlingInGrasp,
                AttemptGraspWithStaticWristPhase.kFloorDrop,
                conditions=[
                    FSMTransitionCondition(
                        condition_type=FSMTransitionConditionType.kAfterThisManySeconds,
                        condition_value=self.post_grasp_settling_time,
                    )
                ]
            )

        else:
            # Create time from end of grasp closing to end of production
            graph.add_edge(
                AttemptGraspWithStaticWristPhase.kGripperClosing,
                AttemptGraspWithStaticWristPhase.kFloorDrop,
                conditions=[
                    FSMTransitionCondition(
                        condition_type=FSMTransitionConditionType.kAfterThisManySeconds,
                        condition_value=self.grasp_closing_time,
                    )
                ]
            )

        return graph

    def total_time(self) -> float:
        return self.settling_time_on_floor + self.grasp_closing_time + self.post_grasp_settling_time + self.drop_time
    