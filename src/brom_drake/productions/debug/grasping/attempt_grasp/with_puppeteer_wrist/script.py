from dataclasses import dataclass
import networkx as nx
import numpy as np
from pydrake.all import RigidTransform, PiecewisePose
from typing import List

# Internal Imports
from brom_drake.utils.leaf_systems.network_fsm import (
    NetworkXFSM, FSMOutputDefinition, FSMTransitionCondition, FSMTransitionConditionType
)
from .phases import AttemptGraspWithPuppeteerWristPhase

# Define some internal classes/enums
@dataclass
class Script:
    """
    *Description*
    
    This class defines the "script" or the sequence of events that is meant to happen in the
    AttemptGrasp production.

    *Attributes*

    settling_time_on_floor: float, optional
        The time (in seconds) to let the object settle on the floor before the gripper approaches.
        Default is 10.0 seconds.

    gripper_approach_time: float, optional
        The time (in seconds) for the gripper to approach the object.
        Default is 5.0 seconds.

    grasp_closing_time: float, optional
        The time (in seconds) for the gripper to close around the object.
        Default is 2.0 seconds.

    post_grasp_settling_time: float, optional
        The time (in seconds) to let the object settle in the grasp after the gripper has closed.
        Default is 0.1 seconds.

    drop_time: float, optional
        The time (in seconds) to let the object fall after releasing the grasp.
        Default is 10.0 seconds.
    """
    settling_time_on_floor: float = 10.0
    gripper_approach_time: float = 5.0
    grasp_closing_time: float = 2.0
    post_grasp_settling_time: float = 0.1
    drop_time: float = 10.0

    def add_all_states_to_networkx_graph(self, graph: nx.DiGraph) -> None:
        """
        *Description*
        
        This method adds all the states to the provided NetworkX graph.
        
        .. note::
            :collapsible:

            One of the states (kObjectSettlingInGrasp) is not explicitly created
            here, but it will be created anyway when the edges are added. (This is a feature
            of NetworkX where nodes can be created implicitly when edges are added.)

        *Parameters*
        
        graph: nx.DiGraph
            A blank NetworkX directed graph to which the states will be added.

        """
        # Setup

        # First State: Object settled
        graph.add_node(
            AttemptGraspWithPuppeteerWristPhase.kObjectSettlingOnFloor,
            outputs=[
                FSMOutputDefinition("start_floor", False),  # Floor trigger
                FSMOutputDefinition("enable_gripper_approach", True),  # Make gripper move towards object
                FSMOutputDefinition("close_gripper", False),  # Gripper trigger
                FSMOutputDefinition("switch_to_memory", "trajectory1"),  # Switch to memory system for gripper pose
            ]
        )

        # Second State: Gripper approach
        graph.add_node(
            AttemptGraspWithPuppeteerWristPhase.kGripperApproach,
            outputs=[
                FSMOutputDefinition("enable_gripper_approach", True),  # Make gripper move towards object
                FSMOutputDefinition("start_floor", False),  # Floor trigger
                FSMOutputDefinition("close_gripper", False),  # Gripper trigger
            ]
        )

        # Third State: Close gripper
        graph.add_node(
            AttemptGraspWithPuppeteerWristPhase.kGripperClosing,
            outputs=[
                FSMOutputDefinition("enable_gripper_approach", True),  # Stop gripper movement
                FSMOutputDefinition("close_gripper", True),
            ]
        )

        # Fourth State: Drop the floor
        graph.add_node(
            AttemptGraspWithPuppeteerWristPhase.kFloorDrop,
            outputs=[
                FSMOutputDefinition("start_floor", True),
                FSMOutputDefinition("switch_to_memory", "memory2"),  # Switch to memory system for gripper pose
            ]
        )

    def build_gripper_approach_trajectory_from_waypoints(self, waypoints_X_WorldGripper: List[RigidTransform]) -> PiecewisePose:
        """
        *Description*
        
        Returns a PiecewisePose trajectory for the gripper base to follow during the
        attempted grasp.

        The trajectory should be an interpolation between two poses that is then held
        until the end of the production.

        *Parameters*

        waypoints_X_WorldGripper: List[RigidTransform]
            The waypoints that the script will create a Pose trajectory for.
            This trajectory will start at the initial waypoint and 

        *Returns*

        gripper_base_trajectory: PiecewisePose
            A PiecewisePose trajectory for the gripper base to follow during the
            attempted grasp.
        """

        # Setup
        n_waypoints = len(waypoints_X_WorldGripper)

        # Compute the Grasp Pose of the gripper in the world frame
        pose_WorldGripper_initial = waypoints_X_WorldGripper[0]

        # For each event in the script, create (i) a time and (ii) a pose
        times = []
        poses = []
        
        # - Create initial pose
        times.append(0.0)
        poses.append(pose_WorldGripper_initial)

        # - Force all poses between (a) pre-grasp pose (will be same as initial pose) and (b) the final grasp pose
        #   to occur during the time window between kGripperApproach and kGripperClosing
        t_pre_grasp = self.start_time_of_phase(
            phase=AttemptGraspWithPuppeteerWristPhase.kGripperApproach
        )
        t_gripper_closing = self.start_time_of_phase(
            phase=AttemptGraspWithPuppeteerWristPhase.kGripperClosing
        )

        times_during_interpolation = np.linspace(start=t_pre_grasp, stop=t_gripper_closing, num=n_waypoints)
        for idx, time_i in enumerate(times_during_interpolation):
            times.append(time_i)
            poses.append(waypoints_X_WorldGripper[idx])

        # Assemble into a PiecewisePose
        gripper_base_trajectory = PiecewisePose.MakeLinear(
            times=times,
            poses=poses,
        )
        return gripper_base_trajectory

    def start_time_of_phase(self, phase: AttemptGraspWithPuppeteerWristPhase) -> float:
        """
        *Description*
        
        This method returns the start time of the given phase.
        
        *Parameters*
        
        phase: AttemptGraspWithPuppeteerWristPhase
            The phase for which to get the start time.
            
        *Returns*
        
        start_time: float
            The start time of the given phase in seconds.
        """
        match phase:
            case AttemptGraspWithPuppeteerWristPhase.kObjectSettlingOnFloor:
                return 0.0
            case AttemptGraspWithPuppeteerWristPhase.kGripperApproach:
                return self.settling_time_on_floor
            case AttemptGraspWithPuppeteerWristPhase.kGripperClosing:
                return self.settling_time_on_floor + self.gripper_approach_time
            case AttemptGraspWithPuppeteerWristPhase.kObjectSettlingInGrasp:
                return self.settling_time_on_floor + self.gripper_approach_time + self.grasp_closing_time
            case AttemptGraspWithPuppeteerWristPhase.kFloorDrop:
                return self.settling_time_on_floor + self.gripper_approach_time + self.grasp_closing_time + self.post_grasp_settling_time
            case _:
                raise ValueError(f"Unrecognized phase {phase}!")

    def to_fsm(self) -> NetworkXFSM:
        """
        *Description*

        This method converts the script to a NetworkXFSM instance.

        *Returns*

        fsm: NetworkXFSM
            The NetworkXFSM LeafSystem which coordinates/sends triggers out according to the script.
        """
        return NetworkXFSM(self.to_networkx_graph())

    def to_networkx_graph(self) -> nx.DiGraph:
        """
        *Description*
        
        This method converts the script to a NetworkX directed graph.

        *Returns*
        
        graph: nx.DiGraph
            The NetworkX directed graph representing the FSM of the script.
        """
        # Setup
        graph = nx.DiGraph()

        # Create NetworkX FSM to trigger the floor trajectory
        self.add_all_states_to_networkx_graph(graph)

        # Create time from init -> Let object settle
        graph.add_edge(
            AttemptGraspWithPuppeteerWristPhase.kObjectSettlingOnFloor,
            AttemptGraspWithPuppeteerWristPhase.kGripperApproach, 
            conditions=[
                FSMTransitionCondition(
                    condition_type=FSMTransitionConditionType.kAfterThisManySeconds,
                    condition_value=self.settling_time_on_floor,
                )
            ]
        )

        # Create time between gripper approach and start og grasp closing
        graph.add_edge(
            AttemptGraspWithPuppeteerWristPhase.kGripperApproach,
            AttemptGraspWithPuppeteerWristPhase.kGripperClosing, 
            conditions=[
                FSMTransitionCondition(
                    condition_type=FSMTransitionConditionType.kAfterThisManySeconds,
                    condition_value=self.gripper_approach_time,
                )
            ]
        )

        if self.post_grasp_settling_time > 0.0:
            # Create time from gripper start to end of grasp closing
            graph.add_edge(
                AttemptGraspWithPuppeteerWristPhase.kGripperClosing,
                AttemptGraspWithPuppeteerWristPhase.kObjectSettlingInGrasp,
                conditions=[
                    FSMTransitionCondition(
                        condition_type=FSMTransitionConditionType.kAfterThisManySeconds,
                        condition_value=self.grasp_closing_time,
                    )
                ]
            )

            # Create time from end of grasp closing to floor drop start
            graph.add_edge(
                AttemptGraspWithPuppeteerWristPhase.kObjectSettlingInGrasp,
                AttemptGraspWithPuppeteerWristPhase.kFloorDrop,
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
                AttemptGraspWithPuppeteerWristPhase.kGripperClosing,
                AttemptGraspWithPuppeteerWristPhase.kFloorDrop,
                conditions=[
                    FSMTransitionCondition(
                        condition_type=FSMTransitionConditionType.kAfterThisManySeconds,
                        condition_value=self.grasp_closing_time,
                    )
                ]
            )

        return graph

    def total_time(self) -> float:
        """
        *Description*

        This method returns the total time of the script.

        *Returns*

        total_time: float
            The total time of the script in seconds.
        """
        return self.settling_time_on_floor + self.gripper_approach_time + self.grasp_closing_time + self.post_grasp_settling_time + self.drop_time
    
