import networkx as nx
import numpy as np
from pydrake.common.eigen_geometry import Quaternion
from pydrake.common.value import AbstractValue
from pydrake.geometry import SceneGraph
from pydrake.math import RotationMatrix
from pydrake.multibody.inverse_kinematics import InverseKinematics
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import ModelInstanceIndex
from pydrake.solvers import Solve, SolutionResult
from pydrake.systems.framework import LeafSystem, BasicVector, Context
from typing import Callable, Tuple

# Internal Imports
from brom_drake.utils.constants import MotionPlan


class PrototypicalPlannerSystem(LeafSystem):
    """
    Description
    -----------
    This function is meant to be a prototypical planner system.
    It will receive a planning algorithm and wrap that algorithm in a LeafSystem
    that works in Brom.
    """
    def __init__(
        self,
        plant: MultibodyPlant,
        scene_graph: SceneGraph,
        planning_algorithm: Callable[
            [np.ndarray, np.ndarray, Callable[[np.ndarray], bool]],
            Tuple[MotionPlan, int],
        ],
        robot_model_idx: ModelInstanceIndex = None,
        **kwargs,
    ):
        """
        Description
        -----------
        Constructor for the PrototypicalPlannerSystem.

        Arguments
        ---------
        plant : MultibodyPlant
            The plant that we are working with.
        scene_graph : SceneGraph
            The scene graph that we are working with.
        planning_algorithm : Callable[[np.ndarray, np.ndarray, Callable[[np.ndarray], bool]], Tuple[MotionPlan, bool]]
            The planning algorithm that we are using.
        robot_model_idx : ModelInstanceIndex
            The robot model index that we are working with.
        """
        LeafSystem.__init__(self)

        # Setup
        self.plant, self.scene_graph = plant, scene_graph
        self.planning_algorithm = planning_algorithm
        self.robot_model_idx = robot_model_idx

        # Define Input Ports
        self.root_context = None  # NOTE: This should be assigned by the user before calling!
        self.plan = None
        self.create_input_ports()

        # Define Output Ports
        self.create_output_ports()

    def check_collision_in_config(
        self,
        q_model: np.ndarray,
    ) -> bool:
        """
        Description
        -----------
        This function checks for collisions in the robot's environment.

        Arguments
        ---------
        q_model : np.ndarray
            The model configuration that we are checking for collisions.
        """
        # Input Processing
        if self.root_context is None:
            raise ValueError("Plant context is not initialized yet!")

        if self.robot_model_idx is None:
            raise ValueError("Robot model index is not set yet!")

        # Setup
        plant_context = self.plant.GetMyMutableContextFromRoot(self.root_context)
        scene_graph_context = self.scene_graph.GetMyMutableContextFromRoot(self.root_context)
        q_init = self.plant.GetPositions(plant_context, self.robot_model_idx)

        # Set the configuration
        self.plant.SetPositions(
            plant_context,
            self.robot_model_idx,
            q_model
        )

        # Check for collisions using the query object
        query_object = self.scene_graph.get_query_output_port().Eval(scene_graph_context)
        # return query_object.HasCollisions()

        # Alternative method to check for collisions
        closest_points = query_object.ComputePointPairPenetration()
        eps0 = 1e-2
        collision_detected = False
        # inspector = self.scene_graph.model_inspector()
        for pair_ii in closest_points:
            if pair_ii.depth > eps0:
                collision_detected = True
                break
            
        # Otherwise return false (after resetting the configuration)
        self.plant.SetPositions(
            plant_context,
            self.robot_model_idx,
            q_init
        )
        return collision_detected

    def compute_plan_if_not_available(self, context: Context, output: AbstractValue):
        """
        Description
        -----------
        This function computes the plan if it is not available.

        Arguments
        ---------
        context : Context
            The context that we are working with.
        output : AbstractValue
            The output that will be produced by the output port
        """
        # Print
        if self.plan is None:
            # Compute plan
            q_start = self.GetInputPort("start_configuration").Eval(context)
            q_goal = self.GetInputPort("goal_configuration").Eval(context)
            if self.robot_model_idx is None:
                self.robot_model_idx = self.GetInputPort("robot_model_index").EvalAbstract(context).get_value()

            if self.root_context is None:
                raise ValueError("Plant context is not initialized yet!")

            self.root_context = self.root_context

            # Plan and extract path
            rrt, goal_node_index = self.planning_algorithm(
                q_start,
                q_goal,
                self.check_collision_in_config
            )

            if goal_node_index == -1:
                raise RuntimeError("No path found! Try increasing the number of iterations or checking your problem!")

            path = nx.shortest_path(rrt, source=0, target=goal_node_index)
            self.plan = np.array([rrt.nodes[node]['q'] for node in path])

        output.SetFrom(
            AbstractValue.Make(self.plan)
        )

    def create_input_ports(self):
        """
        Description:
            This function creates the input ports for the RRT planner.
        """
        # Setup
        N = len(
            self.plant.GetActuatedJointIndices(self.robot_model_idx)
        )

        # Define outputs
        self.DeclareVectorInputPort(
            "start_configuration",
            BasicVector(np.zeros((N,))),
        )
        self.DeclareVectorInputPort(
            "goal_configuration",
            BasicVector(np.zeros((N,))),
        )
        self.DeclareAbstractInputPort(
            "robot_model_index",
            AbstractValue.Make(ModelInstanceIndex(-1))
        )

    def create_output_ports(self):
        """
        Description:
            This function creates the output ports for the RRT planner.
        """
        # Setup

        # Create output for plan
        sample_plan = np.zeros((0, self.n_actuated_dof))
        self.DeclareAbstractOutputPort(
            "motion_plan",
            lambda: AbstractValue.Make(sample_plan),
            self.compute_plan_if_not_available
        )

        # Create output for plan readiness
        self.DeclareAbstractOutputPort(
            "plan_is_ready",
            lambda: AbstractValue.Make(False),
            self.GetPlanIsReady,
        )

    def define_pose_ik_problem(
        self,
        input_pose_vec: np.ndarray,
        eps0: float = 2.5e-2,
    ) -> InverseKinematics:
        """
        Description
        -----------
        Sets up the inverse kinematics problem for the start pose
        input to theis function.
        :return:
        """
        # Setup
        plant_context = self.plant.GetMyMutableContextFromRoot(self.root_context)

        # Create IK Problem
        ik_problem = InverseKinematics(self.plant)

        # Add Pose Target
        ik_problem.AddPositionConstraint(
            self.plant.world_frame(),
            input_pose_vec[:3],
            self.plant.GetFrameByName("ft_frame"),
            (- np.ones((3,)) * eps0).reshape((-1, 1)),
            (+ np.ones((3,)) * eps0).reshape((-1, 1)),
        )

        # Add Orientation Cost (not a hard constraint to help with feasibility)
        ik_problem.AddOrientationCost(
            self.plant.world_frame(),
            RotationMatrix(Quaternion(input_pose_vec[3:]).rotation()),
            self.plant.GetFrameByName("ft_frame"),
            RotationMatrix.Identity(),
            0.25,
        )

        return ik_problem

    @property
    def n_actuated_dof(self) -> int:
        """
        Description
        -----------
        This method uses the plant and the robot model idx
        to identify the numer of degrees of freedom we have for control.
        :return:
        """
        return self.plant.num_actuated_dofs()

    def GetPlanIsReady(self, context, output: AbstractValue):
        """
        Description:
            This function checks if the plan is ready.
        """
        output.SetFrom(
            AbstractValue.Make(self.plan is not None)
        )

    def set_internal_root_context(self, root_context_in: Context):
        self.root_context = root_context_in