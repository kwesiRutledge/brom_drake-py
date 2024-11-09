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
from pydrake.systems.framework import LeafSystem, BasicVector
from typing import Callable

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
            np.ndarray,
        ],
        **kwargs,
    ):
        LeafSystem.__init__(self)

        # Setup
        self.plant, self.scene_graph = plant, scene_graph
        self.planning_algorithm = planning_algorithm

        # Compute dof using robot_model_idx
        self.dim_q = None

        # Define Input Ports
        self.root_context = None  # NOTE: This should be assigned by the user before calling!
        self.plan = None
        self.robot_model_idx = None  # Will be provided by input port
        self.create_input_ports()


        # Define Output Ports
        self.create_output_ports()

    def check_collision_in_config(
        self,
        q_model: np.ndarray,
    ) -> bool:
        """
        Description:
            This function checks for collisions in the robot's environment.
        """
        # Input Processing
        if self.root_context is None:
            raise ValueError("Plant context is not initialized yet!")

        if self.robot_model_idx is None:
            raise ValueError("Robot model index is not set yet!")

        # Setup
        plant_context = self.plant.GetMyMutableContextFromRoot(self.root_context)
        scene_graph_context = self.scene_graph.GetMyMutableContextFromRoot(self.root_context)

        # Set the configuration
        self.plant.SetPositions(
            plant_context,
            self.robot_model_idx,
            q_model
        )

        # TODO(kwesi): Add collision checking logic here!
        query_object = self.scene_graph.get_query_output_port().Eval(scene_graph_context)
        return query_object.HasCollisions()

    def compute_plan_if_not_available(self, context, output: AbstractValue):
        """
        Description:
            This function computes the plan if it is not available.
        """
        # Print
        if self.plan is None:
            # Compute plan
            p_WStart_vec = self.GetInputPort("start_pose").Eval(context)
            p_WGoal_vec = self.GetInputPort("goal_pose").Eval(context)
            self.robot_model_idx = self.GetInputPort("robot_model_index").EvalAbstract(context).get_value()

            print(self.robot_model_idx)

            q_start = self.solve_pose_ik_problem(
                p_WStart_vec,
            )

            q_goal = self.solve_pose_ik_problem(
                p_WGoal_vec,
            )

            base_rrt = self.planning_algorithm(q_start, q_goal, self.check_collision_in_config)

            if self.root_context is None:
                raise ValueError("Plant context is not initialized yet!")

            base_rrt.root_context = self.root_context

            # Plan and extract path
            rrt, found_path = base_rrt.plan(
                q_start, q_goal,
            )

            if not found_path:
                raise RuntimeError("No path found! Try increasing the number of iterations or checking your problem!")

            path = nx.shortest_path(rrt, source=0, target=rrt.number_of_nodes()-1)
            self.plan = np.array([rrt.nodes[node]['q'] for node in path])

        output.SetFrom(
            AbstractValue.Make(self.plan)
        )

    def create_input_ports(self):
        """
        Description:
            This function creates the input ports for the RRT planner.
        """
        self.DeclareVectorInputPort(
            "start_pose",
            BasicVector(np.zeros((7,))),
        )
        self.DeclareVectorInputPort(
            "goal_pose",
            BasicVector(np.zeros((7,))),
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
        # p_WStart = RigidTransform(
        #     Quaternion(p_WStart_vec[3:]),
        #     p_WStart_vec[:3]
        # )

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

        # TODO(kwesi): Add OrientationCosntraint
        ik_problem.AddOrientationConstraint(
            self.plant.world_frame(),
            RotationMatrix(Quaternion(input_pose_vec[3:]).rotation()),
            self.plant.GetFrameByName("ft_frame"),
            RotationMatrix.Identity(),
            0.25,
        )

        return ik_problem

    def solve_pose_ik_problem(
        self,
        input_pose_vec: np.ndarray
    ) -> np.ndarray:
        """
        Description:
        :param input_pose_vec:
        :return:
        """
        # Setup

        # Define Problem
        ik_problem = self.define_pose_ik_problem(input_pose_vec)

        # Solve problem
        ik_program = ik_problem.prog()
        ik_result = Solve(ik_program)

        assert ik_result.get_solution_result() == SolutionResult.kSolutionFound, \
            f"Solution result was {ik_result.get_solution_result()}; need SolutionResult.kSolutionFound to make RRT Plan!"

        q_out = ik_result.get_x_val()

        return q_out


    def set_dimension(self, dim_q: int):
        """
        Description:
            This function sets the dimension of the configuration space.
        """
        self.dim_q = dim_q

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