"""
rrt_plan_generator.py
Description:

    This file defines the RRT planner class, which is used to plan motion for a robot using the Rapidly-exploring Random Tree (RRT) algorithm.
"""
import networkx as nx
import numpy as np
from pydrake.common.eigen_geometry import Quaternion
from pydrake.common.value import AbstractValue
from pydrake.math import RigidTransform
from pydrake.solvers import OsqpSolver, MathematicalProgram, Solve, SolutionResult
from pydrake.multibody.inverse_kinematics import DoDifferentialInverseKinematics, \
    DifferentialInverseKinematicsParameters, InverseKinematics
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import ModelInstanceIndex
from pydrake.systems.framework import LeafSystem, BasicVector, Context

from brom_drake.motion_planning.algorithms.rrt.base import BaseRRTPlanner


class RRTPlanGenerator(LeafSystem):
    """
    RRTPlanGenerator class
    Description:
        This class will call the RRT planning algorithm for
        and then share it in an output port.
    """
    def __init__(
        self,
        plant: MultibodyPlant,
        robot_model_idx: ModelInstanceIndex = None,
        max_rrt_iterations: int = int(1e4),
    ):
        """
        Description:
            This function initializes the RRT planner.
        """
        super().__init__()
        # Setup
        self.robot_model_idx = robot_model_idx
        self.plant = plant
        self.max_rrt_iterations = max_rrt_iterations
        self.plan = None

        # Compute dof using robot_model_idx
        self.dim_q = None

        # Set Up Input and Output Ports
        self.plant_context = None # NOTE: This should be assigned by the user before calling!
        self.create_input_ports()
        self.create_output_ports()

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

            base_rrt = BaseRRTPlanner(self.robot_model_idx, self.plant)

            if self.plant_context is None:
                raise ValueError("Plant context is not initialized yet!")

            base_rrt.plant_context = self.plant_context

            # Plan and extract path
            rrt, found_path = base_rrt.plan(
                q_start, q_goal,
                max_iterations=self.max_rrt_iterations,
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

    def set_internal_plant_context(self, plant_context_in: Context):
        self.plant_context = plant_context_in