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
from pydrake.multibody.inverse_kinematics import DoDifferentialInverseKinematics, \
    DifferentialInverseKinematicsParameters, InverseKinematics
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import ModelInstanceIndex
from pydrake.systems.framework import LeafSystem, BasicVector

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
        robot_model_idx: ModelInstanceIndex,
        max_rrt_iterations: int = 1e4,
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
        self.n_actuated_dof = self.plant.num_actuated_dofs()

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

            p_WStart = RigidTransform(
                Quaternion(p_WStart_vec[3:]),
                p_WStart_vec[:3]
            )

            p_WGoal = RigidTransform(
                Quaternion(p_WGoal_vec[3:]),
                p_WGoal_vec[:3]
            )

            # Convert to configuration space
            ik_params = DifferentialInverseKinematicsParameters(6, 6)
            start_ik_result = InverseKinematics(
                self.plant, self.plant_context,
                with_joint_limits=True,
            )
            print(start_ik_result)
            print("IK Result:", start_ik_result.joint_velocities)
            print("IK Result:", start_ik_result.joint_positions)
            q_start = start_ik_result.vector_q


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

        output.SetFrom(self.plan)

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

    def create_output_ports(self):
        """
        Description:
            This function creates the output ports for the RRT planner.
        """
        # Setup

        # Create output for plan
        sample_plan = np.zeros((0, self.n_actuated_dof))
        self.DeclareAbstractOutputPort(
            "plan",
            lambda: AbstractValue.Make(sample_plan),
            self.compute_plan_if_not_available
        )

        # Create output for plan readiness
        self.DeclareAbstractOutputPort(
            "plan_is_ready",
            lambda: AbstractValue.Make(False),
            self.GetPlanIsReady,
        )



    def set_dimension(self, dim_q):
        """
        Description:
            This function sets the dimension of the configuration space.
        """
        self.dim_q = dim_q

    def plan(self, q_start, q_goal):
        """
        Description:
            This function executes the RRT planning algorithm.
        """
        # Input Processing
        q_start = q_start.flatten()
        q_goal = q_goal.flatten()
        if q_start.shape[0] != self.dim_q or q_goal.shape[0] != self.dim_q:
            raise ValueError(
                f"Start configuration shape ({q_start.shape}) and goal configuration " +
                f"shape ({q_goal.shape}) must match the dimension of the robot ({self.dim_q})."
            )

        # RRT planning logic goes here
        pass

    def GetPlanIsReady(self, context, output: AbstractValue):
        """
        Description:
            This function checks if the plan is ready.
        """
        output.SetFrom(
            AbstractValue.Make(self.plan is not None)
        )