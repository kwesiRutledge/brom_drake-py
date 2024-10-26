"""
rrt_plan_generator.py
Description:

    This file defines the RRT planner class, which is used to plan motion for a robot using the Rapidly-exploring Random Tree (RRT) algorithm.
"""
import networkx as nx
import numpy as np
from pydrake.common.value import AbstractValue
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import ModelInstanceIndex
from pydrake.systems.framework import LeafSystem

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

    def ComputePlanIfNotAvailable(self, context, output):
        """
        Description:
            This function computes the plan if it is not available.
        """
        if self.plan is None:
            # Compute plan
            q_start = self.get_input_port("start_configuration").Eval(context)
            q_goal = self.get_input_port("goal_configuration").Eval(context)

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
        self.DeclareAbstractInputPort(
            "start_configuration",
            AbstractValue.Make(np.zeros((self.n_actuated_dof,))),
        )
        self.DeclareAbstractInputPort(
            "goal_configuration",
            AbstractValue.Make(np.zeros((self.n_actuated_dof,))),
        )

    def create_output_ports(self):
        """
        Description:
            This function creates the output ports for the RRT planner.
        """
        # Setup

        # Create output for plan
        sample_plan = np.zeros((0, self.n_actuated_dof))
        self.DeclareVectorOutputPort(
            "plan",
            AbstractValue.Make(sample_plan),
            self.ComputePlanIfNotAvailable
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