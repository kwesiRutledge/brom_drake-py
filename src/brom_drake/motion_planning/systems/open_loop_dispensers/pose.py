from enum import IntEnum
import numpy as np
from pydrake.all import RigidTransform
from pydrake.common.value import AbstractValue
from pydrake.systems.framework import LeafSystem, Context, BasicVector
from pydrake.trajectories import PiecewisePose, PiecewisePolynomial, PiecewiseQuaternionSlerp

# Internal Imports
from brom_drake.motion_planning.systems.state_of_plan_in_memory import StateOfPlanInMemory

class OpenLoopPosePlanDispenser(LeafSystem):
    def __init__(
        self,
        speed: float = 0.5, # Speed at which to proceed through points.
    ):
        LeafSystem.__init__(self)

        # Setup
        self.speed = speed
        self.plan, self.planned_trajectory = None, None

        # Create input port for the plan (we assume that when we receive it, we will close the port)
        self.plan_is_set = False
        sample_plan = [RigidTransform()]
        self.t0, self.t_final = -1.0, -1.0
        self.plan_port = self.DeclareAbstractInputPort(
            "plan",
            AbstractValue.Make(sample_plan),
        )

        self.plan_ready_port = self.DeclareAbstractInputPort(
            "plan_ready",
            AbstractValue.Make(False),
        )

        self.state_of_plan_in_memory_idx = self.DeclareDiscreteState(
            np.array([StateOfPlanInMemory.kNotSet])
        )

        # Create output ports
        self.define_output_ports()

    def define_output_ports(self):
        """
        Description
        -----------
        This method defines the output ports for this system.
        """
        self.DeclareAbstractOutputPort(
            "pose_in_plan",
            lambda: AbstractValue.Make(RigidTransform()),
            self.GetCurrentPoseInPlan,
        )

        self.DeclareAbstractOutputPort(
            "plan_is_set",
            lambda: AbstractValue.Make(StateOfPlanInMemory.kNotSet),
            self.GetStateOfPlanInMemory,
            {
                self.discrete_state_ticket(
                    self.state_of_plan_in_memory_idx,
                )
            },  # This output should only be updated when the
                # discrete state state_of_plan_in_memory changes
        )

    def GetCurrentPoseInPlan(self, context: Context, output_point: AbstractValue):
        # Setup
        plan_is_ready = self.plan_ready_port.Eval(context)

        # Get the abstract state
        abstract_state = context.get_discrete_state(self.state_of_plan_in_memory_idx)

        # If plan isn't ready, then skip the rest of the logic
        if not plan_is_ready:
            output_point.SetFrom(
                AbstractValue.Make(RigidTransform())
            )
            return

        # If this is the first time that plan_is_ready, then
        # let's save the current time as the time when the plan starts
        if abstract_state[0] == StateOfPlanInMemory.kNotSet:
            # Initialize the system to handle the new plan
            self.initialize_system_for_new_plan(context)

        # Use interpolation to get the current point
        t = context.get_time() - self.t0

        # Mark the plan as set
        context.SetDiscreteState(np.array([
            StateOfPlanInMemory.kSet,
        ]))

        # Output The Current Point
        if t > self.t_final:
            output_point.SetFrom(
                AbstractValue.Make(self.planned_trajectory.GetPose(self.t_final))
            )
        else:
            output_point.SetFrom(
                AbstractValue.Make(self.planned_trajectory.GetPose(t))
            )

    def find_time_between_two_points(self, x1: np.ndarray, x2: np.ndarray)->float:
        """
        Description
        -----------
        This method computes the time that it will take to move between two points.
        :return:
        """
        # Setup

        # Compute the distance between the two points
        dist = np.linalg.norm(x1 - x2)
        return dist / self.speed

    def initialize_system_for_new_plan(self, context: Context):
        """
        Description
        -----------
        This function should lock a couple of things into memory for this object to handle future
        calls to get the plan.

        :param context:
        :return:
        """
        # Setup

        # Get time to start executing the plan
        self.t0 = context.get_time()
        context.SetDiscreteState(
            np.array([StateOfPlanInMemory.kSet])
        )

        # Get times where we should reach each point in the plan and save it into a new map
        self.plan = self.plan_port.Eval(context)

        n_points_in_plan = len(self.plan)
        t_ii = 0.0
        times = [t_ii]
        temp_position_plan = np.array([
            self.plan[ii].translation() for ii in range(n_points_in_plan)
        ])
        temp_quaternion_plan = np.array([
            self.plan[ii].rotation().matrix() for ii in range(n_points_in_plan)
        ])
        for ii in range(n_points_in_plan-1):
            # Compute the time it should take to travel between two positions in the plan
            t_ii += self.find_time_between_two_points(
                temp_position_plan[ii,:],
                temp_position_plan[ii+1, :],
                )
            times += [t_ii]

        self.t_final = t_ii
        position_trajectory = PiecewisePolynomial.FirstOrderHold(times, temp_position_plan.T)
        orientation_trajectory = PiecewiseQuaternionSlerp(times, temp_quaternion_plan)
        self.planned_trajectory = PiecewisePose(
            position_trajectory,
            orientation_trajectory,
        )





    def GetStateOfPlanInMemory(self, context: Context, output: AbstractValue):
        """Plan is set if and only if the internal variable is not None. """
        output.SetFrom(
            context.get_abstract_state(self.state_of_plan_in_memory_idx)[0]
        )

