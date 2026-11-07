from enum import IntEnum
import numpy as np
from pydrake.common.value import AbstractValue
from pydrake.systems.framework import LeafSystem, Context, BasicVector
from pydrake.trajectories import PiecewiseTrajectory, PiecewisePolynomial

# Internal Imports
from brom_drake.motion_planning.systems.state_of_plan_in_memory import (
    StateOfPlanInMemory,
)


class OpenLoopPlanDispenser(LeafSystem):
    """
    **Description**

    This LeafSystem is meant to dispense a plan that the user has given to it
    in an open-loop fashion. (i.e., it will proceed through the plan at a
    constant speed without checking the state of the robot at all).

    **Block Diagram**

    A block diagram of this system is shown below: ::

                            |---------------|
                            |   Open        |
        plan -------->      |   Loop        | --------> point_in_plan
        (np.ndarray[N,n])   |   Plan        |           (np.ndarray[n,])
        plan_ready -------->|   Dispenser   | --------> plan_is_set
        (bool)              |---------------|           (StateOfPlanInMemory)

    where:

    - N is the number of points in the plan
    - n is the number of degrees of freedom in the plan
    - plan is an AbstractValuePort whose value must be an Nxn matrix expressed
      as a numpy array.
    - plan_ready is an AbstractValuePort whose value must be a boolean.
    - point_in_plan is a BasicVectorPort whose value is an n-dimensional vector
        expressed as a numpy array.
    - plan_is_set is an AbstractOutputPort whose value is a StateOfPlanInMemory.

    """

    def __init__(
        self,
        n_dof: int,
        speed: float,  # Speed at which to proceed through points.
    ):
        LeafSystem.__init__(self)

        # Setup
        self.n_dof = n_dof
        self.speed = speed
        self.plan, self.planned_trajectory = None, None

        # Create input port for the plan (we assume that when we receive it, we will close the port)
        self.plan_is_set = False
        sample_plan = np.zeros((0, self.n_dof))
        self.t0, self.t_final = -1.0, -1.0
        self.plan_port = self.DeclareAbstractInputPort(
            "plan",
            AbstractValue.Make(sample_plan),
        )

        self.plan_ready_port = self.DeclareAbstractInputPort(
            "plan_ready",
            AbstractValue.Make(False),
        )

        # Create output ports
        self.DeclareVectorOutputPort(
            "point_in_plan",
            self.n_dof,
            self.GetCurrentPointInPlan,
        )

        self.state_of_plan_in_memory_idx = self.DeclareDiscreteState(
            np.array([StateOfPlanInMemory.kNotSet])
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

    def GetCurrentPointInPlan(self, context: Context, output_point: BasicVector):
        # Setup
        # Unconnected inputs mean "no plan yet" (e.g., a production simulated
        # before a motion planner has been cast). A ready flag without a plan
        # to dispense is also treated as not ready.
        plan_is_connected = self.plan_port.HasValue(context)
        plan = (
            self.plan_port.Eval(context)
            if plan_is_connected
            else np.zeros((0, self.n_dof))
        )
        plan_is_ready = (
            plan_is_connected
            and self.plan_ready_port.HasValue(context)
            and bool(self.plan_ready_port.Eval(context))
        )

        # Get the abstract state
        abstract_state = context.get_discrete_state(self.state_of_plan_in_memory_idx)

        # If plan isn't ready, hold the first configuration of the incoming plan
        # (planners report the configuration to hold while they work). Fall back to
        # zeros only if there is nothing to hold.
        if not plan_is_ready:
            if plan.shape[0] > 0:
                output_point.SetFromVector(np.asarray(plan[0]).flatten())
            else:
                output_point.SetFromVector(np.zeros((self.n_dof,)))
            return

        # If this is the first time that plan_is_ready, then
        # let's save the current time as the time when the plan starts
        if abstract_state[0] == StateOfPlanInMemory.kNotSet:
            # Initialize the system to handle the new plan
            self.initialize_system_for_new_plan(context)

        # Use interpolation to get the current point
        t = context.get_time() - self.t0

        # Mark the plan as set
        context.SetDiscreteState(
            np.array(
                [
                    StateOfPlanInMemory.kSet,
                ]
            )
        )

        # Output The Current Point
        if t < 0.0:
            output_point.SetFromVector(self.planned_trajectory.value(0.0).flatten())
        elif t > self.t_final:
            output_point.SetFromVector(
                self.planned_trajectory.value(self.t_final).flatten()
            )
        else:
            output_point.SetFromVector(self.planned_trajectory.value(t).flatten())

    def find_time_between_two_points(self, x1: np.ndarray, x2: np.ndarray) -> float:
        """
        **Description**

        This method computes the time that it will take to move between two points.
        :return:
        """
        # Setup

        # Compute the distance between the two points
        dist = np.linalg.norm(x1 - x2)
        return dist / self.speed

    def initialize_system_for_new_plan(self, context: Context):
        """
        **Description**

        This function should lock a couple of things into memory for this object to handle future
        calls to get the plan.

        :param context:
        :return:
        """
        # Setup

        # Get time to start executing the plan
        self.t0 = context.get_time()
        context.SetDiscreteState(np.array([StateOfPlanInMemory.kSet]))

        # Get times where we should reach each point in the plan and save it into a new map
        self.plan = self.plan_port.Eval(context)

        # Assign a time to each point, skipping points that repeat the previous
        # one (a zero-length segment would make the times non-increasing).
        points = [self.plan[0, :]]
        times = [0.0]
        for ii in range(1, self.plan.shape[0]):
            dt = self.find_time_between_two_points(points[-1], self.plan[ii, :])
            if dt <= 0.0:
                continue
            points.append(self.plan[ii, :])
            times.append(times[-1] + dt)

        self.t_final = times[-1]
        if len(points) == 1:
            # The plan is a single configuration (e.g., the robot already
            # starts at the goal): hold it.
            self.planned_trajectory = PiecewisePolynomial(
                np.asarray(points[0]).reshape(-1, 1)
            )
        else:
            self.planned_trajectory = PiecewisePolynomial.FirstOrderHold(
                times, np.array(points).T
            )  # Need transpose to make each column a sample

    def GetStateOfPlanInMemory(self, context: Context, output: AbstractValue):
        """Plan is set if and only if the internal variable is not None."""
        output.SetFrom(context.get_abstract_state(self.state_of_plan_in_memory_idx)[0])
