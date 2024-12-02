import numpy as np
from pydrake.all import (
    AbstractValue, Context,
    LeafSystem,
    PiecewisePolynomial, PiecewiseQuaternionSlerp, PiecewisePose,
    RigidTransform,
)

# Internal Imports
from brom_drake.motion_planning.systems.state_of_plan_in_memory import StateOfPlanInMemory

class ProximityPosePlanDispenser(LeafSystem):
    def __init__(
        self,
    ):
        LeafSystem.__init__(self)

        # Setup

        # Create empty variables for the plan and the planned trajectory
        self.plan, self.planned_trajectory = None, None

        # Create input and output ports for the plan
        self.plan_is_set = False
        self.t0, self.t_final = -1.0, -1.0
        self.plan_port, self.plan_ready_port = None, None
        self.declare_input_ports()
        self.declare_output_ports()

        # Create internal state
        self.state_of_plan_in_memory_idx = self.DeclareDiscreteState(
            np.array([StateOfPlanInMemory.kNotSet])
        )

    def declare_input_ports(self):
        """
        Description:
            This function creates the input ports for the plan dispenser.
        """
        # Setup

        # Define ports
        sample_plan = [RigidTransform()]
        self.plan_port = self.DeclareAbstractInputPort(
            "plan",
            AbstractValue.Make(sample_plan),
        )

        self.plan_ready_port = self.DeclareAbstractInputPort(
            "plan_ready",
            AbstractValue.Make(False),
        )

    def declare_output_ports(self):
        """
        Description:
            This function creates the output ports for the plan dispenser.
        """
        # Setup

        # Define ports
        self.DeclareAbstractOutputPort(
            "pose_in_plan",
            lambda: AbstractValue.Make(RigidTransform()),
            self.GetCurrentPoseInPlan,
        )

        self.DeclareAbstractOutputPort(
            "plan_is_set",
            lambda: AbstractValue.Make(False),
            self.GetStateOfPlanInMemory,
        )

    def GetCurrentPoseInPlan(self, context: Context, output_pose: RigidTransform):
        # Setup
        plan_is_ready = self.plan_ready_port.Eval(context)
        plan = self.plan_port.Eval(context)

        # Get the abstract state
        abstract_state = context.get_discrete_state(self.state_of_plan_in_memory_idx)

        # If plan isn't ready, then skip the rest of the logic
        if not plan_is_ready:
            output_pose.SetFrom(RigidTransform())
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
            output_pose.SetFrom(
                self.planned_trajectory.value(self.t_final)
            )
        else:
            output_pose.SetFrom(
                self.planned_trajectory.value(t)
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

    def GetStateOfPlanInMemory(self, context: Context, output: AbstractValue):
        """Plan is set if and only if the internal variable is not None. """
        output.SetFrom(
            context.get_abstract_state(self.state_of_plan_in_memory_idx)[0]
        )

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
        positions_as_array = np.zeros((0, 3))
        for ii in range(n_points_in_plan-1):
            t_ii += self.find_time_between_two_points(
                self.plan[ii].translation(),
                self.plan[ii+1].translation(),
                )
            
            # Save the times, positions and orientaitons separately
            times += [t_ii]
            positions_as_array = np.vstack(
                (positions_as_array, self.plan[ii].translation()),
            )

        # Save final time, position and orientation
        self.t_final = t_ii
        positions_as_array = np.vstack(
            (positions_as_array, self.plan[-1].translation()),
        )

        # Create the planned trajectory
        position_trajectory = PiecewisePolynomial.FirstOrderHold(
            times,
            [p.translation() for p in self.plan],
        )
        orientation_trajectory = PiecewiseQuaternionSlerp(times, [RigidTransform(p).rotation() for p in self.plan])
        
        self.planned_trajectory = PiecewisePose(
            position_trajectory,
            orientation_trajectory,
        )


