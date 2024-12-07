import numpy as np
from pydrake.all import (
    AbstractValue, BasicVector, Context,
    LeafSystem,
    PiecewisePolynomial, PiecewiseQuaternionSlerp, PiecewisePose,
    RigidTransform,
)

# Internal Imports
from brom_drake.motion_planning.systems.proximity_pose_plan_dispenser import (
    ProximityPosePlanDispenserConfig,
    DispenserTransitionRequest,
)
from brom_drake.motion_planning.systems.proximity_pose_plan_dispenser.dispenser_internal_state import (
    DispenserInternalState, DispenserTransitionRequest,
)


class ProximityPosePlanDispenser(LeafSystem):
    """
    Description
    -----------
    This class is responsible for dispensing the plan that is given to it in a way
    that is consistent with the proximity constraint that is given to it.

    Input Ports
    -----------
    plan: AbstractValue (List[RigidTransform])
        This port is used to receive the plan that the system should follow.
    plan_ready: AbstractValue (bool)
        This port is used to receive a boolean that indicates if the plan is ready.
        Lower this (to false) and then raise it (to true) to trigger the system to
        reset the plan.
    """
    def __init__(
        self,
        config: ProximityPosePlanDispenserConfig = ProximityPosePlanDispenserConfig(),
    ):
        LeafSystem.__init__(self)
        # Setup
        self.config = config

        # Create empty variables for the plan and the planned trajectory
        self.plan = []

        # Create input and output ports for the plan
        self.plan_is_set = False
        self.t0, self.t_final = -1.0, -1.0
        self.plan_port, self.request_port = None, None
        self.declare_input_ports()
        self.declare_output_ports()

        # Create internal state
        self.dispenser_state = self.DeclareDiscreteState(
            np.array([DispenserInternalState.kReady])
        )
        self.dispenser_plan_index = 0

        self.last_pose = RigidTransform()

    def advance_plan_if_necessary(self, context: Context):
        """
        Description
        -----------
        This function is responsible for advancing the plan index if the system
        is ready to do so.
        """
        # Setup
        plan = self.plan_port.Eval(context)
        plan_idx = self.dispenser_plan_index
        current_pose = self.current_pose_port.Eval(context)

        state = context.get_discrete_state(self.dispenser_state)[0]

        # If the plan is not ready, then do nothing
        if state != DispenserInternalState.kPlanSet:
            return
        
        # If the plan is ready, then check to see if we should advance the plan
        should_advance = self.config.in_proximity(
            current_pose,
            plan[plan_idx],
        )

        if should_advance and (plan_idx < len(plan) - 1):
            self.dispenser_plan_index += 1
        else:
            # Otherwise, do nothing
            pass

    def declare_input_ports(self):
        """
        Description:
            This function creates the input ports for the plan dispenser.
        """
        # Setup

        # Define ports
        sample_plan = [RigidTransform()]
        self.current_pose_port = self.DeclareAbstractInputPort(
            "current_pose",
            AbstractValue.Make(RigidTransform()),
        )
        
        self.plan_port = self.DeclareAbstractInputPort(
            "plan",
            AbstractValue.Make(sample_plan),
        )

        self.request_port = self.DeclareVectorInputPort(
            "request",
            1,
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

        self.DeclareVectorOutputPort(
            "dispenser_state",
            size=1,
            calc=self.GetInternalState,
        )

    def GetCurrentPoseInPlan(self, context: Context, output_pose: AbstractValue):
        # Setup
        plan = self.plan_port.Eval(context)

        last_pose = self.last_pose

        # Transition the internal state, if needed
        self.transition_internal_state(context)

        # Get the abstract state
        state = context.get_discrete_state(self.dispenser_state)
        plan_idx = self.dispenser_plan_index

        # If the state of the dispenser is reset, then share
        # the LAST pose.
        dispenser_in_reset = state[0] == DispenserInternalState.kReady
        if dispenser_in_reset:
            output_pose.SetFrom(
                AbstractValue.Make(last_pose)
            )
            return
        
        # If the state of the plan is kPlanSet,
        # then check to see if we should ADVANCE to the next part of the plan
        self.advance_plan_if_necessary(context)
        plan_idx = self.dispenser_plan_index

        # Output The Current Point in the Plan
        output_pose.SetFrom(
            AbstractValue.Make(plan[plan_idx])
        )

    def GetInternalState(self, context: Context, output: BasicVector):
        """
        Description
        -----------
        Plan is set if and only if the internal variable is not None.
        """
        # Setup

        # Set the internal state (if necessary)
        self.transition_internal_state(context)

        # Output the internal state
        output.SetFromVector(
            context.get_discrete_state(self.dispenser_state)
        )

    def transition_internal_state(self, context: Context):
        """
        Description
        -----------
        This function is responsible for transitioning the internal state of the system
        based on the request that is given to it.
        """
        # Setup
        current_state = context.get_discrete_state(self.dispenser_state)[0]

        # Collect the request from the port
        request = self.request_port.Eval(context)

        # Transition the state
        dispenser_is_ready = current_state == DispenserInternalState.kReady
        dispenser_plan_is_set = current_state == DispenserInternalState.kPlanSet

        if (request == DispenserTransitionRequest.kRequestSavePlan) and dispenser_is_ready:
            # Save the plan AND transition to the "Plan Set" state
            current_plan = self.plan_port.Eval(context)
            self.plan = current_plan
            self.dispenser_plan_index = 0

            # Set the internal state
            context.SetDiscreteState(
                np.array([DispenserInternalState.kPlanSet])
            )
        elif (request == DispenserTransitionRequest.kRequestReset) and dispenser_plan_is_set:
            # Prepare the system to receive a new plan
            # self.dispenser_plan_index = 0

            # Set the internal state
            context.SetDiscreteState(
                np.array([DispenserInternalState.kReady])
            )
        else:
            # Otherwise, do nothing 
            pass

    # def initialize_system_for_new_plan(self, context: Context):
    #     """
    #     Description
    #     -----------
    #     This function should lock a couple of things into memory for this object to handle future
    #     calls to get the plan.

    #     :param context:
    #     :return:
    #     """
    #     # Setup

    #     # Get time to start executing the plan
    #     self.t0 = context.get_time()
    #     context.SetDiscreteState(
    #         np.array([DispenserInternalState.kSet])
    #     )

    #     # Get times where we should reach each point in the plan and save it into a new map
    #     self.plan = self.plan_port.Eval(context)

    #     n_points_in_plan = len(self.plan)
    #     t_ii = 0.0
    #     times = [t_ii]
    #     positions_as_array = np.zeros((0, 3))
    #     for ii in range(n_points_in_plan-1):
    #         t_ii += self.find_time_between_two_points(
    #             self.plan[ii].translation(),
    #             self.plan[ii+1].translation(),
    #             )
            
    #         # Save the times, positions and orientaitons separately
    #         times += [t_ii]
    #         positions_as_array = np.vstack(
    #             (positions_as_array, self.plan[ii].translation()),
    #         )

    #     # Save final time, position and orientation
    #     self.t_final = t_ii
    #     positions_as_array = np.vstack(
    #         (positions_as_array, self.plan[-1].translation()),
    #     )

    #     # Create the planned trajectory
    #     position_trajectory = PiecewisePolynomial.FirstOrderHold(
    #         times,
    #         [p.translation() for p in self.plan],
    #     )
    #     orientation_trajectory = PiecewiseQuaternionSlerp(times, [RigidTransform(p).rotation() for p in self.plan])
        
    #     self.planned_trajectory = PiecewisePose(
    #         position_trajectory,
    #         orientation_trajectory,
    #     )


