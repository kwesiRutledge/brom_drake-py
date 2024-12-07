"""
proximity_pose_plan_dispenser_test.py: Tests for the proximity_pose_plan_dispenser module.
"""
import numpy as np
from pydrake.all import (
    AbstractValue,
    Diagram, DiagramBuilder,
    ConstantVectorSource, ConstantValueSource,
    RigidTransform,
    RollPitchYaw, RotationMatrix,
)
from typing import Tuple
import unittest

# Internal Imports
from brom_drake.all import (
    add_watcher_and_build,
)
from brom_drake.motion_planning.systems.proximity_pose_plan_dispenser import (
    ProximityPosePlanDispenser,
    DispenserTransitionRequest,
    DispenserInternalState,
)

class TestProximityPosePlanDispenser(unittest.TestCase):
    def test_init1(self):
        """
        Description:
            This test verifies that we can create a ProximityPosePlanDispenser object.
        """
        # Setup
        dispenser = ProximityPosePlanDispenser()

        self.assertTrue(True)
        
    def create_scenario1(self)->Tuple[
        ProximityPosePlanDispenser,
        Diagram,
    ]:
        """
        Description:
            This function creates a scenario for testing the ProximityPosePlanDispenser class.
        """
        # Setup
        builder = DiagramBuilder()

        # Create a plan composed of 4 RigidTransforms
        plan = [
            RigidTransform(),
            RigidTransform(RollPitchYaw(0.0, 0.0, np.pi/2), [1.0, 0.0, 0.0]),
            RigidTransform(RollPitchYaw(0.0, 0.0, np.pi), [1.0, 1.0, 0.0]),
            RigidTransform(RollPitchYaw(0.0, 0.0, 3*np.pi/2), [0.0, 1.0, 0.0]),
        ]

        # Create a source to share the plan
        plan_source = builder.AddSystem(
            ConstantValueSource(AbstractValue.Make(plan))
        )

        # Create a source to share the plan_ready signal
        dispenser_request_source = builder.AddSystem(
            ConstantVectorSource(np.array(
                [DispenserTransitionRequest.kNone],
            ))
        )

        # Create a ProximityPosePlanDispenser object
        dispenser = builder.AddSystem(ProximityPosePlanDispenser())

        # Connect the following sources to the dispenser:
        # - plan_source to plan_port
        # - plan_ready_source to plan_ready_port
        builder.Connect(plan_source.get_output_port(0), dispenser.GetInputPort("plan"))
        builder.Connect(dispenser_request_source.get_output_port(0), dispenser.GetInputPort("request"))

        return builder, dispenser, plan

    def test_advance_plan_if_necessary1(self):
        """
        Description:
            This test verifies that the advance_plan_if_necessary function works as expected.
            We will check that the plan index is advanced when the current pose is within the 
            proximity limit of the plan.
        """
        # Setup
        builder, dispenser, plan = self.create_scenario1()

        # Create the current pose source
        nearby_pose_to_plan_pose0 = RigidTransform()
        current_pose_source = builder.AddSystem(
            ConstantValueSource(AbstractValue.Make(nearby_pose_to_plan_pose0))
        )

        # Connect the current pose source to the dispenser
        builder.Connect(current_pose_source.get_output_port(0), dispenser.GetInputPort("current_pose"))

        # Build the diagram
        watcher, diagram, diagram_context = add_watcher_and_build(builder)

        # Check that the initial plan index is 0
        self.assertEqual(dispenser.dispenser_plan_index, 0)

        # Set the plan in the dispenser and change internal state to planset
        dispenser_context = diagram.GetMutableSubsystemContext(dispenser, diagram_context)
        dispenser_context.SetDiscreteState(
            np.array([DispenserInternalState.kPlanSet])
        )
        dispenser.plan = plan

        # Call advance_plan_if_necessary
        dispenser.advance_plan_if_necessary(
            diagram.GetMutableSubsystemContext(dispenser, diagram_context)
        )

        # Check that the plan index is now 1
        self.assertEqual(dispenser.dispenser_plan_index, 1)

    def test_advance_plan_if_necessary2(self):
        """
        Description:
            This test verifies that the advance_plan_if_necessary function works as expected.
            We will check that the plan index is NOT advanced when the current pose is within the 
            proximity limit of the plan.
        """
        # Setup
        builder, dispenser, plan = self.create_scenario1()

        # Create the current pose source
        further_pose_to_plan_pose0 = RigidTransform(
            RollPitchYaw(0.0, 0.0, np.pi/2), [1.0, 0.0, 0.0]
        )
        current_pose_source = builder.AddSystem(
            ConstantValueSource(AbstractValue.Make(further_pose_to_plan_pose0))
        )

        # Connect the current pose source to the dispenser
        builder.Connect(current_pose_source.get_output_port(0), dispenser.GetInputPort("current_pose"))

        # Build the diagram
        diagram = builder.Build()
        diagram_context = diagram.CreateDefaultContext()

        # Check that the initial plan index is 0
        self.assertEqual(dispenser.dispenser_plan_index, 0)

        # Set the plan in the dispenser and change internal state to planset
        dispenser_context = diagram.GetMutableSubsystemContext(dispenser, diagram_context)
        dispenser_context.SetDiscreteState(
            np.array([DispenserInternalState.kPlanSet])
        )
        dispenser.plan = plan

        # Call advance_plan_if_necessary
        dispenser.advance_plan_if_necessary(
            diagram.GetMutableSubsystemContext(dispenser, diagram_context)
        )

        # Check that the plan index is now 1
        self.assertEqual(dispenser.dispenser_plan_index, 0)

    def create_scenario2(
        self,
        request: DispenserTransitionRequest = DispenserTransitionRequest.kNone,
    )->Tuple[ProximityPosePlanDispenser,Diagram]:
        """
        Description:
            This function creates a scenario for testing the ProximityPosePlanDispenser class.
        """
        # Setup
        builder = DiagramBuilder()

        # Create a plan composed of 4 RigidTransforms
        plan = [
            RigidTransform(RollPitchYaw(0.0, np.pi/2., 0.0), [0.0, 0.0, 0.2]),
            RigidTransform(RollPitchYaw(0.0, 0.0, np.pi/2), [1.0, 0.0, 0.0]),
            RigidTransform(RollPitchYaw(0.0, 0.0, np.pi), [1.0, 1.0, 0.0]),
            RigidTransform(RollPitchYaw(0.0, 0.0, 3*np.pi/2), [0.0, 1.0, 0.0]),
        ]

        # Create a source to share the plan
        plan_source = builder.AddSystem(
            ConstantValueSource(AbstractValue.Make(plan))
        )

        dispenser_request_source = builder.AddSystem(
            ConstantVectorSource(np.array(
                [request],
            ))
        )

        # Create a ProximityPosePlanDispenser object
        dispenser = builder.AddSystem(ProximityPosePlanDispenser())

        # Connect the following sources to the dispenser:
        # - plan_source to plan_port
        # - plan_ready_source to plan_ready_port
        builder.Connect(plan_source.get_output_port(0), dispenser.GetInputPort("plan"))
        builder.Connect(dispenser_request_source.get_output_port(0), dispenser.GetInputPort("request"))

        return builder, dispenser, plan

    def test_GetCurrentPoseInPlan1(self):
        """
        Description
        -----------
        This test verifies that the GetCurrentPoseInPlan function works as expected.
        For scenario2, it should return a pose that is the same as the first pose in the plan.
        """
        # Setup
        builder, dispenser, plan = self.create_scenario2()

        # Create the current pose source
        far_pose0 = RigidTransform(
            RollPitchYaw(0.0, 0.0, np.pi/2), [1.0, 0.0, 0.0]
        )
        current_pose_source = builder.AddSystem(
            ConstantValueSource(AbstractValue.Make(far_pose0))
        )

        # Connect the current pose source to the dispenser
        builder.Connect(current_pose_source.get_output_port(0), dispenser.GetInputPort("current_pose"))

        # Build the diagram
        diagram = builder.Build()
        diagram_context = diagram.CreateDefaultContext()

        # Set the plan in the dispenser and change internal state to planset
        dispenser_context = diagram.GetMutableSubsystemContext(dispenser, diagram_context)
        dispenser_context.SetDiscreteState(
            np.array([DispenserInternalState.kPlanSet])
        )
        dispenser.plan = plan

        # Call GetCurrentPoseInPlan
        pose_out = AbstractValue.Make(RigidTransform())
        dispenser.GetCurrentPoseInPlan(dispenser_context, pose_out)

        pose_out = pose_out.get_value()

        # Check that the pose is the same as the first pose in the plan
        self.assertTrue(pose_out.IsExactlyEqualTo(dispenser.plan[0]))

    def test_GetCurrentPoseInPlan2(self):
        """
        Description
        -----------
        This test verifies that when the pose plan dispenser is in the kReady state, it will
        return the LAST pose in the plan.
        For an initially empty plan, the last pose should be the identity pose.
        """
        # Setup
        builder, dispenser, plan = self.create_scenario2()

        # Create the current pose source
        far_pose0 = RigidTransform(
            RollPitchYaw(0.0, 0.0, np.pi/2), [1.0, 0.0, 0.0]
        )
        current_pose_source = builder.AddSystem(
            ConstantValueSource(AbstractValue.Make(far_pose0))
        )

        # Connect the current pose source to the dispenser
        builder.Connect(current_pose_source.get_output_port(0), dispenser.GetInputPort("current_pose"))

        # Build the diagram
        diagram = builder.Build()
        diagram_context = diagram.CreateDefaultContext()

        # Set the plan in the dispenser and change internal state to planset
        dispenser_context = diagram.GetMutableSubsystemContext(dispenser, diagram_context)
        dispenser.plan = plan

        # Call GetCurrentPoseInPlan
        pose_out = AbstractValue.Make(RigidTransform())
        dispenser.GetCurrentPoseInPlan(dispenser_context, pose_out)

        pose_out = pose_out.get_value()

        # Check that the pose is the same as the first pose in the plan
        self.assertTrue(pose_out.IsExactlyEqualTo(RigidTransform()))

    # TODO(kwesi): Implement this test when we have a way to work through a full plan

    # def test_GetCurrentPoseInPlan3(self):
    #     """
    #     Description
    #     -----------
    #     This test verifies that when the pose plan dispenser is in the kReady state, it will
    #     return the LAST pose in memory.
    #     For a dispenser that previously had a plan, the last pose should be the last pose in the plan.
    #     """
    #     # TODO(kwesi): Implement this test when we have a way to work through a full plan
    #     # (Maybe using open loop dispenser?).
    #     pass

    def test_transition_internal_state1(self):
        """
        Description
        -----------
        This test verifies that if we receive a DispenserTransitionRequest.kRequestSavePlan request,
        and the dispenser is in the kReady state, the plan will be saved and the internal state
        will be transitioned to kPlanSet.
        """
        # Setup
        builder, dispenser, plan = self.create_scenario2(request=DispenserTransitionRequest.kRequestSavePlan)

        # Build the diagram
        diagram = builder.Build()
        diagram_context = diagram.CreateDefaultContext()

        # Set the internal state to kReady
        dispenser_context = diagram.GetMutableSubsystemContext(dispenser, diagram_context)
        dispenser_context.SetDiscreteState(
            np.array([DispenserInternalState.kReady])
        )

        # Set the plan in the dispenser
        dispenser.plan = plan

        # Call transition_internal_state
        dispenser.transition_internal_state(dispenser_context)

        # Check that the internal state is now kPlanSet
        self.assertEqual(
            dispenser_context.get_discrete_state(dispenser.dispenser_state)[0],
            DispenserInternalState.kPlanSet,
            )
        
    def test_transition_internal_state2(self):
        """
        Description
        -----------
        This test verifies that if we receive a DispenserTransitionRequest.kRequestReset request,
        and the dispenser is in the kPlanSet state, the internal state will be transitioned to kReady.
        """
        # Setup
        builder, dispenser, plan = self.create_scenario2(request=DispenserTransitionRequest.kRequestReset)

        # Build the diagram
        diagram = builder.Build()
        diagram_context = diagram.CreateDefaultContext()

        # Set the internal state to kPlanSet
        dispenser_context = diagram.GetMutableSubsystemContext(dispenser, diagram_context)
        dispenser_context.SetDiscreteState(
            np.array([DispenserInternalState.kPlanSet])
        )

        # Set the plan in the dispenser
        dispenser.plan = plan

        # Call transition_internal_state
        dispenser.transition_internal_state(dispenser_context)

        # Check that the internal state is now kReady
        self.assertEqual(
            dispenser_context.get_discrete_state(dispenser.dispenser_state)[0],
            DispenserInternalState.kReady,
        )

    def test_transition_internal_state3(self):
        """
        Description
        -----------
        This test verifies that if we receive a DispenserTransitionRequest.kRequestSavePlan request,
        and the dispenser is in the kPlanSet state, the internal state will NOT be transitioned.
        """
        # Setup
        builder, dispenser, plan = self.create_scenario2(request=DispenserTransitionRequest.kRequestSavePlan)

        # Build the diagram
        diagram = builder.Build()
        diagram_context = diagram.CreateDefaultContext()

        # Set the internal state to kPlanSet
        dispenser_context = diagram.GetMutableSubsystemContext(dispenser, diagram_context)
        dispenser_context.SetDiscreteState(
            np.array([DispenserInternalState.kPlanSet])
        )

        # Set the plan in the dispenser
        dispenser.plan = plan

        # Call transition_internal_state
        dispenser.transition_internal_state(dispenser_context)

        # Check that the internal state is still kPlanSet
        self.assertEqual(
            dispenser_context.get_discrete_state(dispenser.dispenser_state)[0],
            DispenserInternalState.kPlanSet,
        )

if __name__ == "__main__":
    unittest.main()