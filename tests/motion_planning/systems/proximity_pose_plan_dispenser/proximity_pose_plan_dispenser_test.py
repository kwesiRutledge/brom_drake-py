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
        nearby_pose_to_plan_pose0 = RigidTransform(
            RollPitchYaw(0.0, 0.0, np.pi/2), [1.0, 0.0, 0.0]
        )
        current_pose_source = builder.AddSystem(
            ConstantValueSource(AbstractValue.Make(nearby_pose_to_plan_pose0))
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

    def create_scenario2(self)->Tuple[
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
        plan_ready_source = builder.AddSystem(
            ConstantValueSource(AbstractValue.Make(P))
        )

        # Create a ProximityPosePlanDispenser object
        dispenser = builder.AddSystem(ProximityPosePlanDispenser())

        # Connect the following sources to the dispenser:
        # - plan_source to plan_port
        # - plan_ready_source to plan_ready_port
        builder.Connect(plan_source.get_output_port(0), dispenser.GetInputPort("plan"))
        builder.Connect(plan_ready_source.get_output_port(0), dispenser.GetInputPort("request"))

        return builder, dispenser

if __name__ == "__main__":
    unittest.main()