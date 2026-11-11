import unittest

import numpy as np
from pydrake.all import (
    AbstractValue,
    ConstantValueSource,
    DiagramBuilder,
    LeafSystem,
    ModelInstanceIndex,
    RigidTransform,
)

from brom_drake.productions.roles.all import kGraspSelector, PairingType


class FakeGraspSelector(LeafSystem):
    """
    A performer with the ports of the grasp selector role.
    """

    def __init__(self, with_ready_port: bool = True, with_command_port: bool = True):
        LeafSystem.__init__(self)
        self.DeclareAbstractInputPort(
            "target_object_model_index",
            AbstractValue.Make(ModelInstanceIndex(0)),
        )
        self.DeclareAbstractOutputPort(
            "X_ObjectGripper_trajectory",
            lambda: AbstractValue.Make([RigidTransform()]),
            lambda context, output: output.set_value([RigidTransform()]),
        )
        if with_command_port:
            self.DeclareVectorOutputPort(
                "grasp_gripper_command",
                1,
                lambda context, output: output.SetFromVector(np.array([0.7])),
            )
        if with_ready_port:
            self.DeclareAbstractOutputPort(
                "grasp_is_ready",
                lambda: AbstractValue.Make(False),
                lambda context, output: output.set_value(True),
            )


class FakeGraspConsumer(LeafSystem):
    """
    A system that receives the outputs of the grasp selector role.
    """

    def __init__(self):
        LeafSystem.__init__(self)
        self.DeclareAbstractInputPort(
            "X_ObjectGripper_trajectory",
            AbstractValue.Make([RigidTransform()]),
        )
        self.DeclareVectorInputPort("grasp_gripper_command", 1)
        self.DeclareAbstractInputPort("grasp_ready", AbstractValue.Make(False))


class GraspSelectorTest(unittest.TestCase):
    def create_builder_with_external_systems(self) -> DiagramBuilder:
        """
        Creates a builder containing the systems that the role connects a performer to.
        """
        builder = DiagramBuilder()

        builder.AddNamedSystem(
            "target_object_model_index",
            ConstantValueSource(AbstractValue.Make(ModelInstanceIndex(2))),
        )
        builder.AddNamedSystem("grasp_consumer", FakeGraspConsumer())
        return builder

    def test_port_assignments1(self):
        """
        Description
        -----------
        This test verifies that the role defines the expected
        performer ports, and which of them are required.
        """
        # Setup
        assignments = {
            assignment.performer_port_name: assignment
            for assignment in kGraspSelector.port_assignments
        }

        # Verify
        self.assertEqual(
            assignments.keys(),
            {
                "target_object_model_index",
                "X_ObjectGripper_trajectory",
                "grasp_gripper_command",
                "grasp_is_ready",
            },
        )
        self.assertEqual(
            assignments["target_object_model_index"].pairing_type,
            PairingType.kInput,
        )
        self.assertEqual(
            assignments["grasp_gripper_command"].pairing_type,
            PairingType.kOutput,
        )
        self.assertTrue(assignments["grasp_gripper_command"].is_required)
        self.assertFalse(assignments["grasp_is_ready"].is_required)

    def test_connect_performer_ports_to1(self):
        """
        Description
        -----------
        This test verifies that a performer with all of the role's
        ports can be connected and that the diagram passes its outputs along.
        """
        # Setup
        builder = self.create_builder_with_external_systems()
        performer = FakeGraspSelector()

        # Connect + Build
        kGraspSelector.connect_performer_ports_to(builder, performer)
        diagram = builder.Build()
        context = diagram.CreateDefaultContext()

        # Verify
        consumer = diagram.GetSubsystemByName("grasp_consumer")
        consumer_context = consumer.GetMyContextFromRoot(context)
        self.assertEqual(
            len(
                consumer.GetInputPort("X_ObjectGripper_trajectory").Eval(
                    consumer_context
                )
            ),
            1,
        )
        self.assertEqual(
            consumer.GetInputPort("grasp_gripper_command").Eval(consumer_context)[0],
            0.7,
        )
        self.assertTrue(consumer.GetInputPort("grasp_ready").Eval(consumer_context))

    def test_connect_performer_ports_to2(self):
        """
        Description
        -----------
        This test verifies that a performer can omit the optional ready port.
        """
        # Setup
        builder = self.create_builder_with_external_systems()
        performer = FakeGraspSelector(with_ready_port=False)

        # Connect
        kGraspSelector.connect_performer_ports_to(builder, performer)

        # Verify
        self.assertFalse(
            builder.IsConnectedOrExported(
                builder.GetSubsystemByName("grasp_consumer").GetInputPort("grasp_ready")
            )
        )

    def test_connect_performer_ports_to3(self):
        """
        Description
        -----------
        This test verifies that a performer which
        does not output a gripper command is rejected.
        """
        # Setup
        builder = self.create_builder_with_external_systems()
        performer = FakeGraspSelector(with_command_port=False)

        # Verify
        with self.assertRaisesRegex(ValueError, "grasp_gripper_command"):
            kGraspSelector.connect_performer_ports_to(builder, performer)


if __name__ == "__main__":
    unittest.main()
