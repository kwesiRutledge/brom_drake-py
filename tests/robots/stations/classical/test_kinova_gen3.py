"""
Description
-----------
This module tests the Manipulation Station for the Kinova Gen3 robot.
"""

import numpy as np
from pydrake.all import (
    AbstractValue,
    ConstantValueSource,
    ConstantVectorSource,
    DiagramBuilder,
    Simulator,
)
import unittest

# Internal imports
from brom_drake.control import GripperTarget
from brom_drake.robots.gripper_type import GripperType
from brom_drake.stations.classical import KinovaGen3Station, KinovaGen3Variant


class TestKinovaGen3Station(unittest.TestCase):
    def test_init1(self):
        """
        This test verifies that the station contains a 7-DoF, fully actuated arm.
        """
        station = KinovaGen3Station(meshcat_port_number=None)
        station.Finalize()

        self.assertEqual(station.plant.num_positions(station.arm), 7)
        self.assertEqual(station.plant.num_actuated_dofs(station.arm), 7)
        self.assertTrue(station.HasOutputPort("kinova_gen3_arm_position_measured"))
        self.assertTrue(station.HasOutputPort("measured_ee_wrench"))

    def test_init_six_dof1(self):
        """
        This test verifies that the 6-DoF variant of the station contains a
        6-DoF, fully actuated arm.
        """
        station = KinovaGen3Station(
            meshcat_port_number=None,
            variant=KinovaGen3Variant.kSixDof,
        )
        station.Finalize()

        self.assertEqual(station.variant.num_dofs, 6)
        self.assertEqual(station.plant.num_positions(station.arm), 6)
        self.assertEqual(station.plant.num_actuated_dofs(station.arm), 6)
        self.assertEqual(
            station.plant.GetModelInstanceName(station.arm), "kinova_gen3_6dof"
        )

    def test_holds_configuration1(self):
        """
        This test verifies that the station's controller holds the arm
        (with the gripper attached) near a commanded configuration,
        for both variants of the arm.
        """
        for variant, q_des in [
            (
                KinovaGen3Variant.kSevenDof,
                np.array([0.0, 0.5, 0.0, 1.5, 0.0, 1.0, 0.0]),
            ),
            (
                KinovaGen3Variant.kSixDof,
                np.array([0.0, 0.5, 1.5, 0.0, 1.0, 0.0]),
            ),
        ]:
            with self.subTest(variant=variant):
                self.check_holds_configuration(variant, q_des)

    def check_holds_configuration(self, variant: KinovaGen3Variant, q_des):
        """Holds the arm at q_des for 1 second and checks that it stays there."""
        builder = DiagramBuilder()
        station = builder.AddSystem(
            KinovaGen3Station(
                meshcat_port_number=None,
                gripper_type=GripperType.Robotiq_2f_85,
                variant=variant,
            )
        )
        station.Finalize()

        q_source = builder.AddSystem(ConstantVectorSource(q_des))
        builder.Connect(
            q_source.get_output_port(), station.GetInputPort("desired_joint_positions")
        )
        type_source = builder.AddSystem(
            ConstantValueSource(AbstractValue.Make(GripperTarget.kPosition))
        )
        builder.Connect(
            type_source.get_output_port(), station.GetInputPort("gripper_target_type")
        )
        target_source = builder.AddSystem(ConstantVectorSource(np.array([0.0])))
        builder.Connect(
            target_source.get_output_port(), station.GetInputPort("gripper_target")
        )

        diagram = builder.Build()
        diagram_context = diagram.CreateDefaultContext()
        station.plant.SetPositions(
            station.plant.GetMyMutableContextFromRoot(diagram_context),
            station.arm,
            q_des,
        )

        simulator = Simulator(diagram, diagram_context)
        simulator.AdvanceTo(1.0)

        q = station.GetOutputPort("kinova_gen3_arm_position_measured").Eval(
            station.GetMyContextFromRoot(simulator.get_context())
        )
        # The controller's model of the arm includes the gripper,
        # so it compensates for the gripper's weight.
        np.testing.assert_allclose(q, q_des, atol=1e-3)


if __name__ == "__main__":
    unittest.main()
