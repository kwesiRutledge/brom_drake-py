"""
Description:
    This file contains tests for the IdealJointPositionController class.
"""
# External Imports
from importlib import resources as impresources
import os
import unittest

import numpy as np
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import AffineSystem, ConstantVectorSource

from brom_drake.all import add_watcher_and_build
# Internal Imports
from brom_drake.control import IdealJointPositionController
from brom_drake import robots
from brom_drake.robots import find_base_link_name_in
from brom_drake.urdf import DrakeReadyURDFConverter


class TestIdealJointPositionController(unittest.TestCase):
    def setUp(self):
        # Setup the urdf file, if needed
        self.expected_urdf_location = "brom/models/ur102/ur10e.drake.urdf"
        if not os.path.exists(self.expected_urdf_location):
            arm_urdf_path = str(
                impresources.files(robots) / "models/ur/ur10e.urdf",
            )
            arm_urdf = DrakeReadyURDFConverter(arm_urdf_path).convert_urdf()
            self.expected_urdf_location = str(arm_urdf)

    def test_init1(self):
        """
        Description:
            This test checks if the IdealJointPositionController can be initialized.
        :return:
        """
        # Setup
        arm_urdf = self.expected_urdf_location

        # Add arm_urdf to plant
        plant1 = MultibodyPlant(1e-3)
        arm_model_idx = Parser(plant1).AddModels(arm_urdf)[0]
        plant1.Finalize()

        # Create
        controller = IdealJointPositionController(arm_model_idx, plant1)
        self.assertTrue(True)

    def create_simple_affine_system1(self, builder: DiagramBuilder):
        """
        Description:
            This function creates a simple affine system
            which can drive the 6 dof of the ur10e arm.
        :return:
        """
        # Setup

        # Create a simple input for it (an affine system)
        A = np.zeros((6, 6))
        B = np.zeros((6, 1))
        f0 = np.array([0.0, 0.1, 0.1, 0.0, 0.0, 0.0])
        C = np.eye(6)
        D = np.zeros((6, 1))
        y0 = np.zeros((6, 1))
        x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.2, 0.5])
        target_source2 = builder.AddSystem(
            AffineSystem(A, B, f0, C, D, y0)
        )
        target_source2.configure_default_state(x0)

        # Create a simple input for it (an affine system)
        u0 = np.array([0.2])
        affine_system_input = builder.AddSystem(ConstantVectorSource(u0))

        # Connect input to affine system
        builder.Connect(
            affine_system_input.get_output_port(),
            target_source2.get_input_port()
        )

        return target_source2, affine_system_input


    def test_simple_diagram_simulation1(self):
        """
        Description:
            This test checks if the IdealJointPositionController can be simulated in a simple diagram.
            (In other words, this will make sure that the controller can be connected to a plant.
            and that all ports are running correctly.)
        :return:
        """
        # Setup
        arm_urdf = self.expected_urdf_location
        T_sim1 = 4.0

        # Create diagram builder
        builder = DiagramBuilder()

        # Create plant with just the arm
        plant = MultibodyPlant(1e-3)
        arm_model_idx = Parser(plant).AddModels(arm_urdf)[0]
        robot_base_link_name = find_base_link_name_in(arm_urdf)
        plant.WeldFrames(
            plant.world_frame(),
            plant.GetFrameByName(robot_base_link_name, arm_model_idx),
        )
        plant.Finalize()
        builder.AddSystem(plant)

        # Create controller
        controller = IdealJointPositionController(arm_model_idx, plant)
        builder.AddSystem(controller)

        # Create a simple input for it (an affine system)
        target_source2, affine_system_input = self.create_simple_affine_system1(builder)

        # Connect the controller to the target source
        builder.Connect(
            target_source2.get_output_port(),
            controller.get_input_port(0)
        )

        # Add a watcher and build the diagram
        watcher, diagram, diagram_context = add_watcher_and_build(builder)

        # Simulate
        controller_context = controller.CreateDefaultContext()
        controller_context.SetTime(0.0)

        controller.plant_context = diagram.GetMutableSubsystemContext(
            plant, diagram_context
        )

        simulator = Simulator(diagram, diagram_context)
        simulator.Initialize()
        simulator.AdvanceTo(T_sim1)

        # Test is passed if we can fully simulate the system
        self.assertTrue(True)


if __name__ == '__main__':
    unittest.main()