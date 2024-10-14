"""
Description:
    This file contains tests for the kinematic station of the ur10e robot.
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

# Internal Imports
from brom_drake import robots
from brom_drake.all import add_watcher_and_build
from brom_drake.robots.stations.kinematic import UR10eStation as KinematicUR10eStation

class TestKinematicUR10eStation(unittest.TestCase):
    def test_init1(self):
        """
        Description:
            This test checks if the KinematicUR10eStation can be initialized.
        :return:
        """
        # Setup

        # Create
        station = KinematicUR10eStation()

        self.assertTrue(True)

    def test_Finalize1(self):
        """
        Description:
            This test checks if the KinematicUR10eStation can be finalized without
            throwing any errors.
        :return:
        """
        # Setup

        # Create
        station = KinematicUR10eStation()
        station.Finalize()

        self.assertTrue(True)

    @staticmethod
    def add_simple_affine_system1_to(builder: DiagramBuilder):
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

    def test_in_simulation1(self):
        """
        Description:
            This test checks if the KinematicUR10eStation can be simulated without
            throwing any errors.
        :return:
        """
        # Setup
        builder = DiagramBuilder()
        T_sim1 = 0.1

        # Create Diagram
        station = builder.AddSystem(KinematicUR10eStation())
        station.Finalize()
        source1, input_to_source1 = TestKinematicUR10eStation.add_simple_affine_system1_to(builder)

        # Connect source1 to station
        builder.Connect(
            source1.get_output_port(),
            station.GetInputPort("desired_joint_positions")
        )

        # Add Watcher and Build
        watcher, diagram, diagram_context = add_watcher_and_build(builder)

        # Set up simulation
        simulator = Simulator(diagram, diagram_context)
        simulator.set_target_realtime_rate(0.25)

        station.arm_controller.plant_context = diagram.GetSubsystemContext(
            station.arm_controller.plant, diagram_context,
        )

        # Run simulation
        simulator.Initialize()
        simulator.AdvanceTo(T_sim1)

        self.assertTrue(True)



if __name__ == "__main__":
    unittest.main()