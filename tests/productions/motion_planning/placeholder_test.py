"""
placeholder_test.py
Description:

    This file is a placeholder for a test that will be written in the future.
"""

import numpy as np
from pydrake.all import Simulator, DiagramBuilder
from pydrake.common.value import AbstractValue
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.primitives import ConstantValueSource, ConstantVectorSource
import unittest

# Internal Imports
from brom_drake.all import add_watcher_and_build
from brom_drake.robots import GripperType
from brom_drake.stations.kinematic import UR10eStation
from brom_drake.control import CartesianArmController, EndEffectorTarget
from brom_drake.control.grippers import GripperController, GripperTarget


class PlaceholderTest(unittest.TestCase):
    def test_scenario_setup1(self):
        # Setup
        builder = DiagramBuilder()
        time_step = 0.005
        gripper_target = GripperTarget.kPosition

        # Create UR10e object
        # station = UR10eStation()
        station = UR10eStation(
            gripper_type=GripperType.Robotiq_2f_85,
            meshcat_port_number=None,  # Use None for CI
        )
        station.ConnectToMeshcatVisualizer()
        station.Finalize()

        builder.AddSystem(station)

        # Create the GripperTarget value
        q_grip_des = np.array([0])  # open at 0, closed at 1
        gripper_target_source = builder.AddSystem(ConstantVectorSource(q_grip_des))

        # Create the GripperTargetType and connect it to the system
        gripper_target_type_source = builder.AddSystem(
            ConstantValueSource(AbstractValue.Make(gripper_target))
        )
        builder.Connect(
            gripper_target_type_source.get_output_port(),
            station.GetInputPort("gripper_target_type"),
        )

        builder.Connect(
            gripper_target_source.get_output_port(),
            station.GetInputPort("gripper_target"),
        )

        # Create the EndEffectorTarget and target value (then connect to the system)
        ee_target_type = EndEffectorTarget.kPose
        ee_target_type_source = builder.AddSystem(
            ConstantValueSource(AbstractValue.Make(ee_target_type))
        )

        ee_target = np.array([0.5, 0.5, 0.5, 0, 0, 0])
        ee_target_source = builder.AddSystem(ConstantVectorSource(ee_target))

        # Connect the end effector target type and target to the system
        # builder.Connect(
        #     ee_target_type_source.get_output_port(),
        #     station.GetInputPort("ee_target_type")
        # )

        builder.Connect(
            ee_target_source.get_output_port(),
            station.GetInputPort("desired_joint_positions"),
        )

        # Build System
        watcher, diagram, diagram_context = add_watcher_and_build(builder)
        # diagram = builder.Build()
        # diagram_context = diagram.CreateDefaultContext()

        # Set up simulation
        simulator = Simulator(diagram, diagram_context)
        simulator.set_target_realtime_rate(1.0)
        simulator.set_publish_every_time_step(False)

        station.UpdateInternalContexts(diagram_context)

        # Run simulation
        print("Initializing simulation...")
        simulator.Initialize()
        print("Simulation initialized.")
        simulator.AdvanceTo(0.5)

        # Tautology
        self.assertTrue(True)


if __name__ == "__main__":
    unittest.main()
