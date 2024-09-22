"""
motion_planning1.py
Description:

    In this script, we create a UR10e robot and place it in an empty environment that we'll use for motion_planning
    work.
"""
import ipdb
import numpy as np
from pydrake.all import Simulator, DiagramBuilder
from pydrake.common.value import AbstractValue
from pydrake.geometry import Meshcat, MeshcatVisualizer
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.primitives import ConstantValueSource, ConstantVectorSource
import typer

# Internal Imports
from brom_drake.all import add_watcher_and_build
from brom_drake.robots import UR10eStation, GripperType
from brom_drake.control import GripperController, GripperTarget
from brom_drake.control.arms import (
    CartesianArmController, EndEffectorTarget,
    ArmControlMode, JointTarget,
)


def main():
    # Setup
    builder = DiagramBuilder()
    time_step = 1e-5
    gripper_target = GripperTarget.kPosition

    # Create UR10e object
    # station = UR10eStation()
    station = UR10eStation(
        time_step=time_step,
        gripper_type=GripperType.Robotiq_2f_85,
        control_mode=ArmControlMode.kJoint,
    )
    station.ConnectToMeshcatVisualizer()
    station.Finalize()

    builder.AddSystem(station)

    # Create the GripperTarget value
    q_grip_des = np.array([0])  # open at 0, closed at 1
    gripper_target_source = builder.AddSystem(ConstantVectorSource(q_grip_des))

    # Create the GripperTargetType and connect it to the system
    gripper_target_type_source = builder.AddSystem(
        ConstantValueSource(
            AbstractValue.Make(gripper_target)
        )
    )
    builder.Connect(
        gripper_target_type_source.get_output_port(),
        station.GetInputPort("gripper_target_type"))

    builder.Connect(
        gripper_target_source.get_output_port(),
        station.GetInputPort("gripper_target"),
    )

    # Create the JointTarget and target value (then connect to the system)
    joint_target_type = JointTarget.kPosition
    joint_target_type_source = builder.AddSystem(
        ConstantValueSource(
            AbstractValue.Make(joint_target_type)
        )
    )

    joint_target = np.array([0.0, 0.1, 0.0, 0, 0, 0])
    joint_target_source = builder.AddSystem(
        ConstantVectorSource(joint_target)
    )

    # Connect the end effector target type and target to the system
    builder.Connect(
        joint_target_type_source.get_output_port(),
        station.GetInputPort("joint_target_type")
    )

    builder.Connect(
        joint_target_source.get_output_port(),
        station.GetInputPort("joint_target")
    )

    # Build System
    watcher, diagram, diagram_context = add_watcher_and_build(builder)
    # diagram = builder.Build()
    # diagram_context = diagram.CreateDefaultContext()

    # Set up simulation
    simulator = Simulator(diagram, diagram_context)
    simulator.set_target_realtime_rate(0.25)
    simulator.set_publish_every_time_step(False)

    # Run simulation
    print("Initializing simulation...")
    simulator.Initialize()
    print("Simulation initialized.")
    simulator.AdvanceTo(5.0)


if __name__ == "__main__":
    with ipdb.launch_ipdb_on_exception():
        typer.run(main)
