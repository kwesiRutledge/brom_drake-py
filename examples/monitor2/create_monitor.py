import sys

import ipdb
import numpy as np
import matplotlib.pyplot as plt
import typer

# Drake imports
from pydrake.all import (
    RotationMatrix, RigidTransform, CoulombFriction, HalfSpace, RollPitchYaw,
    AddMultibodyPlantSceneGraph, Parser, AddModel,
    MultibodyPlant, DiagramBuilder,
    LeafSystem, BasicVector, LogVectorOutput, SpatialVelocity,
    AffineSystem, ConstantVectorSource,
    DrakeVisualizer, Meshcat, MeshcatVisualizer, Simulator,
)

from manipulation.scenarios import AddMultibodyTriad

from brom_drake.all import add_watcher_and_build


def AddGround(plant):
    """
    Add a flat ground with friction
    """

    # Constants
    transparent_color = np.array([0.5, 0.5, 0.5, 0])
    nontransparent_color = np.array([0.5, 0.5, 0.5, 0.1])

    p_GroundOrigin = [0, 0.0, 0.0]
    R_GroundOrigin = RotationMatrix.MakeXRotation(0.0)
    X_GroundOrigin = RigidTransform(R_GroundOrigin,p_GroundOrigin)

    # Set Up Ground on Plant

    surface_friction = CoulombFriction(
            static_friction = 0.7,
            dynamic_friction = 0.5)
    plant.RegisterCollisionGeometry(
            plant.world_body(),
            X_GroundOrigin,
            HalfSpace(),
            "ground_collision",
            surface_friction)
    plant.RegisterVisualGeometry(
            plant.world_body(),
            X_GroundOrigin,
            HalfSpace(),
            "ground_visual",
            transparent_color)  # transparent

#######################
## Class Definitions ##
#######################

class BlockHandlerSystem(LeafSystem):
    def __init__(self, plant: MultibodyPlant, scene_graph, block_name: str = "block_with_slots"):
        LeafSystem.__init__(self)

        # Constants
        self.block_name = block_name

        # Add the Block to the given plant
        self.plant = plant
        self.block_model_idx = Parser(plant=self.plant).AddModels("./slider-block.urdf")[0]
        self.block_model_name = self.plant.GetModelInstanceName(self.block_model_idx)
        self.block_body_name = "block"

        AddGround(self.plant) #Add ground to plant

        # Add the Triad
        self.scene_graph = scene_graph
        AddMultibodyTriad(plant.GetFrameByName(self.block_body_name), self.scene_graph)

        self.plant.Finalize()
        self.context = self.plant.CreateDefaultContext()

        # Create Input Port for the Slider Block System
        self.desired_pose_port = self.DeclareVectorInputPort("desired_pose",
                                                                BasicVector(6))

        # Create Output Port which should share the pose of the block
        self.DeclareVectorOutputPort(
                "measured_block_pose",
                BasicVector(6),
                self.SetBlockPose,
                {self.time_ticket()}   # indicate that this doesn't depend on any inputs,
                )                      # but should still be updated each timestep

        # Build the diagram

    def SetBlockPose(self, context, output):
        """
        Description:
            This function sets the desired pose of the block.
        """

        # Get Desired Pose from Port
        plant_context = self.context
        pose_as_vec = self.desired_pose_port.Eval(context)

        self.plant.SetFreeBodyPose(
            plant_context,
            self.plant.GetBodyByName(self.block_body_name),
            RigidTransform(RollPitchYaw(pose_as_vec[:3]),pose_as_vec[3:])
        )

        self.plant.SetFreeBodySpatialVelocity(
            self.plant.GetBodyByName(self.block_body_name),
            SpatialVelocity(np.zeros(3),np.array([0.0,0.0,0.0])),
            plant_context
            )

        X_WBlock = self.plant.GetFreeBodyPose(
            plant_context,
            self.plant.GetBodyByName(self.block_body_name)
        )

        pose_as_vector = np.hstack([RollPitchYaw(X_WBlock.rotation()).vector(), X_WBlock.translation()])

        # Create Output
        output.SetFromVector(pose_as_vector)

    def SetInitialBlockState(self,diagram_context):
        """
        Description:
            Sets the initial position to be slightly above the ground (small, positive z value)
            to be .
        """

        # Set Pose
        p_WBlock = [0.0, 0.0, 0.2]
        R_WBlock = RotationMatrix.MakeXRotation(np.pi/2.0) # RotationMatrix.MakeXRotation(-np.pi/2.0)
        X_WBlock = RigidTransform(R_WBlock,p_WBlock)
        print(self.block_model_idx)
        print(self.plant.physical_models())
        self.plant.SetFreeBodyPose(
            self.plant.GetMyContextFromRoot(diagram_context),
            self.plant.GetBodyByName(self.block_body_name),
            X_WBlock)

        # Set Velocities
        self.plant.SetFreeBodySpatialVelocity(
            self.plant.GetBodyByName(self.block_body_name),
            SpatialVelocity(np.zeros(3),np.array([0.0,0.0,0.0])),
            self.plant.GetMyContextFromRoot(diagram_context))

def main(show_plots: bool = True):

    # Building Diagram
    time_step = 0.002

    builder = DiagramBuilder()

    # plant = builder.AddSystem(MultibodyPlant(time_step=time_step)) #Add plant to diagram builder
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-3)
    block_handler_system = builder.AddSystem(BlockHandlerSystem(plant, scene_graph))

    # Connect System To Handler
    # Create system that outputs the slowly updating value of the pose of the block.
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

    command_logger = LogVectorOutput(
        target_source2.get_output_port(),
        builder)
    command_logger.set_name("command_logger")

    # Connect the state of the block to the output of a slowly changing system.
    builder.Connect(
        target_source2.get_output_port(),
        block_handler_system.GetInputPort("desired_pose"))

    u0 = np.array([0.2])
    affine_system_input = builder.AddSystem(ConstantVectorSource(u0))
    builder.Connect(
        affine_system_input.get_output_port(),
        target_source2.get_input_port()
    )

    # Connect to Meshcat
    meshcat0 = Meshcat(port=7001)  # Object provides an interface to Meshcat
    m_visualizer = MeshcatVisualizer(meshcat0)
    m_visualizer.AddToBuilder(builder, scene_graph, meshcat0)

    # Add Watcher and Build
    watcher, diagram, diagram_context = add_watcher_and_build(builder)

    # Set initial pose and vectors
    block_handler_system.SetInitialBlockState(diagram_context)

    # Set up simulation
    simulator = Simulator(diagram, diagram_context)
    block_handler_system.context = block_handler_system.plant.GetMyMutableContextFromRoot(diagram_context)
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(False)

    # Run simulation
    simulator.Initialize()
    simulator.AdvanceTo(15.0)


if __name__ == '__main__':
    with ipdb.launch_ipdb_on_exception():
        typer.run(main)
