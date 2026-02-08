import sys

import ipdb
import numpy as np
import matplotlib.pyplot as plt
import typer

# Drake imports
from pydrake.all import (
    RotationMatrix,
    RigidTransform,
    CoulombFriction,
    HalfSpace,
    RollPitchYaw,
    AddMultibodyPlantSceneGraph,
    Parser,
    AddModel,
    MultibodyPlant,
    DiagramBuilder,
    LeafSystem,
    BasicVector,
    LogVectorOutput,
    SpatialVelocity,
    AffineSystem,
    ConstantVectorSource,
    DrakeVisualizer,
    Meshcat,
    MeshcatVisualizer,
    Simulator,
)

# Internal imports
from brom_drake.utils import AddMultibodyTriad


def AddGround(plant):
    """
    Add a flat ground with friction
    """

    # Constants
    transparent_color = np.array([0.5, 0.5, 0.5, 0])
    nontransparent_color = np.array([0.5, 0.5, 0.5, 0.1])

    p_GroundOrigin = [0, 0.0, 0.0]
    R_GroundOrigin = RotationMatrix.MakeXRotation(0.0)
    X_GroundOrigin = RigidTransform(R_GroundOrigin, p_GroundOrigin)

    # Set Up Ground on Plant

    surface_friction = CoulombFriction(static_friction=0.7, dynamic_friction=0.5)
    plant.RegisterCollisionGeometry(
        plant.world_body(),
        X_GroundOrigin,
        HalfSpace(),
        "ground_collision",
        surface_friction,
    )
    plant.RegisterVisualGeometry(
        plant.world_body(),
        X_GroundOrigin,
        HalfSpace(),
        "ground_visual",
        transparent_color,
    )  # transparent


#######################
## Class Definitions ##
#######################


class BlockHandlerSystem(LeafSystem):
    def __init__(
        self, plant: MultibodyPlant, scene_graph, block_name: str = "block_with_slots"
    ):
        LeafSystem.__init__(self)

        # Constants
        self.block_name = block_name

        # Add the Block to the given plant
        self.plant = plant
        self.block_model_idx = Parser(plant=self.plant).AddModels(
            "./slider-block.urdf"
        )[0]
        self.block_model_name = self.plant.GetModelInstanceName(self.block_model_idx)
        self.block_body_name = "block"

        AddGround(self.plant)  # Add ground to plant

        # Add the Triad
        self.scene_graph = scene_graph
        AddMultibodyTriad(plant.GetFrameByName(self.block_body_name), self.scene_graph)

        self.plant.Finalize()
        self.context = self.plant.CreateDefaultContext()

        # Create Input Port for the Slider Block System
        self.desired_pose_port = self.DeclareVectorInputPort(
            "desired_pose", BasicVector(6)
        )

        # Create Output Port which should share the pose of the block
        self.DeclareVectorOutputPort(
            "measured_block_pose",
            BasicVector(6),
            self.SetBlockPose,
            {self.time_ticket()},  # indicate that this doesn't depend on any inputs,
        )  # but should still be updated each timestep

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
            RigidTransform(RollPitchYaw(pose_as_vec[:3]), pose_as_vec[3:]),
        )

        self.plant.SetFreeBodySpatialVelocity(
            self.plant.GetBodyByName(self.block_body_name),
            SpatialVelocity(np.zeros(3), np.array([0.0, 0.0, 0.0])),
            plant_context,
        )

        X_WBlock = self.plant.GetFreeBodyPose(
            plant_context, self.plant.GetBodyByName(self.block_body_name)
        )

        pose_as_vector = np.hstack(
            [RollPitchYaw(X_WBlock.rotation()).vector(), X_WBlock.translation()]
        )

        # Create Output
        output.SetFromVector(pose_as_vector)

    def SetInitialBlockState(self, diagram_context):
        """
        Description:
            Sets the initial position to be slightly above the ground (small, positive z value)
            to be .
        """

        # Set Pose
        p_WBlock = [0.0, 0.0, 0.2]
        R_WBlock = RotationMatrix.MakeXRotation(
            np.pi / 2.0
        )  # RotationMatrix.MakeXRotation(-np.pi/2.0)
        X_WBlock = RigidTransform(R_WBlock, p_WBlock)
        # print(self.block_model_idx)
        # print(self.plant.physical_models())
        self.plant.SetFreeBodyPose(
            self.plant.GetMyContextFromRoot(diagram_context),
            self.plant.GetBodyByName(self.block_body_name),
            X_WBlock,
        )

        # Set Velocities
        self.plant.SetFreeBodySpatialVelocity(
            self.plant.GetBodyByName(self.block_body_name),
            SpatialVelocity(np.zeros(3), np.array([0.0, 0.0, 0.0])),
            self.plant.GetMyContextFromRoot(diagram_context),
        )


def main(show_plots: bool = True):

    # Building Diagram
    time_step = 0.002

    builder = DiagramBuilder()

    # plant = builder.AddSystem(MultibodyPlant(time_step=time_step)) #Add plant to diagram builder
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-3)
    block_handler_system = builder.AddSystem(BlockHandlerSystem(plant, scene_graph))

    # Connect Handler to Logger
    # state_logger = LogVectorOutput(plant.get_body_spatial_velocities_output_port(), builder)
    state_logger = LogVectorOutput(
        block_handler_system.GetOutputPort("measured_block_pose"), builder
    )
    state_logger.set_name("state_logger")

    # Connect System To Handler
    # Create system that outputs the slowly updating value of the pose of the block.
    A = np.zeros((6, 6))
    B = np.zeros((6, 1))
    f0 = np.array([0.0, 0.1, 0.1, 0.0, 0.0, 0.0])
    C = np.eye(6)
    D = np.zeros((6, 1))
    y0 = np.zeros((6, 1))
    x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.2, 0.5])
    target_source2 = builder.AddSystem(AffineSystem(A, B, f0, C, D, y0))
    target_source2.configure_default_state(x0)

    command_logger = LogVectorOutput(target_source2.get_output_port(), builder)
    command_logger.set_name("command_logger")

    # Connect the state of the block to the output of a slowly changing system.
    builder.Connect(
        target_source2.get_output_port(),
        block_handler_system.GetInputPort("desired_pose"),
    )

    u0 = np.array([0.2])
    affine_system_input = builder.AddSystem(ConstantVectorSource(u0))
    builder.Connect(
        affine_system_input.get_output_port(), target_source2.get_input_port()
    )

    # Connect to Meshcat
    meshcat0 = Meshcat(port=7001)  # Object provides an interface to Meshcat
    mCpp = MeshcatVisualizer(meshcat0)
    mCpp.AddToBuilder(builder, scene_graph, meshcat0)

    diagram = builder.Build()

    # builder.Connect(
    #     plant.get_state_output_port(block),
    #     demux.get_input_port(0))

    # Weld robot to table, with translation in x, y and z
    # p_PlaceOnTable0 = [0.15,0.75,-0.20]
    # R_PlaceOnTableO = RotationMatrix.MakeXRotation(-np.pi/2.0)
    # X_TableRobot = RigidTransform(R_PlaceOnTableO,p_PlaceOnTable0)
    # plant.WeldFrames(
    #     plant.GetFrameByName("simpleDesk"),plant.GetFrameByName("base_link"),X_TableRobot)

    # plant.Finalize()
    # # Draw the frames
    # for body_name in ["base_link", "shoulder_link", "bicep_link", "forearm_link", "spherical_wrist_1_link", "spherical_wrist_2_link", "bracelet_with_vision_link", "end_effector_link"]:
    #     AddMultibodyTriad(plant.GetFrameByName(body_name), scene_graph)

    # diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()

    # Set initial pose and vectors
    block_handler_system.SetInitialBlockState(diagram_context)

    # meshcat.load()
    # diagram.Publish(diagram_context)

    # Set up simulation
    simulator = Simulator(diagram, diagram_context)
    block_handler_system.context = (
        block_handler_system.plant.GetMyMutableContextFromRoot(diagram_context)
    )
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(False)

    # Run simulation
    simulator.Initialize()
    simulator.AdvanceTo(15.0)

    # Collect Data
    state_log = state_logger.FindLog(diagram_context)
    log_times = state_log.sample_times()
    state_data = state_log.data()
    print(state_data.shape)

    command_log = command_logger.FindLog(diagram_context)
    log_times_c = command_log.sample_times()
    command_data = command_log.data()
    print(command_data.shape)

    if show_plots:

        # Plot Data - First Half
        fig = plt.figure()
        ax_list1 = []

        for plt_index1 in range(6):
            ax_list1.append(fig.add_subplot(231 + plt_index1))
            plt.plot(log_times, state_data[plt_index1, :])
            plt.title("State #" + str(plt_index1))

        # Plot Data - Second Half
        fig = plt.figure()
        ax_list2 = []

        for plt_index2 in range(6):
            ax_list2.append(fig.add_subplot(231 + plt_index2))
            plt.plot(log_times_c, command_data[plt_index2, :])
            plt.title("Command #" + str(plt_index2))

        # fig = plt.figure()
        # plt.plot(log_times,state_data[-1,:])

        plt.show()


if __name__ == "__main__":
    with ipdb.launch_ipdb_on_exception():
        typer.run(main)
