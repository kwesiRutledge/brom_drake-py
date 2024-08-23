from importlib import resources as impresources

from pydrake.geometry import HalfSpace
from pydrake.math import RollPitchYaw, RigidTransform, RotationMatrix
from pydrake.multibody.math import SpatialVelocity
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant, CoulombFriction
from pydrake.systems.framework import Diagram, DiagramBuilder, LeafSystem, PortDataType, BasicVector

from manipulation.scenarios import AddMultibodyTriad

import numpy as np

# Internal Imports
from brom_drake import example_helpers as eh


class BlockHandlerSystem(LeafSystem):
    def __init__(
        self,
        plant: MultibodyPlant,
        scene_graph,
        block_name: str = "block_with_slots",
        model_urdf_path: str = None,
    ):
        LeafSystem.__init__(self)

        # Input Processing
        if model_urdf_path is None:
            model_urdf_path = str(
                impresources.files(eh) / "models/slider-block.urdf",
            )

        # Constants
        self.block_name = block_name

        # Add the Block to the given plant
        self.plant = plant
        self.block_model_idx = Parser(plant=self.plant).AddModels(model_urdf_path)[0]
        self.block_model_name = self.plant.GetModelInstanceName(self.block_model_idx)
        self.block_body_name = "block"

        AddGround(self.plant) #Add ground to plant

        # Add the Triad
        self.scene_graph = scene_graph
        AddMultibodyTriad(plant.GetFrameByName(self.block_body_name), self.scene_graph)

        self.plant.Finalize()
        self.context = self.plant.CreateDefaultContext()

        # Create Input Port for the Slider Block System
        self.desired_pose_port = self.DeclareVectorInputPort(
            "desired_pose",
            BasicVector(6),
        )

        # Create Output Port which should share the pose of the block
        self.DeclareVectorOutputPort(
                "measured_block_pose",
                BasicVector(6),
                self.SetBlockPose,
                {self.time_ticket()}   # indicate that this doesn't depend on any inputs,
                )                      # but should still be updated each timestep

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
        X_WBlock = RigidTransform(R_WBlock, p_WBlock)
        self.plant.SetFreeBodyPose(
            self.plant.GetMyContextFromRoot(diagram_context),
            self.plant.GetBodyByName(self.block_body_name),
            X_WBlock)

        # Set Velocities
        self.plant.SetFreeBodySpatialVelocity(
            self.plant.GetBodyByName(self.block_body_name),
            SpatialVelocity(np.zeros(3),np.array([0.0,0.0,0.0])),
            self.plant.GetMyContextFromRoot(diagram_context))

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