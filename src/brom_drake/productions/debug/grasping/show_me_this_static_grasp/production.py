from typing import List, Tuple, Union
from dataclasses import dataclass
import numpy as np
from pydrake.all import (
    GeometryProperties,
    IllustrationProperties,
    RigidBodyFrame,
    RigidTransform,
    RoleAssign,
    SceneGraph,
    SceneGraphInspector,
)
from pydrake.geometry import Meshcat, MeshcatVisualizer, MeshcatVisualizerParams
from pydrake.geometry import Role as DrakeRole
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant
from pydrake.systems.framework import DiagramBuilder, Diagram, Context

from pydrake.systems.primitives import ConstantVectorSource, VectorLogSink

# Internal Imports
from brom_drake.file_manipulation.urdf.drakeify import drakeify_my_urdf
from brom_drake.robots import find_base_link_name_in
from brom_drake.productions.types.debug import BasicGraspingDebuggingProduction
from brom_drake.productions import ProductionID
from brom_drake.productions.roles.role import Role
from brom_drake.utils import Performer, AddMultibodyTriad
from brom_drake.utils.model_instances import (
    get_name_of_first_body_in_urdf,
    find_number_of_positions_in_welded_model,
)
from brom_drake.productions.debug.show_me.show_me_system import ShowMeSystem
from .config import Configuration as ShowMeThisStaticGraspConfiguration

class ShowMeThisStaticGrasp(BasicGraspingDebuggingProduction):
    def __init__(
        self,
        path_to_object: str,
        path_to_gripper: str,
        X_ObjectGripper: RigidTransform = None,
        gripper_joint_positions: Union[List[float], np.ndarray] = None,
        config: ShowMeThisStaticGraspConfiguration = ShowMeThisStaticGraspConfiguration(),
    ):
        # Call the parent constructor
        super().__init__(
            path_to_object=path_to_object,
            path_to_gripper=path_to_gripper,
            X_ObjectGripper=X_ObjectGripper,
            meshcat_port_number=config.base.meshcat_port_number,
            time_step=config.base.time_step,
            target_body_on_gripper=config.target_body_on_gripper,
            gripper_color=config.gripper_color,
            show_gripper_base_frame=config.show_gripper_base_frame,
            show_collision_geometries=config.show_collision_geometries,
        )

        # This will define the following fields for the production:
        # - path_to_object
        # - path_to_gripper
        # - X_ObjectTarget
        # - meshcat_port_number
        # - time_step
        # - target_body_on_gripper
        # - gripper_color
        # - show_gripper_base_frame
        # - show_collision_geometries
        # - plant

        # Create joint position array
        if gripper_joint_positions is None:
            gripper_joint_positions = [0.0] * find_number_of_positions_in_welded_model(self.path_to_gripper)
        self.gripper_joint_positions = gripper_joint_positions

        # Add Name to plantPlant and Scene Graph for easy simulation
        self.plant.set_name("DemonstrateStaticGrasp_plant")

        # Create show me system for holding object in place
        self.show_me_system = None

        self.config = config

    def add_cast_and_build(
        self,
        cast: Tuple[Role, Performer] = [],
    ) -> Tuple[Diagram, Context]:
        super().add_cast_and_build(cast, with_watcher=True)

        # Assign the diagram context to the internal show_me_system
        self.show_me_system.mutable_plant_context = self.plant.GetMyMutableContextFromRoot(
            self.diagram_context,
        )

        return self.diagram, self.diagram_context

    def add_supporting_cast(self):
        """
        Description
        -----------
        This method will add:
        - The user's object model to the builder.
        - The user's gripper model to the builder.
        - The gripper triad to the builder.
        """
        # Setup
        plant: MultibodyPlant = self.plant

        # Add the object to the builder
        self.add_manipuland_to_plant(and_weld_to=plant.world_frame())

        # Add the gripper to the builder
        X_WorldGripper = self.find_X_WorldGripper(
            X_ObjectGripper=self.X_ObjectGripper,
            target_frame_name=self.target_body_name_on_gripper,
            desired_joint_positions=self.gripper_joint_positions,
        )
        self.add_gripper_to_plant(
            and_weld_to=plant.world_frame(),
            with_X_WorldGripper=X_WorldGripper,
        )
        self.add_gripper_controller_and_connect()

        # Connect the plant to the meshcat, if requested
        if self.meshcat_port_number is not None:
            self.connect_to_meshcat()

        # Finalize the plant
        self.plant.Finalize()

    def add_gripper_controller_and_connect(self):
        # Setup
        plant: MultibodyPlant = self.plant

        # Create a system to control the gripper's actuators
        self.show_me_system = ShowMeSystem(
            plant=plant,
            model_index=self.gripper_model_index,
            desired_joint_positions=self.gripper_joint_positions,
        )
        self.builder.AddSystem(self.show_me_system)

        # Add Command to tell the gripper to stay in a particular state
        desired_joint_positions_source = self.builder.AddSystem(
            ConstantVectorSource(np.array(self.gripper_joint_positions)),
        )
        # Connect the source to the system

        self.builder.Connect(
            desired_joint_positions_source.get_output_port(0),
            self.show_me_system.get_input_port(0),
        )

    def find_X_WorldGripper(
        self,
        X_ObjectGripper: RigidTransform,
        target_frame_name: str,
        desired_joint_positions: List[float],
    ) -> RigidTransform:
        """
        Description
        -----------
        This method will return the transform from the world frame to the gripper frame.
        """
        # Setup
        desired_joint_positions = np.array(desired_joint_positions)

        # Create shadow plant containing just the gripper
        shadow_plant = MultibodyPlant(self.time_step)
        model_idcs = Parser(plant=shadow_plant).AddModels(self.path_to_gripper)
        shadow_model_idx = model_idcs[0]
        
        # Get the first body on the gripper
        name_of_first_body_in_gripper = get_name_of_first_body_in_urdf(self.path_to_gripper)

        # Weld the base link to the world frame
        shadow_plant.WeldFrames(
            shadow_plant.world_frame(),
            shadow_plant.GetFrameByName(name_of_first_body_in_gripper),
        )

        # Finalize Plant
        shadow_plant.Finalize()

        shadow_builder = DiagramBuilder()
        shadow_builder.AddSystem(shadow_plant)
        diagram = shadow_builder.Build()
        shadow_diagram_context = diagram.CreateDefaultContext()

        # Set the joint positions
        shadow_plant.SetPositions(
            shadow_plant.GetMyContextFromRoot(shadow_diagram_context),
            shadow_model_idx,
            desired_joint_positions,
        )

        # Get the transform of the gripper's base in the static world frame
        X_GripperBase_GripperTarget = shadow_plant.EvalBodyPoseInWorld(
            shadow_plant.GetMyContextFromRoot(shadow_diagram_context),
            shadow_plant.GetBodyByName(target_frame_name),
        )

        X_Object_GripperTarget = X_ObjectGripper
        X_WorldObject = RigidTransform.Identity() # Identity

        X_Object_GripperBase = X_Object_GripperTarget.multiply(
            X_GripperBase_GripperTarget.inverse()
        )

        return X_WorldObject.multiply(X_Object_GripperBase)

    @property
    def id(self):
        return ProductionID.kDemonstrateStaticGrasp

    @property
    def suggested_roles(self) -> List[Role]:
        return []