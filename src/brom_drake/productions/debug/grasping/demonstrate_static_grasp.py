from typing import List, Tuple, Union

import numpy as np
from manipulation.scenarios import AddMultibodyTriad
from pydrake.all import (
    RigidBodyFrame,
    RigidTransform
)
from pydrake.geometry import Meshcat, MeshcatVisualizer, MeshcatVisualizerParams
from pydrake.geometry import Role as DrakeRole
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant
from pydrake.systems.framework import DiagramBuilder, Diagram, Context

from pydrake.systems.primitives import ConstantVectorSource, VectorLogSink

# Internal Imports
from brom_drake.robots import find_base_link_name_in
from brom_drake.productions.types import BaseProduction
from brom_drake.productions import ProductionID
from brom_drake.productions.roles.role import Role
from brom_drake.utils import Performer
from brom_drake.productions.debug.show_me.show_me_system import ShowMeSystem

class DemonstrateStaticGrasp(BaseProduction):
    def __init__(
        self,
        path_to_object: str,
        path_to_gripper: str,
        X_ObjectTarget: RigidTransform = None,
        meshcat_port_number: int = 7001, # Usually turn off for CI (i.e., make it None)
        show_collision_geometries: bool = False,
        gripper_joint_positions: Union[List[float], np.ndarray] = None,
        time_step: float = 1e-3,
        target_frame_name_on_gripper: str = None,
        **kwargs,
    ):
        super().__init__(**kwargs)

        # Add the model to the Production
        self.path_to_object = path_to_object
        self.path_to_gripper = path_to_gripper
        self.time_step = time_step
        
        if X_ObjectTarget is None:
            X_ObjectTarget = RigidTransform()
        self.X_ObjectTarget = X_ObjectTarget
        
        self.meshcat_port_number = meshcat_port_number
        self.show_collision_geometries = show_collision_geometries

        # Create joint position array
        if gripper_joint_positions is None:
            gripper_joint_positions = [0.0] * self.find_number_of_positions_in_model()
        self.gripper_joint_positions = gripper_joint_positions

        # Assign the target frame on the gripper to the variable
        if target_frame_name_on_gripper is None:
            target_frame_name_on_gripper = self.get_name_of_first_frame_in_gripper()
        else:
            assert target_frame_name_on_gripper in self.get_all_body_names_in_gripper(), \
                f"Target frame {target_frame_name_on_gripper} not found in gripper model; Valid body names are: {self.get_all_body_names_in_gripper()}."


        self.target_frame_name_on_gripper = target_frame_name_on_gripper

        # Add Plant and Scene Graph for easy simulation
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(
            self.builder,
            time_step=1e-3,
        )
        self.plant.set_name("DemonstrateStaticGrasp_plant")

        self.meshcat = None
        self.manipuland_index, self.manipuland_name = None, None
        self.gripper_model_index, self.gripper_model_name = None, None
        self.show_me_system = None

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

        # Add the object to the builder
        self.add_manipuland_to_plant()

        # Add the gripper to the builder
        self.add_gripper_to_plant()

        # Connect the plant to the meshcat, if requested
        if self.meshcat_port_number is not None:
            self.connect_to_meshcat()

        # Finalize the plant
        self.plant.Finalize()

    def add_manipuland_to_plant(self):
        """
        Description
        -----------
        This method will add the manipuland to the plant
        and then weld it to the origin.
        """
        # Setup
        assert self.manipuland_index is None, \
            "The manipuland index is already set. Please DO NOT add the manipuland to the plant before this function."

        plant : MultibodyPlant = self.plant

        # Add the manipuland to the plant
        temp_idcs = Parser(plant=self.plant).AddModels(
            self.path_to_object,
        )
        assert len(temp_idcs) == 1, f"Only one model should be added; received {len(temp_idcs)}"
        self.manipuland_index = temp_idcs[0]
        self.manipuland_name = self.plant.GetModelInstanceName(self.manipuland_index)

        # Weld the first frame in the model to the origin
        manipuland_body_idcs = plant.GetBodyIndices(self.manipuland_index)
        assert len(manipuland_body_idcs) > 0, \
            f"Expected at least one body in the manipuland; received {len(manipuland_body_idcs)}"
        manipuland_body = plant.get_body(manipuland_body_idcs[0])
        frame0 = manipuland_body.body_frame()
        self.plant.WeldFrames(
            self.plant.world_frame(),
            frame0,
        )

    def add_gripper_to_plant(self):
        """
        Description
        -----------
        This method will add the gripper to the plant
        and then weld it to the origin.
        """
        # Setup
        assert self.gripper_model_index is None, \
            "The gripper model index is already set. Please DO NOT add the gripper to the plant before this function."

        plant : MultibodyPlant = self.plant
        

        # Add the gripper to the plant
        temp_idcs = Parser(plant=self.plant).AddModels(
            self.path_to_gripper,
        )
        assert len(temp_idcs) == 1, f"Only one model should be added; received {len(temp_idcs)}"
        self.gripper_model_index = temp_idcs[0]
        self.gripper_model_name = self.plant.GetModelInstanceName(self.gripper_model_index)

        # Weld the first frame in the model to the origin of the manipuland with the
        # specified transform
        target_frame = plant.GetFrameByName(self.target_frame_name_on_gripper)

        # Add the gripper triad to the builder
        AddMultibodyTriad(
            target_frame,
            self.scene_graph,
            # scale=0.1,
        )

        if self.target_frame_name_on_gripper != self.get_name_of_first_frame_in_gripper():
            AddMultibodyTriad(
                plant.GetFrameByName(self.target_frame_name_on_gripper),
                self.scene_graph,
            )

        # Weld the gripper to the manipuland
        plant.WeldFrames(
            plant.world_frame(),
            target_frame,
            self.find_X_WorldGripper(
                self.X_ObjectTarget,
                self.target_frame_name_on_gripper,
                self.gripper_joint_positions,
            ),
        )
        
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

    def connect_to_meshcat(self):
        """
        Description
        -----------
        This method will connect the plant to the meshcat.

        Assumptions
        -----------
        - meshcat_port_number is a positive integer
        """
        # Setup

        # Create meshcat object
        self.meshcat = Meshcat(port=self.meshcat_port_number)  # Object provides an interface to Meshcat
        m_visualizer = MeshcatVisualizer(
            self.meshcat,
        )
        params = MeshcatVisualizerParams(
            role=DrakeRole.kIllustration,
        )
        if self.show_collision_geometries:
            params = MeshcatVisualizerParams(
                role=DrakeRole.kProximity,
            )

        m_visualizer.AddToBuilder(
            self.builder, self.scene_graph, self.meshcat,
            params=params,
        )

    def find_number_of_positions_in_model(self) -> int:
        """
        Description
        -----------
        This method will return the number of positions in the gripper model
        as defined by the user (through the path_to_gripper).

        Returns
        -------
        int
            The number of positions in the model.
        """
        # Setup

        # Create a shadow plant
        shadow_plant = MultibodyPlant(self.time_step)
        model_idcs = Parser(plant=shadow_plant).AddModels(self.path_to_gripper)
        shadow_model_idx = model_idcs[0]

        # Weld the base link to the world frame
        gripper_bodies_indicies = shadow_plant.GetBodyIndices(shadow_model_idx)
        first_body_in_gripper = shadow_plant.get_body(gripper_bodies_indicies[0])
        shadow_plant.WeldFrames(
            shadow_plant.world_frame(),
            first_body_in_gripper.body_frame(),
        )

        # Finalize the shadow plant
        shadow_plant.Finalize()

        return shadow_plant.num_positions()

    def find_X_WorldGripper(
        self,
        X_ObjectTarget: RigidTransform,
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
        name_of_first_body_in_gripper = self.get_name_of_first_frame_in_gripper()

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
        X_GripperBase_Target = shadow_plant.EvalBodyPoseInWorld(
            shadow_plant.GetMyContextFromRoot(shadow_diagram_context),
            shadow_plant.GetBodyByName(target_frame_name),
        )

        X_ObjectGripperBase = X_ObjectTarget.multiply(
            X_GripperBase_Target.inverse(),
        )

        return X_ObjectGripperBase

    def get_all_body_names_in_gripper(self) -> List[str]:
        """
        Description
        -----------
        This method will return all of the body names in the gripper file.

        Returns
        -------
        List[str]
            The list of body names in the gripper.
        """
        # Setup

        # Create shadow plant
        shadow_plant = MultibodyPlant(self.time_step)
        model_idcs = Parser(plant=shadow_plant).AddModels(self.path_to_gripper)
        shadow_model_idx = model_idcs[0]

        # Finalize the shadow plant
        shadow_plant.Finalize()

        return [body.name() for body in shadow_plant.GetBodies()]

    def get_name_of_first_frame_in_gripper(self) -> str:
        """
        Description
        -----------
        This method will return the first frame found in the gripper file.

        Returns
        -------
        RigidBodyFrame
            The first frame on the gripper.
        """
        # Setup

        # Create shadow plant
        shadow_plant = MultibodyPlant(self.time_step)
        model_idcs = Parser(plant=shadow_plant).AddModels(self.path_to_gripper)
        shadow_model_idx = model_idcs[0]

        # Finalize the shadow plant
        shadow_plant.Finalize()

        # Get the first body on the gripper
        gripper_bodies_indicies = shadow_plant.GetBodyIndices(shadow_model_idx)
        first_body_in_gripper = shadow_plant.get_body(gripper_bodies_indicies[0])

        # Return the body frame
        return first_body_in_gripper.body_frame().name()

    @property
    def id(self):
        return ProductionID.kDemonstrateStaticGrasp

    @property
    def suggested_roles(self) -> List[Role]:
        return []