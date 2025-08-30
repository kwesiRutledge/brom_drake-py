from dataclasses import dataclass
import numpy as np
from pydrake.all import (
    Frame,
    LeafSystem,
    ModelInstanceIndex,
    MultibodyPlant,
    Parser,
    PrismaticJoint,
    RevoluteJoint,
    Joint,
    JointActuator,
)
from typing import List, Tuple

# Internal Imports
from .configuration import Configuration as PuppetmakerConfiguration
from brom_drake.file_manipulation.urdf import SimpleShapeURDFDefinition
from brom_drake.file_manipulation.urdf.shapes import SphereDefinition

@dataclass
class PuppetSignature:
    name: str
    model_instance_index: ModelInstanceIndex
    prismatic_joints: List[Joint]
    prismatic_joint_actuators: List[JointActuator]
    revolute_joints: List[Joint]
    revolute_joint_actuators: List[JointActuator]

class Puppetmaker:
    """
    Description
    -----------
    This system is used to actuate a free body that has been added to
    an input plant.
    """
    def __init__(
        self,
        plant: MultibodyPlant,
        puppet_anchored_to: Frame = None,
        frame_on_puppet: Frame = None,
        name: str = None,
        sphere_radius: float = None,
        config: PuppetmakerConfiguration = None,
    ):
        # Save some helpful internal variables
        self.plant = plant

        # Create config
        self.config = self.config_from_initialization_params(
            puppet_anchored_to=puppet_anchored_to,
            frame_on_puppet=frame_on_puppet,
            name=name,
            sphere_radius=sphere_radius,
            config=config,
        )

    def add_actuated_prismatic_joint(
        self,
        axis_dimension: int,
        previous_frame: Frame,
        next_frame: Frame,
        joint_name: str,
    ) -> Tuple[PrismaticJoint, JointActuator]:
        # Setup
        plant: MultibodyPlant = self.plant

        # Create the direction to translate in
        axis_ii = [0, 0, 0]
        axis_ii[axis_dimension] = 1.0

        # Create Joint
        translation_joint_ii = plant.AddJoint(
            PrismaticJoint(
                name=joint_name,
                frame_on_parent=previous_frame,
                frame_on_child=next_frame,
                axis=axis_ii,
            )
        )

        # Create actuator for joint
        translation_actuator_ii = plant.AddJointActuator(
            joint_name + "_actuator",
            translation_joint_ii,
        )
        return translation_joint_ii, translation_actuator_ii

    def add_actuated_revolute_joint(
        self,
        axis_dimension: int,
        previous_frame: Frame,
        next_frame: Frame,
        joint_name: str,
    ) -> Tuple[Joint, JointActuator]:
        # Setup
        plant: MultibodyPlant = self.plant

        # Create the direction to rotate in
        axis_ii = [0, 0, 0]
        axis_ii[axis_dimension] = 1.0

        # Create Joint
        rotation_joint_ii = plant.AddJoint(
            RevoluteJoint(
                name=joint_name,
                frame_on_parent=previous_frame,
                frame_on_child=next_frame,
                axis=axis_ii,
            )
        )

        # Create actuator for joint
        rotation_actuator_ii = plant.AddJointActuator(
            joint_name + "_actuator",
            rotation_joint_ii,
        )
        
        # Return
        return rotation_joint_ii, rotation_actuator_ii

    def add_all_actuated_prismatic_joints(
        self,
        target_model: ModelInstanceIndex,
        massless_sphere_indices: List[ModelInstanceIndex],
    ) -> Tuple[List[Joint], List[JointActuator]]:
        # Setup
        config = self.config
        plant: MultibodyPlant = self.plant

        # Prepare for loop
        joints = []
        joint_actuators = []

        # Create Three Translation Joints
        previous_frame = config.frame_on_parent
        translation_joint_names = self.translation_joint_names(target_model)
        for axis_dimension, joint_name_ii in enumerate(translation_joint_names):
            # Compute the next frame to connect to from sphere_ii
            # It will be the only frame in the sphere model
            sphere_ii = massless_sphere_indices[axis_dimension]
            sphere_ii_bodies = plant.GetBodyIndices(sphere_ii)
            sphere_ii_frame = plant.get_body(sphere_ii_bodies[0]).body_frame()

            joint_ii, joint_actuator_ii =self.add_actuated_prismatic_joint(
                axis_dimension=axis_dimension,
                previous_frame=previous_frame,
                next_frame=sphere_ii_frame,
                joint_name=joint_name_ii,
            )

            # Update previous frame for next iteration of loop
            previous_frame = sphere_ii_frame

            # Save new joints and actuators
            joints.append(joint_ii)
            joint_actuators.append(joint_actuator_ii)

        return joints, joint_actuators

    def add_all_actuated_revolute_joints(
        self,
        target_model: ModelInstanceIndex,
        massless_sphere_indices: List[ModelInstanceIndex],
    ) -> Tuple[List[Joint], List[JointActuator]]:
        # Setup
        config = self.config
        plant: MultibodyPlant = self.plant

        # Prepare output containers
        joints = []
        joint_actuators = []

        # Find the previous joint to connect the FIRST rotation joint
        sphere2 = massless_sphere_indices[2]
        sphere2_bodies = plant.GetBodyIndices(sphere2)
        previous_frame = plant.get_body(sphere2_bodies[0]).body_frame()

        # Create The First 2 Rotation Joints
        rotation_joint_names = self.rotation_joint_names(target_model)
        for axis_dimension, joint_name_ii in enumerate(rotation_joint_names[:2]):
            # Compute the next frame to connect to from sphere_ii
            # It will be the only frame in the sphere model
            sphere_ii = massless_sphere_indices[axis_dimension + 3]
            sphere_ii_bodies = plant.GetBodyIndices(sphere_ii)
            sphere_ii_frame = plant.get_body(sphere_ii_bodies[0]).body_frame()

            joint_ii, joint_actuator_ii = self.add_actuated_revolute_joint(
                axis_dimension=axis_dimension,
                previous_frame=previous_frame,
                next_frame=sphere_ii_frame,
                joint_name=joint_name_ii,
            )

            # Update previous frame for next iteration of loop
            previous_frame = sphere_ii_frame

            # Save new joints and actuators
            joints.append(joint_ii)
            joint_actuators.append(joint_actuator_ii)

        # Connect the last rotation joint to the puppet
        frame_on_puppet = self.find_frame_on_puppet(target_model)
        joint_ii, joint_actuator_ii = self.add_actuated_revolute_joint(
            axis_dimension=2,
            previous_frame=previous_frame,
            next_frame=frame_on_puppet,
            joint_name=rotation_joint_names[2],
        )

        # Save new joints and actuators
        joints.append(joint_ii)
        joint_actuators.append(joint_actuator_ii)

        return joints, joint_actuators

    def add_strings_for(
        self,
        target_model: ModelInstanceIndex,
    ) -> PuppetSignature:
        """Adds the necessary actuators for the puppet's joints."""
        self.add_actuators_for(target_model)

    def add_actuators_for(
        self,
        target_model: ModelInstanceIndex,
    ) -> PuppetSignature:
        """
        Description
        -----------
        Creates actuated joints on the object we want to be the "puppet" so that we can control
        its entire pose. This will be 4 joints:
        1. Translation in X
        2. Translation In Y
        3. Translation In Z
        4. Rotation (Roll-Pitch-Yaw) 
        
        """
        # Setup
        config: PuppetmakerConfiguration = self.config
        plant: MultibodyPlant = self.plant

        # Input Checking
        if plant.is_finalized():
            raise ValueError(
                f"Plant should not be finalized before calling `create_actuators_for_puppet()` method."
            )

        # Create some fictitious bodies which will be used to connect some new actuators
        massless_sphere_indices = self.create_ghost_bodies_for_actuators()

        # Create Three Translation Joints
        prismatic_joints, prismatic_joint_actuators = self.add_all_actuated_prismatic_joints(
            target_model=target_model,
            massless_sphere_indices=massless_sphere_indices,
        )

        # Create the rotational joints
        revolute_joints, revolute_joint_actuators = self.add_all_actuated_revolute_joints(
            target_model=target_model,
            massless_sphere_indices=massless_sphere_indices,
        )

        # Return
        return PuppetSignature(
            name=f"[{config.name}]Puppet for {plant.GetModelInstanceName(target_model)}",
            model_instance_index=target_model,
            prismatic_joints=prismatic_joints,
            prismatic_joint_actuators=prismatic_joint_actuators,
            revolute_joints=revolute_joints,
            revolute_joint_actuators=revolute_joint_actuators,
        )

    def config_from_initialization_params(
        self,
        puppet_anchored_to: Frame|None,
        frame_on_puppet: Frame|None,
        name: str|None,
        sphere_radius: float = None,
        config: PuppetmakerConfiguration = None,
    ) -> PuppetmakerConfiguration:
        # Input Checking
        if config is not None:
            return config # If the config was provided, then simply return that.
        
        # If config is not given, then we will try to create it.

        # Setup
        plant: MultibodyPlant = self.plant

        # Choose Frame to Anchor things to
        if puppet_anchored_to is None:
            puppet_anchored_to = plant.world_frame()

        # Choose name of the puppeteer
        if name is None:
            name = "Puppeteer"

        # Create config with these initial values
        temp_config = PuppetmakerConfiguration(
            frame_on_parent=puppet_anchored_to,
            name=name,
            frame_on_child=frame_on_puppet,
        )

        # Update config with other optional values
        if sphere_radius is not None:
            temp_config.sphere_radius = sphere_radius

        return temp_config

    def create_ghost_bodies_for_actuators(self) -> List[ModelInstanceIndex]:
        """
        Description
        -----------
        This method creates a set of simple spheres which we will use to connect
        actuators to the "puppet". We will reuse the simple shape API's in Brom
        to create a set of new URDFs that will have (i) no mass, and
        (ii) no collision geometry.
        """
        # Setup
        config = self.config

        # Create list of ModelInstances
        out: List[ModelInstanceIndex] = []

        # Create the spheres that connect the translation joints
        for translation_sphere_idx in range(2):
            # Create the sphere name
            sphere_ii_name = f"[{config.name}]translation_{translation_sphere_idx}_to_translation_{translation_sphere_idx+1}"

            # Update the output list
            out.append(
                self.add_massless_sphere_to_plant_with_name(
                    name=sphere_ii_name,
                    color=config.sphere_color,
                )
            )

        # Create the sphere that connects the last translation joint to the first rotation joint
        sphere_2_name = f"[{config.name}]translation_2_to_rotation_0"
        out.append(
            self.add_massless_sphere_to_plant_with_name(
                name=sphere_2_name,
                color=config.sphere_color,
            )
        )

        # Create the spheres that connect the rotation joints
        for rotation_sphere_idx in [0, 1]:
            # Create the name of the sphere
            sphere_jj_name = f"[{config.name}]rotation_{rotation_sphere_idx}_to_rotation_{rotation_sphere_idx+1}"
            # Update the output list
            out.append(
                self.add_massless_sphere_to_plant_with_name(
                    name=sphere_jj_name,
                    color=config.sphere_color,
                )
            )

        return out

    def add_massless_sphere_to_plant_with_name(self, name: str, color: np.ndarray = None) -> ModelInstanceIndex:
        """
        Description
        -----------
        This method adds a sphere with:
        - zero mass
        - no collision geometry,
        - and an associated color
        to the Puppeteer's plant. 
        The sphere's name will be assigned by the input to the function.
        """
        # Setup
        config = self.config
        plant: MultibodyPlant = self.plant

        # Input Processing
        if color is None:
            color = np.array([0.0, 0.0, 1.0, 0.4])

        # Create the sphere shape
        sphere_shape = SphereDefinition(radius=config.sphere_radius)

        # Create a URDF for this sphere
        sphere_urdf_defn = SimpleShapeURDFDefinition(
            name=name,
            shape=sphere_shape,
            create_collision=False,
            mass=1e-3,
            color=color,
        )
        sphere_urdf = sphere_urdf_defn.write_to_file()

        # Add the sphere to the plant
        sphere_model_index = Parser(plant=plant).AddModels(sphere_urdf)[0]

        return sphere_model_index

    def find_frame_on_puppet(self, target_model: ModelInstanceIndex):
        # Setup
        config = self.config
        plant: MultibodyPlant = self.plant

        # Choose the frame on the puppet to attach actuators to
        if config.frame_on_child is not None:
            return self.config.frame_on_child
        else:
            # Search through all frames of the model
            bodies_in_puppet = plant.GetBodyIndices(target_model)
            first_body = plant.get_body(bodies_in_puppet[0])
            first_frame_on_first_body = first_body.body_frame()
            return first_frame_on_first_body


    @property
    def rotation_axis_names(self) -> str:
        return ["Roll", "Pitch", "Yaw"]
    
    def rotation_joint_names(
        self,
        target_model: ModelInstanceIndex,
    ) -> List[str]:
        """
        Description
        -----------
        This method returns the names of all rotation joints created for the puppet.
        """
        # Setup
        config = self.config
        plant: MultibodyPlant = self.plant

        # Find name of plant
        target_model_name = plant.GetModelInstanceName(target_model)

        return [
            f"[{config.name}]{target_model_name}_rotation_joint_{axis_name}"
            for axis_name in self.rotation_axis_names
        ]

    @property
    def translation_axis_names(self) -> str:
        return ["X", "Y", "Z"]

    def translation_joint_names(
        self,
        target_model: ModelInstanceIndex,
    ) -> List[str]:
        """
        Description
        -----------
        This method returns the names of all translation joints created for the puppet.
        """
        # Setup
        config = self.config
        plant: MultibodyPlant = self.plant

        # Find name of plant
        target_model_name = plant.GetModelInstanceName(target_model)

        return [
            f"[{config.name}]{target_model_name}_translation_joint_{axis_name}"
            for axis_name in self.translation_axis_names
        ]