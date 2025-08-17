import numpy as np
from pydrake.all import (
    Frame,
    LeafSystem,
    ModelInstanceIndex,
    MultibodyPlant,
    Parser,
    PrismaticJoint,
    RevoluteJoint,
    Joint
)
from typing import List

# Internal Imports
from .configuration import Configuration as PuppetmakerConfiguration
from brom_drake.file_manipulation.urdf import SimpleShapeURDFDefinition
from brom_drake.file_manipulation.urdf.shapes import SphereDefinition

class Puppetmaker:
    """
    Description
    -----------
    This system is used to actuate a free body that has been added to
    an input plant.
    """
    def __init__(
        self,
        model_instance_index: ModelInstanceIndex,
        plant: MultibodyPlant,
        puppet_anchored_to: Frame = None,
        frame_on_puppet: Frame = None,
        name: str = None,
        sphere_radius: float = None,
        config: PuppetmakerConfiguration = None,
    ):
        # Save some helpful internal variables
        self.plant = plant
        self.target_model = model_instance_index

        # Create config
        self.config = self.config_from_initialization_params(
            puppet_anchored_to=puppet_anchored_to,
            frame_on_puppet=frame_on_puppet,
            name=name,
            sphere_radius=sphere_radius,
            config=config,
        )

        # Create all actuators
        self.translation_joints, self.rotation_joint = [], None
        self.translation_actuators, self.rotation_actuator = [], None
        self.create_actuators_for_puppet()

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
        target_model: ModelInstanceIndex = self.target_model

        # Choose Frame to Anchor things to
        if puppet_anchored_to is None:
            puppet_anchored_to = plant.world_frame()

        # Choose name of the puppeteer
        if name is None:
            name = "Puppeteer"

        # Choose the frame on the puppet to attach actuators to
        if frame_on_puppet is None:
            # Search through all frames of the model
            bodies_in_puppet = plant.GetBodyIndices(target_model)
            first_body = plant.get_body(bodies_in_puppet[0])
            first_frame_on_first_body = first_body.body_frame()
            frame_on_puppet = first_frame_on_first_body


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

    def create_actuators_for_puppet(self):
        """
        Description
        -----------
        Creates joints on the object we want to be the "puppet" so that we can control
        its entire pose. This will be 4 joints:
        1. Translation in X
        2. Translation In Y
        3. Translation In Z
        4. Rotation (Roll-Pitch-Yaw) 
        """
        # Setup
        config: PuppetmakerConfiguration = self.config
        plant: MultibodyPlant = self.plant
        target_model: ModelInstanceIndex = self.target_model

        # Input Checking
        if plant.is_finalized():
            raise ValueError(
                f"Plant should not be finalized before calling `create_actuators_for_puppet()` method."
            )

        # Create some fictitious bodies which will be used to connect some new actuators
        massless_sphere_indices = self.create_ghost_bodies_for_actuators()

        # Create Three Translation Joints
        axis_names = ["X", "Y", "Z"]
        joint_names = [
            f"{config.name}_translation_joint_{axis_name}"
            for axis_name in axis_names
        ]
        previous_frame = config.frame_on_parent
        for axis_dimension, joint_name_ii in enumerate(joint_names):
            # Create the direction to translate in
            axis_ii = [0, 0, 0]
            axis_ii[axis_dimension] = 1.0

            # Compute the next frame to connect to from sphere_ii
            # It will be the only frame in the sphere model
            sphere_ii = massless_sphere_indices[axis_dimension]
            sphere_ii_bodies = plant.GetBodyIndices(sphere_ii)
            sphere_ii_frame = plant.get_body(sphere_ii_bodies[0]).body_frame()

            # Create Joint
            translation_joint_ii = plant.AddJoint(
                PrismaticJoint(
                    name=joint_name_ii,
                    frame_on_parent=previous_frame,
                    frame_on_child=sphere_ii_frame,
                    axis=axis_ii,
                )
            )
            self.translation_joints.append(translation_joint_ii)

            # Create actuator for joint
            translation_actuator_ii = plant.AddJointActuator(
                joint_name_ii + "_actuator",
                translation_joint_ii,
            )
            self.translation_actuators.append(translation_actuator_ii)

            # Update previous frame for next iteration of loop
            previous_frame = sphere_ii_frame

        # Create the rotational joints
        axis_names = ["roll", "pitch", "yaw"]
        joint_names = [
            f"{config.name}_rotation_joint_{axis_name}"
            for axis_name in axis_names
        ]
        for axis_dimension, joint_name_ii in enumerate(joint_names[:2]):
            # Create the direction to translate in
            axis_ii = [0, 0, 0]
            axis_ii[axis_dimension] = 1.0

            # Compute the next frame to connect to from sphere_ii
            # It will be the only frame in the sphere model
            sphere_ii = massless_sphere_indices[axis_dimension+3]
            sphere_ii_bodies = plant.GetBodyIndices(sphere_ii)
            sphere_ii_frame = plant.get_body(sphere_ii_bodies[0]).body_frame()

            # Create Joint
            translation_joint_ii = plant.AddJoint(
                RevoluteJoint(
                    name=joint_name_ii,
                    frame_on_parent=previous_frame,
                    frame_on_child=sphere_ii_frame,
                    axis=axis_ii,
                )
            )
            self.translation_joints.append(translation_joint_ii)

            # Create actuator for joint
            translation_actuator_ii = plant.AddJointActuator(
                joint_name_ii + "_actuator",
                translation_joint_ii,
            )
            self.translation_actuators.append(translation_actuator_ii)

            # Update previous frame for next iteration of loop
            previous_frame = sphere_ii_frame

        # Connect the last sphere to the puppet
        
        # Create the direction to translate in
        axis_ii = [0, 0, 0]
        axis_ii[axis_dimension] = 1.0

        # Create Joint
        translation_joint_ii = plant.AddJoint(
            RevoluteJoint(
                name=joint_name_ii,
                frame_on_parent=previous_frame,
                frame_on_child=config.frame_on_child,
                axis=axis_ii,
            )
        )
        self.translation_joints.append(translation_joint_ii)

        # Create actuator for joint
        translation_actuator_ii = plant.AddJointActuator(
            joint_name_ii + "_actuator",
            translation_joint_ii,
        )
        self.translation_actuators.append(translation_actuator_ii)


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
