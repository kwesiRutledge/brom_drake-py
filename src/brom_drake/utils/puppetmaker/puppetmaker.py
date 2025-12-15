from dataclasses import dataclass
import numpy as np
from pydrake.all import (
    Adder,
    ConstantVectorSource,
    Demultiplexer,
    DiagramBuilder,
    Frame,
    LeafSystem,
    ModelInstanceIndex,
    MultibodyPlant,
    Multiplexer,
    Parser,
    PassThrough,
    PidController,
    PrismaticJoint,
    RevoluteJoint,
    Joint,
    JointActuator,
)
from typing import List, Tuple

from brom_drake.utils.model_instances import get_all_bodies_in

# Internal Imports
from .configuration import Configuration as PuppetmakerConfiguration
from .puppet_signature import PuppetSignature, PuppeteerJointSignature, AllJointSignatures
from brom_drake.file_manipulation.urdf import SimpleShapeURDFDefinition
from brom_drake.file_manipulation.urdf.shapes import SphereDefinition
from brom_drake.utils.leaf_systems.rigid_transform_to_vector_system import (
    RigidTransformToVectorSystem,
    RigidTransformToVectorSystemConfiguration,
)

class Puppetmaker:
    """
    Description
    -----------
    This object is used to:
    - Add "invisible" actuators for moving a free body (i.e., `add_strings_for` or `add_actuators_for`), and
    - Control actuate a free body that has been added with the expected "puppet" actuators (i.e., `add_puppet_controller_for)

    Notes
    -----
    - The puppet is assumed to be a free body (i.e., it has no existing joints connecting it to the world).
      After adding the puppet actuators, the puppet will NO LONGER be a free body.
    - You must call add_strings_for or add_actuators_for BEFORE finalizing the plant.
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
        """
        Description
        -----------
        This method will add an prismatic joint in the direction `axis_dimension`
        between `previous_frame` and `next_frame`, along with an actuator to control it.
        
        Arguments
        ---------
        :param axis_dimension: The axis to translate in (0 for X, 1 for Y, 2 for Z).
        :type axis_dimension: int
        :param previous_frame: The frame of the parent side of the joint.
        :type previous_frame: Frame
        :param next_frame: The frame of the child side of the joint.
        :type next_frame: Frame
        :param joint_name: The name of the joint we are creating.
        :type joint_name: str

        Returns
        -------
        :return: The created prismatic joint and its actuator.
        :rtype: Tuple[PrismaticJoint, JointActuator]
        """
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
    ) -> List[PuppeteerJointSignature]:
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

        return [
            PuppeteerJointSignature(
                joint=joints[ii],
                joint_actuator=joint_actuators[ii]
            )
            for ii in range(len(joints))
        ]

    def add_all_actuated_revolute_joints(
        self,
        target_model: ModelInstanceIndex,
        massless_sphere_indices: List[ModelInstanceIndex],
    ) -> List[PuppeteerJointSignature]:
        """
        Description
        -----------

        Notes
        -----
        This function expects 3 massless sphere inputs.
        The first sphere in the list should be the one representing the last 
        translation sphere (i.e., the one we should connect the first revolute sphere to.)
        """
        # Setup
        config = self.config
        plant: MultibodyPlant = self.plant

        # Prepare output containers
        joints = []
        joint_actuators = []

        # Find the previous joint to connect the FIRST rotation joint to
        # (This should be the sphere used for the last translation joint.)
        sphere2 = massless_sphere_indices[0]
        sphere2_bodies = plant.GetBodyIndices(sphere2)
        previous_frame = plant.get_body(sphere2_bodies[0]).body_frame()

        # Create The First 2 Rotation Joints
        rotation_joint_names = self.rotation_joint_names(target_model)
        for axis_dimension, joint_name_ii in enumerate(rotation_joint_names[:2]):
            # Compute the next frame to connect to from sphere_ii
            # It will be the only frame in the sphere model
            sphere_ii = massless_sphere_indices[axis_dimension+1]
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

        return [
            PuppeteerJointSignature(
                joint=joints[ii],
                joint_actuator=joint_actuators[ii]
            )
            for ii in range(len(joints))
        ]

    def add_strings_for(
        self,
        target_model: ModelInstanceIndex,
    ) -> PuppetSignature:
        """Adds the necessary actuators for the puppet's joints."""
        return self.add_actuators_for(target_model)

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
        translation_ghost_models, revolute_ghost_models = self.create_ghost_bodies_for_actuators()

        # Create Three Translation Joints
        prismatic_signatures = self.add_all_actuated_prismatic_joints(
            target_model=target_model,
            massless_sphere_indices=translation_ghost_models,
        )

        # Create the rotational joints
        revolute_signatures = self.add_all_actuated_revolute_joints(
            target_model=target_model,
            massless_sphere_indices=[translation_ghost_models[-1]] + revolute_ghost_models,
        )

        # Return
        return PuppetSignature(
            name=f"[{config.name}] Puppet Signature for {plant.GetModelInstanceName(target_model)}",
            model_instance_index=target_model,
            prismatic_ghost_bodies=translation_ghost_models,
            revolute_ghost_bodies=revolute_ghost_models,
            joints=AllJointSignatures(
                prismatic=prismatic_signatures,
                revolute=revolute_signatures,
            ),
        )

    def add_feedforward_sum_system_for(
        self,
        signature: PuppetSignature,
        builder: DiagramBuilder,
    ) -> Tuple[Adder, ConstantVectorSource]:
        """
        Description
        -----------
        This method will create a simple Adder system to add feedforward
        inputs to the PID controller output.
        """
        # Setup
        plant: MultibodyPlant = self.plant
        n_actuators_for_puppet = signature.n_joints

        # Create the adder system
        feedforward_adder = builder.AddNamedSystem(
            name=f"[{self.config.name}] Feedforward Adder for puppeteering \"{signature.name}\"",
            system=Adder(
                num_inputs=2,
                size=n_actuators_for_puppet,
            ),
        )

        # Create a constant vector source to provide feedforward gravity compensation
        # for the linear actuators
        gravity_field = plant.gravity_field()
        gravity_vector = gravity_field.gravity_vector() # Returns the gravity vector in world frame in units of m/s^2

        # Calculate mass of the puppet
        bodies_in_puppet = get_all_bodies_in(plant, signature.model_instance_index)
        total_mass = np.sum(
            [body_ii.default_mass() for body_ii in bodies_in_puppet]
        )

        gravity_compensation = np.zeros((n_actuators_for_puppet,))
        gravity_compensation[:3] = -total_mass * gravity_vector

        # Create the constant vector source
        gravity_compensation_source = builder.AddNamedSystem(
            name=f"[{self.config.name}] Gravity Compensation for puppeteering \"{signature.name}\"",
            system=ConstantVectorSource(gravity_compensation),
        )

        # Connect the gravity compensation to the adder
        builder.Connect(
            gravity_compensation_source.get_output_port(0),
            feedforward_adder.get_input_port(0),
        )

        return feedforward_adder, gravity_compensation_source

    def add_puppet_controller_for(
        self,
        signature: PuppetSignature,
        builder: DiagramBuilder,
        Kp: np.ndarray = None,
        Kd: np.ndarray = None,
    ) -> Tuple[RigidTransformToVectorSystem, PassThrough|None]:
        """
        Description
        -----------
        This method will create a simple PID controller to actuate the puppet
        to a desired pose.

        Arguments
        ---------
        signature : PuppetSignature
            The signature of the puppet to be controlled.
        builder : DiagramBuilder
            The diagram builder to add the controller to.
        Kp : np.ndarray, optional
            The proportional gain for the PID controller. If not provided, a default value will be used.
        Kd : np.ndarray, optional
            The derivative gain for the PID controller. If not provided, a default value will be used.

        Returns
        -------
        pose_to_vector_system : RigidTransformToVectorSystem
            The system that converts the desired pose to a vector format.
            This system is what provides the target to the PID controller.
            You can think of this as the "reference" signal for the controller.
        passthrough_system : PassThrough|None
            A passthrough system that can be used to provide inputs to the
            "puppet" if the Puppet contains actuated joints.
            If the puppet does not contain any actuated joints, then this will be None.
        """
        # Setup
        plant: MultibodyPlant = self.plant
        n_actuators_for_puppet = signature.n_joints

        # Input Processing
        # - Verify that plant is finalized already
        if not self.plant.is_finalized():
            raise ValueError("Plant is not finalized yet.\nPlant must be finalized before calling this method!")

        if Kp is None:
            bodies_in_puppet = plant.GetBodyIndices(signature.model_instance_index)
            m, gravity = 0.0, 9.81
            for body_ii in bodies_in_puppet:
                m += plant.get_body(body_ii).default_mass()
            Kp = np.zeros((n_actuators_for_puppet,))
            Kp[:3] = np.array([1.0*m*gravity]*3)
            Kp[3:] = np.array([0.1*m*gravity]*3)

        if Kd is None:
            Kd = np.sqrt(Kp)

        # Create a demultiplexer to combine all actuator inputs
        actuator_demux, pass_through_system = self.create_actuator_demux(signature, builder)

        # Create the PID Controller
        controller: PidController = builder.AddSystem(
            PidController(
                kp=Kp,
                ki=np.zeros(Kp.shape),
                kd=Kd,
            )
        )

        # Create the feedforward adder system
        feedforward_adder, _ = self.add_feedforward_sum_system_for(signature, builder)

        # Connect the feedforward adder to the controller
        self.combine_feedforward_and_controller_for(
            builder=builder,
            feedforward_adder=feedforward_adder,
            controller=controller,
        )

        # Connect the adder (which combines the PID Controller and the feedforward term)
        # to the demultiplexer
        builder.Connect(
            feedforward_adder.get_output_port(0),
            actuator_demux.get_input_port(0),
        )

        # Connect a PoseToVector System to the PID controller
        # as reference.
        p2v_config = RigidTransformToVectorSystemConfiguration(
            name=f"[{self.config.name}]Pose to vector for puppeteering \"{signature.name}\"",
            output_format="vector_xyz_euler(rpy)",
        )
        pose_to_vector = builder.AddSystem(
            RigidTransformToVectorSystem(p2v_config)
        )

        # Reference signal is a state which will be the
        # target pose as well as the target twist
        zero_twist_source = builder.AddSystem(
            ConstantVectorSource(np.zeros((6,)))
        )

        target_state_mux = builder.AddNamedSystem(
            name=f"[{self.config.name}] Target State Mux for puppeteering \"{signature.name}\"",
            system=Multiplexer(input_sizes=[6, 6]),
        )

        # Connect mux inputs
        builder.Connect(
            pose_to_vector.get_output_port(0),
            target_state_mux.get_input_port(0),
        )
        builder.Connect(
            zero_twist_source.get_output_port(0),
            target_state_mux.get_input_port(1),
        )
        
        # Connect mux output (i.e., the desired state) to PID
        builder.Connect(
            target_state_mux.get_output_port(0),
            controller.get_input_port_desired_state(),
        )

        self.connect_plant_to_controller(builder, controller, signature)

        return pose_to_vector, pass_through_system

    def combine_feedforward_and_controller_for(
        self,
        builder: DiagramBuilder,
        feedforward_adder: Adder,
        controller: PidController,
    ) -> Tuple[Adder, ConstantVectorSource]:
        """
        Description
        -----------
        This method will connect the output of the PID controller to one input
        of the feedforward adder.
        """
        # Setup

        # Connect the output of the controller to one input of the adder
        builder.Connect(
            controller.get_output_port(0),
            feedforward_adder.get_input_port(1),
        )

        return feedforward_adder, None

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

    def connect_plant_to_controller(
        self,
        builder: DiagramBuilder,
        controller: PidController,
        signature: PuppetSignature,
    ) -> None:
        """
        Connect the plant to the controller.
        """
        # Setup
        plant: MultibodyPlant = self.plant

        # Create a massive multiplexer to combine all of the
        # states (2 each) of the models (there should be 6) in the puppet
        state_mux = builder.AddNamedSystem(
            name=f"[{self.config.name}] State Mux for puppeteering \"{signature.name}\"",
            system=Multiplexer(num_scalar_inputs=2*signature.n_models),
        )

        # Create connections to the state of each model (in the puppet)
        # to new demultiplexers
        all_state_demuxes: List[Demultiplexer] = []
        for ii, model_ii in enumerate(signature.all_models):
            # Create demultiplexer for this model
            n_state_ii = plant.num_multibody_states(model_ii)
            print(f"Model #{ii}'s states: {plant.GetStateNames(model_ii)}")
            state_demux_ii: Demultiplexer = builder.AddSystem(
                Demultiplexer(size=n_state_ii),
            )

            # Connect the plant state to the demux
            builder.Connect(
                plant.get_state_output_port(model_ii),
                state_demux_ii.get_input_port(0),
            )

            all_state_demuxes.append(state_demux_ii)

        # Connect all demuxes to the state mux
        for ii, demux_ii in enumerate(all_state_demuxes):
            model_ii = signature.all_models[ii]

            # Find position component of puppet joint state and
            # send it to the state_mux
            position_ii_state_index = self.find_index_of_position_state_in(
                model_instance_index=model_ii,
                signature=signature,
            )
            builder.Connect(
                demux_ii.get_output_port(position_ii_state_index),
                state_mux.get_input_port(ii),
            )

            # Find velocity component of puppet joint state and
            # send it to the state_mux
            velocity_ii_state_index = self.find_index_of_velocity_state_in(
                model_instance_index=model_ii,
                signature=signature,
            )
            builder.Connect(
                demux_ii.get_output_port(velocity_ii_state_index),
                state_mux.get_input_port(signature.n_models + ii),
            )

        # Connect the output of the state demux to the controller
        builder.Connect(
            state_mux.get_output_port(0),
            controller.get_input_port_estimated_state(),
        )

    def create_actuator_demux(
        self,
        signature: PuppetSignature,
        builder: DiagramBuilder,
    ) -> Tuple[Demultiplexer, PassThrough|None]:
        """
        Description
        -----------
        This method creates a demultiplexer to split the actuator inputs
        to the puppet's various models.
        """
        # Setup
        plant: MultibodyPlant = self.plant
        n_actuators_for_puppet = signature.n_joints

        # Create a demultiplexer to combine all actuator inputs
        actuator_demux = builder.AddSystem(
            Demultiplexer(size=n_actuators_for_puppet),
        )

        # Connect demultiplexer to each of the actuators
        pass_through_system = None # Default output, if the puppet has no actuators

        for ii, model_ii in enumerate(signature.all_models):
            if plant.num_actuators(model_ii) == 0:
                continue

            print(f"n_actuators for model {plant.GetModelInstanceName(model_ii)}: {plant.num_actuators(model_ii)}")

            # for port_jj in plant.GetActuatorNames(model_ii):
            #     print(f"Actuator on model {plant.GetModelInstanceName(model_ii)}: {port_jj}")
                
            if plant.num_actuators(model_ii) == 1:
                # If there is only one actuator for this model,
                # then it is the actuator that we created and 
                # we can connect it directly to the demux input

                builder.Connect(
                    actuator_demux.get_output_port(ii),
                    plant.get_actuation_input_port(model_ii),
                )
            else:
                # If there is more than one actuator for this model,
                # then we will need to create a passthrough system to
                # pass the "default" actuator inputs for the puppet's (default) actuated joints
                # and a separate passthrough system to pass the "puppet" actuator input for puppeteering.
                
                default_actuator_pass_through, puppeteer_pass_through = self.create_actuator_passthrough_for_puppet_with_actuated_joints(
                    signature=signature,
                    builder=builder,
                )
                pass_through_system = default_actuator_pass_through

                # Connect the puppeteer passthrough to the demux
                builder.Connect(
                    actuator_demux.get_output_port(ii),
                    puppeteer_pass_through.get_input_port(),
                )                

        return actuator_demux, pass_through_system
    
    def create_actuator_passthrough_for_puppet_with_actuated_joints(
        self,
        signature: PuppetSignature,
        builder: DiagramBuilder,
    ) -> Tuple[PassThrough, PassThrough]:
        """
        Description
        -----------
        This method creates two passthrough systems:
        1. passes the "default" actuator inputs for the puppet's (default) actuated joints,
        2. passes the "puppet" actuator input for puppeteering.
        """
        # Setup
        plant: MultibodyPlant = self.plant
        puppet_model_idx = signature.model_instance_index
        n_default_actuators_in_puppet = plant.num_actuators(puppet_model_idx)
        
        # Create a Multiplexer system to combine all actuator inputs
        passthrough_mux = builder.AddNamedSystem(
            name=f"[{self.config.name}] Total Actuation Mux for puppeteering \"{signature.name}\" WITH default inputs needed",
            system=Multiplexer(input_sizes=[n_default_actuators_in_puppet-1,1]),
        )

        # Create a passthrough system to pass the combined actuator inputs to the puppet
        default_actuator_inputs_passthrough = builder.AddSystem(
            PassThrough(value=np.zeros((n_default_actuators_in_puppet-1,))),
        )

        # Connect the default actuator inputs to the multiplexer
        builder.Connect(
            default_actuator_inputs_passthrough.get_output_port(0),
            passthrough_mux.get_input_port(0),
        )

        # Connect the puppet actuator input to the multiplexer
        puppet_actuator_input = builder.AddSystem(
            PassThrough(value=np.zeros((1,))),
        )
        builder.Connect(
            puppet_actuator_input.get_output_port(0),
            passthrough_mux.get_input_port(1),
        )

        # connect the multiplexer to the plant's input for the
        # puppet's FULL set of actuators (both the ones we added and the ones that it already contains)
        builder.Connect(
            passthrough_mux.get_output_port(),
            plant.get_actuation_input_port(puppet_model_idx),
        )

        return default_actuator_inputs_passthrough, puppet_actuator_input

    def create_ghost_bodies_for_actuators(
        self,
    ) -> Tuple[List[ModelInstanceIndex], List[ModelInstanceIndex]]:
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
        translation_ghost_models: List[ModelInstanceIndex] = []
        revolute_ghost_models: List[ModelInstanceIndex] = []

        # Create the spheres that connect the translation joints
        for translation_sphere_idx in range(2):
            # Create the sphere name
            sphere_ii_name = f"[{config.name}]translation_{translation_sphere_idx}_to_translation_{translation_sphere_idx+1}"

            # Update the output list
            translation_ghost_models.append(
                self.add_massless_sphere_to_plant_with_name(
                    name=sphere_ii_name,
                    color=config.sphere_color,
                )
            )

        # Create the sphere that connects the last translation joint to the first rotation joint
        sphere_2_name = f"[{config.name}]translation_2_to_rotation_0"
        translation_ghost_models.append(
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
            revolute_ghost_models.append(
                self.add_massless_sphere_to_plant_with_name(
                    name=sphere_jj_name,
                    color=config.sphere_color,
                )
            )

        return translation_ghost_models, revolute_ghost_models

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
            mass=config.sphere_mass,
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

    def find_index_of_position_state_in(
        self,
        model_instance_index: ModelInstanceIndex,
        signature: PuppetSignature,
    ) -> int|None:
        """
        Description
        -----------
        This method finds the names of the "position" component for the
        state of the joint connected to model_instance_index.
        """
        # Setup
        plant: MultibodyPlant = self.plant
        target_model: ModelInstanceIndex = signature.model_instance_index

        # For each state state name,
        for ii, state_name_ii in enumerate(plant.GetStateNames(model_instance_index)):
            print("state name: ", state_name_ii)
            # Check to see if name is either:
            # - A Translational joint state name (for the target model)
            for translation_joint_name_jj in self.translation_joint_names(target_model, remove_puppeteer_prefix=True):
                print("- translation_joint_name: ", translation_joint_name_jj)
                if (translation_joint_name_jj in state_name_ii) and ("_x" in state_name_ii):
                    return ii
                
            # - A Rotational Joint state name
            for rotational_joint_name_jj in self.rotation_joint_names(target_model, remove_puppeteer_prefix=True):
                print("- rotational_joint_name: ", rotational_joint_name_jj)
                if (rotational_joint_name_jj in state_name_ii) and ("_q" in state_name_ii):
                    return ii
                
        # If none of the states contained a joint name, then return None
        return None


    def find_index_of_velocity_state_in(
        self,
        model_instance_index: ModelInstanceIndex,
        signature: PuppetSignature,
    ) -> int|None:
        """
        Description
        -----------
        This method finds the names of the "velocity" component for the
        state of the joint connected to model_instance_index.
        """
        # Setup
        plant: MultibodyPlant = self.plant
        target_model: ModelInstanceIndex = signature.model_instance_index

        # For each state state name,
        for ii, state_name_ii in enumerate(plant.GetStateNames(model_instance_index)):
            # Check to see if name is either:
            # - A Translational joint state name
            for translation_joint_name_jj in self.translation_joint_names(target_model, remove_puppeteer_prefix=True):
                if (translation_joint_name_jj in state_name_ii) and ("_v" in state_name_ii):
                    return ii
                
            # - A Rotational Joint state name
            for rotational_joint_name_jj in self.rotation_joint_names(target_model, remove_puppeteer_prefix=True):
                if (rotational_joint_name_jj in state_name_ii) and ("_w" in state_name_ii):
                    return ii
                
        # If none of the states contained a joint name, then return None
        return None

    @property
    def rotation_axis_names(self) -> List[str]:
        return ["roll", "pitch", "yaw"]

    def rotation_joint_names(
        self,
        target_model: ModelInstanceIndex,
        remove_puppeteer_prefix: bool = False,
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
        puppeteer_prefix = "" if remove_puppeteer_prefix else f"[{config.name}]"

        return [
            f"{puppeteer_prefix}{target_model_name}_rotation_joint_{axis_name}"
            for axis_name in self.rotation_axis_names
        ]

    @property
    def translation_axis_names(self) -> List[str]:
        return ["X", "Y", "Z"]

    def translation_joint_names(
        self,
        target_model: ModelInstanceIndex,
        remove_puppeteer_prefix: bool = False,
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
        puppeteer_prefix = "" if remove_puppeteer_prefix else f"[{config.name}]"

        return [
            f"{puppeteer_prefix}{target_model_name}_translation_joint_{axis_name}"
            for axis_name in self.translation_axis_names
        ]