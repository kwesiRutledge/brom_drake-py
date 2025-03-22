from typing import List, Tuple, Union

import numpy as np
from manipulation.scenarios import AddMultibodyTriad
from pydrake.all import (
    Adder,
    GeometryProperties,
    IllustrationProperties,
    JointActuator,
    ModelInstanceIndex,
    PidController,
    PrismaticJoint,
    RigidBodyFrame,
    RigidTransform,
    SceneGraphInspector,
)
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant
from pydrake.systems.framework import DiagramBuilder, Diagram, Context

from pydrake.systems.primitives import ConstantVectorSource, VectorLogSink

# Internal Imports
from brom_drake.file_manipulation.urdf.drakeify import drakeify_my_urdf
from brom_drake.file_manipulation.urdf.shapes.box import BoxDefinition
from brom_drake.file_manipulation.urdf.simple_writer import SimpleShapeURDFDefinition
from brom_drake.robots import find_base_link_name_in
from brom_drake.productions.types.debug import BasicGraspingDebuggingProduction
from brom_drake.productions import ProductionID
from brom_drake.productions.roles.role import Role
from brom_drake.utils import Performer, collision_checking
from brom_drake.utils.model_instances import (
    get_name_of_first_body_in_urdf,
    find_number_of_positions_in_welded_model,
)
from brom_drake.productions.debug.show_me.show_me_system import ShowMeSystem

class AttemptGrasp(BasicGraspingDebuggingProduction):
    def __init__(
        self,
        path_to_object: str,
        path_to_gripper: str,
        X_ObjectTarget: RigidTransform = None,
        meshcat_port_number: int = 7001, # Usually turn off for CI (i.e., make it None)
        show_collision_geometries: bool = False,
        gripper_joint_positions: Union[List[float], np.ndarray] = None,
        time_step: float = 1e-3,
        target_body_on_gripper: str = None,
        gripper_color: List[float] = None,
        show_gripper_base_frame: bool = False,
    ):
        # Call the parent constructor
        super().__init__(
            path_to_object=path_to_object,
            path_to_gripper=path_to_gripper,
            X_ObjectTarget=X_ObjectTarget,
            meshcat_port_number=meshcat_port_number,
            time_step=time_step,
            target_body_on_gripper=target_body_on_gripper,
            gripper_color=gripper_color,
            show_gripper_base_frame=show_gripper_base_frame,
            show_collision_geometries=show_collision_geometries,
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

        self.floor_model_index = None
        self.floor_actuator = None
        self.floor_shape = None

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

    def add_floor_to_plant(
        self,
        floor_mass: float = 1_000.0,
        plant: MultibodyPlant = None,
    ) -> Tuple[
        float, List[float], ModelInstanceIndex, JointActuator
    ]:
        """
        Description
        -----------
        This method will add a floor to the plant.
        """
        # Setup
        if plant is None:
            plant: MultibodyPlant = self.plant

        # Create a box urdf for the floor
        floor_shape = [10.0, 10.0, 0.1] # Length, Width, Height
        floor_geometry_defn = BoxDefinition(size=floor_shape)

        # Create a urdf for the floor
        floor_urdf_defn = SimpleShapeURDFDefinition(
            name="floor",
            shape=floor_geometry_defn,
            mass=floor_mass,
        ) 
        floor_urdf = floor_urdf_defn.write_to_file()

        # Add the floor to the plant
        floor_model_idcs = Parser(plant=plant).AddModels(floor_urdf)
        floor_model_index = floor_model_idcs[0]

        # Create joint + actuator for the floor
        floor_joint = plant.AddJoint(
            PrismaticJoint(
                name="floor_joint",
                frame_on_parent=plant.world_frame(),
                frame_on_child=plant.GetFrameByName(get_name_of_first_body_in_urdf(floor_urdf)),
                axis=[0, 0, 1],
            )
        )

        floor_actuator = plant.AddJointActuator(
            "floor_elevation_actuator",
            floor_joint,
        )

        return floor_mass, floor_shape, floor_model_index, floor_actuator

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
        self.add_manipuland_to_plant()

        # Add the gripper to the builder
        X_WorldGripper = self.find_X_WorldGripper(
            X_ObjectTarget=self.X_ObjectTarget,
            target_frame_name=self.target_body_name_on_gripper,
            desired_joint_positions=self.gripper_joint_positions,
        )
        self.add_gripper_to_plant(
            and_weld_to=plant.world_frame(),
            with_X_WorldGripper=X_WorldGripper,
        )

        # Add the floor
        floor_mass, self.floor_shape, self.floor_model_index, self.floor_actuator = self.add_floor_to_plant()

        # Connect the plant to the meshcat, if requested
        if self.meshcat_port_number is not None:
            self.connect_to_meshcat()

        # Finalize the plant
        self.plant.Finalize()

        # Add controllers for gripper AND floor
        self.add_gripper_controller_and_connect()
        self.add_floor_controller_and_connect(floor_mass)

        # Create defaults for plant
        self.set_plant_defaults()

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

    def add_floor_controller_and_connect(self, floor_mass: float):
        # Setup
        plant: MultibodyPlant = self.plant

        # Find the z position of the floor
        z_floor = self.find_floor_z_via_line_search()

        # Connect a PID Controller to the floor actuator
        kp = np.array([[floor_mass * 9.81]])
        ki = 0.1 * np.sqrt(kp)
        kd = np.sqrt(kp)
        floor_controller = self.builder.AddSystem(
            PidController(kp=kp, ki=ki, kd=kd),
        )

        # Create a feedforward term that we will add to the PID controller's
        # output to keep the floor at a constant height
        # (i.e., the weight of the object)
        feedforward_term = self.builder.AddSystem(
            ConstantVectorSource(np.array([[floor_mass * 9.81]])),  # Weight of the object
        )

        # Summer to combine the PID output and the feedforward term
        summer = self.builder.AddSystem(
            Adder(2, 1),
        )

        self.builder.Connect(
            floor_controller.get_output_port(0),  # PID output
            summer.get_input_port(0),  # summer input
        )

        self.builder.Connect(
            feedforward_term.get_output_port(0),  # Weight of the object
            summer.get_input_port(1),  # Feedforward term
        )

        # Connect the target height to the PID controller
        target_height_source = self.builder.AddSystem(
            ConstantVectorSource(np.array([[z_floor],[0.0]])),  # Target height (z position)
        )

        # Connect the source to the floor actuator
        self.builder.Connect(
            summer.get_output_port(0),
            plant.get_actuation_input_port(self.floor_model_index),
        )

        # Connect the target height to the PID controller
        self.builder.Connect(
            target_height_source.get_output_port(0),
            floor_controller.get_input_port_desired_state(),
        )

        # Connect the current height of the floor to the PID controller
        self.builder.Connect(
            plant.get_state_output_port(self.floor_model_index),
            floor_controller.get_input_port_estimated_state(),
        )

    def find_floor_z_via_bounding_box(
        self,
        debug_flag: bool = True,
    ) -> float:
        """
        Description
        -----------
        This method will find the transform needed to put the floor at so that
        the object sits "just" on top of it.

        WARNING: This method does not work and I don't know why. (i.e., sometimes
        the floor is in collision with the object)
        """
        # Setup
        shadow_builder = DiagramBuilder()

        # Create shadow plant + scene graph
        shadow_plant, shadow_scene_graph = AddMultibodyPlantSceneGraph(
            shadow_builder,
            time_step=self.time_step,
        )
        shadow_plant.set_name("DemonstrateStaticGrasp_shadow_plant")
        shadow_scene_graph.set_name("DemonstrateStaticGrasp_shadow_scene_graph")

        # Add the object to the shadow plant
        model_idcs = Parser(plant=shadow_plant).AddModels(self.path_to_object)
        assert len(model_idcs) == 1, f"Only one model should be added; received {len(model_idcs)}"
        model_idx = model_idcs[0]
        model_name = shadow_plant.GetModelInstanceName(model_idx)

        # Weld the base link to the world frame
        # manipuland_body_idcs = shadow_plant.GetBodyIndices(model_idx)
        # assert len(manipuland_body_idcs) > 0, \
        #     f"Expected at least one body in the manipuland; received {len(manipuland_body_idcs)}"
        # manipuland_body = shadow_plant.get_body(manipuland_body_idcs[0])
        # frame0 = manipuland_body.body_frame()
        # shadow_plant.WeldFrames(
        #     shadow_plant.world_frame(),
        #     frame0,
        # )

        # Finalize the shadow plant
        shadow_plant.Finalize()

        # Search through the geometries in the scene and find the manipuland's
        # geometry. Attempt to compute the bounding box around it.
        shadow_sg_inspector: SceneGraphInspector = shadow_scene_graph.model_inspector()
        all_geometries = shadow_scene_graph.model_inspector().GetAllGeometryIds()
        objects_minimum_z = 1_000.0
        for ii, geometry_ii in enumerate(all_geometries):
            # Announce geometry value:
            if debug_flag:
                print(f"Geometry ID {ii}:")
                print(f"- Raw value: {geometry_ii}")

            # Get shape of the geometry
            shape_ii = shadow_sg_inspector.GetShape(geometry_ii)
            if debug_flag:
                print(f"- Shape {ii}:")
                print(f"  + Raw value: {shape_ii}")
                print(f"  + Type: {type(shape_ii)}")

            # Get the convex hull
            ch_ii = shape_ii.GetConvexHull()
            if debug_flag:
                print(f"- Convex Hull {ii}:")
                print(f"  + Raw value: {ch_ii}")

            bb_ii = ch_ii.CalcBoundingBox()
            if debug_flag:
                print(f"- Bounding Box {ii}:")
                print(f"  + Raw value: {bb_ii}")
            lower_left, upper_right = bb_ii

            if lower_left[2] < objects_minimum_z:
                objects_minimum_z = lower_left[2]
                print(f"  + New z_out: {objects_minimum_z}")
        
        # If we didn't find any geometry, raise an error
        if objects_minimum_z == 1_000.0:
            raise ValueError("Unable to find geometry for the manipuland.")
        
        # If we found a geometry, then we need to set the floor height
        # using the known shape of the table

        return objects_minimum_z - self.floor_shape[2]*0.5  # Subtract the height of the floor

    def find_floor_z_via_line_search(
        self,
        debug_flag: bool = False,
    ) -> float:
        """
        Description
        -----------
        Use bounding box method to generate an initial guess for the floor height
        and then perform a line search to find the height that actually
        is collision free.
        """
        # Setup
        z_floor = self.find_floor_z_via_bounding_box(debug_flag=debug_flag)
        floor_shape = self.floor_shape

        # Check for collisions at the initial guess
        collision_detected = self.floor_z_creates_collisions(
            floor_z_translation=z_floor,
            debug_flag=debug_flag,
        )
        if not collision_detected:
            # If no collision detected, return the initial guess
            return z_floor
        
        # If we have a collision, then we need to perform a line search
        # to find the height that is collision free
        
        # Start with a small step size
        step_size = floor_shape[2] * 0.25  # 25% of the floor height

        # Perform a line search to find the height that is collision free
        candidate_z_floor = z_floor - step_size
        while True:
            # Check for collisions at the current height
            collision_detected = self.floor_z_creates_collisions(
                floor_z_translation=candidate_z_floor,
                debug_flag=debug_flag,
            )
            if not collision_detected:
                # If no collision detected, return the current height
                print(f"Found collision-free height: {candidate_z_floor}")
                return candidate_z_floor
            

            print(f"Collision detected at height: {candidate_z_floor}")
            # If we have a collision, then increase the height
            candidate_z_floor -= step_size

            # If we exceed a certain height, break out of the loop
            if candidate_z_floor < -10.0:
                break        
                
        raise ValueError("Unable to find a collision-free height for the floor.")

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
        X_GripperBase_Target = shadow_plant.EvalBodyPoseInWorld(
            shadow_plant.GetMyContextFromRoot(shadow_diagram_context),
            shadow_plant.GetBodyByName(target_frame_name),
        )

        X_ObjectGripperBase = X_ObjectTarget.multiply(
            X_GripperBase_Target.inverse(),
        )

        return X_ObjectGripperBase

    def floor_z_creates_collisions(
        self,
        floor_z_translation: float = 0.0,
        debug_flag: bool = False,
    ) -> bool:
        # Setup
        shadow_builder = DiagramBuilder()

        # Create shadow plant + scene graph
        shadow_plant, shadow_scene_graph = AddMultibodyPlantSceneGraph(
            shadow_builder,
            time_step=self.time_step,
        )
        shadow_plant.set_name("DemonstrateStaticGrasp_shadow_plant")
        shadow_scene_graph.set_name("DemonstrateStaticGrasp_shadow_scene_graph")

        # Add the object to the shadow plant
        model_idcs = Parser(plant=shadow_plant).AddModels(self.path_to_object)
        assert len(model_idcs) == 1, f"Only one model should be added; received {len(model_idcs)}"
        model_idx = model_idcs[0]
        model_name = shadow_plant.GetModelInstanceName(model_idx)

        # # Add the gripper to the shadow plant
        # temp_idcs = Parser(plant=shadow_plant).AddModels(self.path_to_gripper)
        # assert len(temp_idcs) == 1, f"Only one model should be added; received {len(temp_idcs)}"
        # shadow_gripper_model_index = temp_idcs[0]
        # shadow_gripper_model_name = shadow_plant.GetModelInstanceName(shadow_gripper_model_index)

        # Add the floor to the shadow plant
        _, _, shadow_floor_model_index, _ = self.add_floor_to_plant(plant=shadow_plant)

        # Finalize the shadow plant
        shadow_plant.Finalize()

        # Create diagram and set position of the floor to be floor_z_translation
        shadow_diagram = shadow_builder.Build()
        shadow_diagram_context = shadow_diagram.CreateDefaultContext()

        shadow_plant.SetPositions(
            shadow_plant.GetMyContextFromRoot(shadow_diagram_context),
            shadow_floor_model_index,
            np.array([floor_z_translation]),
        )

        return collision_checking.using_point_pair_penetration(
            shadow_plant, shadow_scene_graph, shadow_diagram_context,
        )

    @property
    def id(self):
        return ProductionID.kDemonstrateStaticGrasp

    def set_plant_defaults(self):
        """
        Description
        -----------
        This method will set the default state of the plant to the desired joint positions.
        """
        # Setup
        plant: MultibodyPlant = self.plant

        # Find the z position of the floor
        z_floor = self.find_floor_z_via_line_search()

        # Set the default state of the floor to be at the desired height
        plant.SetDefaultPositions(
            self.floor_model_index,
            np.array([z_floor]),
        )

    @property
    def suggested_roles(self) -> List[Role]:
        return []
    
    