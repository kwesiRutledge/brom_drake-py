from typing import List, Tuple, Union

import numpy as np
from manipulation.scenarios import AddMultibodyTriad
from pydrake.all import (
    GeometryProperties,
    IllustrationProperties,
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
from brom_drake.utils import Performer
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

    def add_floor_to_plant(self):
        """
        Description
        -----------
        This method will add a floor to the plant.
        """
        # Setup
        plant: MultibodyPlant = self.plant

        # Find the z position of the floor
        z_floor = self.find_floor_z()

        # Create a box urdf for the floor
        floor_geometry_defn = BoxDefinition(
            size=[10.0, 10.0, 0.1]
        )
        floor_thickness = floor_geometry_defn.size[2]

        # Create a urdf for the floor
        floor_urdf_defn = SimpleShapeURDFDefinition(
            name="floor",
            shape=floor_geometry_defn,
        ) 
        floor_urdf = floor_urdf_defn.write_to_file()

        # Add the floor to the plant
        floor_model_idcs = Parser(plant=plant).AddModels(floor_urdf)
        self.floor_model_index = floor_model_idcs[0]

        # Create joint + actuator for the floor
        floor_joint = plant.AddJoint(
            PrismaticJoint(
                name="floor_joint",
                frame_on_parent=plant.world_frame(),
                frame_on_child=plant.GetFrameByName(get_name_of_first_body_in_urdf(floor_urdf)),
                axis=[0, 0, 1],
            )
        )

        self.floor_actuator = plant.AddJointActuator(
            "floor_elevation_actuator",
            floor_joint,
        )

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
            X_ObjectTarget=self.X_ObjectTarget,
            target_frame_name=self.target_body_name_on_gripper,
            desired_joint_positions=self.gripper_joint_positions,
        )
        self.add_gripper_to_plant(
            and_weld_to=plant.world_frame(),
            with_X_WorldGripper=X_WorldGripper,
        )

        # Add the floor
        self.add_floor_to_plant()

        # Add controllers for gripper AND floor

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

    def add_floor_controller_and_connect(self):
        # Setup
        plant: MultibodyPlant = self.plant

        # Connect a constant vector source to the floor actuator
        floor_actuator_source = self.builder.AddSystem(
            ConstantVectorSource(np.array([0.0])),
        )

        # Connect the source to the floor actuator
        self.builder.Connect(
            floor_actuator_source.get_output_port(0),
            plant.get_actuation_input_port(self.floor_model_index),
        )

    def find_floor_z(
        self,
        debug_flag: bool = True,
    ) -> float:
        """
        Description
        -----------
        This method will find the transform needed to put the floor at so that
        the object sits "just" on top of it.
        """
        # Setup
        
        # Create shadow plant + scene graph
        shadow_plant, shadow_scene_graph = AddMultibodyPlantSceneGraph(
            self.builder,
            time_step=self.time_step,
        )
        shadow_plant.set_name("DemonstrateStaticGrasp_shadow_plant")

        # Add the object to the shadow plant
        model_idcs = Parser(plant=shadow_plant).AddModels(self.path_to_object)
        assert len(model_idcs) == 1, f"Only one model should be added; received {len(model_idcs)}"
        model_idx = model_idcs[0]
        model_name = shadow_plant.GetModelInstanceName(model_idx)

        # Weld the base link to the world frame
        manipuland_body_idcs = shadow_plant.GetBodyIndices(model_idx)
        assert len(manipuland_body_idcs) > 0, \
            f"Expected at least one body in the manipuland; received {len(manipuland_body_idcs)}"
        manipuland_body = shadow_plant.get_body(manipuland_body_idcs[0])
        frame0 = manipuland_body.body_frame()
        shadow_plant.WeldFrames(
            shadow_plant.world_frame(),
            frame0,
        )

        # Finalize the shadow plant
        shadow_plant.Finalize()

        # Create a context for the shadow plant
        shadow_diagram = DiagramBuilder().AddSystem(shadow_plant)

        # Search through the geometries in the scene and find the manipuland's
        # geometry. Attempt to compute the bounding box around it.
        shadow_sg_inspector: SceneGraphInspector = shadow_scene_graph.model_inspector()
        all_geometries = shadow_scene_graph.model_inspector().GetAllGeometryIds()
        z_out = 1_000.0
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

            if lower_left[2] < z_out:
                z_out = lower_left[2]
                print(f"  + New z_out: {z_out}")
            
        return z_out

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
        name_of_first_body_in_gripper = get_name_of_first_body_in_urdf()

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

    @property
    def id(self):
        return ProductionID.kDemonstrateStaticGrasp

    @property
    def suggested_roles(self) -> List[Role]:
        return []