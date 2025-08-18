from dataclasses import dataclass
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    Frame,
    Meshcat,
    MeshcatVisualizer,
    MeshcatVisualizerParams,
    MultibodyPlant,
    Parser,
    RigidTransform
)
from pydrake.all import Role as DrakeRole
from typing import List

# Internal Imports
from brom_drake.file_manipulation.urdf.drakeify import drakeify_my_urdf
from brom_drake.productions.types.base import BaseProduction
from brom_drake.utils.model_instances import (
    get_name_of_first_body_in_urdf,
    get_name_of_all_bodies_in_urdf,
)
from brom_drake.utils.triads import AddMultibodyTriad

class BasicGraspingDebuggingProduction(BaseProduction):
    """
    Description
    -----------
    This base class is used to define several functions that are useful
    for any production that is used for basic grasping tasks.
    """
    def __init__(
        self, 
        path_to_object: str,
        path_to_gripper: str,
        X_ObjectGripper: RigidTransform,
        meshcat_port_number: int = None,
        time_step: float = 0.001,
        target_body_on_gripper: str = None,
        gripper_color: List[float] = None,
        show_gripper_base_frame: bool = False,
        show_collision_geometries: bool = False,
    ):
        """
        Description
        -----------
        Defines the following fields for the production:
        
        Arguments
        ----------
        path_to_object: str
            A string representing the path to the object urdf file.
        path_to_gripper: str
            A string representing the path to the gripper urdf file.
        X_GripperObject: RigidTransform
            A RigidTransform object representing the transform from the object frame to the target frame.
        meshcat_port_number: int (Optional, default=None)
            An integer representing the port number for the meshcat server.
            If this is None, the meshcat server will not be started.
        time_step: float (Optional, default=0.001)
            A float representing the time step for the simulation.
        target_body_on_gripper: str (Optional, default=None)
            A string representing the name of the body on the gripper that will be used as the target frame.
            If this is None, the first body in the gripper will be used.
        gripper_color: List[float]
            A list of floats representing the color of the gripper in RGBA format.
        show_gripper_base_frame: bool
            A boolean flag that indicates whether or not to show the base frame of the gripper.
        show_collision_geometries: bool
            A boolean flag that indicates whether or not to show the collision geometries of the gripper.
        """
        # Call the base class constructor
        super().__init__()

        # Add the model to the Production
        self.path_to_object = path_to_object
        self.path_to_gripper = path_to_gripper
        self.time_step = time_step
        self.gripper_color = gripper_color
        self.show_gripper_base_frame = show_gripper_base_frame
        
        if X_ObjectGripper is None:
            X_ObjectGripper = RigidTransform()
        self.X_ObjectGripper = X_ObjectGripper
        
        self.meshcat_port_number = meshcat_port_number
        self.show_collision_geometries = show_collision_geometries

        # Assign the target frame on the gripper to the variable
        if target_body_on_gripper is None:
            target_body_on_gripper = get_name_of_first_body_in_urdf(self.path_to_gripper)
        else:
            assert target_body_on_gripper in get_name_of_all_bodies_in_urdf(self.path_to_gripper), \
                f"Target body \"{target_body_on_gripper}\" not found in gripper model; Valid body names are: {self.get_all_body_names_in_gripper()}."


        self.target_body_name_on_gripper = target_body_on_gripper

        # Add Plant and Scene Graph for easy simulation
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(
            self.builder,
            time_step=1e-3,
        )

        # Create empty variables to be filled in later.
        self.meshcat = None
        self.manipuland_index, self.manipuland_name = None, None
        self.gripper_model_index, self.gripper_model_name = None, None

    def add_gripper_to_plant(self, and_weld_to: Frame = None, with_X_WorldGripper: RigidTransform = None):
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
        gripper_color = self.gripper_color

        show_gripper_base_frame = self.show_gripper_base_frame
        if with_X_WorldGripper is None:
            with_X_WorldGripper = RigidTransform()
        

        # Input Processing
        if gripper_color is not None:
            # Convert gripper URDF to Drake-ready URDF
            recolored_gripper_urdf = drakeify_my_urdf(
                self.path_to_gripper,
                overwrite_old_logs=True,
                log_file_name="BasicGraspingDebuggingProduction_AddManipulandToPlant_gripper.log",
                replace_colors_with=gripper_color,
            )
            self.path_to_gripper = str(recolored_gripper_urdf)

        # Add the gripper to the plant
        temp_idcs = Parser(plant=self.plant).AddModels(
            self.path_to_gripper,
        )
        assert len(temp_idcs) == 1, f"Only one model should be added; received {len(temp_idcs)}"
        self.gripper_model_index = temp_idcs[0]
        self.gripper_model_name = self.plant.GetModelInstanceName(self.gripper_model_index)

        # Draw the MultibodyTriad for the
        # - Target Frame on the Gripper
        # - Base Link of the Gripper
        target_frame = plant.GetFrameByName(self.target_body_name_on_gripper)
        gripper_base_frame = plant.GetFrameByName(
            get_name_of_first_body_in_urdf(self.path_to_gripper)
        )

        # Add the target triad to the builder
        AddMultibodyTriad(
            target_frame,
            self.scene_graph,
        )

        # Add the gripper triad to the builder
        target_is_different_from_gripper_base = target_frame.body().name() != gripper_base_frame.body().name()
        if show_gripper_base_frame and target_is_different_from_gripper_base:
            AddMultibodyTriad(
                gripper_base_frame,
                self.scene_graph,
                # scale=0.1,
            )
            

        # Weld the gripper to the manipuland
        if and_weld_to is not None:
            plant.WeldFrames(
                and_weld_to,
                gripper_base_frame,
                with_X_WorldGripper,
            )

    def add_manipuland_to_plant(self, and_weld_to: Frame = None):
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

        if and_weld_to is not None:
            # Weld the first frame in the model to the frame given by and_weld_to
            assert isinstance(and_weld_to, Frame), \
                f"and_weld_to must be a Frame; received {type(and_weld_to)}"
            
            manipuland_body_idcs = plant.GetBodyIndices(self.manipuland_index)
            assert len(manipuland_body_idcs) > 0, \
                f"Expected at least one body in the manipuland; received {len(manipuland_body_idcs)}"
            manipuland_body = plant.get_body(manipuland_body_idcs[0])
            frame0 = manipuland_body.body_frame()

            plant.WeldFrames(
                self.plant.world_frame(),
                frame0,
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