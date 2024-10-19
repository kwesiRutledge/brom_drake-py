from importlib import resources as impresources
import numpy as np
from pydrake.common.eigen_geometry import Quaternion
from pydrake.geometry import MeshcatVisualizer, Meshcat
from pydrake.math import RigidTransform
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant
from pydrake.systems.framework import DiagramBuilder

# Internal Imports
import brom_drake.robots as robots
from brom_drake.example_helpers import AddGround
from brom_drake.robots.stations.kinematic import UR10eStation as KinematicUR10eStation
from brom_drake.scenes import SceneID
from brom_drake.scenes.types.motion_planning import OfflineMotionPlanningScene
from brom_drake.urdf import drakeify_my_urdf


class ShelfPlanningScene(OfflineMotionPlanningScene):
    def __init__(
        self,
        time_step=1e-3,
        shelf_pose: RigidTransform = None, # The pose of the shelf
        meshcat_port_number: int = 7001, # Usually turn off for CI (i.e., make it None)
        **kwargs,
    ):
        super().__init__(time_step=time_step, **kwargs)

        self.time_step = time_step
        self.shelf_pose = shelf_pose
        self.meshcat_port_number = meshcat_port_number

        # Input Processing
        if self.shelf_pose is None:
            shelf_orientation = Quaternion(1, 0, 0 , 0)
            shelf_position = np.array([-0.4, 0.8, 0.0])
            self.shelf_pose = RigidTransform(
                shelf_orientation, shelf_position,
            )

        # If the base link name is not provided,
        # then we will try to do a smart search for it later.
        self.meshcat = None
        self.station, self.plant = None, None

    def add_all_secondary_cast_members_to_builder(self):
        """
        Add all secondary cast members to the builder.
        :return:
        """
        # Setup
        builder = self.builder
        self.add_ur10e_station() # Use the UR10e station's plant + scene graph for all other objects
        self.plant = self.station.plant # The station's plant is what we will use for the rest of the scene

        # Add obstacles
        self.add_shelf(self.plant)

        # Add the ground as well as the start and goal locations
        AddGround(self.plant)
        self.add_start_and_goal_to_this_plant(self.plant)



        self.station.Finalize()

        # Connect to Meshcat
        # if self.use_meshcat:
        #     self.meshcat = Meshcat(port=7001)  # Object provides an interface to Meshcat
        #     m_visualizer = MeshcatVisualizer(self.meshcat)
        #     m_visualizer.AddToBuilder(builder, self.scene_graph, self.meshcat)

        # Print message to user
        print("Added all secondary cast members to the builder.")

    def add_shelf(self, plant: MultibodyPlant):
        """
        Add the shelf to the scene.
        :return:
        """
        # Setup
        urdf_file_path = str(
            impresources.files(robots) / "models/bookshelf/bookshelf.urdf"
        )

        # Convert the URDF
        new_urdf_path = drakeify_my_urdf(
            urdf_file_path,
            overwrite_old_logs=True,
            log_file_name="bookshelf-conversion.log",
        )

        # Add the shelf to the plant
        model_idcs = Parser(plant=plant).AddModels(str(new_urdf_path))
        self.model_idx = model_idcs[0]

        # Weld the bookshelf to the world frame
        plant.WeldFrames(
            plant.world_frame(),
            plant.GetFrameByName("base_footprint", self.model_idx),
            self.shelf_pose,
        )

    def add_ur10e_station(self):
        """
        Add the Kinematic UR10e station to the scene for simple motion planning.
        :return:
        """
        # Setup
        builder = self.builder

        # Add station
        self.station = KinematicUR10eStation(
            time_step=self.time_step,
            meshcat_port_number=self.meshcat_port_number,
        )
        builder.AddSystem(self.station)

    @property
    def id(self) -> SceneID:
        return SceneID.kShelfPlanning1

    @property
    def start_pose(self):
        """
        Get the start pose. This should be defined by the subclass.
        :return:
        """
        start_position = np.array([-0.2, 0.9, 0.3])
        start_orientation = Quaternion(1, 0, 0, 0)
        return RigidTransform(start_orientation, start_position)

    @property
    def goal_pose(self):
        """
        Get the goal pose. This should be defined by the subclass.
        :return:
        """
        goal_position = np.array([+0.2, 1.0, 0.65])
        goal_orientation = Quaternion(1, 0, 0, 0)
        return RigidTransform(goal_orientation, goal_position)