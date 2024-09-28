from importlib import resources as impresources
import numpy as np
from pydrake.geometry import MeshcatVisualizer, Meshcat
from pydrake.math import RigidTransform
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph

# Internal Imports
import brom_drake.robots as robots
from brom_drake.scenes import SceneID
from brom_drake.scenes.types.motion_planning import OfflineMotionPlanningScene
from brom_drake.urdf import drakeify_my_urdf


class ShelfPlanningScene(OfflineMotionPlanningScene):
    def __init__(
        self,
        time_step=1e-3,
        use_meshcat: bool = True, # Usually turn off for CI
        **kwargs,
    ):
        super().__init__(**kwargs)

        self.time_step = time_step
        self.use_meshcat = use_meshcat
        self.shelf_position = np.array([-0.4, 0.6, 0.0])

        # If the base link name is not provided,
        # then we will try to do a smart search for it later.
        self.plant, self.scene_graph, self.meshcat = None, None, None

    def add_all_secondary_cast_members_to_builder(self, builder):
        """
        Add all secondary cast members to the builder.
        :param builder: The DiagramBuilder object where we will add all of the secondary cast members.
        :return:
        """
        # Setup
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(
            builder,
            time_step=self.time_step,
        )
        self.plant.set_name("ShowMeThisModel_plant")
        self.add_shelf()

        # Connect to Meshcat
        if self.use_meshcat:
            self.meshcat = Meshcat(port=7001)  # Object provides an interface to Meshcat
            m_visualizer = MeshcatVisualizer(self.meshcat)
            m_visualizer.AddToBuilder(builder, self.scene_graph, self.meshcat)

        # Finalize plant and connect it to system
        self.plant.Finalize()

    def add_shelf(self):
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
        model_idcs = Parser(plant=self.plant).AddModels(str(new_urdf_path))
        self.model_idx = model_idcs[0]

        # Weld the bookshelf to the world frame
        self.plant.WeldFrames(
            self.plant.world_frame(),
            self.plant.GetFrameByName("base_footprint", self.model_idx),
            RigidTransform(
                p=self.shelf_position,
            ),
        )

    @property
    def id(self) -> SceneID:
        return SceneID.kShelfPlanning1