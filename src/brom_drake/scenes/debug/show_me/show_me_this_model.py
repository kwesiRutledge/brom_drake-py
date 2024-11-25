from typing import List, Tuple

import numpy as np
from manipulation.scenarios import AddMultibodyTriad
from pydrake.geometry import Meshcat, MeshcatVisualizer, MeshcatVisualizerParams
from pydrake.geometry import Role as DrakeRole
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder, Diagram, Context

from pydrake.systems.primitives import ConstantVectorSource, VectorLogSink

# Internal Imports
from brom_drake.robots import find_base_link_name_in
from brom_drake.scenes.types import BaseScene
from brom_drake.scenes import SceneID
from brom_drake.scenes.roles.role import Role
from brom_drake.utils import Performer
from .show_me_system import ShowMeSystem

class ShowMeThisModel(BaseScene):
    def __init__(
        self,
        path_to_model: str,
        with_these_joint_positions: List[float] = None,
        base_link_name: str = None,
        time_step: float = 1e-3,
        meshcat_port_number: int = 7001, # Usually turn off for CI (i.e., make it None)
        show_collision_geometries: bool = False,
        **kwargs,
    ):
        super().__init__(**kwargs)

        # Add the model to the scene
        self.path_to_model = path_to_model
        self.q_des = with_these_joint_positions
        self.base_link_name = base_link_name
        self.time_step = time_step
        self.meshcat_port_number = meshcat_port_number
        self.show_collision_geometries = show_collision_geometries

        # If the base link name is not provided,
        # then we will try to do a smart search for it later.
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(
            self.builder,
            time_step=self.time_step,
        )
        self.plant.set_name("ShowMeThisModel_plant")

        self.meshcat = None
        self.model_index, self.model_name = None, None
        self.show_me_system = None

    def add_all_secondary_cast_members_to_builder(self):
        """
        Description
        -----------
        This method will add just the user's model to the builder.

        :return:
        """
        # Setup

        # Add Model
        model_idcs = Parser(plant=self.plant).AddModels(self.path_to_model)
        assert len(model_idcs) == 1, \
            f"Only one model should be added in this scene;" + \
            f" received {len(model_idcs)} in the file {self.path_to_model}."
        self.model_index = model_idcs[0]
        self.model_name = self.plant.GetModelInstanceName(model_idcs[0])

        # Collect the expected number of actuated joints
        n_dofs = self.plant.num_actuated_dofs()
        if self.q_des is None:
            self.q_des = np.zeros(n_dofs)
        else:
            assert len(self.q_des) == n_dofs, \
                f"Expected {n_dofs} joint positions; received {len(self.q_des)}."

        # Add ShowMeSystem
        self.show_me_system = self.builder.AddSystem(
            ShowMeSystem(
                plant=self.plant,
                model_index=self.model_index,
                desired_joint_positions=np.array(self.q_des),
            ),
        )

        # Add Source
        desired_joint_positions_source = self.builder.AddSystem(
            ConstantVectorSource(np.array(self.q_des)),
        )

        # Connect the source to the system
        self.builder.Connect(
            desired_joint_positions_source.get_output_port(0),
            self.show_me_system.get_input_port(0),
        )

        # Add Sink
        output_joints_sink = self.builder.AddSystem(
            VectorLogSink(len(self.q_des)),
        )

        # Connect the system to the sink
        self.builder.Connect(
            self.show_me_system.get_output_port(0),
            output_joints_sink.get_input_port(0),
        )

        # Add A Triad to base?
        AddMultibodyTriad(self.plant.world_frame(), self.scene_graph)

        # Try to collect the base link name if it is not provided
        if self.base_link_name is None:
            self.base_link_name = find_base_link_name_in(self.path_to_model)

        # Weld the base link to the world frame
        self.plant.WeldFrames(
            self.plant.world_frame(),
            self.plant.GetFrameByName(self.base_link_name, self.model_index),
        )

        # Connect to Meshcat, if requested
        if self.meshcat_port_number is not None:
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

        # Finalize plant and connect it to system
        self.plant.Finalize()

    def cast_scene_and_build(
        self,
        cast: Tuple[Role, Performer] = [],
    ) -> Tuple[Diagram, Context]:
        super().cast_scene_and_build(cast)

        # Assign the diagram context to the internal show_me_system
        self.show_me_system.mutable_plant_context = self.plant.GetMyMutableContextFromRoot(
            self.diagram_context,
        )

        return self.diagram, self.diagram_context

    @property
    def id(self):
        return SceneID.kShowMeThisModel

    @property
    def suggested_roles(self) -> List[Role]:
        return []
