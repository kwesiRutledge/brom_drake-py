from typing import List

import numpy as np
from manipulation.scenarios import AddMultibodyTriad
from pydrake.geometry import Meshcat, MeshcatVisualizer
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder

import xml.etree.ElementTree as ET

from pydrake.systems.primitives import ConstantVectorSource, VectorLogSink

# Internal Imports
from brom_drake.scenes.types import BaseScene
from brom_drake.scenes import SceneID
from brom_drake.scenes.roles.role import Role
from .show_me_system import ShowMeSystem

class ShowMeThisModel(BaseScene):
    def __init__(
        self,
        path_to_model: str,
        desired_joint_positions: List[float],
        base_link_name: str = None,
        time_step: float = 1e3,
        **kwargs,
    ):
        super().__init__(**kwargs)

        # Add the model to the scene
        self.path_to_model = path_to_model
        self.q_des = desired_joint_positions
        self.base_link_name = base_link_name
        self.time_step = time_step

        # If the base link name is not provided,
        # then we will try to do a smart search for it later.
        self.plant, self.scene_graph, self.meshcat = None, None, None
        self.model_idx, self.model_name = None, None
        self.show_me_system = None

    def add_all_secondary_cast_members_to_builder(self, builder: DiagramBuilder):
        """
        Description
        -----------
        This method will add just the user's model to the builder.

        :param builder: A DiagramBuilder object from Drake.
        :return:
        """
        # Setup
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(
            builder,
            time_step=self.time_step,
        )
        self.plant.set_name("ShowMeThisModel_plant")
        self.meshcat = Meshcat(port=7001)  # Object provides an interface to Meshcat

        # Add Model
        model_idcs = Parser(plant=self.plant).AddModels(self.path_to_model)
        assert len(model_idcs) == 1, \
            f"Only one model should be added in this scene;" + \
            f" received {len(model_idcs)} in the file {self.path_to_model}."
        self.model_idx = model_idcs[0]
        self.model_name = self.plant.GetModelInstanceName(model_idcs[0])

        # Add ShowMeSystem
        self.show_me_system = builder.AddSystem(
            ShowMeSystem(
                plant=self.plant,
                model_name=self.model_name,
                desired_joint_positions=np.array(self.q_des),
            ),
        )

        # Add Source
        desired_joint_positions_source = builder.AddSystem(
            ConstantVectorSource(np.array(self.q_des)),
        )

        # Connect the source to the system
        builder.Connect(
            desired_joint_positions_source.get_output_port(0),
            self.show_me_system.get_input_port(0),
        )

        # Add Sink
        output_joints_sink = builder.AddSystem(
            VectorLogSink(len(self.q_des)),
        )

        # Connect the system to the sink
        builder.Connect(
            self.show_me_system.get_output_port(0),
            output_joints_sink.get_input_port(0),
        )

        # Add A Triad to base?
        AddMultibodyTriad(self.plant.world_frame(), self.scene_graph)

        # Try to collect the base link name if it is not provided
        if self.base_link_name is None:
            self.base_link_name = self.find_a_good_base_link_name()

        # Weld the base link to the world frame
        self.plant.WeldFrames(
            self.plant.world_frame(),
            self.plant.GetFrameByName(self.base_link_name, self.model_idx),
        )

        # Connect to Meshcat
        m_visualizer = MeshcatVisualizer(self.meshcat)
        m_visualizer.AddToBuilder(builder, self.scene_graph, self.meshcat)

        # Finalize plant and connect it to system
        self.plant.Finalize()

    def find_a_good_base_link_name(self)->str:
        """
        Description
        -----------
        This method will try to find a good base link for the model.
        :return:
        """
        # Setup

        # Parse the model using xml
        if ".urdf" in self.path_to_model:
            original_xml = ET.ElementTree(file=self.path_to_model)
            link_names = self.find_all_link_names(original_xml)

            # Find a link that contains the word "base"
            for link_name in link_names:
                if "base" in link_name.lower():
                    return link_name # Return the first one we find

        else:
            raise ValueError(
                "We can only smartly find base links in .urdf files, for now.\n" +
                "File an issue if you want more support in the future."
            )

        # If we can't find a base link, then we will raise an error
        raise ValueError(
            "We could not find a good base link in the model.\n" +
            "Please provide the base link name manually by adding \"base\" to one of the urdf's links."
        )


    # TODO(kwesi): Maybe move this to a utilities file?
    @staticmethod
    def find_all_link_names(xml_tree: ET.ElementTree)->List[str]:
        """
        Description
        -----------
        This method will find all the link names in the xml tree.
        :param xml_tree: The xml tree that we would like to investigate.
        :return:
        """
        # Setup
        link_names = []

        # Find all the links
        for link in xml_tree.findall(".//link"):
            link_names.append(link.attrib["name"])

        return link_names

    @property
    def id(self):
        return SceneID.kShowMeThisModel

    @property
    def suggested_roles(self) -> List[Role]:
        return []
