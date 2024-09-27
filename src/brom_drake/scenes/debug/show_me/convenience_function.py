from pathlib import Path
from typing import Union, List, Tuple

from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder, Diagram, Context

#Internal Imports
from brom_drake.all import add_watcher_and_build
from .show_me_system import ShowMeSystem
from .show_me_this_model import ShowMeThisModel

def show_me_this_model_in_sim(
    path_to_model: Union[str, Path],
    desired_joint_positions: List[float],
    time_step: float = 1e-3,
    base_link_name: str = None,
)-> Tuple[ShowMeThisModel, Diagram, Context, Simulator]:
    # Setup
    builder = DiagramBuilder()

    # Define Scene
    scene = ShowMeThisModel(
        path_to_model=str(path_to_model),
        desired_joint_positions=desired_joint_positions,
        base_link_name=base_link_name,
        time_step=time_step,
    )

    diagram = scene.cast_scene_and_build(builder)
    diagram_context = diagram.CreateDefaultContext()

    # Set up simulation
    simulator = Simulator(diagram)
    simulator_context = simulator.get_mutable_context()
    scene.show_me_system.mutable_plant_context = scene.plant.GetMyMutableContextFromRoot(
        simulator_context,
    )

    return scene, diagram, diagram_context, simulator
