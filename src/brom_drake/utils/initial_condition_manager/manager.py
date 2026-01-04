from dataclasses import dataclass, field
import numpy as np
from pydrake.all import (
    Context,
    Frame,
    ModelInstanceIndex,
    MultibodyPlant,
    RigidTransform,
)
from typing import List

# Internal Imports
from brom_drake.utils.initial_condition_manager.initial_condition import InitialCondition

@dataclass
class InitialConditionManager:
    _ic_tuples: List[InitialCondition] = field(default_factory=list)

    def add_initial_condition(self, ic_tuple: InitialCondition):
        self._ic_tuples.append(ic_tuple)

    def add_initial_configuration(self, model_instance_index: ModelInstanceIndex, configuration: np.ndarray):
        ic_tuple = InitialCondition(
            model_instance_index=model_instance_index,
            configuration=configuration
        )
        self._ic_tuples.append(ic_tuple)

    def add_initial_pose(self, model_instance_index: ModelInstanceIndex, pose_wrt_parent: RigidTransform):
        ic_tuple = InitialCondition(
            model_instance_index=model_instance_index,
            pose_wrt_parent=pose_wrt_parent
        )
        self._ic_tuples.append(ic_tuple)

    def set_all_initial_conditions(self, plant: MultibodyPlant, diagram_context: Context = None):
        """
        *Description*
        
        Set all initial conditions (both the pose and configuration) 
        for the given plant.
        """
        self.set_all_initial_configurations(plant, diagram_context=diagram_context)
        self.set_all_initial_poses(plant, diagram_context=diagram_context)

    def set_all_initial_configurations(self, plant: MultibodyPlant, diagram_context: Context = None):
        """Set all known initial configurations for the given plant."""
        for ic_tuple in self._ic_tuples:
            ic_tuple.set_initial_configuration(plant, diagram_context=diagram_context)


    def set_all_initial_poses(self, plant: MultibodyPlant, diagram_context: Context = None):
        """Set all known initial poses for the given plant."""
        for ic_tuple in self._ic_tuples:
            ic_tuple.set_initial_pose(plant, diagram_context=diagram_context)