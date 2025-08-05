from dataclasses import dataclass
from pathlib import Path
from pydrake.geometry.optimization import HPolyhedron
from pydrake.all import ModelInstanceIndex

# Internal Imports
from brom_drake.utils.pick_and_place.phase import PickAndPlacePhase

@dataclass
class PickAndPlaceTargetDescription:
    file_path: Path
    goal_region: HPolyhedron
    model_instance_index: ModelInstanceIndex = None
    _name: str = None

    @property
    def name(self) -> str:
        # If a name was provided, then provide that instead.
        if self._name is not None:
            return self._name
        
        # If a name was not provided, then extract a name from the file_path.
        file_name = self.file_path.name
        return file_name[:file_name.find('.')]
        
    def name_for_pick_and_place_phase(self, phase: PickAndPlacePhase) -> str:
        return self.name + "_" + phase.to_str()
