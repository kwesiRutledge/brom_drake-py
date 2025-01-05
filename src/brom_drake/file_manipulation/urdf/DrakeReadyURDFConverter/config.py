from dataclasses import dataclass
from enum import IntEnum

class MeshReplacementStrategy(IntEnum):
    """
    The different strategies for replacing mesh files.
    """
    kDoNotReplace = 0 
    kWithObj = 1
    kWithMinimalEnclosingCylinder = 2

@dataclass(frozen=True)
class MeshReplacementStrategies:
    """
    Description
    -----------
    A dataclass that specifies the different strategies for replacing mesh files in:
    - the <collision> section of the URDF
    - the <visual> section of the URDF
    """
    collision_meshes: MeshReplacementStrategy = MeshReplacementStrategy.kWithObj
    visual_meshes: MeshReplacementStrategy = MeshReplacementStrategy.kWithObj

@dataclass
class DrakeReadyURDFConverterConfig:
    """
    Description
    -----------
    A dataclass that specifies how the DrakeReadyURDFConverter
    should convert the URDF file.
    """
    output_urdf_file_path: str = None
    overwrite_old_logs: bool = False
    overwrite_old_models: bool = False
    log_file_name: str = "conversion.log"
    mesh_replacement_strategies: MeshReplacementStrategies = MeshReplacementStrategies()
    add_missing_actuators: bool = True