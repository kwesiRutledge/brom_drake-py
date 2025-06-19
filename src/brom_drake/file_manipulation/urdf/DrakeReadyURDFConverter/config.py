from dataclasses import dataclass
from enum import IntEnum
from typing import List

class MeshReplacementStrategy(IntEnum):
    """
    The different strategies for replacing mesh files.
    """
    kDoNotReplace = 0 
    kWithObj = 1
    kWithMinimalEnclosingCylinder = 2
    kWithConvexDecomposition = 3

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

    Attributes
    ----------
    output_urdf_file_path: str
        The path where the converted URDF file will be saved. If None, it defaults to the models directory.
    coacd_log_level: str
        The log level for the coacd tool. Options are "off", "info", "error".
    """
    output_urdf_file_path: str = None
    overwrite_old_logs: bool = False
    overwrite_old_models: bool = False
    log_file_name: str = "conversion.log"
    mesh_replacement_strategies: MeshReplacementStrategies = MeshReplacementStrategies()
    add_missing_actuators: bool = True
    replace_colors_with: List[float] = None
    coacd_log_level: str = "info"