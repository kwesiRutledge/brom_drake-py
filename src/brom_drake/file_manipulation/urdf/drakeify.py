"""
convenience.py
Description:

    This file contains a couple of convenience functions that are useful for the user of the URDF conversion
    feature.
"""

from typing import Union
from pathlib import Path

# Internal Imports
from .DrakeReadyURDFConverter.converter import DrakeReadyURDFConverter
from brom_drake.file_manipulation.urdf.DrakeReadyURDFConverter.config import (
    MeshReplacementStrategy, MeshReplacementStrategies, DrakeReadyURDFConverterConfig
)

def drakeify_my_urdf(
    urdf_file_path: Union[str, Path],
    overwrite_old_logs: bool = False,
    overwrite_old_models: bool = False,
    log_file_name: str = "conversion.log",
    collision_mesh_replacement_strategy: MeshReplacementStrategy = MeshReplacementStrategy.kWithObj,
) -> Path:
    """
    Description
    -----------
    This function provides a convenience function for the user to convert a URDF file
    into a "Drake-ready" URDF file.

    Arguments
    ---------
    urdf_file_path: str
        A string representing the path to the URDF file that you would like to convert.
    overwrite_old_models: bool (optional)
        A boolean flag that indicates whether or not to overwrite old models.
        Default is False.
    overwrite_old_logs: bool (optional)
        A boolean flag that indicates whether or not to overwrite old logs.
        Default is False.
    log_file_name: str (optional)
        A string representing the name of the log file.
        Default is "conversion.log".
    collision_mesh_replacement_strategy: MeshReplacementStrategy (optional)
        An enum representing the strategy for replacing collision meshes.
        Default is MeshReplacementStrategy.kWithObj.
    :return:
    """
    # Input Processing
    if not isinstance(urdf_file_path, str) and not isinstance(urdf_file_path, Path):
        raise ValueError(
            f"urdf_file_path must be of type str or Path; received {urdf_file_path} of type {type(urdf_file_path)}."
        )

    if isinstance(urdf_file_path, str):
        urdf_file_path = Path(urdf_file_path)

    # Create config for converter
    config = DrakeReadyURDFConverterConfig(
        overwrite_old_logs=overwrite_old_logs,
        overwrite_old_models=overwrite_old_models,
        log_file_name=log_file_name,
        mesh_replacement_strategies=MeshReplacementStrategies(
            collision_meshes=collision_mesh_replacement_strategy,
        ),
    )

    # Use converter
    converter = DrakeReadyURDFConverter(
        urdf_file_path,
        config=config,
    )

    # Convert the URDF
    new_urdf_path = converter.convert_urdf()

    return new_urdf_path
