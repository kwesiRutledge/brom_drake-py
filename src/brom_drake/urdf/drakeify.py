"""
convenience.py
Description:

    This file contains a couple of convenience functions that are useful for the user of the URDF conversion
    feature.
"""

from typing import Union
from pathlib import Path

# Internal Imports
from .DrakeReadyURDFConverter import DrakeReadyURDFConverter

def drakeify_my_urdf(
    urdf_file_path: Union[str, Path],
    overwrite_old_logs: bool = False,
    log_file_name: str = "conversion.log",
) -> Path:
    """
    Description:

        This function provides a convenience function for the user to convert a URDF file
        into a "Drake-ready" URDF file.
    :param urdf_file:
    :param output_dir:
    :param overwrite_old_logs:
    :param log_file_name:
    :return:
    """
    # Input Processing
    if not isinstance(urdf_file_path, str) and not isinstance(urdf_file_path, Path):
        raise ValueError(
            f"urdf_file_path must be of type str or Path; received {urdf_file_path} of type {type(urdf_file_path)}."
        )

    if isinstance(urdf_file_path, str):
        urdf_file_path = Path(urdf_file_path)

    # Use converter
    converter = DrakeReadyURDFConverter(
        urdf_file_path,
        overwrite_old_logs=overwrite_old_logs,
        log_file_name=log_file_name,
    )

    # Convert the URDF
    new_urdf_path = converter.convert_urdf()

    return new_urdf_path
