from enum import IntEnum
from pathlib import Path
from typing import List
from pydrake.systems.framework import (
    LeafSystem,
    OutputPort,
    PortDataType,
)

class PathOrganizationConvention(IntEnum):
    """
    **Description**

    This enum is used to define the organization convention for the file paths.
    """

    kFlat = 0  # e.g. "plant_generalized_output_dim_0.png"
    kHierarchical = 1  # e.g. "system_plant/port_generalized_output/dim_0.png"

def compute_safe_system_name(system_name: str) -> str:
    """
    *Description*

    This function returns a filesystem-safe version of the system name.

    *Returns*

    safe_system_name: str
        The filesystem-safe version of the system name.
    """
    # First, let's check to see how many "/" exist in the name
    slash_occurences = [i for i, letter in enumerate(system_name) if letter == "/"]
    if len(slash_occurences) > 0:
        system_name = system_name[
            slash_occurences[-1] + 1 :
        ]  # truncrate string based on the last slash

    # Second, replace all spaces with underscores
    system_name = system_name.replace(" ", "_")

def generate_all_file_paths_for_ports_data(
    output_port: OutputPort,
    file_format: str,
    organization_convention: PathOrganizationConvention,
    dimension_names: dict[int, str] = None,
) -> List[Path]:
    """
    *Description*

    Generates ALL file path for the data of the given output port.
    For ports that have multiple dimensions, this will generate a file path for each dimension.
    For example, if the output port is a 3-dimensional vector, this function will generate 3 file paths,
    one for each dimension.

    *Parameters*

    output_port: OutputPort
        The output port for which to generate the file path.
    file_format: str
        The file format for the data (e.g., "npy", "csv", etc.).
    """
    # Identify the number of dimensions of the output port
    if output_port.get_data_type() == PortDataType.kVectorValued:
        num_dimensions = output_port.size()
    else:
        raise ValueError(f"Unsupported output port data type: {output_port.get_data_type()}")
    
    # Create dimension names dict, if it is not provided
    if dimension_names is None:
        dimension_names = {dimension: f"dim{dimension}" for dimension in range(num_dimensions)}

    # Generate file paths for each dimension
    file_paths = []
    for dimension, dimension_name in dimension_names.items():
        file_path = file_path_for_port_data_dimension(
            output_port=output_port,
            file_format=file_format,
            organization_convention=organization_convention,
            dimension_name=dimension_name,
        )
        file_paths.append(file_path)

    return file_paths

def file_path_for_port_data_dimension(
    output_port: OutputPort,
    file_format: str,
    dimension_name: str = None,
    organization_convention: PathOrganizationConvention = PathOrganizationConvention.kFlat,
) -> Path:
    """
    *Description*

    Generates a file path for the data of the given output port and dimension.

    *Parameters*

    output_port: OutputPort
        The output port for which to generate the file path.

    dimension_name: str, optional
        The name of the dimension of the output port for which to generate the file path.
        By default, this is set to None, which means that the file path will be generated for the first dimension of the output port.

    file_format: str
        The file format for the data (e.g., "npy", "csv", etc.).

    organization_convention: PathOrganizationConvention, optional
        The organization convention for the file paths. Defaults to PathOrganizationConvention.kFlat.
    """
    # Setup
    name_is_empty = dimension_name is None or dimension_name == ""

    # Collect System and Port Names
    # - System Name
    system: LeafSystem = output_port.get_system()
    system_name = system.get_name()
    safe_system_name = compute_safe_system_name(system_name)

    # - Port Name
    port_name = output_port.get_name()

    # Generate File Path
    if organization_convention == PathOrganizationConvention.kFlat:
        file_path = f"{safe_system_name}_{port_name}_{dimension_name}.{file_format}"
        file_path = file_path.replace("/", "_")  # Replace any "/" in the system or port names with "_"

        # Remove the dimension part of the file path if the output port is not vector-valued (i.e., if it only has one dimension)
        if name_is_empty:
            file_path = file_path.replace(f"_{dimension_name}", "")

    elif organization_convention == PathOrganizationConvention.kHierarchical:
        file_path = f"system_{safe_system_name}/port_{port_name}/{dimension_name}.{file_format}"

        # Remove the dimension part of the file path if the output port is not vector-valued (i.e., if it only has one dimension)
        if name_is_empty:
            file_path = file_path.replace(f"/{dimension_name}", "")

    else:
        raise ValueError(f"Unsupported organization convention: {organization_convention}")

    return Path(file_path)
