from enum import IntEnum
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

def generate_all_file_paths_for_ports_data(
    output_port: OutputPort,
    file_format: str,
    organization_convention: PathOrganizationConvention,
) -> str:
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
    
    # Generate file paths for each dimension
    file_paths = []
    for dimension in range(num_dimensions):
        file_path = file_path_for_port_data_dimension(
            output_port=output_port,
            dimension=dimension,
            file_format=file_format,
            organization_convention=organization_convention,
        )
        file_paths.append(file_path)

    return file_paths

def file_path_for_port_data_dimension(
    output_port: OutputPort,
    file_format: str,
    dimension: int = None,
    organization_convention: PathOrganizationConvention = PathOrganizationConvention.kFlat,
) -> str:
    """
    *Description*

    Generates a file path for the data of the given output port and dimension.

    *Parameters*

    output_port: OutputPort
        The output port for which to generate the file path.

    dimension: int, optional
        The dimension of the output port for which to generate the file path.
        By default, this is set to 0, which means that the file path will be generated for the first dimension of the output port.

    file_format: str
        The file format for the data (e.g., "npy", "csv", etc.).

    organization_convention: PathOrganizationConvention, optional
        The organization convention for the file paths. Defaults to PathOrganizationConvention.kFlat.
    """
    # Setup
    port_size = output_port.size() if output_port.get_data_type() == PortDataType.kVectorValued else 1

    # Collect System and Port Names
    # - System Name
    system: LeafSystem = output_port.get_system()
    system_name = system.get_name()

    # - Port Name
    port_name = output_port.get_name()

    # Generate File Path
    if organization_convention == PathOrganizationConvention.kFlat:
        file_path = f"{system_name}_{port_name}_dim_{dimension}.{file_format}"

        # Remove the dimension part of the file path if the output port is not vector-valued (i.e., if it only has one dimension)
        if port_size == 1:
            file_path = file_path.replace(f"_dim_{dimension}", "")

    elif organization_convention == PathOrganizationConvention.kHierarchical:
        file_path = f"system_{system_name}/port_{port_name}/dim_{dimension}.{file_format}"

        # Remove the dimension part of the file path if the output port is not vector-valued (i.e., if it only has one dimension)
        if port_size == 1:
            file_path = file_path.replace(f"/dim_{dimension}", "")

    else:
        raise ValueError(f"Unsupported organization convention: {organization_convention}")

    return file_path
