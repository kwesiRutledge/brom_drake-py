

URDF_CONVERSION_LOG_LEVEL_NAME = "BROM_URDF_CONVERSION"
URDF_CONVERSION_LEVEL = 21

def does_drake_parser_support(filename: str):
    """
    Description
    -----------
    This function cleanly answers whether the given 3d object
    file is supported by Drake.
    :param filename:
    :return:
    """
    return ".obj" in filename # TODO(kwesi): Determine if .sdf should be put here.