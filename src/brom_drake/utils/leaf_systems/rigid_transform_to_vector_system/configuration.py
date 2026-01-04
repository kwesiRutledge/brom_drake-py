from dataclasses import dataclass

@dataclass
class Configuration:
    """
    *Description*

    A dataclass that specifies the configuration for the RigidTransformToVectorSystem.

    *Attributes*

    name: str
        The name of the system.

    output_format: str, optional
        The format of the output vector. Options are:

        - "vector_xyz_quat(wxyz)"
        - "vector_xyz_euler(rpy)"

        This determines if the output vector will represent orientation as a quaternion or as roll-pitch-yaw Euler angles.
        If the output vector is in quaternion format, then the output will be 7-dimensional.
        If the output vector is in Euler angle format, then the output will be 6-dimensional.

        By default, this is set to "vector_xyz_quat(wxyz)".
    """
    name: str = "rigid_transform_to_vector_system"
    output_format: str = "vector_xyz_quat(wxyz)" # options: "vector_xyz_quat(wxyz)", "vector_xyz_euler(rpy)"