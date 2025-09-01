from dataclasses import dataclass

@dataclass
class Configuration:
    name: str = "rigid_transform_to_vector_system"
    output_format: str = "vector_xyz_quat(wxyz)" # options: "vector_xyz_quat(wxyz)", "vector_xyz_euler(rpy)"