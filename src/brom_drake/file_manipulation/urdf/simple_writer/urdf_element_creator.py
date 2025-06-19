from pathlib import Path
from pydrake.all import Quaternion, RigidTransform
import numpy as np
from typing import Union
from xml.etree import ElementTree as ET

# Internal imports
from brom_drake.file_manipulation.urdf.shapes.shape_definition import ShapeDefinition

class URDFElementCreator:
    """
    A class to create URDF elements quickly and easily.
    """
    @staticmethod
    def create_collision_element(
        name: str,
        pose_ParentMesh: Union[None, np.ndarray, RigidTransform] = None,
        mesh_file_path: Union[str, None] = None,
        mesh_scale: np.ndarray = np.array([1.0, 1.0, 1.0]),
    ) -> ET.Element:
        """
        Create a collision element for the URDF.
        
        Parameters
        ----------
        name: str
            The name of the collision element.
        pose_ParentMesh: Union[None, np.ndarray, RigidTransform], optional
            The pose of the parent mesh in the collision element. If None, a default RigidTransform is used.
            If provided as a numpy array, it should be a 7-element array representing position and quaternion.
            (Specifically, the first 3 elements are the position, and the last 4 elements are the quaternion [x,y,z,w].)
        
        Returns
        -------
        ET.Element
            The created collision element.
        """
        # Setup

        if pose_ParentMesh is None:
            pose_ParentMesh = RigidTransform()
        
        if isinstance(pose_ParentMesh, np.ndarray):
            assert pose_ParentMesh.shape == (7,), \
                "pose_ParentMesh should be a 7-element array representing a RigidTransform."
            pose_ParentMesh: RigidTransform = RigidTransform(
                p = pose_ParentMesh[:3],  # Position (x, y, z)
                quaternion= Quaternion(pose_ParentMesh[[-1,3,4,5]]) # Quaternion given as (w, x, y, z)
            )

        if mesh_file_path is None:
            raise NotImplementedError(
                f"Mesh file path is currently required for creating a collision element."
            )

        # Create the collision element
        collision = ET.Element("collision", {"name": name})

        # Add the origin sub-element to the collision element
        collision.append(
            URDFElementCreator.create_origin_element(
                pose_ParentOrigin=pose_ParentMesh
            )
        )

        # If a mesh file path is provided, add a geometry sub-element
        # with a mesh sub-element inside it
        if mesh_file_path is not None:
            collision.append(
                URDFElementCreator.create_geometry_element(
                    mesh_file_path=mesh_file_path,
                    mesh_scale=mesh_scale,
                )
            )

        return collision
    
    @staticmethod
    def create_origin_element(
        pose_ParentOrigin: RigidTransform,
    ) -> ET.Element:
        """
        Create an origin element for the URDF.
        
        Parameters
        ----------
        translation: np.ndarray
            The translation vector (x, y, z).
        rotation: Quaternion
            The rotation as a Quaternion.
        
        Returns
        -------
        ET.Element
            The created origin element.
        """
        # Setup
        translation = pose_ParentOrigin.translation()
        rotation = pose_ParentOrigin.rotation()
        rotation_as_rpy = rotation.ToRollPitchYaw()

        # Create the origin element
        origin = ET.Element("origin")
        origin.set("xyz", f"{translation[0]} {translation[1]} {translation[2]}")
        origin.set("rpy", f"{rotation_as_rpy.roll_angle()} {rotation_as_rpy.pitch_angle()} {rotation_as_rpy.yaw_angle()}")
        
        return origin
    
    @staticmethod   
    def create_geometry_element(
        mesh_file_path: str = None,
        definition: ShapeDefinition = None,
        mesh_scale: np.ndarray = np.array([1.0, 1.0, 1.0])
    ) -> ET.Element:
        """
        Create a geometry element for the URDF.
        
        Parameters
        ----------
        mesh_file_path: Path, optional
            The path to the mesh file. If provided, a mesh sub-element will be created.
        definition: ShapeDefinition, optional
            The dimensions of the geometry. Required for box and cylinder types.
        
        Returns
        -------
        ET.Element
            The created geometry element.
        """
        # Create the geometry element
        geometry = ET.Element("geometry")
        
        # If a path to a mesh file is provided, add a mesh sub-element
        if mesh_file_path is not None:
            mesh = ET.SubElement(geometry, "mesh")
            mesh.set("filename", str(mesh_file_path))
            # # If the mesh file path is relative, make it absolute
            # if not str(mesh_file_path).startswith("/"):
            #     mesh.set("filename", f"{mesh_file_path}")

            # Set the scale for the mesh
            mesh.set("scale", f"{mesh_scale[0]} {mesh_scale[1]} {mesh_scale[2]}")
            

        # If a shape definition is provided, add the appropriate sub-element
        # TODO: Implement support for different shape definitions
        
        return geometry
