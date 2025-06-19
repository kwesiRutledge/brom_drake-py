from pathlib import Path
import xml.etree.ElementTree as ET
from typing import Union

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

def create_transmission_element_for_joint(
    actuated_joint_name: str,
) -> ET.Element:
    """
    Description
    -----------
    This function creates a transmission element for the given joint.
    :param actuated_joint_name: The name of the joint that will be actuated
    :return: XML Element defining the new transmission for the actuated joint
    """

    # Create the transmission element
    transmission = ET.Element("transmission")

    # Create an inner type element
    transmission_type_element = ET.Element("type")
    transmission_type_element.text = "transmission_interface/SimpleTransmission"

    # Create inner joint element (reference to the actuated_joint_name
    joint_element = ET.Element("joint")
    joint_element.set("name", actuated_joint_name)

    # Create inner actuator element
    actuator_element = ET.Element("actuator")
    actuator_element.set("name", f"{actuated_joint_name}_actuator")

    # Assemble transmission element
    transmission.append(transmission_type_element)
    transmission.append(joint_element)
    transmission.append(actuator_element)

    return transmission

def tree_contains_transmission_for_joint(
    tree: ET.ElementTree,
    actuated_joint_name: str,
) -> bool:
    """
    Description
    -----------
    This function determines if the given tree contains a transmission
    element for the given actuated joint.
    :param tree:
    :param actuated_joint_name:
    :return:
    """
    # Setup
    root = tree.getroot()

    # Check to see if the root is a transmission element
    if root.tag == "transmission":
        # Check if the transmission element contains the actuated joint
        for child in root:
            if child.tag == "joint" and child.attrib["name"] == actuated_joint_name:
                return True

    # Check each of the tree's children to see if there is a transmission element
    for child in root:
        child_tree_contains_transmission_for_joint = tree_contains_transmission_for_joint(
            ET.ElementTree(child),
            actuated_joint_name,
        )
        if child_tree_contains_transmission_for_joint:
            return True

    # If we've searched through the full sub-tree and don't see the actuated joint,
    # then we return False
    return False

def get_mesh_element_in(collision_element: ET.Element) -> Union[ET.Element, None]:
    """
    Description
    -----------
    This function finds the mesh element in the given collision element.

    Parameters
    ----------
    collision_element: ET.Element
        The collision element to search for a mesh element
    
    Returns
    -------
    ET.Element or None
        The mesh element if found, otherwise None
    """
    # Check if the collision element has a <geometry> child
    geometry = collision_element.find("geometry")
    if geometry is not None:
        # Check if the geometry has a <mesh> child
        mesh = geometry.find("mesh")
        if mesh is not None:
            return mesh
    
    # If no mesh element is found, return None
    return None

def find_mesh_file_path_in(collision_element: ET.Element) -> Union[Path, None]:
    """
    Description
    -----------
    This function finds the mesh filename in the given collision element.

    Parameters
    ----------
    collision_element: ET.Element
        The collision element to search for a mesh filename
    
    Returns
    -------
    str or None
        The mesh filename if found, otherwise None
    """
    # Setup

    # Algorithm
    mesh_elt = get_mesh_element_in(collision_element)
    if mesh_elt is not None:
        # Return the filename attribute of the mesh element
        return Path(
            mesh_elt.attrib.get("filename", None)
        )
    
    # If no mesh filename is found, return None
    return None