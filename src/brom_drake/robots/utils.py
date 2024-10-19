"""
Description:
    This file contains a set of helpful utilities that can be used to help with the
    development of the robot models.
"""

from typing import List
import xml.etree.ElementTree as ET

def find_base_link_name_in(path_to_robot_model: str) -> str:
    """
    Description
    -----------
    This method will try to find a good base link for the model.
    :param path_to_robot_model: The path to the robot model.
    :return:
    """
    # Setup
    path_to_model = path_to_robot_model

    # Parse the model using xml
    if ".urdf" in path_to_model:
        original_xml = ET.ElementTree(file=path_to_model)
        link_names = find_all_link_names(original_xml)

        # Find a link that contains the word "base"
        for link_name in link_names:
            if "base" in link_name.lower():
                return link_name # Return the first one we find

    else:
        raise ValueError(
            "We can only smartly find base links in .urdf files, for now.\n" +
            "File an issue if you want more support in the future."
        )

    # If we can't find a base link, then we will raise an error
    raise ValueError(
        "We could not find a good base link in the model.\n" +
        "Please provide the base link name manually by adding \"base\" to one of the urdf's links."
    )


def find_all_link_names(xml_tree: ET.ElementTree ) -> List[str]:
    """
    Description
    -----------
    This method will find all the link names in the xml tree.
    :param xml_tree: The xml tree that we would like to investigate.
    :return:
    """
    # Setup
    link_names = []

    # Find all the links
    for link in xml_tree.findall(".//link"):
        link_names.append(link.attrib["name"])

    return link_names