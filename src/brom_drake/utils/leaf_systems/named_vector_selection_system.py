import numpy as np
from pydrake.all import AffineSystem
from typing import List

def define_named_vector_selection_system(
    all_element_names: List[str],
    sequence_of_names_for_output: List[str],
) -> AffineSystem:
    """
    *Description*
    
    This method creates an Affine System that will receive as input
    a vector where each element is named according to `all_element_names`
    and returns an output vector that selects only the elements
    specified in `desired_inputs`.

    *Parameters*
    
    name : str
        The name of the system to be added.

    all_element_names : List[str]
        The list of all element names in the input vector.

    desired_inputs : List[str]
        The list of desired element names to be selected for output.

    *Returns*
    
    AffineSystem
        The created NamedVectorSelectionSystem.
    """
    # Setup
    n_x = len(all_element_names)
    n_y = len(sequence_of_names_for_output)

    # Create the selection matrix
    D = np.zeros((n_y, n_x))
    for i, desired_name in enumerate(sequence_of_names_for_output):
        if desired_name not in all_element_names:
            raise ValueError(
                f"Desired input '{desired_name}' not found in all_element_names."
            )
        j = all_element_names.index(desired_name)
        D[i, j] = 1.0

    # Create the Affine System
    return AffineSystem(
        D=D,
    )