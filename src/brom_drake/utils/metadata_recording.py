from pydrake.systems.framework import LeafSystem


def create_summary_for_LeafSystem(leaf_system: LeafSystem) -> dict:
    """
    Create a summary dictionary for a given LeafSystem.
    This can be used to record metadata about the system.
    """
    summary = {
        "name": leaf_system.GetSystemName(),
        "input_ports": [
            leaf_system.get_input_port(input_port_index).GetName() 
            for input_port_index in range(leaf_system.num_input_ports())
        ],
        "output_ports": [
            leaf_system.get_output_port(output_port_index).GetName() 
            for output_port_index in range(leaf_system.num_output_ports())
        ],
    }
    return summary
