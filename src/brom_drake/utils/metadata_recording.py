from pydrake.systems.framework import LeafSystem


def create_summary_for_LeafSystem(leaf_system: LeafSystem) -> dict:
    """
    Create a summary dictionary for a given LeafSystem.
    This can be used to record metadata about the system.
    """
    summary = {
        "input_ports": [
            leaf_system.get_input_port(input_port_index).get_name() 
            for input_port_index in range(leaf_system.num_input_ports())
        ],
        "output_ports": [
            leaf_system.get_output_port(output_port_index).get_name() 
            for output_port_index in range(leaf_system.num_output_ports())
        ],
    }
    return summary
