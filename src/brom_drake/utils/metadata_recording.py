from pydrake.systems.framework import LeafSystem


def create_summary_for_LeafSystem(leaf_system: LeafSystem) -> dict:
    """
    Create a summary dictionary for a given LeafSystem.
    This can be used to record metadata about the system.
    """
    summary = {
        "name": leaf_system.GetSystemName(),
        "input_ports": [port.GetName() for port in leaf_system.get_input_ports()],
        "output_ports": [port.GetName() for port in leaf_system.get_output_ports()],
    }
    return summary
