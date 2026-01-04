from brom_drake.utils.type_checking import is_rigid_transform
from pydrake.systems.framework import OutputPort, PortDataType
from typing import Any

def assert_port_is_supported(output_port: OutputPort):
    """
    *Description*
    
    Checks to see if the port is of the correct type for plotting.

    """
    # Algorithm
    if output_port.get_data_type() == PortDataType.kVectorValued:
        return
    
    # If the output port is kAbstractValued, then there are a specific
    # instantiations that we support.

    # Collect the example value for the port and then check that value
    example_allocation = output_port.Allocate()
    example_value = example_allocation.get_value()

    assert_abstract_value_ports_type_is_supported(example_value)

def assert_abstract_value_ports_type_is_supported(output_port_value: Any):
    # Check to see if AbstractValue port contains RigidTransform
    if is_rigid_transform(output_port_value):
        return
    elif type(output_port_value) == bool: # if the output_value is a boolean
        return

    # Check to see if the object is a list of one of the supported types
    if type(output_port_value) is list:
        # Then check every element of the list
        for example_elt in output_port_value:
            assert_abstract_value_ports_type_is_supported(
                output_port_value=example_elt
            )

        # If all of the above checks passed,
        # then return! We are good!
        return        

    # Raise error otherwise
    raise create_port_value_type_error(output_port_value)


def create_port_value_type_error(example_value: Any) -> ValueError:
    """
    *Description*
    
    Creates an error message for the port value type.
    
    *Parameters*
    
    example_value: Any
        The type of value in the problematic output port.

    *Returns*
    
    error_out: ValueError
        The error message.
    """
    # Return
    return ValueError(
        f"This watcher only supports output ports that are:\n" +
        f"- Vector valued ports (i.e., of type {PortDataType.kVectorValued}.\n" +
        f"- Abstract valued ports containing:\n" +
        f"  + RigidTransforms\n" +
        f"  + Booleans\n" +
        f"  + list[T], where T is a type from the above list."
        f"Received port of type {PortDataType.kAbstractValued} with underlying type {type(example_value)}."
    )