from pydrake.common.value import AbstractValue
from pydrake.math import RigidTransform
from pydrake.systems.framework import Context, LeafSystem

class AbstractListSelectionSystem(LeafSystem):
    """
    *Description*

    This System receives selects a single element from an AbstractValue
    that contains a list. The index of the element to select is specified
    at construction time.

    *Diagram*

    The LeafSystem's input and output ports can be illustrated with the following block: ::

                            |---------------|
            list_in ------->|               |
            (List)          | List          |
                            | Selection     | ----> element_out
                            | System        |       (Element Type)
                            |               |
                            |---------------|

    """
    def __init__(
        self,
        index: int,
        output_type: type = RigidTransform,
        output_port_name: str = "element_out"
    ):
        LeafSystem.__init__(self)

        self.index = index

        # Define Input Ports
        self.list_input_port = self.DeclareAbstractInputPort(
            "list_in",
            AbstractValue.Make([output_type()])  # Placeholder; actual type will be checked at runtime
        )

        # Define Output Port
        self.DeclareAbstractOutputPort(
            output_port_name,
            lambda: AbstractValue.Make(output_type()),  # Placeholder; actual type will be checked at runtime
            self.CalcOutputElement,
        )
        
        self.set_name("ListSelectionSystem")   
        
    def CalcOutputElement(self, context: Context, output: AbstractValue):
        """
        *Description*

        This callback function computes the selected element from the input list.
        """
        # Retrieve current input value
        input_list = self.GetInputPort("list_in").Eval(context)

        assert isinstance(input_list, list), \
            f"Expected list_in port to contain \"list\" objects, but received type {type(input_list)}"
        
        # Check index validity
        if not (0 <= self.index < len(input_list)):
            raise IndexError(
                f"Index {self.index} is out of bounds for input list of length {len(input_list)}."
            )
        
        # Select the element at the specified index
        selected_element = input_list[self.index]

        # Set the output value
        output.SetFrom(
            AbstractValue.Make(selected_element)
        )