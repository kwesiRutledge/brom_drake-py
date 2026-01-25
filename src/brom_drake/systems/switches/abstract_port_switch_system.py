from pydrake.common.value import AbstractValue
from pydrake.systems.framework import Context, InputPort, InputPortIndex, LeafSystem

class AbstractPortSwitch(LeafSystem):
    """
    *Description*
    
    This system is a simplification of the PortSwitch system primitive
    that is built into Drake. It allows for you to command the switch
    with a simple input (i.e., a string or an int) instead of
    a more complicated type.
    """

    def __init__(
        self,
        selector_type_in: type = int,
        input_type: type = str
    ):
        LeafSystem.__init__(self)
        
        # Setup
        self._selector_type_in = selector_type_in
        self._input_type = input_type

        # Initialize the system with all input ports
        self._input_ports = {}
        self._current_input_port = None

        # Declare the port selector input port
        self.declare_port_selector_input_port(self._selector_type_in)

        # Declare the value output port
        self.DeclareAbstractOutputPort(
            "value",
            lambda: AbstractValue.Make(input_type()),
            self.DoCalcValue,
        )

    def declare_port_selector_input_port(
        self,
        type_in: type = str
    ) -> InputPort:
        # Setup
        port_name = "port_selector"

        # Conditional based on type_in
        if type_in == str:
            self.port_selector_port = self.DeclareAbstractInputPort(
                port_name,
                AbstractValue.Make("model_name"),
            )
        elif type_in == int:
            self.port_selector_port = self.DeclareVectorInputPort(
                name=port_name,
                size=1,
            )
        elif type_in == InputPortIndex:
            self.port_selector_port = self.DeclareAbstractInputPort(
                port_name,
                AbstractValue.Make(InputPortIndex),
            )
        else:
            raise NotImplementedError(
                f"Type {type_in} is not supported. Please use str."
            )

    def DeclareInputPort(self, name: str) -> InputPort:
        """
        *Description*
        
        This method will declare an input port for the system.
        """
        # Setup
        selector_type_in = self._selector_type_in

        # Create the input port
        input_port = self.DeclareAbstractInputPort(
            name,
            AbstractValue.Make(self._input_type()),
        )

        # Update the input ports dictionary
        if selector_type_in == str:
            self._input_ports[name] = input_port
        elif selector_type_in == int:
            self._input_ports[selector_type_in] = input_port
        elif selector_type_in == InputPortIndex:
            self._input_ports[input_port.get_index()] = input_port
        else:
            raise NotImplementedError(
                f"Type {selector_type_in} is not supported. Please use str, int, or InputPortIndex."
            )
        
        return input_port
    
    def get_port_selector_input_port(self) -> InputPort:
        """
        *Description*
        
        This method will return the port selector input port.
        """
        return self.port_selector_port
    
    def DoCalcValue(self, context: Context, output: AbstractValue):
        """
        *Description*
        
        This method will calculate the value of the output port.
        """
        # Setup
        selector_type_in = self._selector_type_in

        # Get the port selector value
        port_selection = self.port_selector_port.Eval(context)
        if selector_type_in == int:
            port_selection = int(port_selection[0])

        # Vet the port selector value
        assert isinstance(port_selection, self._selector_type_in), \
            f"Expected {self._selector_type_in}; received {type(port_selection)}"
        
        assert len(self._input_ports) > 0, \
            "No input ports have been declared. Please declare at least one input port."

        # Get the input port
        input_port = self._input_ports[port_selection]

        # Get the value from the input port
        value = input_port.Eval(context)

        # Set the output value
        output.SetFrom(AbstractValue.Make(value))