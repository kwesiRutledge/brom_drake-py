from pydrake.all import (
    AbstractValue,
    BasicVector,
    Context,
    InputPort,
    LeafSystem,
)

class FlexiblePortSwitch(LeafSystem):
    """
    Description
    -----------
    This system is a simplification of the PortSwitch system primitive
    that is built into Drake. It allows for you to command the switch
    with a simple input (i.e., a string or an int) instead of
    a more complicated type.
    """

    def __init__(self, dim: int, selector_type_in: type = str):
        LeafSystem.__init__(self)
        
        # Setup
        self.dim = dim
        self.type_in = selector_type_in

        # Initialize the system with no input ports
        self._input_ports = {}
        self._current_input_port = None

        # Declare the port selector input port
        self.declare_port_selector_input_port("port_selector", selector_type_in)

        # Declare the value output port
        self.DeclareVectorOutputPort(
            "value",
            self.dim,
            self.DoCalcValue,
        )

    def declare_port_selector_input_port(self, name: str, type_in: type = str) -> InputPort:
        # Setup

        # Conditional based on type_in
        if type_in == str:
            self.port_selector_port = self.DeclareAbstractInputPort(
                "port_selector",
                AbstractValue.Make("model_name"),
            )
        else:
            raise NotImplementedError(
                f"Type {type_in} is not supported. Please use str."
            )

    def DeclareInputPort(self, name: str) -> InputPort:
        """
        Description
        -----------
        This method will declare an input port for the system.
        """
        # Setup
        input_port = self.DeclareVectorInputPort(
            name=name,
            size=self.dim,
        )
        
        # Update the input ports dictionary
        self._input_ports[name] = input_port
        
        return input_port
    
    def get_port_selector_input_port(self) -> InputPort:
        """
        Description
        -----------
        This method will return the port selector input port.
        """
        return self.port_selector_port
    
    def DoCalcValue(self, context: Context, output: BasicVector):
        """
        Description
        -----------
        This method will calculate the value of the output port.
        """
        # Setup

        # Get the port selector value
        port_selector = self.port_selector_port.Eval(context)

        # Vet the port selector value
        assert isinstance(port_selector, self.type_in), \
            f"Expected {self.type_in}; received {type(port_selector)}"
        assert len(self._input_ports) > 0, \
            "No input ports have been declared. Please declare at least one input port."

        # Get the input port
        input_port = self._input_ports[port_selector]

        # Get the value from the input port
        value = input_port.Eval(context)

        # Set the output value
        output.SetFrom(value)