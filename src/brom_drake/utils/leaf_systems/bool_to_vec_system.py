import numpy as np
from pydrake.all import (
    AbstractValue, BasicVector,
    Context,
    LeafSystem,
)

class BoolToVectorSystem(LeafSystem):
    """
    *Description*
    
    This LeafSystem receives boolean inputs (i.e., AbstractValue inputs
    containing bool values) and returns an output vector of size 1 (i.e., BasicVector) where the
    single element is 1 if the input bool is True and 0 if the input bool
    is False.

    *Diagram*
    
    The LeafSystem's input and output ports can be illustrated with the following block: ::

                        |---------------|
                        | Bool          |
        bool_in ---->   | To            | ---> vector_out 
        (bool)          | VectorSystem  | (BasicVector[1])
                        |---------------|

    """
    def __init__(self):
        LeafSystem.__init__(self)
        self.boolean_input_port = self.DeclareAbstractInputPort(
            "bool_in",
            AbstractValue.Make(False),
        )
        self.DeclareVectorOutputPort(
            "vector_xyz_quat(wxyz)",
            1,
            self.CalcVectorOutput,
        )

    def CalcVectorOutput(self, context: Context, output: BasicVector):
        """
        *Description*
        
        Callback function for calculating the output vector of the `BoolToVectorSystem`.
        This function takes in a bool and outputs a vector of size 1.
        """
        # Setup

        # Get the bool input
        bool_input = self.boolean_input_port.Eval(context)
        output.SetFromVector(
            np.array([int(bool_input)])
        )