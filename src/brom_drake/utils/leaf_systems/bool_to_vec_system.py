import numpy as np
from pydrake.all import (
    AbstractValue, BasicVector,
    Context,
    LeafSystem,
)

class BoolToVectorSystem(LeafSystem):
    """
    Description
    -----------
    This system will take in a bool and output a vector
    of size 1.

    Diagram
    -------
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
        Description
        -----------
        This function takes in a bool and outputs a vector of size 1.
        """
        # Setup

        # Get the bool input
        bool_input = self.boolean_input_port.Eval(context)
        output.SetFromVector(
            np.array([int(bool_input)])
        )