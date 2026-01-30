"""
Description
-----------
This file tests the BoolToVectorSystem class.
"""

import numpy as np
from pydrake.all import (
    AbstractValue,
    ConstantValueSource,
    DiagramBuilder,
    Simulator,
    VectorLogSink,
)
import unittest

# Internal Imports
from brom_drake.systems.conversion import BoolToVectorSystem

class BoolToVecSystemTest(unittest.TestCase):
    def test_in_sim1(self):
        # Setup
        builder = DiagramBuilder()

        # Create a system that outputs True always
        true_source = builder.AddSystem(
            ConstantValueSource(AbstractValue.Make(True))
        )

        # Attempt to connect a Boolean AbstractValue system to the BoolToVectorSystem
        b2vec_sys = builder.AddSystem(BoolToVectorSystem())
        b2vec_sys.set_name("BoolToVectorSystem")

        builder.Connect(true_source.get_output_port(0), b2vec_sys.get_input_port(0))

        # Connect the BoolToVectorSystem to the DiagramBuilder
        vector_sink = builder.AddSystem(VectorLogSink(1))
        builder.Connect(b2vec_sys.get_output_port(0), vector_sink.get_input_port(0))

        # Build the Diagram and simulate for 1 second
        diagram = builder.Build()
        diagram_context = diagram.CreateDefaultContext()
        simulator = Simulator(diagram, diagram_context)
        simulator.Initialize()
        simulator.AdvanceTo(1.0)

        # Get the logs of the output
        vector_log = vector_sink.FindLog(diagram_context)
        log_data = vector_log.data()
        self.assertEqual(log_data.shape, (1, 2))

        self.assertTrue(
            np.all(log_data == 1)
        )



if __name__ == "__main__":
    unittest.main()