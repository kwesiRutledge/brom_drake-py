from brom_drake.systems.switches.flexible_port_switch import FlexiblePortSwitch
import numpy as np
from pydrake.all import (
    BasicVector,
    DiagramBuilder,
    InputPort,
    InputPortIndex,
    LeafSystem,
    PortDataType,
    Simulator,
    ConstantVectorSource,
)
import unittest


class TestFlexiblePortSwitch(unittest.TestCase):
    def test_init1(self):
        """
        Description
        -----------
        This test verifies that we can initialize the FlexiblePortSwitch without any errors.
        """
        # Setup
        switch0 = FlexiblePortSwitch(2)

        # Check that the switch is initialized correctly (i.e., that
        # - the output port has the correct size (2)
        # - there is only one input port (it should not be a vector input port; it should be abstract)

        self.assertEqual(switch0.get_output_port(0).size(), 2)
        self.assertEqual(switch0.num_input_ports(), 1)
        input_port0: InputPort = switch0.get_input_port(0)
        self.assertTrue(input_port0.get_data_type() == PortDataType.kAbstractValued)

    def test_init2(self):
        """
        Description
        -----------
        This test verifies that an exception is raised if we attempt to create
        a FlexiblePortSwitch with a selector type that is not supported.
        In this case, we will use a LeafSystem as the selector type.
        """

        try:
            switch0 = FlexiblePortSwitch(2, selector_type_in=LeafSystem)
            self.fail("Expected NotImplementedError not raised.")
        except NotImplementedError as e:
            self.assertTrue(True)

    def test_DeclareInputPort1(self):
        """
        Description
        -----------
        This test verifies that we can declare an input port without any errors.
        We will check that the input port changes the number of input ports
        of the FlexiblePortSwitch AND that the input port is of the correct type
        with the correct dimensions.
        """
        # Setup
        switch0 = FlexiblePortSwitch(2)

        # Check that the switch is initialized correctly (i.e., that
        # - the output port has the correct size (2)
        # - there is only one input port (it should not be a vector input port; it should be abstract)

        self.assertEqual(switch0.get_output_port(0).size(), 2)
        self.assertEqual(switch0.num_input_ports(), 1)
        input_port0: InputPort = switch0.get_input_port(0)

        self.assertTrue(input_port0.get_data_type() == PortDataType.kAbstractValued)

        # Declare the input port
        port_name = "input_port_0"
        switch0.DeclareInputPort(name=port_name)
        self.assertEqual(switch0.num_input_ports(), 2)

        # check that the input port named "input_port_0" is a vector input port
        input_port1: InputPort = switch0.GetInputPort(port_name)
        self.assertTrue(input_port1.get_data_type() == PortDataType.kVectorValued)
        self.assertEqual(input_port1.size(), 2)

    def test_DeclareInputPort2(self):
        """
        Description
        -----------
        This test verifies that we can correctly create a FlexiblePortSwitch
        with a switch type of int.
        We will create a FlexiblePortSwitch that produces a vector osf size 7
        and will declare 3 input ports for it.
        """
        # Setup
        switch0 = FlexiblePortSwitch(7, selector_type_in=int)

        # Check that the switch is initialized correctly (i.e., that
        # - the output port has the correct size (7)
        # - there is only one input port (it should not be a vector input port; it should be abstract)
        self.assertEqual(switch0.get_output_port(0).size(), 7)
        self.assertEqual(switch0.num_input_ports(), 1)
        input_port0: InputPort = switch0.get_input_port(0)
        self.assertTrue(input_port0.get_data_type() == PortDataType.kVectorValued)

        # Declare three input ports with unique names
        port_names = ["input_port_0", "input_port_1", "input_port_2"]
        for port_name in port_names:
            switch0.DeclareInputPort(name=port_name)

        # Check that the number of input ports is now 4
        self.assertEqual(switch0.num_input_ports(), 4)

        # Check that the input ports are of the correct type
        for port_name in port_names:
            input_port: InputPort = switch0.GetInputPort(port_name)
            self.assertTrue(input_port.get_data_type() == PortDataType.kVectorValued)
            self.assertEqual(input_port.size(), 7)

    def test_DeclareInputPort3(self):
        """
        Description
        -----------
        This test verifies that we can correctly create a FlexiblePortSwitch
        with a switch type of InputPortIndex.
        We will create a FlexiblePortSwitch that produces a vector osf size 7
        and will declare 3 input ports for it.
        """
        # Setup
        switch0 = FlexiblePortSwitch(7, selector_type_in=InputPortIndex)

        # Check that the switch is initialized correctly (i.e., that
        # - the output port has the correct size (7)
        # - there is only one input port (it should not be a vector input port; it should be abstract)
        self.assertEqual(switch0.get_output_port(0).size(), 7)
        self.assertEqual(switch0.num_input_ports(), 1)
        input_port0: InputPort = switch0.get_input_port(0)
        self.assertTrue(input_port0.get_data_type() == PortDataType.kAbstractValued)

        # Declare three input ports with unique names
        port_names = ["input_port_0", "input_port_1", "input_port_2"]
        for port_name in port_names:
            switch0.DeclareInputPort(name=port_name)

        # Check that the number of input ports is now 4
        self.assertEqual(switch0.num_input_ports(), 4)

    def test_DoCalcValue1(self):
        """
        Description
        -----------
        This test verifies that we can correctly calculate the value that should
        be sent through the port, for a proper diagram in Drake.
        """
        # Setup
        builder = DiagramBuilder()

        # Create a FlexiblePortSwitch with a switch type of int
        switch0 = FlexiblePortSwitch(7, selector_type_in=int)
        builder.AddSystem(switch0)

        # Declare three input ports with unique names
        port_names = ["input_port_0", "input_port_1"]
        for port_name in port_names:
            switch0.DeclareInputPort(name=port_name)

        # Create two DIFFERENT CONSTANT vector sources and connect them
        # to the input ports
        values = [np.array([1, 2, 3, 4, 5, 6, 7]), np.array([7, 6, 5, 4, 3, 2, 1])]
        sources = []
        for ii, value_ii in enumerate(values):
            source_ii = builder.AddSystem(ConstantVectorSource(value_ii))
            sources.append(source_ii)
            # Connect the source to the input port
            input_port_ii = switch0.GetInputPort(port_names[ii])
            builder.Connect(source_ii.get_output_port(0), input_port_ii)

        # Create a constant vector source for the selector input port
        selector_value = np.array([1])
        selector_source = builder.AddSystem(ConstantVectorSource(selector_value))
        builder.Connect(
            selector_source.get_output_port(0), switch0.get_port_selector_input_port()
        )

        # Build the diagram
        diagram = builder.Build()
        diagram_context = diagram.CreateDefaultContext()

        # Define Simulator
        simulator = Simulator(diagram, diagram_context)
        simulator.Initialize()
        simulator.AdvanceTo(0.1)

        # Attempt to calculate the value of the output port
        output_value = BasicVector(7)
        switch0.DoCalcValue(switch0.GetMyContextFromRoot(diagram_context), output_value)

        # Check that the output value is correct
        expected_value = values[selector_value[0]]
        self.assertTrue(np.allclose(output_value.get_value(), expected_value))


if __name__ == "__main__":
    unittest.main()
