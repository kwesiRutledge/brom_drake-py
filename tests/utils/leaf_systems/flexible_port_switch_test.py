from brom_drake.utils.leaf_systems.flexible_port_switch import FlexiblePortSwitch
from pydrake.all import (
    InputPort,
    PortDataType,
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
        input_port0 : InputPort = switch0.get_input_port(0)
        self.assertTrue(input_port0.get_data_type() == PortDataType.kAbstractValued)

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
        input_port0 : InputPort = switch0.get_input_port(0)

        self.assertTrue(input_port0.get_data_type() == PortDataType.kAbstractValued)

        # Declare the input port
        port_name = "input_port_0"
        switch0.DeclareInputPort(name=port_name)
        self.assertEqual(switch0.num_input_ports(), 2)

        # check that the input port named "input_port_0" is a vector input port
        input_port1 : InputPort = switch0.GetInputPort(port_name)
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
        input_port0 : InputPort = switch0.get_input_port(0)
        self.assertTrue(input_port0.get_data_type() == PortDataType.kVectorValued)

        # Declare three input ports with unique names
        port_names = ["input_port_0", "input_port_1", "input_port_2"]
        for port_name in port_names:
            switch0.DeclareInputPort(name=port_name)

        # Check that the number of input ports is now 4
        self.assertEqual(switch0.num_input_ports(), 4)

        # Check that the input ports are of the correct type
        for port_name in port_names:
            input_port : InputPort = switch0.GetInputPort(port_name)
            self.assertTrue(input_port.get_data_type() == PortDataType.kVectorValued)
            self.assertEqual(input_port.size(), 7)


if __name__ == "__main__":
    unittest.main()