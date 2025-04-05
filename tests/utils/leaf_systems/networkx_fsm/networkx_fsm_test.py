import networkx as nx
from pydrake.all import (
    AbstractValue,
    ConstantValueSource,
    Simulator,
)
from pydrake.systems.framework import DiagramBuilder
import unittest

# Internal Imports
from brom_drake.all import add_watcher_and_build
from brom_drake.utils.leaf_systems.network_fsm import (
    NetworkXFSM,
    FSMTransitionCondition,
    FSMTransitionConditionType,
    FSMOutputDefinition,
)

class TestNetworkXFSM(unittest.TestCase):
    def setUp(self):
        # Create simple Digraphs for testing
        self.disconnectedDigraph = self.createDisconnectedDigraph()
        self.connectedDigraphWithMultipleRoots = self.createConnectedDigraphWithMultipleRoots()
        self.completelyLabeledButNoConditionedDigraph = self.createFullyLabeledButNoConditionedDigraph()

    def createDisconnectedDigraph(self) -> nx.DiGraph:
        # Setup
        graph = nx.DiGraph()

        # Create a disconnected Digraph for testing
        graph.add_node(0)
        graph.add_node(1)
        graph.add_node(2)

        # connect 0 -> 1
        graph.add_edge(0, 1)

        return graph
    
    def createConnectedDigraphWithMultipleRoots(self) -> nx.DiGraph:
        # Setup
        graph = nx.DiGraph()

        # Create a disconnected Digraph for testing
        graph.add_node(0)
        graph.add_node(1)
        graph.add_node(2)

        # connect 0 -> 1
        graph.add_edge(0, 1)
        graph.add_edge(2, 1)

        return graph
    
    def createPartiallyLabeledDigraph(self) -> nx.DiGraph:
        """
        Description
        -----------
        This function creates a partially labeled Digraph for testing.
        The labels on each edge will be named by "signal"
        """
        # Setup
        graph = nx.DiGraph()

        # Create a disconnected Digraph for testing
        graph.add_node(0)
        graph.add_node(1)
        graph.add_node(2)
        graph.add_node(3)

        # connect 0 -> 1 and 1 -> 2
        graph.add_edge(0, 1, conditions=[])
        graph.add_edge(1, 2, conditions=[])
        graph.add_edge(2, 3)

        return graph
    
    def createFullyLabeledButNoConditionedDigraph(self) -> nx.DiGraph:
        """
        Description
        -----------
        This function creates a partially labeled Digraph for testing.
        The labels on each edge will be named by "signal"
        """
        # Setup
        graph = nx.DiGraph()

        # Create a disconnected Digraph for testing
        graph.add_node(0)
        graph.add_node(1)
        graph.add_node(2)
        graph.add_node(3)

        # connect 0 -> 1 and 1 -> 2
        graph.add_edge(0, 1, conditions=[])
        graph.add_edge(1, 2, conditions=[])
        graph.add_edge(2, 3, conditions=[])

        return graph
    
    def createSimpleTwoNodeFSM(self) -> nx.DiGraph:
        # Setup
        graph = nx.DiGraph()

        # Create a disconnected Digraph for testing
        graph.add_node(
            0,
            outputs=[
                FSMOutputDefinition("output1", True),
            ]
        )
        graph.add_node(1)

        # connect 0 -> 1
        graph.add_edge(0, 1, conditions=[
            FSMTransitionCondition(
                input_port_name="signal",
                condition_type=FSMTransitionConditionType.kEqual,
                condition_value=True
            )
        ])

        return graph
    
    def createTwoNodeFSMWithUninitializedOutput(self) -> nx.DiGraph:
        # Setup
        graph = nx.DiGraph()

        # Create a disconnected Digraph for testing
        graph.add_node(
            0,
            outputs=[
                FSMOutputDefinition("output1", True),
            ]
        )
        graph.add_node(
            1,
            outputs=[
                FSMOutputDefinition("output1", False),
                FSMOutputDefinition("output2", False),
            ]
        )

        # connect 0 -> 1
        graph.add_edge(0, 1, conditions=[
            FSMTransitionCondition(
                input_port_name="signal",
                condition_type=FSMTransitionConditionType.kEqual,
                condition_value=True
            )
        ])

        return graph

    def test_supports_this_digraph1(self):
        """
        Description
        -----------
        This test verifies that we can check if a disconnected Digraph is a valid FSM.
        """
        # Setup
        digraph = self.disconnectedDigraph

        # Run
        result = NetworkXFSM.supports_this_digraph(digraph)

        # Verify
        self.assertIsNotNone(result)

    def test_supports_this_digraph2(self):
        """
        Description
        -----------
        This test verifies that we can check if a connected Digraph with multiple roots 
        is a valid FSM (it should not be).
        """
        # Setup
        digraph = self.connectedDigraphWithMultipleRoots

        # Run
        result = NetworkXFSM.supports_this_digraph(digraph)

        # Verify
        self.assertIsNotNone(result)

    def test_supports_this_digraph3(self):
        """
        Description
        -----------
        This test verifies that we can check if a partially labeled Digraph
        is a valid FSM (it should not be).
        """
        # Setup
        digraph = self.createPartiallyLabeledDigraph()

        # Run
        result = NetworkXFSM.supports_this_digraph(digraph, debug_flag=False)

        # Verify
        self.assertIsNotNone(result)

    def test_supports_this_digraph4(self):
        """
        Description
        -----------
        This test verifies that we can check if a fully labeled Digraph
        is a valid FSM (it should not be).
        """
        # Setup
        digraph = self.completelyLabeledButNoConditionedDigraph

        # Run
        result = NetworkXFSM.supports_this_digraph(digraph)

        # Verify
        self.assertIsNotNone(result)


    def test_supports_this_digraph5(self):
        """
        Description
        -----------
        This test verifies that we can check if a simple two-node FSM is a valid FSM.
        This one should be a correct FSM.
        """
        # Setup
        digraph = self.createSimpleTwoNodeFSM()

        # Run
        result = NetworkXFSM.supports_this_digraph(digraph)

        # Verify
        self.assertIsNone(result)

    def test_supports_this_digraph6(self):
        """
        Description
        -----------
        This test verifies that the supports_this_digraph() method
        correctly raises an error when the input DiGraph contains
        an output that is not initialized.
        """
        # Setup
        digraph = self.createTwoNodeFSMWithUninitializedOutput()

        # Run
        result = NetworkXFSM.supports_this_digraph(digraph)

        # Verify
        self.assertIsNotNone(result)

    def test_create_input_ports1(self):
        """
        Description
        -----------
        This test verifies that we can create the input ports for the FSM.
        """
        # Setup
        fsm_graph = self.createSimpleTwoNodeFSM()
        fsm = NetworkXFSM(fsm_graph)

        # Verify that the input ports were created
        self.assertGreater(
            len(list(fsm.input_port_dict.keys())),
            0,
        )

        # Verify that the input port with name "signal" was created
        try:
            signal_port = fsm.GetInputPort("signal")
            self.assertTrue(True)
        except Exception as e:
            self.assertTrue(False)

    def test_CalcFSMState1(self):
        """
        Description
        -----------
        This test verifies that the CalcFSMState method correctly
        transitions from the start node to the next node on
        the correct conditions being met.
        The FSM we use is a simple two-node FSM.
        """
        # Setup
        fsm_graph = self.createSimpleTwoNodeFSM()
        builder = DiagramBuilder()

        # Create a simple diagram
        fsm = builder.AddSystem(
            NetworkXFSM(fsm_graph)
        )

        signal_source = builder.AddSystem(
            ConstantValueSource(
                AbstractValue.Make(True)
            )
        )

        # Connect the signal source to the FSM
        builder.Connect(
            signal_source.get_output_port(),
            fsm.GetInputPort("signal")
        )


        # Build Diagram and advance it for 1 second
        watcher, diagram, diagram_context = add_watcher_and_build(builder)
        simulator = Simulator(diagram, diagram_context)

        # Run
        simulator.AdvanceTo(0.25)
        simulator.AdvanceTo(0.50)

        # Extract the FSM state data
        watcher_keys = list(watcher.port_watchers.keys())
        first_key = watcher_keys[0]

        fsm_state_log = watcher.port_watchers[first_key]["fsm_state"].logger.FindLog(diagram_context)
        fsm_state_log_data = fsm_state_log.data().flatten()

        # Verify that the current state is now 1
        self.assertEqual(fsm_state_log_data[-1], 1)

    def test_CalcFSMState2(self):
        """
        Description
        -----------
        This test verifies that the CalcFSMState method correctly
        transitions from the start node to the next node on
        the correct conditions being met.
        The condition that we will use is a time-based one.
        The FSM we use is a simple two-node FSM.
        """
        # Setup
        builder = DiagramBuilder()

        # Create a simple DiGraph to explain the transitions
        fsm_graph = nx.DiGraph()
        fsm_graph.add_node(
            0,
            outputs=[
                FSMOutputDefinition("output1", True),
            ]
        )
        fsm_graph.add_node(
            1,
            outputs=[
                FSMOutputDefinition("output1", False),
            ]
        )
        fsm_graph.add_edge(
            0, 1,
            conditions=[
                FSMTransitionCondition(
                    condition_type=FSMTransitionConditionType.kAfterThisManySeconds,
                    condition_value=0.75,
                )
            ]
        )

        # Create a simple diagram
        fsm = builder.AddSystem(
            NetworkXFSM(fsm_graph)
        )

        # Build Diagram and advance it for 1 second
        watcher, diagram, diagram_context = add_watcher_and_build(builder)
        simulator = Simulator(diagram, diagram_context)

        # Get key needed to extract watcher data for fsm state
        watcher_keys = list(watcher.port_watchers.keys())
        first_key = watcher_keys[0]
        fsm_state_logger = watcher.port_watchers[first_key]["fsm_state"].logger
        output1_logger = watcher.port_watchers[first_key]["output1"].logger

        # Run for 0.25 seconds and check that:
        # - the current state is still 0
        # - the output1 is still 1
        simulator.AdvanceTo(0.25)

        fsm_state_log = fsm_state_logger.FindLog(diagram_context)
        fsm_state_log_data = fsm_state_log.data().flatten()
        self.assertEqual(int(fsm_state_log_data[-1]), 0)
        
        output1_log = output1_logger.FindLog(diagram_context)
        output1_log_data = output1_log.data().flatten()
        self.assertEqual(int(output1_log_data[-1]), 1)

        # Run until 1 second and check that:
        # - the current state is now 1
        # - the output1 is now 0
        simulator.AdvanceTo(1.00)

        fsm_state_log = fsm_state_logger.FindLog(diagram_context)
        fsm_state_log_data = fsm_state_log.data().flatten()
        self.assertEqual(int(fsm_state_log_data[-1]), 1)

        output1_log = output1_logger.FindLog(diagram_context)
        output1_log_data = output1_log.data().flatten()
        self.assertEqual(int(output1_log_data[-1]), 0)

    def test_CalcFSMState3(self):
        """
        Description
        -----------
        This test verifies that the CalcFSMState method correctly
        transitions from the start node to the next 2 nodes on
        the correct conditions being met.
        The condition that we will use two time-based conditions
        to make sure that the timer is reset.
        The FSM we use is a simple two-node FSM.
        """
        # Setup
        builder = DiagramBuilder()

        # Create a simple DiGraph to explain the transitions
        fsm_graph = nx.DiGraph()
        fsm_graph.add_node(
            0,
            outputs=[
                FSMOutputDefinition("output1", True),
            ]
        )
        fsm_graph.add_node(
            1,
            outputs=[
                FSMOutputDefinition("output1", False),
            ]
        )
        fsm_graph.add_node(
            2,
            outputs=[
                FSMOutputDefinition("output1", True),
            ]
        )
        fsm_graph.add_edge(
            0, 1,
            conditions=[
                FSMTransitionCondition(
                    condition_type=FSMTransitionConditionType.kAfterThisManySeconds,
                    condition_value=0.75,
                )
            ]
        )
        fsm_graph.add_edge(
            1, 2,
            conditions=[
                FSMTransitionCondition(
                    condition_type=FSMTransitionConditionType.kAfterThisManySeconds,
                    condition_value=0.75,
                )
            ]
        )

        # Create a simple diagram
        fsm = builder.AddSystem(
            NetworkXFSM(fsm_graph)
        )

        # Build Diagram and advance it for 1 second
        watcher, diagram, diagram_context = add_watcher_and_build(builder)
        simulator = Simulator(diagram, diagram_context)

        # Collect data for the fsm state
        watcher_keys = list(watcher.port_watchers.keys())
        first_key = watcher_keys[0]
        fsm_state_logger = watcher.port_watchers[first_key]["fsm_state"].logger
        output1_logger = watcher.port_watchers[first_key]["output1"].logger

        # Run for 0.25 seconds and check that:
        # - the current state is still 0
        # - the output1 is still 1
        simulator.AdvanceTo(0.25)

        fsm_state_log = fsm_state_logger.FindLog(diagram_context)
        fsm_state_log_data = fsm_state_log.data().flatten()
        self.assertEqual(int(fsm_state_log_data[-1]), 0)

        output1_log = output1_logger.FindLog(diagram_context)
        output1_log_data = output1_log.data().flatten()
        self.assertEqual(int(output1_log_data[-1]), 1)

        # Run until 1 second and check that:
        # - the current state is now 1, and
        # - the output1 is now 0
        simulator.AdvanceTo(1.00)

        fsm_state_log = fsm_state_logger.FindLog(diagram_context)
        fsm_state_log_data = fsm_state_log.data().flatten()
        self.assertEqual(int(fsm_state_log_data[-1]), 1)

        output1_log = output1_logger.FindLog(diagram_context)
        output1_log_data = output1_log.data().flatten()
        self.assertEqual(int(output1_log_data[-1]), 0)

        # Run until 1.25 seconds and check that:
        # - the current state is still 1, and
        # - the output1 is still 0
        simulator.AdvanceTo(1.25)

        fsm_state_log = fsm_state_logger.FindLog(diagram_context)
        fsm_state_log_data = fsm_state_log.data().flatten()
        self.assertEqual(int(fsm_state_log_data[-1]), 1)

        output1_log = output1_logger.FindLog(diagram_context)
        output1_log_data = output1_log.data().flatten()
        self.assertEqual(int(output1_log_data[-1]), 0)

        # Run until 1.75 seconds and check that the current state is now 2
        simulator.AdvanceTo(1.75)
        
        fsm_state_log = fsm_state_logger.FindLog(diagram_context)
        fsm_state_log_data = fsm_state_log.data().flatten()
        self.assertEqual(int(fsm_state_log_data[-1]), 2)

        output1_log = output1_logger.FindLog(diagram_context)
        output1_log_data = output1_log.data().flatten()
        self.assertEqual(int(output1_log_data[-1]), 1)

if __name__ == "__main__":
    unittest.main()