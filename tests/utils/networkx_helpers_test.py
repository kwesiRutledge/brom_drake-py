import networkx as nx
import unittest

# Internal Imports
from brom_drake.utils.networkx_helpers import digraph_is_fsm_ready

class TestNetworkxHelpers(unittest.TestCase):
    def setUp(self):
        # Create simple Digraphs for testing
        self.disconnectedDigraph = self.createDisconnectedDigraph()
        self.connectedDigraphWithMultipleRoots = self.createConnectedDigraphWithMultipleRoots()

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
        graph.add_edge(0, 1, flag=True)
        graph.add_edge(1, 2, flag=False)
        graph.add_edge(2, 3)

        return graph

    def test_digraph_is_fsm_ready1(self):
        """
        Description
        -----------
        This test verifies that we can check if a disconnected Digraph is a valid FSM.
        """
        # Setup
        digraph = self.disconnectedDigraph

        # Run
        result = digraph_is_fsm_ready(digraph)

        # Verify
        self.assertFalse(result)

    def test_digraph_is_fsm_ready2(self):
        """
        Description
        -----------
        This test verifies that we can check if a connected Digraph with multiple roots 
        is a valid FSM (it should not be).
        """
        # Setup
        digraph = self.connectedDigraphWithMultipleRoots

        # Run
        result = digraph_is_fsm_ready(digraph)

        # Verify
        self.assertFalse(result)

    def test_digraph_is_fsm_ready3(self):
        """
        Description
        -----------
        This test verifies that we can check if a partially labeled Digraph
        is a valid FSM (it should not be).
        """
        # Setup
        digraph = self.createPartiallyLabeledDigraph()

        # Run
        result = digraph_is_fsm_ready(digraph, debug_flag=True)

        # Verify
        self.assertFalse(result)

if __name__ == "__main__":
    unittest.main()
