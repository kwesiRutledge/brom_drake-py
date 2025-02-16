import networkx as nx
import unittest

# Internal Imports
from brom_drake.utils.leaf_systems.network_fsm import NetworkXFSM

class TestNetworkXFSM(unittest.TestCase):
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
        graph.add_edge(0, 1, conditions=[])
        graph.add_edge(1, 2, conditions=[])
        graph.add_edge(2, 3)

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
        result = NetworkXFSM.supports_this_digraph(digraph, debug_flag=True)

        # Verify
        self.assertIsNotNone(result)


if __name__ == "__main__":
    unittest.main()