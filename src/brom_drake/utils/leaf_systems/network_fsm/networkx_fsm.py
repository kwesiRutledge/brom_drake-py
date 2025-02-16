import networkx as nx
from pydrake.all import (
    LeafSystem,
)
from typing import List, Union

# Internal Imports
from brom_drake.utils.leaf_systems.network_fsm.fsm_transition_condition import (
    FSMTransitionCondition, FSMTransitionConditionType
)
from brom_drake.utils.leaf_systems.network_fsm.fsm_edge_definition import FSMEdgeDefinition
from brom_drake.utils.leaf_systems.network_fsm.errors import (
    EdgeContainsNoConditionsError,
    MultipleConnectedComponentsError,
    NumberOfStartNodesError,
)

class NetworkXFSM(LeafSystem):
    def __init__(
        self,
        fsm_graph: nx.DiGraph,
    ):
        LeafSystem.__init__(self)

        # Prepare to Create FSM from Graph
        self.fsm_graph = fsm_graph

        # Check if the FSM is valid
        if not self.supports_this_digraph(self.fsm_graph):
            raise ValueError("The given directed graph is not a valid FSM.")

        # Collect all edge definitions from the graph
        self.edge_definitions = self.collect_edge_definitions(self.fsm_graph)

        # Create Input and Output Ports
        self.input_port_dict = {}
        self.create_input_ports()

    def create_input_ports(self):
        """
        Description
        -----------
        This function creates the input ports for the FSM.
        """
        # Setup
        fsm_graph = self.fsm_graph

        # Iterate through all edges of the FSM graph
        # and identify:
        #   - The input ports that are required for each edge
        input_port_names = []
        for ii, edge_ii in enumerate(fsm_graph.edges):
            edge_data = fsm_graph.edges[edge_ii]

            # Get the input port names
            for jj, name_jj in enumerate(edge_data["input_ports"]):
                # Add the input port if it is not already in the list
                if name_jj not in input_port_names: 
                    # Add all input port names to this list
                    input_port_names.append(name_jj)

                    # Check the value of the input port
                    self.input_port_dict[name_jj] = []
                    
    def collect_edge_definitions(
        self,
        fsm_graph: nx.DiGraph,
    ) -> List[FSMEdgeDefinition]:
        """
        Description
        -----------
        This function collects the edge definitions from the FSM graph.
        It uses the data from the graph to create an internal representation
        (FSMEdgeDefinition) for each edge in the graph.
        """
        # Setup
        edge_definitions = []

        # Iterate through all edges of the FSM graph
        # and identify:
        #   - The input ports that are required for each edge
        for ii, edge_ii in enumerate(fsm_graph.edges):
            src_ii, dst_ii = edge_ii 
            print(f"Edge: {src_ii} -> {dst_ii}")

            edge_data = fsm_graph.edges[edge_ii]

            # Get the input port names, conditions, and condition values
            input_port_names = edge_data["input_port_names"]
            condition_types = edge_data["condition_types"]
            condition_values = edge_data["condition_values"]

            assert len(input_port_names) == len(condition_types), \
                "Number of input ports and condition types must match."
            assert len(input_port_names) == len(condition_values), \
                "Number of input ports and condition values must match."
            
            # Create the FSMTransitionCondition objects
            conditions = []
            for jj, name_jj in enumerate(input_port_names):
                condition = FSMTransitionCondition(
                    input_port_name=name_jj,
                    condition_type=condition_types[jj],
                    condition_value=condition_values[jj],
                )
                conditions.append(condition)


            # Create the FSMEdgeDefinition object
            edge_definition = FSMEdgeDefinition(
                conditions=conditions,
                src=src_ii,
                dst=dst_ii,
            )

            # Add the edge definition to the list
            edge_definitions.append(edge_definition)

        return edge_definitions
    
    @staticmethod
    def supports_this_digraph(
        digraph: nx.DiGraph,
        debug_flag: bool = False,
    ) -> Union[None, ValueError]:
        """
        Description
        -----------
        This function checks if the given directed graph is a valid FSM.

        Parameters
        ----------
        digraph : nx.DiGraph
            The directed graph to check.

        Returns
        -------
        recognized_exception: Union[None, ValueError]
            Either an error if the graph is not a valid FSM, or None if the graph is valid.
        """

        # Check if the graph is connected
        connected_components = list(nx.weakly_connected_components(digraph))
        if debug_flag:
            print(f"Found {len(connected_components)} connected components.")
        
        if len(connected_components) > 1:
            return MultipleConnectedComponentsError(connected_components)


        # Check if the graph has a single start node
        start_nodes = [
            node
            for node in digraph.nodes
            if len(list(digraph.predecessors(node))) == 0
        ]
        if debug_flag:
            print(f"Found {len(start_nodes)} start nodes.")

        if len(start_nodes) != 1:
            return NumberOfStartNodesError(start_nodes)

        # Check that all edges have a unique label
        # TODO: List all edges and verify that each edge has at least one label
        for edge in digraph.edges:
            # print(edge)
            # print(digraph.edges[edge])
            if digraph.edges[edge] == {}:
                return EdgeContainsNoConditionsError(edge)
            
            

        return True