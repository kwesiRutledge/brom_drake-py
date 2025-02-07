import networkx as nx

def digraph_is_fsm_ready(digraph: nx.DiGraph, debug_flag: bool = False) -> bool:
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
    bool
        True if the graph can be used to define a valid FSM, False otherwise.
    """

    # Check if the graph is connected
    connected_components = list(nx.weakly_connected_components(digraph))
    if debug_flag:
        print(f"Found {len(connected_components)} connected components.")
    
    if len(connected_components) > 1:
        return False


    # Check if the graph has a single start node
    start_nodes = [
        node
        for node in digraph.nodes
        if len(list(digraph.predecessors(node))) == 0
    ]
    if debug_flag:
        print(f"Found {len(start_nodes)} start nodes.")

    if len(start_nodes) != 1:
        return False

    # Check that all edges have a unique label
    # TODO: List all edges and verify that each edge has at least one label
    for edge in digraph.edges:
        print(edge)
        print(digraph.edges[edge])
        if digraph.edges[edge] == {}:
            return False

    return True