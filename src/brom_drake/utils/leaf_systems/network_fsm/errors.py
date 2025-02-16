
from typing import List, Set, Tuple

class MultipleConnectedComponentsError(ValueError):
    """
    Description
    -----------
    Raised when the FSM graph has multiple connected components.
    """
    def __init__(self, connected_components: List[Set[int]]):
        self.connected_components = connected_components
        self.message = f"Found {len(connected_components)} connected components in the FSM graph.\n"
        self.message += f"Connected components are: {connected_components}.\n"
        super().__init__(self.message)

class NumberOfStartNodesError(ValueError):
    """
    Description
    -----------
    Raised when the FSM graph has multiple start nodes.
    """
    def __init__(self, start_nodes: List[int]):
        self.start_nodes = start_nodes
        self.message = f"Found {len(start_nodes)} start nodes in the FSM graph.\n"
        self.message += f"Start nodes are: {start_nodes}.\n"
        super().__init__(self.message)

class EdgeContainsNoConditionsError(ValueError):
    """
    Description
    -----------
    Raised when an edge in the FSM graph contains no conditions.
    """
    def __init__(self, edge: Tuple[int, int]):
        self.edge = edge
        self.message = f"Edge {edge} contains 0 conditions; need at least one!\n"
        super().__init__(self.message)