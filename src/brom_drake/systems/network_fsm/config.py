from dataclasses import dataclass

@dataclass
class NetworkXFSMConfig:
    """
    Configuration for the NetworkXFSM system.
    
    Attributes:
        name (str): Name of the FSM.
        initial_state (str): Initial state of the FSM.
        states (list): List of states in the FSM.
        transitions (dict): Mapping of transitions between states.
    """
    name: str = "NetworkXFSM"
    log_file_name: str = "network_fsm.log"
    show_logs_in_terminal: bool = True