from dataclasses import dataclass

@dataclass
class Configuration:
    meshcat_port_number: int = None
    time_step: float = 1e-3