"""
Description:
    This file defines a Role class that is used to populate the
    scene with ONLY systems that match the appropriate structure/signature.
"""
from dataclasses import dataclass

@dataclass
class Role:
    name: str
    description: str
    required_input_ports: list[str] # TODO(kwesi): Should this be a list of tuples? With port names AND types?
    required_output_ports: list[str]

    def __str__(self):
        return self.name