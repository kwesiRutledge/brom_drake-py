"""
Description:
    This file defines a Role class that is used to populate the
    scene with ONLY systems that match the appropriate structure/signature.
"""

class Role:
    def __init__(self, name, description):
        self.name = name
        self.description = description
        self.required_input_ports = []
        self.required_output_ports = []

    def __str__(self):
        return self.name