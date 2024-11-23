from dataclasses import dataclass

@dataclass
class InertiaDefinition:
    """
    A dataclass that defines the inertia of a simple shape.
    """
    ixx: float = 1.0
    ixy: float = 0.0
    ixz: float = 0.0
    iyy: float = 1.0
    iyz: float = 0.0
    izz: float = 1.0

    def as_list(self) -> list:
        """
        Convert the inertia to a list.
        :return: A list of the inertia values.
        """
        return [self.ixx, self.ixy, self.ixz, self.iyy, self.iyz, self.izz]

    def as_map(self) -> dict:
        """
        Convert the inertia to a map.
        :return: A map of the inertia values.
        """
        return {
            "ixx": f"{self.ixx}",
            "ixy": f"{self.ixy}",
            "ixz": f"{self.ixz}",
            "iyy": f"{self.iyy}",
            "iyz": f"{self.iyz}",
            "izz": f"{self.izz}",
        }