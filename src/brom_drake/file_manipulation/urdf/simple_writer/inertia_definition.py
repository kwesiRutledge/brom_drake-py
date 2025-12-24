from dataclasses import dataclass

@dataclass
class InertiaDefinition:
    """
    *Description*

    A dataclass that defines the inertia of a simple shape.

    This is defined to match the URDF specifications for inertia.

    *Attributes*

    ixx: float
        The xx component of the inertia tensor.

    ixy: float
        The xy component of the inertia tensor.

    ixz: float
        The xz component of the inertia tensor.

    iyy: float
        The yy component of the inertia tensor.

    iyz: float
        The yz component of the inertia tensor.

    izz: float
        The zz component of the inertia tensor.
    """
    ixx: float = 1.0
    ixy: float = 0.0
    ixz: float = 0.0
    iyy: float = 1.0
    iyz: float = 0.0
    izz: float = 1.0

    def as_list(self) -> list[float]:
        """
        *Description*

        Convert the inertia to a list.

        *Returns*

        list[float]
            A list of the inertia values.
        """
        return [self.ixx, self.ixy, self.ixz, self.iyy, self.iyz, self.izz]

    def as_map(self) -> dict[str, str]:
        """
        *Description*

        Convert the inertia to a map.

        *Returns*

        dict[str, str]
            A map of the inertia values.
        """
        return {
            "ixx": f"{self.ixx}",
            "ixy": f"{self.ixy}",
            "ixz": f"{self.ixz}",
            "iyy": f"{self.iyy}",
            "iyz": f"{self.iyz}",
            "izz": f"{self.izz}",
        }