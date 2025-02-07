from dataclasses import dataclass
import numpy as np

@dataclass
class PlanningNode:
    id: int
    q: np.ndarray

    def __hash__(self):
        return hash(self.id)
    
    def __eq__(self, other):
        return self.id == other.id