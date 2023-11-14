from typing import List
from dataclasses import dataclass

@dataclass
class GaitPhase:
    stiffness: List[float]
    damping: List[float]
    equilibrium: List[float]

class GaitMode:
    """Describes a mostly closed cycle of repeated gait phases"""
    
    def __init__(self, gait_phases: List[GaitPhase]):
        """Constructs a Gait Mode with an ordered list of phases"""

        self.gait_phases = gait_phases

    def get(self, idx: int) -> GaitPhase:
        """Retrieves the gait phase at the given index"""

        if idx < 0 or idx >= len(self.gait_phases):
            raise ValueError(f"Invalid index given for gait phase: {idx}")

        return self.gait_phases[idx]