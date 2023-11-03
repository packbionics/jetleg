from abc import ABCMeta
from typing import List

from jetleg_control.data import SensorData
from jetleg_control.rule_list import RuleList
from jetleg_control.gait_mode import GaitMode, GaitPhase


class ClassifierInterface(metaclass=ABCMeta):
    """Interface defining the common methods of a gait classifier"""
    
    @classmethod
    def __subclasshook__(cls, __subclass: type) -> bool:
        return (
            hasattr(__subclass, "classify") and
            callable(__subclass.classify)
        )

class RuleBasedClassifier:
    """Gait classifier which follows pre-programmed rules for determining gait"""

    def __init__(self, initial_gait_phase: GaitPhase, rule_list: RuleList, gait_modes: List[GaitMode]):
        """Constructs a rule-based classifier"""

        self.gait_phase = initial_gait_phase

        self.rule_list = rule_list
        self.gait_modes = gait_modes

    def classify(self, sensor_data: SensorData) -> GaitPhase:
        """Determines the desired gait phase based on given sensor data"""

        results = self.rule_list.evaluate(self.gait_phase, sensor_data)

        # Check if a phase could not be classified
        if results is None:
            raise ValueError(f"RuleBasedClassifier could not determine the gait phase from given sensor data: {sensor_data}")
        
        # Otherwise, return the classified phase
        mode_idx = results[0]
        phase_idx = results[1]

        return self.gait_modes[mode_idx].get(phase_idx)
