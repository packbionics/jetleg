from typing import Callable, Tuple, Dict

from jetleg_control.gait_mode import GaitPhase
from jetleg_control.data import SensorData


ConditionFunc = Callable[[GaitPhase, SensorData], bool]

class RuleList:
    
    def __init__(self):
        """Constructs an empty mapping from condition to corresponding value"""

        self.rule_map = Dict[ConditionFunc, int]

    def add_rule(self, condition_test: ConditionFunc, val: int):
        """Adds a new rule to the dictionary of (rule, value)"""

        self.rule_map.update({condition_test: val})

    def evaluate(self, gait_phase: GaitPhase, sensor_data: SensorData) -> Tuple[int, int] | None:
        """Iterates through the set of rules until one passes and returns its value"""

        for rule in self.rule_map:
            if rule(gait_phase, sensor_data):
                return self.rule_map.get(rule)
            
        return None