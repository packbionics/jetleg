# Copyright 2023 Pack Bionics
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


from typing import Callable, Tuple

from jetleg_control.gait_mode import GaitPhase
from jetleg_control.data import SensorData


ConditionFunc = Callable[[GaitPhase, SensorData], bool]


class RuleList:

    def __init__(self):
        """Constructs an empty mapping from condition to corresponding value"""

        self.rule_map = dict()

    def add_rule(self, condition_test: ConditionFunc, val: Tuple[int, int]):
        """Adds a new rule to the dictionary of (rule, value)"""

        self.rule_map.update({condition_test: val})

    def evaluate(self, gait_phase: GaitPhase, sensor_data: SensorData) -> Tuple[int, int] | None:
        """Iterates through the set of rules until one passes and returns its value"""

        for rule in self.rule_map:
            if rule(gait_phase, sensor_data):
                return self.rule_map.get(rule)

        return None
