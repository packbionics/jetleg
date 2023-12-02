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


from abc import ABCMeta
from typing import Callable

from jetleg_control.data import SensorData
from jetleg_control.gait_phase import GaitPhase


class ClassifierInterface(metaclass=ABCMeta):
    """Interface defining the common methods of a gait classifier."""

    @classmethod
    def __subclasshook__(cls, __subclass: type) -> bool:
        return (
            hasattr(__subclass, "classify") and
            callable(__subclass.classify)
        )


class RuleBasedClassifier:
    """Gait classifier which follows pre-programmed rules for determining gait."""

    TransitionModel = Callable[[GaitPhase, SensorData], GaitPhase]

    def __init__(self, initial_gait_phase: GaitPhase, transitions: TransitionModel):
        """Construct a rule-based classifier."""
        self.gait_phase = initial_gait_phase
        self.transitions = transitions

    def classify(self, sensor_data: SensorData) -> GaitPhase:
        """Determine the desired gait phase based on given sensor data."""
        self.gait_phase = self.transitions(self.gait_phase, sensor_data)
        return self.gait_phase
