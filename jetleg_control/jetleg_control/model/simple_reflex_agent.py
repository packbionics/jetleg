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


from typing import Iterable

from jetleg_control.data import Action, Percept
from jetleg_control.gait_phase import GaitPhase


class SimpleReflexAgent:
    """Gait classifier which follows pre-programmed rules for determining gait."""

    def __init__(self, gait_phases: Iterable[GaitPhase]):
        self.gait_phases = gait_phases

    def get_action(self, percept: Percept) -> Action:
        """Determine the desired gait phase based on given sensor data."""
        gait_phase = self._make_transition(percept)

        action = Action(gait_phase)
        return action

    def _make_transition(self, percept: Percept) -> GaitPhase:
        """Specify transitions from any arbitrary discrete gait phase."""
        phases = self.gait_phases
        if percept.gait_phase == phases[0]:
            return self.first_phase_transitions(percept)
        else:
            return self.second_phase_transitions(percept)

    def first_phase_transitions(self, percept: Percept):
        """Specify transitions from one gait phase."""
        gait_phases = self.gait_phases
        if percept.imu_mean < 0:
            return gait_phases[0]
        else:
            return gait_phases[1]

    def second_phase_transitions(self, percept: Percept):
        """Specify transitions from one gait phase."""
        gait_phases = self.gait_phases
        if percept.imu_mean >= 0:
            return gait_phases[1]
        else:
            return gait_phases[0]
