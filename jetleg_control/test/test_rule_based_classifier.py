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


import pytest

from jetleg_control.gait_phase import GaitPhase
from jetleg_control.data import SensorData
from jetleg_control.classifier import RuleBasedClassifier


@pytest.fixture
def phases():
    return [GaitPhase([0.0, 0.0], [0.0, 0.0], [0.0, 0.0]),
            GaitPhase([1.0, 0.0], [0.0, 0.0], [0.0, 0.0]),
            GaitPhase([2.0, 0.0], [0.0, 0.0], [0.0, 0.0]),
            GaitPhase([3.0, 0.0], [0.0, 0.0], [0.0, 0.0])]


@pytest.fixture
def transition_model(phases):

    def make_transition(phase: GaitPhase, data: SensorData) -> GaitPhase:
        if phase == phases[0]:
            if data.imu_mean > 5:
                return phases[0]
            else:
                return phases[1]
        elif phase == phases[1]:
            if data.imu_mean > 7:
                return phases[1]
            elif data.imu_mean < 5:
                return phases[2]
            else:
                return phases[0]
        elif phase == phases[2]:
            if data.imu_mean > 0:
                return phases[2]
            else:
                return phases[3]
        else:
            if data.imu_mean < 5:
                return phases[3]
            else:
                return phases[0]

    return make_transition


def test_init(phases, transition_model):

    classifier = RuleBasedClassifier(phases[0], transition_model)

    assert classifier.gait_phase == phases[0]
    assert classifier.transitions == transition_model
