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


from jetleg_control.rule_list import RuleList
from jetleg_control.gait_mode import GaitPhase
from jetleg_control.data import SensorData


def test_evaluate():
    rule_list = RuleList()

    gait_phase = GaitPhase(0.0, 0.0, 0.0)
    wrong_phase = GaitPhase(0.0, 0.0, 0.0)

    # Test equality of gait phase
    def same_phase_condition(x, y): return x == gait_phase
    rule_list.add_rule(same_phase_condition, (0, 1))

    result = rule_list.evaluate(gait_phase, None)
    assert result is not None

    assert result == (0, 1)

    # Retrieve None when no rule applies
    result = rule_list.evaluate(wrong_phase, None)
    assert result is None

    # Test less than relation of sensor data
    def less_than_condition(x, y): return y.imu_mean < 0.0
    rule_list.add_rule(less_than_condition, (3, 2))

    pass_data = SensorData(-0.5)
    fail_data = SensorData(0.5)

    result = rule_list.evaluate(None, pass_data)
    assert result is not None

    assert result == (3, 2)

    result = rule_list.evaluate(None, fail_data)
    assert result is None

    # Make sure previously added rules still apply
    result = rule_list.evaluate(gait_phase, None)
    assert result is not None

    assert result == (0, 1)
