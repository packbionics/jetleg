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


from jetleg_control.gait_mode import GaitMode, GaitPhase
from jetleg_control.rule_list import RuleList


def construct() -> RuleList:

    num_phases = 4
    phases = list()

    for _ in range(num_phases):
        phases.append(GaitPhase())

    startphase = phases[2]

    singleton_gait_modes = [GaitMode(phases)]

    # Define set of rules for making transitions
    def rule1(x, y): return x == phases[0] and y.imu_mean > 5
    def rule2(x, y): return x == phases[0] and y.imu_mean <= 5

    def rule3(x, y): return x == phases[1] and y.imu_mean > 7
    def rule4(x, y): return x == phases[1] and y.imu_mean <= 5
    def rule5(x, y): return x == phases[1] and y.imu_mean <= 7 and y.imu_mean > 5

    def rule6(x, y): return x == phases[2] and y.imu_mean > 0
    def rule7(x, y): return x == phases[2] and y.imu_mean <= 0

    def rule8(x, y): return x == phases[3] and y.imu_mean < 5
    def rule9(x, y): return x == phases[3] and y.imu_mean >= 5

    simple_rule_list = RuleList()

    simple_rule_list.add_rule(rule1, (0, 0))
    simple_rule_list.add_rule(rule2, (0, 1))

    simple_rule_list.add_rule(rule3, (0, 1))
    simple_rule_list.add_rule(rule4, (0, 2))
    simple_rule_list.add_rule(rule5, (0, 0))

    simple_rule_list.add_rule(rule6, (0, 2))
    simple_rule_list.add_rule(rule7, (0, 3))

    simple_rule_list.add_rule(rule8, (0, 3))
    simple_rule_list.add_rule(rule9, (0, 0))

    return startphase, simple_rule_list, singleton_gait_modes


def test_init():
    assert True
    # classifier = RuleBasedClassifier(startphase, simple_rule_list, singleton_gait_modes)

    # assert len(classifier.gait_modes) == 1
    # assert classifier.gait_phase == startphase
