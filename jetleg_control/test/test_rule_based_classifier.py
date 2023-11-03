from jetleg_control.classifier import RuleBasedClassifier
from jetleg_control.gait_mode import GaitMode, GaitPhase
from jetleg_control.data import SensorData
from jetleg_control.rule_list import RuleList


def test_init():


    num_phases = 4
    phases = list()

    for _ in range(num_phases):
        phases.append(GaitPhase())

    startphase = phases[2]

    # Define set of rules for making transitions
    rule1 = lambda x, y: x == phases[0] and y.imu_mean > 5
    rule2 = lambda x, y: x == phases[0] and y.imu_mean <= 5

    rule3 = lambda x, y: x == phases[1] and y.imu_mean > 7
    rule4 = lambda x, y: x == phases[1] and y.imu_mean <= 5
    rule5 = lambda x, y: x == phases[1] and y.imu_mean <= 7 and y.imu_mean > 5

    rule6 = lambda x, y: x == phases[2] and y.imu_mean > 0
    rule7 = lambda x, y: x == phases[2] and y.imu_mean <= 0

    rule8 = lambda x, y: x == phases[3] and y.imu_mean < 5
    rule9 = lambda x, y: x == phases[3] and y.imu_mean >= 5

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
    
    singleton_gait_modes = [GaitMode(phases)]

    classifier = RuleBasedClassifier(startphase, simple_rule_list, singleton_gait_modes)

    assert len(classifier.gait_modes) == 1
    assert classifier.gait_phase == startphase
