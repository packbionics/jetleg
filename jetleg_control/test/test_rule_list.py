from jetleg_control.rule_list import RuleList
from jetleg_control.gait_mode import GaitPhase
from jetleg_control.data import SensorData


def test_evaluate():
    rule_list = RuleList()

    gait_phase = GaitPhase()
    wrong_phase = GaitPhase()

    # Test equality of gait phase
    same_phase_condition = lambda x, y: x == gait_phase
    rule_list.add_rule(same_phase_condition, (0, 1))

    result = rule_list.evaluate(gait_phase, None)
    assert result is not None

    assert result == (0, 1)

    # Retrieve None when no rule applies
    result = rule_list.evaluate(wrong_phase, None)
    assert result is None

    # Test less than relation of sensor data
    less_than_condition = lambda x, y: y.imu_mean < 0.0
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
    