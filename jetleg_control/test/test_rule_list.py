from jetleg_control.rule_list import RuleList
from jetleg_control.gait_mode import GaitPhase


def test_evaluate():
    rule_list = RuleList()

    gait_phase = GaitPhase()

    same_phase_condition = lambda x, y: x == gait_phase
    rule_list.add_rule(same_phase_condition, (0, 1))

    result = rule_list.evaluate(gait_phase, None)
    assert result is not None

    assert result == (0, 1)
    