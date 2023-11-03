from jetleg_control.gait_mode import GaitMode, GaitPhase


def test_get():
    gait_phases = list()

    # Number of phases to add to gait mode
    n = 4
    for i in range(n):
        gait_phases.append(GaitPhase())

    gait_mode = GaitMode(gait_phases=gait_phases)

    for i in range(n):
        assert gait_mode.get(i) is not None

    try:
        gait_mode.get(-1)
        assert False
    except ValueError as e:
        assert str(e) == "Invalid index given for gait phase: -1"

    try:
        gait_mode.get(4)
        assert False
    except ValueError as e:
        assert str(e) == "Invalid index given for gait phase: 4"