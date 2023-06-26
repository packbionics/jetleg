class ImpedanceParameters:

    def __init__(self):
        pass

    def get_params(self):
        pass


class GaitState:

    def __init__(self):
        self._impedance_params = ImpedanceParameters()
        self._next_state = None

    def set_next_state(self, gait_state):
        self._next_state = gait_state

    def transition(self):
        return self._next_state

    def get_impedance_params(self):
        return self._impedance_params.get_params()